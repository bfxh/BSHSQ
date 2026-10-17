//! **M6 浮体系泊金样**（vxl-only 通道）—— `ROUTE` M6 出口判据「帆/浮体/车金样」里的**浮体**那一个。
//!
//! 场景（把 M5 浮箱的介质场浮力 + M6 的约束族拼起来）：体素水槽 + **铸装水块**（`[8,8,8]@0.05`，
//! 口径同 `m5_float_box`）+ **轻盒浮体**（`0.12 m` 立方、密度 `300`）+ 一根**系泊绳**
//! （`JointKind::Distance`，锚点钉在池底 `y = 1.0`，绳长 = 初始间距 `0.30 m`）。
//!
//! 两个子场景，判据全按 **`f32` 位模式**逐位（口径同 `m4_cantilever` / `m5_float_box`）：
//! ① **被拉住**（`break_force = 0` ⇒ 绳永不断）：浮力把绳绷紧，浮体停在绳长处 —— 钉末态 `y` 与 `v_y`；
//! ② **被扯断**（`break_force = 5 N`）：浮力（净 ≈ 12 N）超过绳的断裂力 ⇒ 绳断，浮体浮到水面 ——
//!    钉末态 `y`、`v_y`、以及 `is_broken`。
//!
//! ⚠️ **绳长取的是初始间距**：`rest ≠ 初始距离` 时求解器会把间距直接压到 0（`rest` 形同虚设），
//! 已登记 `docs/OPEN-PROBLEMS.md`。本档因此让浮体**从绳长处起步**，靠浮力去绷紧它。
//!
//! 运行：`cargo run --release -p vxl-phys --example m6_float_mooring -- [ticks]`
//! 门禁配方见 `scripts/gate_gold.sh`。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};

const GATE_TICKS: usize = 720;
/// 绳长 = 浮体初始离锚点的距离（见头注：必须等于初始间距）。
const ROPE_LEN: f32 = 0.30;
/// 断裂档的绳断裂力（N）：浮体重 ≈ 5.1 N 挂在绳上 ⇒ 取 2 N 必然被扯断。
const BREAK_FORCE: f32 = 2.0;

/// 水槽 + 铸装水块 + 系泊轻盒。`break_force = 0` ⇒ 绳永不断。
fn scene(break_force: f32) -> (World, u32, u32) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    // 围堰：只留中心格 (2,2) 敞开 ⇒ 腔体 x,z ∈ [-0.25, 0.25]（同 `m5_float_box`）。
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    let voxel_id = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    w.add_fluid(sys, &[voxel_id]);
    // 锚点在**水面上方**（像系在桥墩上的浮标）：浮力往上顶、绳往下拉，才能稳定在绳长处。
    // （第一版把锚点放池底 ⇒ 绳把浮体往池底拽，直接穿模到 y=0.99 且还在下落。）
    let body_y = 1.30;
    let anchor = w.add_static(
        Shape::Sphere { radius: 0.02 },
        Vec3::new(0.0, body_y + ROPE_LEN, 0.0),
        Quat::IDENTITY,
    );
    let body = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, body_y, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    let joint =
        Joint::new(JointKind::Distance, anchor, body, Vec3::ZERO, Vec3::ZERO).with_rest(ROPE_LEN);
    let j = w.joints.add_breakable(joint, break_force);
    (w, body, j)
}

/// 跑 `ticks`，返回 `(粒子数, 末态浮体心 y, 末态 v_y, 绳是否断了)`。
fn run(break_force: f32, ticks: usize) -> (usize, f32, f32, bool) {
    let (mut w, body, j) = scene(break_force);
    for _ in 0..ticks {
        w.step();
    }
    let n = w
        .fluids()
        .first()
        .map(|(sys, ..)| sys.positions().len())
        .unwrap_or(0);
    let bi = body as usize;
    (
        n,
        w.bodies.position[bi].y,
        w.bodies.linvel[bi].y,
        w.joints.is_broken(j),
    )
}

/// 冻结基线（配方 = [`GATE_TICKS`] tick）。**存 `f32` 位模式**（判定是逐位的）。
struct Frozen {
    n: usize,
    y_held: u32,
    vy_held: u32,
    y_broken: u32,
    vy_broken: u32,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
///
/// ⚠️ `vy_held` 在修 `Distance` 关节的冲量符号之前是 `0x35c0_b434`，修后变成 `0xb5bf_4bca`
/// —— **只有符号翻转、绝对值不变**（两端都约 `1.4e-6`，浮体静止在绳长处）。同一次修复里
/// `y_held` / `y_broken` / `vy_broken` 三项**逐位不变** ⇒ 修好的是"发散"而不是"平衡点"。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        n: 512,
        y_held: 0x3fa6_6666,
        vy_held: 0xb5bf_4bca,
        y_broken: 0x3f9a_ddfb,
        vy_broken: 0x3c9b_fa9f,
    })
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);

    let (n, y_held, vy_held, held_broken) = run(0.0, ticks);
    let (_, y_broken, vy_broken, broken_broken) = run(BREAK_FORCE, ticks);
    println!(
        "M6-FLOAT-MOORING ticks={ticks} 粒子={n}\n  \
         被拉住：y={y_held:.9} v_y={vy_held:.9} 断={held_broken}\n  \
         被扯断：y={y_broken:.9} v_y={vy_broken:.9} 断={broken_broken}"
    );
    println!(
        "  位模式 held y={:#010x} v_y={:#010x} | broken y={:#010x} v_y={:#010x}",
        y_held.to_bits(),
        vy_held.to_bits(),
        y_broken.to_bits(),
        vy_broken.to_bits()
    );

    let Some(fz) = frozen(ticks) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方 = {GATE_TICKS} tick）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    if n != fz.n {
        bad.push(format!("粒子数 {n} ≠ 基线 {}", fz.n));
    }
    if y_held.to_bits() != fz.y_held {
        bad.push(format!("[被拉住] y {y_held:.9} ≠ 基线 {:#010x}", fz.y_held));
    }
    if vy_held.to_bits() != fz.vy_held {
        bad.push(format!(
            "[被拉住] v_y {vy_held:.9} ≠ 基线 {:#010x}",
            fz.vy_held
        ));
    }
    if y_broken.to_bits() != fz.y_broken {
        bad.push(format!(
            "[被扯断] y {y_broken:.9} ≠ 基线 {:#010x}",
            fz.y_broken
        ));
    }
    if vy_broken.to_bits() != fz.vy_broken {
        bad.push(format!(
            "[被扯断] v_y {vy_broken:.9} ≠ 基线 {:#010x}",
            fz.vy_broken
        ));
    }
    if held_broken {
        bad.push("[被拉住] 绳不该断，实测断了".to_string());
    }
    if !broken_broken {
        bad.push("[被扯断] 绳应当断，实测没断".to_string());
    }
    if bad.is_empty() {
        println!("✅ 金样基线 PASS（m6_float_mooring / {ticks} tick）");
    } else {
        println!(
            "❌ 金样基线 FAIL（m6_float_mooring / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
