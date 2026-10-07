//! **M5 浮箱金样**（vxl-only 通道）—— `ROUTE` M5 出口判据「溃坝/浮箱金样」里的**浮箱**那一半
//! （溃坝在 `m5_dam_break.rs`）。
//!
//! 场景（与 `tests/fluid_coupling.rs::buoyancy_raises_light_box` **同构**）：体素水槽
//! （中心腔 `0.5×0.5 m`、地板顶 `y = 1.0`）+ **铸装水块** `[8,8,8]@0.05`（按沉降后几何
//! 就近就位 —— `PLAN-0.3.md` §4.2 的"必须铸装"结论，带落差入盆会触发驻留瞬态顶心喷泉）
//! + **轻盒**（`0.12 m` 立方、密度 `300`）从**深潜位** `y = 1.10` 靠浮力上浮。
//!
//! **判据**（三项，全部按 **`f32` 位模式**逐位，口径同 `m4_cantilever` / `m5_dam_break`）：
//! ① 粒子数（守恒）；② 末态**盒心 `y`**（吃水深度）；③ 末态盒的 **`v_y`**（应≈0 = 已到平衡）。
//!
//! **与 `fluid_coupling.rs` 的分工**：那里判"**方向**对"（轻盒上浮 / 重盒下沉 / 干区对照照常
//! 下落），本档钉"**数值**对"（同配方末态逐位可复现）—— 单测管方向、金样管数值，与 M3/M4
//! 同一款分工。
//!
//! 运行：`cargo run --release -p vxl-phys --example m5_float_box -- [ticks]`
//! 门禁配方：`720` —— **实测选窗**：180 tick 时盒子还在动（`v_y = −0.191`）、360 仍在动
//! （`+0.057`）、**720 起完全入睡**（`v_y` 精确 `0`，且 720 与 1200 的 `y` 逐位相同）
//! ⇒ 取 720 才钉的是"浮到平衡"，而不是"某一瞬间的相位"。见 `scripts/gate_gold.sh`。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

const GATE_TICKS: usize = 720;

/// 水槽 + 铸装水块 + 水中轻盒 ⇒ 返回 `(世界, 轻盒 id)`。
fn scene() -> (World, u32) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    // 围堰：只留中心格 (2,2) 敞开 ⇒ 腔体 x,z ∈ [-0.25, 0.25]（同 `fluid_coupling.rs`）。
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
    // 轻盒从**水面以下 ~0.15 m** 起步 ⇒ 必须靠浮力升上来（不是"放对位置"）。
    let light = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.10, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    (w, light)
}

/// 跑 `ticks`，返回 `(粒子数, 末态盒心 y, 末态 v_y)`。
fn run(ticks: usize) -> (usize, f32, f32) {
    let (mut w, light) = scene();
    for _ in 0..ticks {
        w.step();
    }
    let n = w
        .fluids()
        .first()
        .map(|(sys, ..)| sys.positions().len())
        .unwrap_or(0);
    let y = w.bodies.position[light as usize].y;
    let vy = w.bodies.linvel[light as usize].y;
    (n, y, vy)
}

/// 冻结基线（配方 = `180` tick）。**存 `f32` 位模式**（判定是逐位的，见头注）。
struct Frozen {
    n: usize,
    y: u32,
    vy: u32,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        n: 512,
        y: 0x3f9b_5cf5,
        vy: 0x0000_0000,
    })
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);

    let (n, y, vy) = run(ticks);
    println!(
        "M5-FLOAT-BOX ticks={ticks} 粒子={n} 盒心y={y:.9} v_y={vy:.9}（地板顶 1.0、起点 1.10）"
    );
    println!(
        "  位模式 y={:#010x} v_y={:#010x}",
        y.to_bits(),
        vy.to_bits()
    );

    let Some(fz) = frozen(ticks) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方 = {GATE_TICKS} tick）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    if n != fz.n {
        bad.push(format!("粒子数 {n} ≠ 基线 {}", fz.n));
    }
    if y.to_bits() != fz.y {
        bad.push(format!(
            "盒心 y {y:.9}（{:#010x}）≠ 基线 {:#010x}",
            y.to_bits(),
            fz.y
        ));
    }
    if vy.to_bits() != fz.vy {
        bad.push(format!(
            "v_y {vy:.9}（{:#010x}）≠ 基线 {:#010x}",
            vy.to_bits(),
            fz.vy
        ));
    }
    if bad.is_empty() {
        println!(
            "✅ 金样基线 PASS（m5_float_box / {ticks} tick）：粒子 {n}，盒心 y {y:.9}，v_y {vy:.9}"
        );
    } else {
        println!(
            "❌ 金样基线 FAIL（m5_float_box / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
