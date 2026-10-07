//! **M5 溃坝金样**（vxl-only 通道）—— `ROUTE` M5 出口判据「溃坝/浮箱金样」里的**溃坝**那一半
//! （浮箱要刚-液双向，属另一半）。
//!
//! 场景（与演示 `examples/dam_break.rs` 同构，但**不转储渲染帧**、只出判据量）：
//! 体素盆（地板 2 层 + 围堰 1 层、内域 `1.5×1.5 m`）+ **自由水柱** `0.25×0.25×0.65 m`
//! （`6×6×14 = 504` 粒、间距 `0.05`）贴 `−x` 围堰 ⇒ 失支撑坍塌、波前沿向 `+x` 推进。
//!
//! **判据**（三项确定性量，全部按 **`f32` 位模式**比较，口径同 `m4_cantilever`）：
//! ① 粒子数（守恒）；② 末态**质心** `(x,y,z)`；③ 末态**最大 `x`**（波前沿推进到哪）。
//! **为什么用这三样**：它们是"流动真的发生了"的机器无关签名（质心随坍塌前移、前沿推进），
//! 且都是 SPH 的确定性输出（同配方两次连跑应逐位一致 —— 本档已实测确认）。
//!
//! ⚠️ **不是"驻留瞬态"断言**：自由柱坍塌本就该飞溅（`dam_break.rs` 头注的分工），本档只钉
//! "同配方下流动形状可复现"，不声称末态是物理平衡态。
//!
//! 运行：`cargo run --release -p vxl-phys --example m5_dam_break -- [ticks]`
//! 门禁配方：`400`（见 `scripts/gate_gold.sh`）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Vec3};

const GATE_TICKS: usize = 400;

/// 体素盆 + 自由水柱（与 `dam_break.rs` 的几何逐字相同，方便两处对照）。
fn scene() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 0 || ix == 4 || iz == 0 || iz == 4 {
                vol.set(ix, 2, iz, true);
            }
        }
    }
    let voxel_id = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.73, 1.05, -0.125),
        [6, 6, 14],
        0.05,
    );
    w.add_fluid(sys, &[voxel_id]);
    w
}

/// 跑 `ticks`，返回 `(粒子数, 末态质心, 末态最大 x)`；流体缺失时退回零值（门禁禁 `unwrap`）。
fn run(ticks: usize) -> (usize, [f32; 3], f32) {
    let mut w = scene();
    for _ in 0..ticks {
        w.step();
    }
    let Some((sys, ..)) = w.fluids().first() else {
        return (0, [0.0; 3], 0.0);
    };
    let ps = sys.positions();
    if ps.is_empty() {
        return (0, [0.0; 3], 0.0);
    }
    let mut com = Vec3::ZERO;
    let mut max_x = f32::NEG_INFINITY;
    for p in ps {
        com += *p;
        max_x = max_x.max(p.x);
    }
    let com = com * (1.0 / ps.len() as f32);
    (ps.len(), [com.x, com.y, com.z], max_x)
}

/// 冻结基线（配方 = `400` tick）。**存 `f32` 位模式**（判定是逐位的，见头注）。
struct Frozen {
    n: usize,
    com: [u32; 3],
    max_x: u32,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        n: 504,
        com: [0x3e51_4a69, 0x3f81_3aa2, 0xbe39_a57b],
        max_x: 0x3f3f_d96f,
    })
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);

    let (n, com, max_x) = run(ticks);
    println!(
        "M5-DAM-BREAK ticks={ticks} 粒子={n} 质心=({:.9},{:.9},{:.9}) 最大x={max_x:.9}",
        com[0], com[1], com[2]
    );
    println!(
        "  位模式 com=[{:#010x},{:#010x},{:#010x}] max_x={:#010x}",
        com[0].to_bits(),
        com[1].to_bits(),
        com[2].to_bits(),
        max_x.to_bits()
    );

    let Some(fz) = frozen(ticks) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方 = {GATE_TICKS} tick）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    if n != fz.n {
        bad.push(format!("粒子数 {n} ≠ 基线 {}", fz.n));
    }
    for (k, (g, w)) in com.iter().zip(fz.com.iter()).enumerate() {
        if g.to_bits() != *w {
            bad.push(format!(
                "质心[{k}] {g:.9}（{:#010x}）≠ 基线 {w:#010x}",
                g.to_bits()
            ));
        }
    }
    if max_x.to_bits() != fz.max_x {
        bad.push(format!(
            "最大 x {max_x:.9}（{:#010x}）≠ 基线 {:#010x}",
            max_x.to_bits(),
            fz.max_x
        ));
    }
    if bad.is_empty() {
        println!(
            "✅ 金样基线 PASS（m5_dam_break / {ticks} tick）：粒子 {n}，质心 ({:.9},{:.9},{:.9})，最大x {max_x:.9}",
            com[0], com[1], com[2]
        );
    } else {
        println!(
            "❌ 金样基线 FAIL（m5_dam_break / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
