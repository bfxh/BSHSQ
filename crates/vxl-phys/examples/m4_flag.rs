//! **M4 旗飘金样**（vxl-only 通道）—— `ROUTE` M4 出口判据「悬臂/旗飘金样」里的**旗飘**那一半
//! （悬臂在 `m4_cantilever.rs`）。
//!
//! 场景：`0.3 m × 0.2 m` 的旗面（**xy 平面**、6×4 格、`z = 0`）**左列钉在旗杆上**；
//! 恒定风沿 **`+z` 垂直吹向旗面**（`AeroConfig::wind`；Bridson 线化面元气动 + 升力，
//! 见 `vxl-phys-soft/src/cloth_aero.rs`）⇒ 旗面被吹离原平面、飘动。重力 `-y`。
//! **冻结基线自检**只在门禁配方上判定（其余 tick 数 = 自由实验，会显式打印"跳过"）。
//!
//! **判据**：自由端**上角**与**中点**的坐标 + 最大应变，全部按 **`f32` 位模式逐位**比较
//! （与 `m4_cantilever.rs` 同一口径：十进制往返要 9 位有效数字才不失真，位模式没有这问题）。
//! ⚠️ 旗是**动**的（飘动不收敛到静止）：末态读数取决于**配方 tick 数**，所以它只能当
//! "同配方逐位可复现"的冻结基线，不能当"稳态解"引用。
//!
//! 运行：`cargo run --release -p vxl-phys --example m4_flag -- [ticks]`
//! 门禁配方：`600`（见 `scripts/gate_gold.sh`）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

const GATE_TICKS: usize = 600;
/// 从旗杆向外的格数 / 竖直格数（顶点数 = `(GX+1)·(GY+1)`）。
const GX: usize = 6;
const GY: usize = 4;
const LEN: f32 = 0.3;
const HGT: f32 = 0.2;
/// 风速（沿 `+z`，垂直吹向旗面）。
const WIND: f32 = 8.0;

/// 顶点索引：`iy·(GX+1) + ix`（`ix` = 离旗杆的格数、`iy` = 竖直格数）。
const fn vid(iy: usize, ix: usize) -> usize {
    iy * (GX + 1) + ix
}
/// 判据取的两个点：自由端（`ix = GX`）的上角与中点。
const TIP_TOP: usize = vid(GY, GX);
const TIP_MID: usize = vid(GY / 2, GX);

/// 旗面 + 旗杆 + 恒风。
fn flag() -> ClothSheet {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
    let (nx, ny) = (GX as u32, GY as u32);
    for iy in 0..=ny {
        for ix in 0..=nx {
            pts.push(Vec3::new(
                LEN * ix as f32 / nx as f32,
                HGT * iy as f32 / ny as f32,
                0.0,
            ));
        }
    }
    for iy in 0..ny {
        for ix in 0..nx {
            let a = iy * (nx + 1) + ix;
            let (c, d) = (a + 1, a + nx + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.bend_compliance = Stiffness::Standard.alpha();
    s.aero.enabled = true;
    s.aero.cfg.wind = [0.0, 0.0, WIND];
    for i in 0..s.pos.len() {
        if s.pos[i].x.abs() < 1e-6 {
            s.set_pinned(i, true); // 旗杆：`x = 0` 那一列
        }
    }
    s
}

/// 跑 `ticks`，返回 `(自由端上角, 自由端中点, 最大应变)`；布片缺失时退回零值（门禁禁 `unwrap`）。
fn run(ticks: usize) -> ([f32; 3], [f32; 3], f32) {
    let mut w = World::new(PhysConfig::default());
    let idx = w.add_cloth(flag());
    for _ in 0..ticks {
        w.step();
    }
    let Some(s) = w.cloth(idx) else {
        return ([0.0; 3], [0.0; 3], 0.0);
    };
    let (a, b) = (s.pos[TIP_TOP], s.pos[TIP_MID]);
    ([a.x, a.y, a.z], [b.x, b.y, b.z], s.max_strain())
}

/// 冻结基线（配方 = `600` tick）。**存 `f32` 位模式**（判定是逐位的，见头注）。
struct Frozen {
    top: [u32; 3],
    mid: [u32; 3],
    strain: u32,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        top: [0x3c1f_5a2d, 0x3e0d_8f6d, 0x3e93_8150],
        mid: [0x3da6_f157, 0x3d8e_ea24, 0x3e8c_e74a],
        strain: 0x3cc1_8568,
    })
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);

    let (top, mid, strain) = run(ticks);
    println!(
        "M4-FLAG ticks={ticks} 自由端上角=({:.9},{:.9},{:.9}) 中点=({:.9},{:.9},{:.9}) 最大应变={strain:.9}",
        top[0], top[1], top[2], mid[0], mid[1], mid[2]
    );
    println!(
        "  位模式 top=[{:#010x},{:#010x},{:#010x}] mid=[{:#010x},{:#010x},{:#010x}] strain={:#010x}",
        top[0].to_bits(),
        top[1].to_bits(),
        top[2].to_bits(),
        mid[0].to_bits(),
        mid[1].to_bits(),
        mid[2].to_bits(),
        strain.to_bits()
    );

    let Some(fz) = frozen(ticks) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方 = {GATE_TICKS} tick）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    for (name, got, want) in [("上角", top, fz.top), ("中点", mid, fz.mid)] {
        for (k, (g, w)) in got.iter().zip(want.iter()).enumerate() {
            if g.to_bits() != *w {
                bad.push(format!(
                    "{name}[{k}] {g:.9}（{:#010x}）≠ 基线 {w:#010x}",
                    g.to_bits()
                ));
            }
        }
    }
    if strain.to_bits() != fz.strain {
        bad.push(format!(
            "最大应变 {strain:.9}（{:#010x}）≠ 基线 {:#010x}",
            strain.to_bits(),
            fz.strain
        ));
    }
    if bad.is_empty() {
        println!(
            "✅ 金样基线 PASS（m4_flag / {ticks} tick）：应变 {strain:.9}，上角 ({:.9},{:.9},{:.9})，中点 ({:.9},{:.9},{:.9})",
            top[0], top[1], top[2], mid[0], mid[1], mid[2]
        );
    } else {
        println!(
            "❌ 金样基线 FAIL（m4_flag / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
