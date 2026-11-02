//! **接触图着色深度**（`PLAN-gpu §17.2` 第三片 / SoA+SIMD 的前置判断）。
//!
//! 问的是：**把顺序 GS 换成"按色分批的并行 GS"需要多少色**。同一色的约束两两不共享体 ⇒
//! 色内可并行（SIMD/多线程）且结果确定；色数决定"多少趟顺序批"。
//! ⚠️ 彩色 GS ≠ 顺序 GS（收敛序不同）⇒ 只能做**新档**，不与主机档比逐位。
//!
//! 运行：`cargo run --release -p vxl-phys --example contact_coloring_probe -- [ticks]`
use vxl_phys::{BodyBuildExt, DiagExt, HeightField, PhysConfig, Quat, Shape, Vec3, World};

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;

fn build(ticks: u32) -> World {
    let cfg = PhysConfig {
        velocity_iterations: 16,
        substeps: 4,
        threads: 8,
        shock_iterations: 2,
        stabilization_iterations: 2,
        wake_gate_k: 8,
        settled_hold_iterations: 4,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let off = (SIDE as f32 - 1.0) * 0.5 * SPACING;
    for layer in 0..LAYERS {
        let y = 0.25 + layer as f32 * SPACING;
        for row in 0..SIDE {
            for col in 0..SIDE {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * SPACING - off, y, row as f32 * SPACING - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    for _ in 0..ticks {
        w.step();
    }
    w
}

fn main() {
    let ticks: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);
    let w = build(ticks);
    let ms = w.manifolds();
    let n_b = w.bodies.len();

    // 贪心**边着色**：按流形序逐条取"两端都没用过的最小色号"。
    // `used[b]` 是 64 位色掩码；超过 64 色则退化为逐条线性探测（本场景不会）。
    let mut used = vec![0u64; n_b];
    let mut colors = 0u32;
    let mut counts: Vec<u32> = Vec::new();
    for m in ms {
        let (a, b) = (m.a.min(m.b) as usize, m.a.max(m.b) as usize);
        let taken = used[a] | used[b];
        let mut c = 0u32;
        while c < 64 && taken & (1u64 << c) != 0 {
            c += 1;
        }
        if c == 64 {
            // 兜底：线性找第一个空位（只计数，不改结论口径）。
            while counts.get(c as usize).copied().unwrap_or(0) > 0 && c > 0 {
                c += 1;
                if c > 10_000 {
                    break;
                }
            }
        }
        let bit = 1u64 << (c.min(63));
        used[a] |= bit;
        used[b] |= bit;
        if counts.len() <= c as usize {
            counts.resize(c as usize + 1, 0);
        }
        counts[c as usize] += 1;
        colors = colors.max(c + 1);
    }
    let total: u32 = counts.iter().sum();
    let max_c = counts.iter().copied().max().unwrap_or(0);
    let min_c = counts.iter().copied().min().unwrap_or(0);
    let avg = total.checked_div(colors).unwrap_or(0);
    println!(
        "m1pile {LAYERS}×{SIDE}² @ {ticks} tick：流形 {} / 体 {n_b}",
        ms.len()
    );
    println!("贪心边着色：色数 {colors} · 每色约束 min {min_c} / 均 {avg} / max {max_c}");
    println!(
        "按色并行：最宽的一色可同时处理 {max_c} 条约束（4-wide SIMD ⇒ {} 批/色）",
        max_c.div_ceil(4)
    );
    println!("（前 8 色规模：{:?}）", &counts[..colors.min(8) as usize]);
}
