//! **M1 只读探针：一 tick 内「新接触点」比例**（`docs/M1-RULING.md` §3.5 的复算入口）。
//!
//! 用法：`cargo run --release -p vxl-phys --example contact_churn -- <tower|pile> <layers> [side|ticks] [ticks]`
//!   tower 25 120   —— 25 层单柱（对齐 tower25）
//!   pile  5 20 120 —— 5 层 × 20×20（对齐 pile5）

use std::collections::BTreeSet;

use vxl_phys::world_step::contact_probe;
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

fn main() {
    let mut args = std::env::args().skip(1);
    let mode = args.next().unwrap_or_else(|| "tower".to_string());
    let layers: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(12);
    let (side, ticks): (usize, u64) = if mode == "pile" {
        (
            args.next().and_then(|s| s.parse().ok()).unwrap_or(20),
            args.next().and_then(|s| s.parse().ok()).unwrap_or(120),
        )
    } else {
        (1, args.next().and_then(|s| s.parse().ok()).unwrap_or(120))
    };

    // substeps 必须 >1：单子步时"tick 内新增"恒为 0
    let mut w = World::new(PhysConfig {
        substeps: 4,
        ..PhysConfig::default()
    });
    w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.1, 6.0),
        },
        Vec3::new(0.0, -0.1, 0.0),
        Quat::IDENTITY,
    );
    for l in 0..layers {
        for iz in 0..side {
            for ix in 0..side {
                let x = (ix as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                let z = (iz as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(x, 0.25 + 0.52 * l as f32, z),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }

    contact_probe::enable(true);
    for _ in 0..ticks {
        w.step();
    }
    let rows = contact_probe::take();
    contact_probe::enable(false);

    let mut cur = u64::MAX;
    let mut seen: BTreeSet<u64> = BTreeSet::new();
    let (mut new_pts, mut all_pts) = (0u64, 0u64);
    let mut worst = (0.0f64, 0u64);
    for (tick, _sub, ids) in rows {
        if tick != cur {
            cur = tick;
            seen.clear();
            for id in &ids {
                seen.insert(*id);
            }
            continue;
        }
        for id in &ids {
            all_pts += 1;
            if !seen.contains(id) {
                new_pts += 1;
                seen.insert(*id);
            }
        }
        if all_pts > 0 && (new_pts as f64 / all_pts as f64) > worst.0 {
            worst = (new_pts as f64 / all_pts as f64, tick);
        }
    }
    println!(
        "[churn] {mode} {layers} 层 × {side}² / {ticks} tick / substeps=4：新增点 {new_pts} / 观测点 {all_pts} = {:.2}%",
        new_pts as f64 / all_pts.max(1) as f64 * 100.0
    );
    println!(
        "[churn] 最差单 tick：{:.2}%（tick {}）",
        worst.0 * 100.0,
        worst.1
    );
}
