//! **M1 立项前的成本判据**：现役"几何排序"到底值不值得被 VBD 换掉？
//!
//! 现役 rope↔刚体耦合每次施加前对命中做 `sort_by(几何序)`（`rope.rs` 的 ② 段）。
//! VBD 的第一收益就是**去掉这次排序**。若排序在子步里占比可忽略，这条收益就不成立，
//! M1 应直接把力气放到②（高刚度比单调性）而不是去省它。
//!
//! 本探针用与判据同规模的场景（33 节点绳 + 1 个 0.6 宽盒）量两件事：
//! ① 单次 `sort_by`（33 条命中）的耗时；② 同一场景下 `Rope::step` 单次子步的耗时。
//! 本机读数写进账本；机器无关的**占比**才是判据。

use std::time::Instant;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// 与 `rope.rs` 的 `BodyHit` 同布局的替身（只为量排序成本；不参与物理）。
#[derive(Clone, Copy)]
struct Hit {
    ord: f32,
    x: f32,
}

fn hits(n: usize) -> Vec<Hit> {
    (0..n)
        .map(|i| {
            let t = i as f32 / (n.max(2) - 1) as f32;
            Hit {
                ord: (t * std::f32::consts::TAU).sin(),
                x: -t,
            }
        })
        .collect()
}

fn time_sort(n: usize, reps: u32) -> f64 {
    let base = hits(n);
    let mut buf = base.clone();
    // 预热
    for _ in 0..100 {
        buf.copy_from_slice(&base);
        buf.sort_by(|a, c| a.ord.total_cmp(&c.ord).then(a.x.total_cmp(&c.x)));
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        buf.copy_from_slice(&base);
        buf.sort_by(|a, c| a.ord.total_cmp(&c.ord).then(a.x.total_cmp(&c.x)));
    }
    t0.elapsed().as_secs_f64() / reps as f64
}

fn time_substep(reps: u32) -> f64 {
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    let proxy = RigidProxy {
        body: 0,
        shape: Shape::Box {
            half: Vec3::new(0.3, 0.05, 0.3),
        },
        pos: Vec3::new(0.0, 1.2, 0.0),
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
        local_inv_inertia: Vec3::ZERO,
        inv_mass: 1.0,
    };
    let bodies = [proxy];
    // 预热：让盒落到绳上（建立接触）
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &bodies);
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        r.step(DT, G, &NoProviders, 0, &bodies);
    }
    t0.elapsed().as_secs_f64() / reps as f64
}

#[test]
fn sort_cost_versus_substep() {
    let ts33 = time_sort(33, 20_000);
    let ts64 = time_sort(64, 20_000);
    let step = time_substep(2_000);
    println!("[cost] sort 33 条命中：{:.1} ns", ts33 * 1e9);
    println!("[cost] sort 64 条命中：{:.1} ns", ts64 * 1e9);
    println!("[cost] Rope::step 单子步：{:.1} µs", step * 1e6);
    println!(
        "[cost] 占比：33 条 {:.4}% / 64 条 {:.4}%",
        ts33 / step * 100.0,
        ts64 / step * 100.0
    );
    // 判据（机器无关的占比）：实测 **3.4%（33 条）/ 6.6%（64 条）⇒ VBD 的「去排序」收益成立**，
    // M1 照原计划接线。这里只守异常上界：>20% 说明命中数暴涨或子步退化（先查场景，不是先信 VBD）。
    assert!(
        ts33 / step < 0.20,
        "排序占比异常（{:.1}%）：先查命中数/子步是否退化",
        ts33 / step * 100.0
    );
}
