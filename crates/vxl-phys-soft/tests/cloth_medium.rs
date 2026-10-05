//! **布 × 介质（湿布阻力）**的软体侧判据：`cloth.medium` 是**数据**（门面按面心采好），
//! `predict` 里由 `cloth_medium::inject` 消费 ⇒ 这里直接填数据、跑 `step` 看效果。
//!
//! 判据：① 均匀介质流把静止的布**带向流的方向**；② `medium` 空（默认）⇒ 与"不填"**逐位相同**；
//! ③ `density = 0` 的样本（真空）⇒ 同样逐位不动。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys_core::interop::{MediumSample, NoProviders};
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 0.4×0.4 的水平布（2×2 格、9 粒子、厚 0.01、密度 1000 ⇒ 每粒子 ≈0.178 kg）。
fn sheet() -> ClothSheet {
    let n = 2usize;
    let size = 0.2f32;
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard)
}

/// 每个三角面填一份均匀介质样本。
fn fill(sc: &mut ClothSheet, density: f32, velocity: Vec3) {
    sc.medium.samples = sc
        .tris
        .iter()
        .map(|_| MediumSample {
            density,
            velocity,
            ..MediumSample::VACUUM
        })
        .collect();
}

/// 跑 `n` 个子步（重力 −y、无提供者、无刚体代理）。
fn run(sc: &mut ClothSheet, n: usize) {
    for _ in 0..n {
        sc.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0, &[]);
    }
}

/// 速度位图（逐位比较用）。
fn vel_bits(sc: &ClothSheet) -> Vec<u32> {
    sc.vel
        .iter()
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

/// ① 均匀介质流（+x）把静止的布带向 +x。
#[test]
fn medium_flow_accelerates_the_sheet_along_it() {
    let mut sc = sheet();
    fill(&mut sc, 1000.0, Vec3::new(1.0, 0.0, 0.0));
    run(&mut sc, 8);
    let mean_vx = sc.vel.iter().map(|v| v.x).sum::<f32>() / sc.vel.len().max(1) as f32;
    assert!(
        mean_vx > 0.05,
        "介质流 (+x) 应把布带起来，实得 mean_vx={mean_vx}"
    );
}

/// ② 空 `medium`（默认）与显式 `density = 0` ⇒ 都与"完全不填"逐位相同。
#[test]
fn empty_or_vacuum_medium_is_bitwise_neutral() {
    let mut a = sheet();
    let mut b = sheet();
    let mut c = sheet();
    fill(&mut b, 0.0, Vec3::new(1.0, 0.0, 0.0));
    c.medium.clear();
    run(&mut a, 6);
    run(&mut b, 6);
    run(&mut c, 6);
    assert_eq!(
        vel_bits(&a),
        vel_bits(&b),
        "density=0 的样本不应改变任何一位"
    );
    assert_eq!(vel_bits(&a), vel_bits(&c), "空 medium 应是零成本关档");
}
