//! **湿质量**（布吸水后有效质量上升）的软体侧判据 —— `ROUTE.md` §4「湿布（质量+阻力、双向）」
//! 里"质量"那半（阻力 + 双向在 `cloth_medium.rs`）。
//!
//! **开关**：`cloth.medium.wet_mass`（**默认关** ⇒ 默认档逐位不变）。开启后每个子步从逐面
//! `occupied` 重算逐顶点有效质量倍率，并刷新 `inv_mass`（**从干质量 `mass` 重算** ⇒ 离水可逆）。
//!
//! **判据**：① `occupied = 0` ⇒ 倍率精确 `1.0` ⇒ 与「完全不填」的 `inv_mass` **逐位相同**；
//! ② 全浸没 ⇒ 每个顶点 `inv_mass` 都下降；③ 同样的介质阻力下，湿布比干布**更慢**。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{MediumSample, NoProviders};
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 0.4×0.4 的水平布（2×2 格、9 粒子；同 `cloth_medium.rs` 的场景）。
fn sheet() -> ClothSheet {
    let (n, size) = (2usize, 0.2f32);
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

/// 每个三角面填一份均匀介质样本（`occupied` 默认 0，调用方按需覆盖）。
fn fill(sc: &mut ClothSheet, density: f32) {
    sc.medium.samples = sc
        .tris
        .iter()
        .map(|_| MediumSample {
            density,
            velocity: Vec3::new(1.0, 0.0, 0.0),
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

/// `inv_mass` 的位图（逐位比较用）。
fn inv_bits(sc: &ClothSheet) -> Vec<u32> {
    sc.inv_mass.iter().map(|m| m.to_bits()).collect()
}

/// 平均 `vx`（"谁更慢"的读数）。
fn mean_vx(sc: &ClothSheet) -> f32 {
    sc.vel.iter().map(|v| v.x).sum::<f32>() / sc.vel.len().max(1) as f32
}

#[test]
fn submerged_sheet_is_heavier_and_slower() {
    let (mut wet, mut nil, mut none) = (sheet(), sheet(), sheet());
    wet.medium.wet_mass = true;
    fill(&mut wet, 1000.0);
    for s in wet.medium.samples.iter_mut() {
        s.occupied = 1.0; // 全浸没
    }
    fill(&mut nil, 1000.0); // `occupied` 留 0：同样的阻力，但不吸水
    none.medium.clear(); // 完全不填
    run(&mut wet, 8);
    run(&mut nil, 8);
    run(&mut none, 8);
    assert_eq!(
        inv_bits(&nil),
        inv_bits(&none),
        "`occupied = 0` 的倍率必须精确 1.0 ⇒ 与「完全不填」逐位相同"
    );
    let heavier = wet.inv_mass.iter().zip(&nil.inv_mass).all(|(w, d)| *w < *d);
    assert!(heavier, "全浸没 ⇒ 每个顶点的 inv_mass 都应变小");
    assert!(
        mean_vx(&wet) < mean_vx(&nil),
        "同样的阻力下湿布应更慢：wet={} nil={}",
        mean_vx(&wet),
        mean_vx(&nil)
    );
}
