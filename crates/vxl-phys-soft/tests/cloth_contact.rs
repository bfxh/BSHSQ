//! **布料接触判据**（软体切片 2）：提供者球采样 + 库仑锥（与 rope 同款口径，
//! 共享实现 `cloth::sphere_contacts_project`——rope 委托调用 ⇒ 两域行为可对齐比较）。
//!
//! 判据三条：① **平地落定**（下落 → 停在地板上、无穿透、应变有界）；
//! ② **缓坡黏住**（15° < atan μ ⇒ 静摩擦锥扛住，位移 ≈ 0——rope 同款阈值）；
//! ③ **陡坡滑动**（35° > atan μ ⇒ 滑动，位移大）。②③ 合起来 = 锥的**阈值型**判据
//! （与解析 `tanθ_crit = μ` 对拍）。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};
use vxl_phys_terrain::mesh::TriMesh;

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 **y = 0 平面**）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
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
    (pts, tris)
}

/// 平地板（y ≡ 0；与 `rope_minimal` / `provider_shape_coverage` 同一套三角化）。
fn flat_mesh() -> TriMesh {
    let (verts, tris) = plate(8, 4.0);
    TriMesh::new(verts, tris)
}

/// 斜坡网格：`y = −tan(θ)·x`（**下坡朝 +x**）；`θ = 0` 即平地。
fn slope_mesh(theta_deg: f32) -> TriMesh {
    let k = theta_deg.to_radians().tan();
    let (mut verts, tris) = plate(8, 4.0);
    for v in &mut verts {
        v.y = -k * v.x;
    }
    TriMesh::new(verts, tris)
}

/// ① **平地落定**：布片从 1 m 落到地板上 ⇒ 停住（粒子中心 ≈ 球半径高度、无穿透、应变有界）。
#[test]
fn cloth_rests_on_flat_floor() {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += 1.0;
    }
    let floor = flat_mesh();
    for _ in 0..600 {
        sheet.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &floor, 1, &[]);
    }
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for p in &sheet.pos {
        lo = lo.min(p.y);
        hi = hi.max(p.y);
    }
    let strain = sheet.max_strain();
    println!(
        "[判据①平地] 粒子中心 y ∈ [{lo:+.4}, {hi:+.4}]（球半径 {}；应 ≈ 半径高度）最大应变 = {strain:.4}",
        sheet.radius
    );
    assert!(
        sheet.pos.iter().all(|p| p.is_finite()),
        "平地落定出现非有限值"
    );
    assert!(
        lo > sheet.radius - 0.05,
        "粒子最低点 {lo:.4} 深穿地板（球中心应 ≈ 半径高度 {}）",
        sheet.radius
    );
    assert!(
        hi < sheet.radius + 0.1,
        "粒子最高点 {hi:.4} 仍悬空 ⇒ 没落定（或接触没建上）"
    );
    assert!(
        strain < 0.05,
        "落定后应变 {strain:.4} ≥ 5% ⇒ 冲击把片扯坏了"
    );
}

/// 坡上滑移量（质心 x 位移；下坡朝 +x）。**出生在坡面上**（粒子中心 = 表面 + 法向×半径
/// ⇒ 零冲击）——阈值型判据量的是**静摩擦锥**；落体冲击的瞬态见下方留档注。
fn slope_drift(theta_deg: f32, ticks: usize) -> f32 {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    let k = theta_deg.to_radians().tan();
    let ny = 1.0 / (1.0 + k * k).sqrt(); // 坡面法线的 y 分量（朝上）
    for p in &mut sheet.pos {
        p.y = -k * p.x + ny * sheet.radius;
    }
    let floor = slope_mesh(theta_deg);
    let x0: f32 = sheet.pos.iter().map(|p| p.x).sum::<f32>() / sheet.pos.len() as f32;
    for _ in 0..ticks {
        sheet.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &floor, 1, &[]);
    }
    let x1: f32 = sheet.pos.iter().map(|p| p.x).sum::<f32>() / sheet.pos.len() as f32;
    x1 - x0
}

/// ②+③ **锥的阈值型判据**（与解析 `tanθ_crit = μ = 0.5` 对拍；rope 同款阈值）：
/// 15°（tan = 0.268 < μ）⇒ 黏住；35°（tan = 0.700 > μ）⇒ 滑动。
///
/// **⚠️ 冲击瞬态留档（探针实测，2026-09-28）**：若从 0.1 m **落到**坡上，15°/μ=0.5 会先滑
/// **2.45 m 后停住**（μ=1.0 ⇒ 1.94 m；substeps=32 ⇒ 一直滑出网格边缘 ±4 m）——机制 =
/// 弹跳相位里锥预算（`μ·depth`）吃不完弹跳再生的切向速度，静摩擦最终接住。⇒ 阈值型判据
/// 用"出生在面上"的**静摩擦**口径；冲击瞬态的锥增强属后续切片。
#[test]
fn friction_cone_threshold_on_slope() {
    let sticky = slope_drift(15.0, 300);
    let sliding = slope_drift(35.0, 300);
    println!("[判据②③锥] 15° 位移 = {sticky:+.4}（应 ≈ 0 黏住）；35° 位移 = {sliding:+.4}（应 > 0.5 滑动）");
    assert!(
        sticky.abs() < 0.1,
        "15° < atan μ 应被静摩擦黏住（位移 {sticky:+.4}）⇒ 锥口径坏（恒定蠕变 = 只扣超出部分的那个错）"
    );
    assert!(
        sliding > 0.5,
        "35° > atan μ 应滑动（位移 {sliding:+.4}）⇒ 锥把片冻死了 ⇒ 锥写反或没启用"
    );
}

/// **零接触金丝雀**：`provider_count = 0` ⇒ 与无接触时逐位同（布片自由落体穿过地板）。
#[test]
fn zero_providers_means_free_fall_through() {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += 1.0;
    }
    for _ in 0..600 {
        sheet.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0, &[]);
    }
    let lo = sheet.pos.iter().map(|p| p.y).fold(f32::MAX, f32::min);
    println!("[金丝雀] 零提供者 ⇒ 最低点 y = {lo:+.2}（应自由落体 ≈ −47）");
    assert!(
        lo < -30.0,
        "零提供者应自由落体（最低点 {lo:+.2}）⇒ 接触在没提供者时也生效了 ⇒ 路由坏"
    );
}
