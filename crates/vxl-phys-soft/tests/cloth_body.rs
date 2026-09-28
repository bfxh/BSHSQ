//! **布片 × 刚体判据**（软体切片 2b-i）：静态/睡眠代理 = 墙（逐粒子解析穿透
//! `rigid::shape_penetration` + 库仑锥）。**动态代理的反应两腿 `body_dv`/`body_dx`
//! 属 2b-ii**——此处如实不受理（不是漏）。
//!
//! 判据两条：① **盒顶落定**（与窄相那条 `trimesh_is_held_by_box` 同场景，但走**软体域代理
//! 路径**——布片不过窄相）；② **越边垂落**（布片比盒顶宽 ⇒ 中间粒子在盒顶、边缘粒子
//! 沿盒侧面**垂下**——多面接触 + 布片"搭"的形态）。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{ClothSheet, RigidProxy, Stiffness};

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

/// 静态盒代理（`inv_mass = 0` ⇒ 墙）。
fn static_box(half: Vec3, top: f32) -> RigidProxy {
    RigidProxy {
        body: 0,
        shape: Shape::Box { half },
        pos: Vec3::new(0.0, top - half.y, 0.0),
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
        local_inv_inertia: Vec3::ZERO,
        inv_mass: 0.0,
    }
}

/// ① **盒顶落定**：布片（1×1）落到静态盒（顶 2×2，顶面 y = 1）上 ⇒ 粒子停在盒顶。
#[test]
fn cloth_rests_on_static_box() {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += 2.0;
    }
    let bodies = vec![static_box(Vec3::new(1.0, 0.5, 1.0), 1.0)];
    for _ in 0..600 {
        sheet.step(
            1.0 / 60.0,
            Vec3::new(0.0, -9.81, 0.0),
            &NoProviders,
            0,
            &bodies,
        );
    }
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for p in &sheet.pos {
        lo = lo.min(p.y);
        hi = hi.max(p.y);
    }
    let strain = sheet.max_strain();
    println!(
        "[判据①盒顶] 粒子 y ∈ [{lo:+.4}, {hi:+.4}]（盒顶 1.0 + 球半径 {}）；应变 = {strain:.4}",
        sheet.radius
    );
    assert!(
        sheet.pos.iter().all(|p| p.is_finite()),
        "盒顶落定出现非有限值"
    );
    assert!(
        lo > 1.0 - 0.05,
        "粒子最低点 {lo:.4} 深穿盒顶（应 ≈ 1.0 + 半径）⇒ 代理路径没建接触"
    );
    assert!(hi < 1.0 + 0.1, "粒子最高点 {hi:.4} 仍悬空 ⇒ 没落定");
    assert!(
        strain < 0.05,
        "落定后应变 {strain:.4} ≥ 5% ⇒ 冲击把片扯坏了"
    );
}

/// ② **越边垂落**：布片（1.4×1.4，`plate(2, 0.7)`）比盒顶（1×1，顶 y = 1）宽 ⇒
/// 中间粒子**在盒顶**（y ≈ 1）、最外圈粒子**垂到盒顶之下**（y < 0.8）——"搭"的形态。
#[test]
fn cloth_drapes_over_box_edge() {
    let (pts, tris) = plate(2, 0.7);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += 2.0;
    }
    let bodies = vec![static_box(Vec3::new(0.5, 0.5, 0.5), 1.0)];
    for _ in 0..900 {
        sheet.step(
            1.0 / 60.0,
            Vec3::new(0.0, -9.81, 0.0),
            &NoProviders,
            0,
            &bodies,
        );
    }
    // 中心粒子（角点 4 = (0,0)）：应留在盒顶上；角粒子（0 = (−0.7,−0.7)）：应垂到盒顶之下。
    let center_y = sheet.pos[4].y;
    let corner_y = sheet.pos[0].y;
    let strain = sheet.max_strain();
    println!(
        "[判据②垂落] 中心 y = {center_y:+.4}（盒顶 1.0）；角粒子 y = {corner_y:+.4}（应垂到 < 0.8）；应变 = {strain:.4}"
    );
    assert!(sheet.pos.iter().all(|p| p.is_finite()), "垂落出现非有限值");
    assert!(
        (center_y - 1.0).abs() < 0.1,
        "中心粒子应停在盒顶（y = {center_y:+.4}）"
    );
    assert!(
        corner_y < 0.8,
        "角粒子应沿盒侧垂到盒顶之下（y = {corner_y:+.4}）⇒ 边缘没搭上/被锥冻住"
    );
    assert!(strain < 0.08, "垂落后应变 {strain:.4} 过大 ⇒ 求解散架");
}
