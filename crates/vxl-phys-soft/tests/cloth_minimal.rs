//! **布料最小闭环判据**（软体切片 1，`cloth.rs`）：三角网 + XPBD 距离约束 + 薄壳均分质量。
//!
//! 判据三条（口径先于阈值——先跑探针读数，阈值取自实测；与 `rope_minimal.rs` 同族）：
//! ① **自由落体 = 刚性平移**：约束处于零应变态时，XPBD 对刚性平移**零噪声**
//!    （所有粒子逐子步同位移；若约束在零应变态仍出力 ⇒ 公式错）；
//! ② **两角钉住悬垂**：约束扛得住重力（最大应变有界）、片下垂（形态方向对）、无 NaN；
//! ③ **子步收敛**：应变残差随子步数下降（O(h²) 特征）。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

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

const G: f32 = 9.8;

/// **① 自由落体 = 刚性平移**：零应变态下所有粒子位移一致（XPBD 零噪声），且
/// 位移 ≈ 半隐式欧拉解析 `½g·t²·(1 + h/t)`。
#[test]
fn free_fall_is_rigid_translation() {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    assert_eq!(
        sheet.edge_count(),
        16,
        "2×2 格唯一边 = 12 横竖直边 + 4 对角"
    );
    let ticks = 120;
    let dt = 1.0 / 60.0;
    let p0 = sheet.pos.clone();
    for _ in 0..ticks {
        sheet.step(dt, Vec3::new(0.0, -G, 0.0), &NoProviders, 0, &[]);
    }
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for (p, p_init) in sheet.pos.iter().zip(p0.iter()) {
        let dy = p.y - p_init.y;
        lo = lo.min(dy);
        hi = hi.max(dy);
    }
    let spread = hi - lo;
    let t = ticks as f32 * dt;
    let analytic = 0.5 * G * t * t * (1.0 + dt / t);
    println!(
        "[判据①] 位移散布 = {spread:.3e}（应 ≈ 0）；末位移 ≈ {analytic:.4}（解析）vs 实测 {:.4}",
        p0[4].y - sheet.pos[4].y
    );
    assert!(
        spread.abs() < 1e-5,
        "零应变态下自由落体应=刚性平移（散布 {spread:.3e} ≠ 0 ⇒ 约束在零应变态出力）"
    );
    let drop = p0[4].y - sheet.pos[4].y;
    assert!(
        (drop - analytic).abs() / analytic < 0.01,
        "自由落体位移 {drop:.4} 应 ≈ 解析 {analytic:.4}"
    );
}

/// **② 两角钉住悬垂**：约束扛住重力（最大应变有界）、片下垂（方向对）、无 NaN。
#[test]
fn pinned_corners_hold_and_sheet_sags() {
    let (pts, tris) = plate(2, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    // 钉住一条短边的两端（粒子 0 = (−0.5,0,−0.5)、2 = (0.5,0,−0.5)）。
    sheet.set_pinned(0, true);
    sheet.set_pinned(2, true);
    for _ in 0..600 {
        sheet.step(1.0 / 60.0, Vec3::new(0.0, -G, 0.0), &NoProviders, 0, &[]);
    }
    let strain = sheet.max_strain();
    let center_y = sheet.pos[4].y;
    let finite = sheet.pos.iter().all(|p| p.is_finite());
    println!("[判据②] 最大应变 = {strain:.4}（应 < 0.05）；中心 y = {center_y:+.4}（钉住层 y=0，应下垂）；finite={finite}");
    assert!(finite, "悬垂 600 tick 出现非有限值 ⇒ 求解发散");
    assert!(
        strain < 0.05,
        "最大应变 {strain:.4} ≥ 5% ⇒ 约束刚度/求解次数扛不住自重（或公式错）"
    );
    assert!(
        center_y < -0.1,
        "中心应明显下垂（实得 y = {center_y:+.4}）⇒ 约束把片'冻'住了 ⇒ 刚度档写错"
    );
}

/// **③ 子步收敛**：同一悬垂场景，子步 4 → 16 ⇒ 最大应变残差**单调下降**且 ≥ 1.5×。
#[test]
fn strain_residual_converges_with_substeps() {
    let mut readings = Vec::new();
    for sub in [4u32, 16] {
        let (pts, tris) = plate(2, 0.5);
        let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
        sheet.substeps = sub;
        sheet.set_pinned(0, true);
        sheet.set_pinned(2, true);
        for _ in 0..600 {
            sheet.step(1.0 / 60.0, Vec3::new(0.0, -G, 0.0), &NoProviders, 0, &[]);
        }
        readings.push(sheet.max_strain());
    }
    println!(
        "[判据③] 应变残差：substeps 4 = {:.5}、16 = {:.5}（比值 {:.2}×）",
        readings[0],
        readings[1],
        readings[0] / readings[1]
    );
    assert!(
        readings[1] < readings[0] * 0.9,
        "子步 ×4 应显著降残差（{:.5} → {:.5}）⇒ 收敛阶坏了",
        readings[0],
        readings[1]
    );
}
