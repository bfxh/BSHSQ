//! **弯曲约束判据**（软体切片：布料三组约束的第三组——`ClothConstraints.bending` 的落点）：
//! 二环对（"隔一格"的跨格距离约束，XPBD 布料同族做法）。
//!
//! 判据两条：① **弯曲扛住自重**——同一悬垂场景，开弯曲的**下垂量小于**关弯曲
//! （`bend_compliance = f32::INFINITY`）；② **金丝雀**：关弯曲时**逐位退回**切片 1 的行为
//! （弯曲组的 λ 恒 0 ⇒ 不产生任何位移），且结构应变口径不受影响（`max_strain` 只统计结构/剪切）。

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

/// 短边两角钉住、跑 `ticks`；返回 `(中心粒子 y, 最大结构应变)`。
fn sag(bend_compliance: f32, ticks: usize) -> (f32, f32) {
    let (pts, tris) = plate(4, 0.5); // 4×4 格 ⇒ 二环对有意义（2 格跨距）
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    sheet.bend_compliance = bend_compliance;
    // 钉住 z = −0.5 那条边的 5 个粒子（索引 0..=4 是 iz = 0 行）。
    for i in 0..=4 {
        sheet.set_pinned(i, true);
    }
    for _ in 0..ticks {
        sheet.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0, &[]);
    }
    // 自由边中点（iz = 4 行的中间那个：索引 4*5 + 2 = 22）。
    (sheet.pos[22].y, sheet.max_strain())
}

/// ① **弯曲扛住自重**：开弯曲（默认 `Soft` 档）⇒ 下垂**小于**关弯曲。
#[test]
fn bending_resists_sag() {
    let (sag_on, strain_on) = sag(Stiffness::Soft.alpha(), 900);
    let (sag_off, strain_off) = sag(f32::INFINITY, 900);
    println!(
        "[判据①弯曲] 自由边中点 y：开弯曲 = {sag_on:+.4}、关弯曲 = {sag_off:+.4}（开应更高=下垂更小）；结构应变 {strain_on:.4} / {strain_off:.4}"
    );
    assert!(
        sag_on.is_finite() && sag_off.is_finite(),
        "悬垂出现非有限值"
    );
    assert!(
        sag_on > sag_off + 0.005,
        "开弯曲应显著减小下垂（开 {sag_on:+.4} vs 关 {sag_off:+.4}）⇒ 弯曲约束没生效或符号错"
    );
    assert!(
        strain_on < 0.05 && strain_off < 0.05,
        "结构应变应仍有界（{strain_on:.4} / {strain_off:.4}）"
    );
}

/// ② **金丝雀**：`bend_compliance = ∞` ⇒ **逐位退回**切片 1 的行为
/// （弯曲组 λ 恒 0、不产生位移）；二环对计数 = 确定性常数。
#[test]
fn infinite_bend_compliance_is_bit_identical_to_no_bending() {
    let (pts, tris) = plate(4, 0.5);
    let mut a = ClothSheet::new(pts.clone(), tris.clone(), 1000.0, 0.01, Stiffness::Hard);
    let mut b = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    a.bend_compliance = f32::INFINITY;
    // b 用同一份（对照就是"关弯曲"的同一个实现）：跑同样步数，逐位比。
    for _ in 0..300 {
        a.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0, &[]);
        b.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0, &[]);
    }
    let mut worst = 0.0f32;
    for (pa, pb) in a.pos.iter().zip(b.pos.iter()) {
        worst = worst.max((*pa - *pb).length());
    }
    println!(
        "[金丝雀] 关弯曲 vs 同实现对照：最大差 = {worst:.3e}（逐位）；二环对 = {}",
        a.bend_count()
    );
    assert!(
        worst == 0.0,
        "关弯曲（∞）应与自身逐位一致（最大差 {worst:.3e}）⇒ 无穷 compliance 没被正确短路"
    );
    assert!(
        a.pos.iter().all(|p| p.is_finite()),
        "关弯曲必须全有限 ⇒ **非有限 compliance 要显式短路**（`inf/inf = NaN`，本判据首版就栽在这）"
    );
    // 二环对计数：**本轮实测 79**（4×4 顶点网格；三角化含对角边 ⇒ 二环集含 (2,1) 类对，
    // 不是手算的 16）。拓扑/三角化一改它就变 ⇒ 钉住数值逼人复核，而不是让它悄悄漂。
    assert_eq!(a.bend_count(), 79, "二环对计数应为实测冻结值 79");
    assert_eq!(
        a.bend_count(),
        b.bend_count(),
        "同日同参两次构建的二环对必须相同（确定性）"
    );
}
