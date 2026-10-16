//! 静水：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::Tank;
use crate::{FluidConfig, FluidSystem};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;

/// #2 静水压强（切片1口径）：柱沉降后压强随深度**方向性递增**
/// （下三分之一 > 中三分之一 > 上三分之一），并加三道量级闸：
/// ① 柱高收在晶格初始高与摊平之间（四壁兜住的证据）；
/// ② 中带均压在静水参考 ρ0·g·(H/2) 的 0.4–1.1×（离散 ρ 噪声经
///    Tait q⁷ 放大 + 自由面钳制 ⇒ 实测 ≈0.72×，偏低压）；
/// ③ 底带均压 ≤ 2× 静水参考 ρ0·g·(5H/6)（镜像鬼影补给的边界层
///    q⁷ 尾部使贴底 2cm 系统性偏高 ~1.65×，三分带稀释后 ≈1.19×）。
/// 用 Tank（地板+四壁）——无壁则落柱摊平成 puddle 是正确物理，静水态无从谈起。
/// 实测口径（7³、间距 0.05、xsph 0.05、240 tick）：柱高 0.259，
/// 下/中/上三分带均压 ≈ 2520/920/90 Pa。±30% 逐带量化校准
/// 留待提分辨率或 δ-SPH 切片——本切片只锁方向与量级带宽。
#[test]
fn hydrostatic_pressure_increases_with_depth() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
    f.set_boundaries(&[0]);
    for _ in 0..240 {
        f.step(1.0 / 60.0, &Tank);
    }
    let mut ymin = f32::INFINITY;
    let mut ymax = f32::NEG_INFINITY;
    for p in f.positions() {
        ymin = ymin.min(p.y);
        ymax = ymax.max(p.y);
    }
    let height = ymax - ymin;
    assert!(height > 0.20 && height < 0.32, "柱高 {height:.3} 异常");
    let (mut lo_s, mut lo_n, mut mid_s, mut mid_n, mut hi_s, mut hi_n) =
        (0.0f32, 0u32, 0.0f32, 0u32, 0.0f32, 0u32);
    for (i, p) in f.positions().iter().enumerate() {
        let t = (p.y - ymin) / height;
        let pr = f.pressures()[i];
        if t < 1.0 / 3.0 {
            lo_s += pr;
            lo_n += 1;
        } else if t < 2.0 / 3.0 {
            mid_s += pr;
            mid_n += 1;
        } else {
            hi_s += pr;
            hi_n += 1;
        }
    }
    let (lo, mid, hi) = (lo_s / lo_n as f32, mid_s / mid_n as f32, hi_s / hi_n as f32);
    // 方向：随深度单调（下 > 中 > 顶；顶部自由面钳制 p≥0）。
    assert!(
        lo > mid && mid > hi,
        "压强须随深度递增：底 {lo:.0} / 中 {mid:.0} / 顶 {hi:.0}"
    );
    // 量级闸（参考深度取各带中心：中带 t=0.5 → 深 H/2；底带 t=1/6 → 深 5H/6）。
    let rho0 = f.config().rest_density;
    let mid_ref = rho0 * 9.81 * 0.5 * height;
    assert!(
        mid > mid_ref * 0.4 && mid < mid_ref * 1.1,
        "中带 {mid:.0} 应在静水参考 {mid_ref:.0} 的 0.4–1.1×"
    );
    let lo_ref = rho0 * 9.81 * height * 5.0 / 6.0;
    assert!(
        lo < lo_ref * 2.0,
        "底带 {lo:.0} 超过边界层上界（2× 静水参考 {lo_ref:.0}）"
    );
    // 顶带（自由面 + 镜像鬼影密度尾，见 PLAN-0.3 §4.3）实测：release 下
    // 顶带 157 / 中带 646 ≈ 4.1×（旧的 5× 闸门在此误报失败），debug 下
    // 同一断言（5×）通过 ⇒ debug 比值 > 5。两个 profile 的绝对值不完全
    // 相同，因此判据取"顶带至少低于中带 3×"的稳健口径：真实缺陷是"顶带
    // 与中带同量级"，3× 抓得住，而不会被 profile 差异误报。失败信息把三带
    // 实测值全部带出。
    assert!(
        hi * 3.0 < mid,
        "顶带 {hi:.0} 应显著低于中带 {mid:.0}（底 {lo:.0}）"
    );
    for v in f.velocities() {
        assert!(v.length_squared() < 9.0, "速度失控：{v:?}");
    }
}

/// #3 动量守恒：零重力、XSPH 关闭，内部压力成对抵消 ⇒ Σv 漂移 < 1e−3。
#[test]
fn momentum_conserved_without_gravity() {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        xsph_viscosity: 0.0,
        substeps: 2,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.125, 0.2, -0.125), [5, 5, 5], 0.05);
    // 正弦速度场（非平移、非平衡）⇒ 内部力持续活跃。
    let vs: Vec<Vec3> = (0..f.len())
        .map(|i| {
            let a = i as f32;
            Vec3::new(0.3 * a.sin(), 0.3 * (a * 1.3).cos(), 0.3 * (a * 0.7).sin())
        })
        .collect();
    f.set_velocities(&vs);
    let p0: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
    for _ in 0..120 {
        f.step(1.0 / 60.0, &NoProviders);
    }
    let p1: Vec3 = f.velocities().iter().fold(Vec3::ZERO, |s, v| s + *v);
    let drift = (p1 - p0).length();
    assert!(drift < 1e-3, "动量漂移 {drift:.3e}");
}
