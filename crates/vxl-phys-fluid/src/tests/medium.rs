//! 介质采样：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::still_block;
use vxl_phys_core::interop::{MediumField, NoProviders};
use vxl_phys_core::Vec3;

/// #8 **介质场采样**（`MediumField`，2a 第一刀）：块内晶格点读到 ≈ρ0。
/// 容差比内部密度测试宽（同核同式但**不含自身项与鬼影项**，且 5³ 块只剩 2 格余量）。
#[test]
fn medium_sample_center_reads_rest_density() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let c = Vec3::new(0.025, 0.325, 0.025); // origin + (2+½)·0.05 = 晶格点上
    let s = f.sample(c);
    let rho0 = f.config().rest_density;
    assert!(
        (s.density - rho0).abs() < 0.15 * rho0,
        "块中心密度 {:?} 应≈ρ0 {:?}",
        s.density,
        rho0
    );
    assert!(s.occupied > 0.85, "占用率 {:?} 应接近 1", s.occupied);
}

/// #9 采样口径：**无介质处返回真空**（核带外 ⇒ 不是"零密度的一团水"）。
#[test]
fn medium_sample_far_is_vacuum() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let far = Vec3::new(0.025, 0.325 + 10.0 * f.h, 0.025);
    let s = f.sample(far);
    assert_eq!(s.density, 0.0, "核带外不应有密度");
    assert_eq!(s.occupied, 0.0, "核带外占用率应为 0");
}

/// #10 采样速度 = **Shepard 平均**（Σwᵥ/Σw）⇒ 均匀流场下应与输入一致。
#[test]
fn medium_sample_velocity_follows_uniform_flow() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let v = Vec3::new(1.5, -0.25, 0.75);
    let vs = vec![v; f.len()];
    f.set_velocities(&vs);
    let s = f.sample(Vec3::new(0.025, 0.325, 0.025));
    assert!(
        (s.velocity - v).length() < 1e-5,
        "采样速度 {:?} 应≈{:?}",
        s.velocity,
        v
    );
}

// ───────────────────────── 2b：Akinci 两层边界粒子 ─────────────────────────
// 判据见 `docs/M1-EXIT.md` §4：① 近壁密度天然正确 ② 稳定性 ③ 对无流体场景
// 逐位不变（由 `crates/vxl-phys` 的四哈希门守）④ 造价。
// 端到端（体真被浮起来 / 体推水）在 `crates/vxl-phys/tests/fluid_boundary.rs`。
