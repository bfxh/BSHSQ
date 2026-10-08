//! **空气作为 `MediumField`** 的判据（`ROUTE §4`「风 × 液」的接口半，2026-10-08）。
//!
//! 判据链：
//! ① 采样与位置无关（均匀风场，逐位相同）、`velocity = wind`、`occupied = 1`；
//! ② `air_density <= 0` ⇒ 显式真空（关档）；
//! ③ `deposit` 累加吸收账（按调用序），**不改风速**（运动学背景的金丝雀）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_aero::{AeroConfig, AeroState};
use vxl_phys_core::interop::{MediumField, MediumSample};
use vxl_phys_core::Vec3;

fn air(wind: [f32; 3], density: f32) -> AeroState {
    AeroState::new(AeroConfig {
        air_density: density,
        drag_coefficient: 1.0,
        lift_slope: 5.0,
        wind,
    })
}

#[test]
fn air_is_uniform_and_carries_the_wind() {
    let m = air([1.5, -0.25, 0.0], 1.225);
    let a = m.sample(Vec3::ZERO);
    let b = m.sample(Vec3::new(3.0, -2.0, 7.5));
    assert_eq!(a, b, "均匀风场 ⇒ 采样与位置无关（逐位）");
    assert_eq!(a.density, 1.225);
    assert_eq!(a.velocity, Vec3::new(1.5, -0.25, 0.0));
    assert_eq!(a.occupied, 1.0, "开放空气处处可用");
    assert_eq!((a.viscosity, a.temperature), (0.0, 0.0), "不编造黏性/温度");
}

#[test]
fn zero_density_is_explicit_vacuum() {
    let m = air([9.0, 0.0, 0.0], 0.0);
    assert_eq!(m.sample(Vec3::ZERO), MediumSample::VACUUM, "关档 = 真空");
}

#[test]
fn deposit_absorbs_momentum_without_changing_the_wind() {
    let mut m = air([2.0, 0.0, 0.0], 1.225);
    let before = m.sample(Vec3::ZERO);
    let j1 = Vec3::new(0.3, -0.1, 0.05);
    let j2 = Vec3::new(-0.2, 0.4, 0.0);
    m.deposit(Vec3::ZERO, j1, 0.0, 0.0);
    m.deposit(Vec3::new(1.0, 0.0, 0.0), j2, 0.0, 0.0);
    assert_eq!(m.absorbed, j1 + j2, "吸收账 = 注入动量之和（按调用序）");
    let after = m.sample(Vec3::ZERO);
    assert_eq!(after, before, "金丝雀：运动学背景 ⇒ 吸收不改风（逐位）");
    m.deposit(Vec3::ZERO, Vec3::ZERO, 0.0, 0.0);
    assert_eq!(m.absorbed, j1 + j2, "零动量沉积不记账");
}
