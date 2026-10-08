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
        air_mass: 0.0,
    })
}

fn air_with_slot(wind: [f32; 3], air_mass: f32) -> AeroState {
    AeroState::new(AeroConfig {
        wind,
        air_mass,
        ..AeroConfig::default()
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

/// **动量槽（双向那一半）**：有限 `air_mass` ⇒ 风速按 `Δv = J/M` 变慢，且 `M·Δv = J` 闭合；
/// 默认（`air_mass = 0`）被上面的金丝雀钉住（吸收不改风）。
#[test]
fn momentum_slot_slows_the_wind_by_the_absorbed_momentum() {
    let mut m = air_with_slot([3.0, 0.0, 0.0], 8.0);
    let v0 = m.sample(Vec3::ZERO).velocity;
    let j = Vec3::new(-2.0, 0.5, 0.0); // 大气**吸收**的动量（受体拿到 −j）
    m.deposit(Vec3::ZERO, j, 0.0, 0.0);
    let v1 = m.sample(Vec3::ZERO).velocity;
    let dv = v1 - v0;
    let want = j * (1.0 / 8.0);
    let rel = (dv - want).length() / want.length();
    assert!(rel < 1e-6, "风速变化应 = J/M：rel={rel:e}（dv={dv:?}）");
    let acc = (dv * 8.0 - j).length() / j.length();
    assert!(acc < 1e-6, "动量账 M·Δv = J 应闭合：rel={acc:e}");
    assert!(v1.x < v0.x, "受体拿到 +x 动量 ⇒ 大气沿 +x 变慢");
    let ke_drop = 0.5 * 8.0 * (v0.length_squared() - v1.length_squared());
    assert!(
        ke_drop > 0.0,
        "大气变慢 ⇒ 它的动能下降（要拿能量得先有能量）：{ke_drop:e}"
    );
}
