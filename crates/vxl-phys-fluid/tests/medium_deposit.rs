//! **点式反作用沉积（`MediumField::deposit`）**的判据（2026-10-05 落地）：
//! ① `Σ m·Δv` **严格守恒**（分摊系数 `k = 1/(Σw·m)` 的全部意义所在）；
//! ② 无近粒（真空）或 `momentum = 0` ⇒ **逐位不变**（不打空气、零成本关档）；
//! ③ 沉积后 `sample` 读到的介质速度朝注入方向走（表示通道接得上）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()` / glob `use`（新文件零基线）。
use vxl_phys_core::interop::{MediumField, NoProviders};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// 流体总动量 `Σ m·v`（只算流体粒子段）。
fn momentum(f: &FluidSystem) -> Vec3 {
    let m = f.particle_mass();
    f.velocities().iter().fold(Vec3::ZERO, |a, v| a + *v * m)
}

/// 4³ 水块（间距 0.05、核半径默认）——`x` 取块心 ⇒ 27 邻域必有多粒。
fn block() -> (FluidSystem, Vec3) {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [4, 4, 4], 0.05);
    // 邻域网格在 `step` 的密度轮里才建（`sample`/`deposit` 都读它）。
    f.step(1.0 / 60.0, &NoProviders);
    // 清掉压力/黏度带出来的速度（本判据只看**沉积**那一下的动量账）。
    f.set_velocities(&vec![Vec3::ZERO; f.len()]);
    (f, Vec3::new(0.0, 0.275, 0.0))
}

#[test]
fn deposit_conserves_total_momentum() {
    let (mut f, x) = block();
    let before = momentum(&f);
    let j = Vec3::new(0.7, -0.3, 0.11);
    f.deposit(x, j, 0.0, 0.0);
    let got = momentum(&f) - before;
    let err = (got - j).length();
    assert!(
        err <= 1e-4 * j.length(),
        "Σ m·Δv 应等于注入动量：got={got:?} want={j:?} err={err:.3e}"
    );
}

#[test]
fn deposit_into_vacuum_or_zero_is_bitwise_no_op() {
    let (mut f, _) = block();
    let before = momentum(&f);
    // ① 真空（远离块）
    f.deposit(
        Vec3::new(50.0, 50.0, 50.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.0,
        0.0,
    );
    // ② 零动量（块内有近粒，但动量 0 ⇒ 首行短路）
    f.deposit(Vec3::new(0.0, 0.275, 0.0), Vec3::ZERO, 0.0, 0.0);
    let after = momentum(&f);
    assert_eq!(
        before.x.to_bits(),
        after.x.to_bits(),
        "真空/零动量：x 逐位不变"
    );
    assert_eq!(
        before.y.to_bits(),
        after.y.to_bits(),
        "真空/零动量：y 逐位不变"
    );
    assert_eq!(
        before.z.to_bits(),
        after.z.to_bits(),
        "真空/零动量：z 逐位不变"
    );
}

#[test]
fn deposited_momentum_shows_up_in_sample() {
    let (mut f, x) = block();
    let v0 = f.sample(x).velocity;
    let j = Vec3::new(0.0, 1.0, 0.0);
    f.deposit(x, j, 0.0, 0.0);
    let v1 = f.sample(x).velocity;
    assert!(
        v1.y > v0.y,
        "注入 +y 动量后，介质采样速度应沿 +y 增大：v0={v0:?} v1={v1:?}"
    );
}
