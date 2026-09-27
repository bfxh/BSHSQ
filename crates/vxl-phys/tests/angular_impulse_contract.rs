//! **角冲量注入契约**（§8.4.28）：`force`/`torque` 是**每子步消费并清零**的累加器
//! （`Integrator::integrate_velocities`：`ω += I⁻¹·τ·dt_sub` 之后 `τ = 0`）。因此**在 tick 末
//! （子步全部完成之后）注入的力矩只被一个子步消费** ⇒ 要交付"整 tick 的角冲量 `J_τ`"，
//! 注入值必须是 **`J_τ / dt_sub`**（`dt_sub = dt / substeps`），**不是** `J_τ / dt`
//! （后者只交付 `1/substeps`，默认 2 ⇒ **差 2×**）。
//!
//! **本条为什么现在就能立**：门面回填角冲量的路径（`rope_pass` 的 `bodies.torque += …`）当前**停用**
//! （接触模型看不见转动，§8.4.10），所以它的口径错误**不影响任何可观测行为**；但契约**可以**用等价注入
//! 钉住——本文件直接按"tick 末注入"的方式写累加器（`w.bodies.torque[b] += …` 在 `w.step()` 之前，
//! 与绳索在 `domain_pass` 里注入**同为"只被一个子步消费"**）。⇒ 启用转动反作用时（2c）照这一行写即可。
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

/// 造一个世界 + 一个球体（`rot = identity` ⇒ 世界惯量 = 本体系惯量，便于逐轴对照）。
fn world_with_sphere() -> (World, usize, Vec3) {
    let mut w = World::new(PhysConfig::default());
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 10.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let i_inv = w.bodies.local_inv_inertia[b];
    (w, b, i_inv)
}

/// **正确口径**：`J_τ / dt_sub` ⇒ Δω 恰好是 `I⁻¹·J_τ`（整 tick 的角冲量）。
#[test]
fn end_of_tick_torque_injection_delivers_the_full_impulse() {
    let (mut w, b, i_inv) = world_with_sphere();
    let dt_sub = w.config.dt / w.config.substeps.max(1) as f32;
    let j_tau = Vec3::new(0.0, 0.0, 2.0); // 打算交付的"整 tick 角冲量"
    w.bodies.torque[b] += j_tau * (1.0 / dt_sub);
    let w0 = w.bodies.angvel(b);
    w.step();
    let dw = w.bodies.angvel(b) - w0;
    let want = i_inv.mul_per_elem(j_tau);
    println!(
        "substeps={} dt_sub={dt_sub:.6} | Δω=({:.5},{:.5},{:.5}) | 期望 I⁻¹·J_τ=({:.5},{:.5},{:.5})",
        w.config.substeps, dw.x, dw.y, dw.z, want.x, want.y, want.z
    );
    assert!(
        (dw.z - want.z).abs() <= 1e-4 * want.z.abs(),
        "Δω.z 该等于 I⁻¹·J_τ（实测 {:.6} vs 期望 {:.6}）——差 1/2 说明注入用了 `J_τ/dt` 而不是 `J_τ/dt_sub`",
        dw.z,
        want.z
    );
}

/// **金丝雀**：把口径换成"÷ tick dt"（即老写法）⇒ 只交付 `1/substeps`、判据会红。
/// 这条证明上一条**能分辨两种口径**（否则它只是个恒真断言）。
#[test]
fn wrong_caliber_halves_the_impulse_canary() {
    let (mut w, b, i_inv) = world_with_sphere();
    let substeps = w.config.substeps.max(1);
    let j_tau = Vec3::new(0.0, 0.0, 2.0);
    w.bodies.torque[b] += j_tau * (1.0 / w.config.dt); // ← 老写法（少了 substeps 这一因子）
    let w0 = w.bodies.angvel(b);
    w.step();
    let dw = w.bodies.angvel(b) - w0;
    let want = i_inv.mul_per_elem(j_tau);
    let ratio = dw.z / want.z;
    println!("老口径（÷ tick dt）实测 Δω/期望 = {ratio:.6}（应 ≈ 1/{substeps}）");
    assert!(
        (ratio - 1.0 / substeps as f32).abs() < 1e-3,
        "老口径该只交付 1/{substeps}（实测比值 {ratio:.6}）——比值变了说明累加器语义改了，\
         那本文件两条判据都要跟着重写"
    );
}
