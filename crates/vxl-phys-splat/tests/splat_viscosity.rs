//! **高斯粒子自场黏性判据**（②物理代理第三片，2026-10-08）。
//!
//! 判据链：
//! ① **等速度 ⇒ 逐位无变化**（金丝雀：均匀流场不该被黏性改动）；
//! ② **反向运动 ⇒ 相互拉平**（相对速度减小、总动能不增 —— XSPH 只耗散）；
//! ③ **动量守恒**：不等质量下 `Σ m_k·Δv_k ≈ 0`（对称系数逐对构造）；
//! ④ **关档 / 无速度槽 ⇒ 空操作**；⑤ 两跑逐位一致。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::viscosity::{apply_self_viscosity, SelfViscosity};
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 两核场（`opacities` 决定质量；速度由调用方 `set_kernel_velocities` 给）。
fn two(opacities: [f32; 2], dx: f32) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.medium_density = 1000.0;
    for (i, op) in opacities.iter().enumerate() {
        f.push(Splat {
            center: Vec3::new(i as f32 * dx, 0.0, 0.0),
            scale: Vec3::splat(0.2),
            rot: Mat3::IDENTITY,
            opacity: *op,
            color: [1.0, 1.0, 1.0],
        });
    }
    f
}

fn kinetic(f: &GaussianSplatField) -> f32 {
    let mut e = 0.0f32;
    for (k, v) in f.kernel_velocities().iter().enumerate() {
        e += 0.5 * f.splats()[k].mass(f.medium_density) * v.length_squared();
    }
    e
}

#[test]
fn equal_velocities_are_untouched() {
    let mut f = two([1.0, 1.0], 0.1);
    assert!(f.set_kernel_velocities(&[Vec3::new(1.0, 0.0, 0.0); 2]));
    let before = f.kernel_velocities().to_vec();
    let imp = apply_self_viscosity(&mut f, 1.0 / 60.0, SelfViscosity::default());
    assert_eq!(imp, Vec3::ZERO, "等速度 ⇒ 逐位无冲量");
    assert_eq!(f.kernel_velocities(), before, "等速度 ⇒ 速度逐位不变");
}

#[test]
fn opposing_velocities_are_smoothed_and_dissipate() {
    let mut f = two([1.0, 1.0], 0.1);
    assert!(f.set_kernel_velocities(&[Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0),]));
    let e0 = kinetic(&f);
    apply_self_viscosity(&mut f, 1.0 / 60.0, SelfViscosity::default());
    let v = f.kernel_velocities();
    assert!(v[0].x > -1.0, "左核应被拉平（v={v:?}）");
    assert!(v[1].x < 1.0, "右核应被拉平（v={v:?}）");
    assert!(
        (v[0].x + v[1].x).abs() < 1e-9,
        "等质量 ⇒ 速度应等大反向：v={v:?}"
    );
    let e1 = kinetic(&f);
    assert!(e1 < e0, "只耗散：KE {e0} → {e1}");
}

#[test]
fn momentum_is_conserved_with_unequal_masses() {
    let mut f = two([0.5, 1.5], 0.1);
    assert!(f.set_kernel_velocities(&[Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0),]));
    let (m0, m1) = (
        f.splats()[0].mass(f.medium_density),
        f.splats()[1].mass(f.medium_density),
    );
    // 步前动量（判据要的是 Δp = 0，不是 p = 0）
    let p0 = f.kernel_velocities()[0] * m0 + f.kernel_velocities()[1] * m1;
    let imp = apply_self_viscosity(&mut f, 1.0 / 60.0, SelfViscosity::default());
    let v = f.kernel_velocities();
    let p1 = v[0] * m0 + v[1] * m1;
    let scale = p1.length().max(1e-12);
    assert!(
        (p1 - p0).length() / scale < 1e-5,
        "动量须守恒：rel={}（p0={p0:?} p1={p1:?}）",
        (p1 - p0).length() / scale
    );
    assert!(
        imp.length() / scale < 1e-5,
        "返回的净冲量也应≈0：rel={}",
        imp.length() / scale
    );
}

#[test]
fn off_and_missing_slot_are_no_ops() {
    // 关档：ε = 0
    let mut f = two([1.0, 1.0], 0.05);
    assert!(f.set_kernel_velocities(&[Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0),]));
    let before = f.kernel_velocities().to_vec();
    let imp = apply_self_viscosity(&mut f, 1.0 / 60.0, SelfViscosity { epsilon: 0.0 });
    assert_eq!(imp, Vec3::ZERO);
    assert_eq!(f.kernel_velocities(), before, "关档必须空操作");

    // 无速度槽（空 = 未登记）⇒ 空操作，且不该凭空建槽
    let mut g = two([1.0, 1.0], 0.05);
    let imp2 = apply_self_viscosity(&mut g, 1.0 / 60.0, SelfViscosity::default());
    assert_eq!(imp2, Vec3::ZERO);
    assert!(g.kernel_velocities().is_empty(), "无槽 ⇒ 仍为空");
}

#[test]
fn viscosity_step_is_deterministic() {
    let run = || {
        let mut f = two([0.7, 1.3], 0.09);
        assert!(f.set_kernel_velocities(&[Vec3::new(-0.8, 0.2, 0.0), Vec3::new(0.6, -0.1, 0.3),]));
        for _ in 0..20 {
            apply_self_viscosity(&mut f, 1.0 / 60.0, SelfViscosity::default());
        }
        (f.kernel_velocities()[0], f.kernel_velocities()[1])
    };
    assert_eq!(run(), run(), "同场景两跑逐位一致");
}
