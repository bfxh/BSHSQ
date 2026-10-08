//! **高斯粒子自场压力判据**（②物理代理第二片，2026-10-08）。
//!
//! 判据链：
//! ① **自密度口径自洽**：孤立核在自身中心处的场密度 = `medium_density·opacity`（P11 质量口径的
//!    直接推论）⇒ `rest_density` 取同值时 p = 0；
//! ② **远距离无力**（金丝雀）：两核相距 ≫ 截断半径 ⇒ 净冲量**逐位为 0**、位置与速度不变；
//! ③ **重叠 ⇒ 相互推开**（斥力）+ 速度方向相反；
//! ④ **动量守恒**：`Σ m_k·Δv_k ≈ 0`（逐对反对称构造；残差只来自求和序），且**不等质量**时
//!    轻的核分到更大 |Δv|（金丝雀：证明质量真的进了力）；
//! ⑤ **关档**（`sound_speed = 0`）⇒ 空操作；⑥ 两跑逐位一致。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::MediumField as _;
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::pressure::{apply_self_pressure, SelfPressure};
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 建场：`centers[i]` 放第 i 颗核（尺度固定 0.2、不透明度取 `opacities[i]`）。
fn field(centers: &[Vec3], opacities: &[f32]) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.medium_density = 1000.0;
    for (c, op) in centers.iter().zip(opacities) {
        f.push(Splat {
            center: *c,
            scale: Vec3::splat(0.2),
            rot: Mat3::IDENTITY,
            opacity: *op,
            color: [1.0, 1.0, 1.0],
        });
    }
    f
}

#[test]
fn isolated_kernel_self_density_matches_the_mass_reading() {
    let f = field(&[Vec3::ZERO], &[1.0]);
    let rho = f.sample(Vec3::ZERO).density;
    assert!(
        (rho - 1000.0).abs() < 1e-2,
        "孤立核自密度应 = medium_density·opacity = 1000：ρ={rho}"
    );
    // 两核但**相距 ≫ 截断**（4σ = 0.8）⇒ 互相读不到 ⇒ 无斥力（金丝雀）
    let mut far = field(&[Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0)], &[1.0, 1.0]);
    let before: Vec<Vec3> = vec![far.splats()[0].center, far.splats()[1].center];
    let imp = apply_self_pressure(&mut far, 1.0 / 60.0, SelfPressure::default());
    assert_eq!(imp, Vec3::ZERO, "远距离必须逐位无冲量");
    assert_eq!(far.kernel_velocities(), vec![Vec3::ZERO; 2]);
    assert_eq!(far.splats()[0].center, before[0]);
    assert_eq!(far.splats()[1].center, before[1]);
}

#[test]
fn overlapping_kernels_repel_and_conserve_momentum() {
    let mut f = field(&[Vec3::ZERO, Vec3::new(0.1, 0.0, 0.0)], &[1.0, 1.0]);
    let m = f.splats()[0].mass(f.medium_density);
    let imp = apply_self_pressure(&mut f, 1.0 / 60.0, SelfPressure::default());
    let v = f.kernel_velocities();
    assert!(v[0].x < 0.0 && v[1].x > 0.0, "应沿连线相互推开：v={v:?}");
    assert!(
        (v[0].x + v[1].x).abs() < 1e-9,
        "等质量 ⇒ 速度应等大反向：v={v:?}"
    );
    let scale = imp.length().max(1e-12);
    assert!(
        scale < 1e-6 * m.max(1.0),
        "净冲量应≈0（逐对反对称）：|imp|={}",
        imp.length()
    );
    // 用等价的"逐核动量"口径再算一遍：Σ m·Δv（同一份速度读数）
    let sum = (v[0] + v[1]) * m;
    assert!(sum.length() < 1e-12, "Σ m·Δv 逐位为 0：{sum:?}");
}

#[test]
fn light_kernel_takes_more_velocity_than_the_heavy_one() {
    let mut f = field(&[Vec3::ZERO, Vec3::new(0.1, 0.0, 0.0)], &[0.5, 1.5]);
    apply_self_pressure(&mut f, 1.0 / 60.0, SelfPressure::default());
    let v = f.kernel_velocities();
    assert!(v[0].length() > v[1].length(), "轻核应分到更大速度：v={v:?}");
    // 动量账：Σ m·Δv ≈ 0（质量不同也成立 —— 这是逐对构造出来的）
    let (m0, m1) = (
        f.splats()[0].mass(f.medium_density),
        f.splats()[1].mass(f.medium_density),
    );
    let sum = v[0] * m0 + v[1] * m1;
    let scale = (v[0] * m0).length().max(1e-12);
    assert!(
        sum.length() / scale < 1e-6,
        "不等质量下动量仍须守恒：rel={}",
        sum.length() / scale
    );
}

#[test]
fn zero_sound_speed_is_a_no_op_and_step_is_deterministic() {
    let mut f = field(&[Vec3::ZERO, Vec3::new(0.05, 0.0, 0.0)], &[1.0, 1.0]);
    let imp = apply_self_pressure(
        &mut f,
        1.0 / 60.0,
        SelfPressure {
            sound_speed: 0.0,
            ..SelfPressure::default()
        },
    );
    assert_eq!(imp, Vec3::ZERO, "关档必须空操作");
    assert!(
        f.kernel_velocities().is_empty(),
        "关档不该建速度槽（空 = 未登记）"
    );

    let run = || {
        let mut g = field(
            &[
                Vec3::ZERO,
                Vec3::new(0.12, 0.0, 0.0),
                Vec3::new(0.0, 0.11, 0.0),
            ],
            &[1.0, 0.8, 1.2],
        );
        for _ in 0..30 {
            apply_self_pressure(&mut g, 1.0 / 60.0, SelfPressure::default());
        }
        (
            g.kernel_velocities()[0],
            g.kernel_velocities()[1],
            g.kernel_velocities()[2],
        )
    };
    assert_eq!(run(), run(), "同场景两跑逐位一致");
}
