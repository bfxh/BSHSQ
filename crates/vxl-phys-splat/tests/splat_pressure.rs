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
//! ⑦ **扫距判据（内聚为何不在此口径内）**：全带宽 `(0, 4σ)` 上两核**只会互相推开**（或无力）——
//!    P11 质量口径让孤立核自密度恒 = ρ0、任何邻居都把 ρ 抬高 ⇒ `p = B((ρ/ρ0)^7−1) ≥ 0` 恒成立
//!    ⇒ **没有"稀疏带吸引"**；要做内聚/表面张力得**另立机制**（见 `KNOWLEDGE.md` §K）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{MediumField as _, NoProviders};
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::particles::{step_particles, ParticleStep};
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

/// **扫距判据**：分离度 `d/σ ∈ (0, 4)` 全带上，两核的**相对速度永远不减小**（推开或无力）。
/// 这把"内聚/表面张力需要另立机制"钉成证据，而不是一句代数断言。
#[test]
fn pressure_pair_is_never_attractive_across_the_whole_band() {
    let sigma = 0.2f32;
    let mut worst = 0.0f32; // 带内最负的相对速度变化（若为负 ⇒ 存在吸引支）
    let mut nearest = 0.0f32;
    for k in 1..40 {
        let d = sigma * (k as f32) * 0.1; // 0.1σ .. 3.9σ
        let mut f = field(&[Vec3::ZERO, Vec3::new(d, 0.0, 0.0)], &[1.0, 1.0]);
        apply_self_pressure(&mut f, 1.0 / 60.0, SelfPressure::default());
        let v = f.kernel_velocities();
        // 相对速度（右减左）沿 +x：>0 = 推开、=0 = 无力、<0 = 吸引
        let rel = v[1].x - v[0].x;
        if rel < worst {
            worst = rel;
            nearest = d;
        }
    }
    println!("扫距：带内最负相对速度 = {worst:.3e}（出现在 d={nearest:.4}）");
    assert!(
        worst >= 0.0,
        "全带内不许出现吸引（最负 {worst:.3e} @ d={nearest}）"
    );
}

/// 内聚档（`cohesion = 2300 Pa`）——量级锚：`B = c²ρ0/γ = 1000/7 ≈ 142.9 Pa`，
/// 目标平衡 **密度比 1.5**（`ρ_eq/ρ0 = (1 + cohesion/B)^{1/7} = 1.5`）⇒ 解析平衡间距
/// `d_eq = σ·√(−2 ln 0.5) ≈ 1.177σ`（σ = 0.2 ⇒ 0.2354）。
fn cohesion_cfg() -> SelfPressure {
    SelfPressure {
        cohesion: 2300.0,
        ..SelfPressure::default()
    }
}

/// **解析平衡 vs 实测符号翻转**：`p(ρ) = cohesion` 给出的平衡间距两侧，净力必须异号。
/// 这条把"内聚有界且可标定"钉住（不是"找个系数看起来像"）。
#[test]
fn cohesion_equilibrium_matches_the_analytic_density() {
    let sigma = 0.2f32;
    let b = 1.0f32 * 1.0 * 1000.0 / 7.0; // sound_speed²·ρ0/γ
    let ratio = (1.0 + 2300.0 / b).powf(1.0 / 7.0); // ρ_eq/ρ0（= 1.5）
    let e = ratio - 1.0;
    let d_eq = sigma * (-2.0 * e.ln()).sqrt();
    assert!(
        (ratio - 1.5).abs() < 1e-3,
        "量级锚：平衡密度比应 ≈1.5，得 {ratio}"
    );

    let rel_at = |d: f32| {
        let mut f = field(&[Vec3::ZERO, Vec3::new(d, 0.0, 0.0)], &[1.0, 1.0]);
        apply_self_pressure(&mut f, 1.0 / 240.0, cohesion_cfg());
        let v = f.kernel_velocities();
        v[1].x - v[0].x
    };
    let inside = rel_at(d_eq * 0.9);
    let outside = rel_at(d_eq * 1.1);
    println!("内聚平衡：d_eq={d_eq:.4} | 0.9·d_eq 相对速度={inside:.3e} | 1.1·d_eq={outside:.3e}");
    assert!(
        inside > 0.0,
        "平衡点内侧必须是**斥力**（实得 {inside:.3e}）"
    );
    assert!(
        outside < 0.0,
        "平衡点外侧必须是**吸引**（实得 {outside:.3e}）"
    );
}

/// **自由演化收敛**：两核从平衡点外侧出发（零重力、每步阻尼 0.95）⇒ 间距收敛到 `d_eq` 附近、
/// 速度收敛、无 NaN（内聚是**有界**的：近距仍被压力顶住，不会塌成一点）。
#[test]
fn cohesion_pair_converges_to_the_equilibrium() {
    let sigma = 0.2f32;
    let d_eq = sigma * 1.176_7;
    let mut f = field(&[Vec3::ZERO, Vec3::new(d_eq * 1.6, 0.0, 0.0)], &[1.0, 1.0]);
    let cfg = cohesion_cfg();
    let step = ParticleStep {
        gravity: Vec3::ZERO,
        damping: 0.95,
    };
    for _ in 0..900 {
        apply_self_pressure(&mut f, 1.0 / 240.0, cfg);
        step_particles(&mut f, 1.0 / 240.0, &NoProviders, &[], step);
    }
    let d = (f.splats()[1].center.x - f.splats()[0].center.x).abs();
    let v = f.kernel_velocities();
    println!(
        "内聚收敛：d={d:.4}（目标 {d_eq:.4}）| |v|={:.3e}",
        v[0].length()
    );
    assert!(d.is_finite() && d > 0.0, "间距必须有限：d={d}");
    assert!(
        (d - d_eq).abs() < 0.25 * d_eq,
        "应收敛到平衡间距附近：d={d:.4} vs d_eq={d_eq:.4}"
    );
    assert!(v[0].length() < 0.05, "应已收敛：|v|={}", v[0].length());
}

/// **动量守恒**（内聚档、不等质量）：`Δp = 0` 仍是逐对构造出来的（与斥力同一套反对称写法）。
#[test]
fn cohesion_preserves_momentum_with_unequal_masses() {
    let mut f = field(&[Vec3::ZERO, Vec3::new(0.35, 0.0, 0.0)], &[0.5, 1.5]);
    let (m0, m1) = (
        f.splats()[0].mass(f.medium_density),
        f.splats()[1].mass(f.medium_density),
    );
    let p0 = f.kernel_velocities().first().copied().unwrap_or(Vec3::ZERO) * m0
        + f.kernel_velocities().get(1).copied().unwrap_or(Vec3::ZERO) * m1;
    let imp = apply_self_pressure(&mut f, 1.0 / 240.0, cohesion_cfg());
    let v = f.kernel_velocities();
    let p1 = v[0] * m0 + v[1] * m1;
    let scale = p1.length().max(1e-12);
    assert!(
        (p1 - p0).length() / scale < 1e-6,
        "内聚档动量仍须守恒：Δp={:?}",
        p1 - p0
    );
    assert!(imp.length() / scale < 1e-6, "净冲量≈0：{imp:?}");
    assert!(v[1].x < v[0].x, "外侧核应被拉向内侧（吸引）：v={v:?}");
}
