//! **高斯粒子的自场压力**（②物理代理第二片：让"高斯粒子"真的有相互作用，2026-10-08）。
//!
//! 液体侧的口径照搬（`vxl-phys-fluid::fluid_force`）：Tait `p = B·((ρ/ρ0)^7 − 1)`、
//! `p ≥ 0` 钳制（**张力抑制**，同一条理由：负压经对称式会变成巨大吸力）、对称压力项
//! `m_j·(p_k/ρ_k² + p_j/ρ_j²)·(−∇W_kj)` ⇒ **逐对反对称、总动量守恒**。差别只在"核"：
//! 这里的 `W` 就是**本场自己的归一化高斯**，密度取 `MediumField::sample`（P11 定案后，
//! 质量与密度是同一本账）。
//!
//! 口径：
//! - 归一化核 `w_j(p) = opacity_j·exp(−½α_j(p)) / m_j(ρ=1)`（∫w = 1）；密度
//!   `ρ_k = Σ_j m_j·w_j(c_k) = sample(c_k).density`（kg/m³，与 `Splat::mass` 同源）。
//! - `−∇W` 取**对称化**形式 `(W̄/ℓ²)·r`：`W̄ = (w_k(c_j) + w_j(c_k))/2`（各向异性也对称）、
//!   `ℓ = (r_k(r̂) + r_j(r̂))/2`（两核沿连线方向的支撑半径均值）、`r = c_k − c_j`
//!   ⇒ `m_k·dv_k = −m_j·dv_j` **逐对精确**。
//! - **只做斥力**（`p ≥ 0`）：没有内聚项 ⇒ 远距离核与孤立核都不受力（判据里的金丝雀）。
//! - `medium_density ≤ 0` / `sound_speed ≤ 0` / `dt ≤ 0` / 核数 < 2 ⇒ 空操作。
//! - 确定性：核按注册序枚举对（用既有的候选迭代）、无 HashMap、无浮点归约顺序变化。
use crate::GaussianSplatField;
use vxl_phys_core::Vec3;

/// 自场压力参数（`sound_speed = 0` ⇒ 关档）。
#[derive(Clone, Copy, Debug)]
pub struct SelfPressure {
    /// 静止密度 ρ0（kg/m³）。孤立核的自密度 = `medium_density·opacity` ⇒ 两者相等时 p = 0。
    pub rest_density: f32,
    /// 声速 c（m/s）：`B = c²ρ0/γ` —— 压力的硬软旋钮。
    pub sound_speed: f32,
}

impl Default for SelfPressure {
    fn default() -> Self {
        Self {
            rest_density: 1000.0,
            sound_speed: 1.0,
        }
    }
}

/// Tait 指数（与液体侧同值；7 走整数次幂乘法展开，避免 `powf` 的平台差异）。
const GAMMA: f32 = 7.0;

/// 归一化高斯在 `p` 处的值 `w = opacity·exp(−½α)/m(ρ=1)`（截断与 `density_grad` 同）。
fn kernel_norm(field: &GaussianSplatField, s: &crate::Splat, p: Vec3) -> f32 {
    let ax = s.axes();
    let a = GaussianSplatField::alpha(s, &ax, p);
    if a > field.cut {
        return 0.0;
    }
    (-0.5 * a).exp() * s.opacity / s.mass(1.0)
}

/// 施加一个 `dt` 的自场压力（只斥力）；返回**净冲量** `Σ m_k·Δv_k`（判据/审计：逐对反对称 ⇒ ≈0）。
pub fn apply_self_pressure(field: &mut GaussianSplatField, dt: f32, cfg: SelfPressure) -> Vec3 {
    let n = field.splats.len();
    if disabled(field, dt, cfg, n) {
        return Vec3::ZERO;
    }
    let (rho, press, mass) = thermo(field, cfg, n);
    if field.kern_vel.len() != n {
        field.kern_vel.clear();
        field.kern_vel.resize(n, Vec3::ZERO);
    }
    let dv = pair_impulses(field, &rho, &press, &mass, dt);
    let mut impulse = Vec3::ZERO;
    for k in 0..n {
        field.kern_vel[k] += dv[k];
        impulse += dv[k] * mass[k];
    }
    impulse
}

/// 关档判据（任一条成立 ⇒ 空操作）。
fn disabled(field: &GaussianSplatField, dt: f32, cfg: SelfPressure, n: usize) -> bool {
    dt.is_nan()
        || dt <= 0.0
        || n < 2
        || cfg.sound_speed.is_nan()
        || cfg.sound_speed <= 0.0
        || cfg.rest_density <= 0.0
        || field.medium_density <= 0.0
}

/// 每核的热力学量 `(ρ, p, m)`：Tait + 张力抑制（只保留斥力）。
fn thermo(
    field: &GaussianSplatField,
    cfg: SelfPressure,
    n: usize,
) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let rho0 = cfg.rest_density;
    let b = cfg.sound_speed * cfg.sound_speed * rho0 / GAMMA;
    let mut rho = vec![0.0f32; n];
    let mut press = vec![0.0f32; n];
    let mut mass = vec![0.0f32; n];
    for k in 0..n {
        rho[k] = field.density_grad(field.splats[k].center).0 * field.medium_density;
        mass[k] = field.splats[k].mass(field.medium_density);
        let q = rho[k] / rho0;
        let q2 = q * q;
        let q4 = q2 * q2;
        press[k] = (b * (q * q2 * q4 - 1.0)).max(0.0); // 只斥力（张力抑制）
    }
    (rho, press, mass)
}

/// 逐对速度增量（`m_k·dv_k = −m_j·dv_j` **逐对精确** ⇒ 总动量守恒）。
fn pair_impulses(
    field: &GaussianSplatField,
    rho: &[f32],
    press: &[f32],
    mass: &[f32],
    dt: f32,
) -> Vec<Vec3> {
    let n = field.splats.len();
    let mut dv = vec![Vec3::ZERO; n];
    for k in 0..n {
        // 候选 = 与 `density_grad` 同一条路（网格序 × 注册序；网格脏时全扫）
        for j in field.candidate_ids(field.splats[k].center) {
            if j <= k {
                continue;
            }
            let r = field.splats[k].center - field.splats[j].center;
            let d2 = r.length_squared();
            if d2.is_nan() || d2 <= 0.0 {
                continue; // 完全重合：方向不可定
            }
            let w = 0.5
                * (kernel_norm(field, &field.splats[k], field.splats[j].center)
                    + kernel_norm(field, &field.splats[j], field.splats[k].center));
            if w.is_nan() || w <= 0.0 {
                continue;
            }
            let rhat = r * (1.0 / d2.sqrt());
            let ell =
                0.5 * (field.splats[k].radius_along(rhat) + field.splats[j].radius_along(rhat));
            if ell.is_nan() || ell <= 0.0 {
                continue;
            }
            let coef = (press[k] / (rho[k] * rho[k]) + press[j] / (rho[j] * rho[j]))
                * (w / (ell * ell))
                * dt;
            let a = r * coef; // 单位质量冲量方向项
            dv[k] += a * mass[j];
            dv[j] -= a * mass[k];
        }
    }
    dv
}
