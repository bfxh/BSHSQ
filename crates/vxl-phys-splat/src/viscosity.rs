//! **高斯粒子的自场黏性**（②物理代理第三片：与自场压力配对，凑齐对称 SPH 动量方程，2026-10-08）。
//!
//! 口径照抄液体侧 XSPH 的**对称形式**（`vxl-phys-fluid::fluid_force` 的注释：
//! `Σ m_j·2/(ρ_k+ρ_j)·W·(v_j − v_k)`，系数逐对对称 ⇒ 与压力项同样动量守恒）；核 = 本场自己的
//! 归一化高斯（`pressure::kernel_norm`，各向异性取对称平均）。
//!
//! 口径与边界：
//! - `dv_k = ε·dt·Σ_j m_j·2/(ρ_k+ρ_j)·w̄_kj·(v_j − v_k)`；`m_k·dv_k = −m_j·dv_j` **逐对精确**。
//! - **只耗散**：系数恒正 ⇒ 相对速度被拉平（动能不增）；等速度 ⇒ 逐位无变化（金丝雀）。
//! - `ε ≤ 0` / 无速度槽（`kern_vel` 空 = 未登记）/ `dt ≤ 0` / `medium_density ≤ 0` / 核数 < 2 ⇒ 空操作。
//! - 确定性：核按注册序枚举对（沿用候选迭代）、无 HashMap、无浮点归约顺序变化。
use crate::pressure::{densities, kernel_norm};
use crate::GaussianSplatField;
use vxl_phys_core::Vec3;

/// 自场黏性参数（`epsilon = 0` ⇒ 关档）。
#[derive(Clone, Copy, Debug)]
pub struct SelfViscosity {
    /// XSPH 无量纲系数（与液体侧 `xsph_viscosity` 同量纲）。
    pub epsilon: f32,
}

impl Default for SelfViscosity {
    fn default() -> Self {
        Self { epsilon: 0.1 }
    }
}

/// 施加一个 `dt` 的自场黏性；返回**净冲量** `Σ m_k·Δv_k`（逐对反对称 ⇒ ≈0）。
pub fn apply_self_viscosity(field: &mut GaussianSplatField, dt: f32, cfg: SelfViscosity) -> Vec3 {
    let n = field.splats.len();
    if disabled(field, dt, cfg, n) {
        return Vec3::ZERO;
    }
    let rho = densities(field, n);
    let mass: Vec<f32> = (0..n)
        .map(|k| field.splats[k].mass(field.medium_density))
        .collect();
    let dv = pair_smoothing(field, &rho, &mass, dt, cfg);
    let mut impulse = Vec3::ZERO;
    for k in 0..n {
        field.kern_vel[k] += dv[k];
        impulse += dv[k] * mass[k];
    }
    impulse
}

/// 关档判据（任一条成立 ⇒ 空操作）。
fn disabled(field: &GaussianSplatField, dt: f32, cfg: SelfViscosity, n: usize) -> bool {
    dt.is_nan()
        || dt <= 0.0
        || n < 2
        || cfg.epsilon.is_nan()
        || cfg.epsilon <= 0.0
        || field.medium_density <= 0.0
        || field.kern_vel.len() != n // 没有速度槽（空 = 未登记）⇒ 无黏性可言
}

/// 逐对速度平滑（对称系数 ⇒ `m_k·dv_k = −m_j·dv_j` 逐对精确）。
fn pair_smoothing(
    field: &GaussianSplatField,
    rho: &[f32],
    mass: &[f32],
    dt: f32,
    cfg: SelfViscosity,
) -> Vec<Vec3> {
    let n = field.splats.len();
    let mut dv = vec![Vec3::ZERO; n];
    for k in 0..n {
        for j in field.candidate_ids(field.splats[k].center) {
            if j <= k {
                continue;
            }
            let w = 0.5
                * (kernel_norm(field, &field.splats[k], field.splats[j].center)
                    + kernel_norm(field, &field.splats[j], field.splats[k].center));
            if w.is_nan() || w <= 0.0 {
                continue;
            }
            let denom = rho[k] + rho[j];
            if denom.is_nan() || denom <= 0.0 {
                continue;
            }
            // 对称系数（只看 k/j 的和与 w̄ ⇒ 与交换无关）
            let c = cfg.epsilon * dt * (2.0 / denom) * w;
            let a = (field.kern_vel[j] - field.kern_vel[k]) * c;
            dv[k] += a * mass[j];
            dv[j] -= a * mass[k];
        }
    }
    dv
}
