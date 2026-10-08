//! **液面表面驱动**（`ROUTE.md` §4「风 × 液（表面驱动）」那一格的第一片，2026-10-08）。
//!
//! 物理：任何 `MediumField`（生产路径上是 `vxl_phys_aero::AeroState` = 空气）被逐粒采样，只对
//! **自由表面**粒子（密度亏超过阈值：`ρᵢ < surface_ratio·ρ0`）施加
//! Bridson 线化阻力
//!   `F = ½·ρ_air·Cd·A·|u|·u`，`u = v_air − vᵢ`，`A = spacing²`（粒子的支流面积）
//! 并把**反作用** `−F·dt` 沉积回介质（→ 大气 `absorbed` 审计）。内部粒子（`ρᵢ ≥ ρ0`）不受力 ——
//! 这正是"表面驱动"与"整体拖拽"的区别。
//!
//! **阈值的实测依据**（8³ 水块 @ spacing 0.05，本机一轮实测）：内部粒子的残差亏 ≤ 1e-4·ρ0
//! （实测 `ρmax = 999.99994`），自由表面亏 ≥ 0.4·ρ0（实测角点 `ρmin = 515.15 = 0.515·ρ0`）
//! ⇒ 默认 `surface_ratio = 0.9` 落在两个量级中间。**别用严格 `ρ < ρ0`**：那会把整个已松弛的
//! 流体都当表面，判据测到的就变成"整体拖拽"（本机实测：整块 60 tick 平移 0.108 m）。
//!
//! 口径与边界：
//! - **速度空间注入**（半隐式）：直接给 `velᵢ` 加 `F·dt/mᵢ`，与 SPH 自己的力管线并列；
//!   密度取**上一 tick 末**的快照（门面段位：`aero_pass` 在体子步里、`fluid_pass` 在 tick 末）。
//!   ⚠️ `dens ≤ 0` = **密度快照尚未算过**（首 tick 之前）⇒ 该粒子**不驱动**（与状态桥的
//!   "空段 = 未登记"同一取舍；否则整块会在首 tick 被均匀踢一脚，判据测到的就不是表面驱动）。
//! - 返回**流体侧净注入动量** `Σ F·dt`（守恒对账用；介质侧同时记 `−该值`）。
//! - `Cd ≤ 0` / `dt ≤ 0` / 无流体 ⇒ 空操作（零成本短路）；`wind = 0` 且流体静止 ⇒ 逐位不变。
//! - 确定性：按粒子索引序、无分配、纯 f32。
use crate::FluidSystem;
use vxl_phys_core::interop::MediumField;
use vxl_phys_core::Vec3;

/// 表面驱动参数。
#[derive(Clone, Copy, Debug)]
pub struct SurfaceDrag {
    /// 阻力系数（生产路径取自气动域 `AeroConfig::drag_coefficient`）。
    pub cd: f32,
    /// **自由表面阈值**：`ρᵢ < surface_ratio·ρ0` 的粒子按自由表面驱动（见模块头的实测依据）。
    pub surface_ratio: f32,
}

impl Default for SurfaceDrag {
    fn default() -> Self {
        Self {
            cd: 1.0,
            surface_ratio: 0.9,
        }
    }
}

/// 逐粒表面驱动（返回值 = 流体侧净注入动量，N·s）。
pub fn surface_drag(
    sys: &mut FluidSystem,
    medium: &mut dyn MediumField,
    cfg: SurfaceDrag,
    dt: f32,
) -> Vec3 {
    let n = sys.n_fluid;
    if cfg.cd <= 0.0 || dt <= 0.0 || n == 0 {
        return Vec3::ZERO;
    }
    let area = sys.particle_spacing() * sys.particle_spacing();
    let surface = cfg.surface_ratio * sys.cfg.rest_density;
    let mut injected = Vec3::ZERO;
    for i in 0..n {
        // 自由表面 = **密度亏超过阈值**；`ρ ≤ 0`（含 NaN）= 还没算过密度 ⇒ 不驱动。
        let rho = sys.dens[i];
        if rho.is_nan() || rho <= 0.0 || rho >= surface {
            continue;
        }
        let m = sys.pmass[i];
        if m.is_nan() || m <= 0.0 {
            continue;
        }
        let s = medium.sample(sys.pos[i]);
        if s.density <= 0.0 || s.occupied <= 0.0 {
            continue;
        }
        let u = s.velocity - sys.vel[i];
        let sp = u.length();
        if sp <= 0.0 {
            continue;
        }
        let dp = u * (0.5 * s.density * cfg.cd * area * sp * dt);
        sys.vel[i] += dp * (1.0 / m);
        injected += dp;
        medium.deposit(sys.pos[i], -dp, 0.0, 0.0);
    }
    injected
}
