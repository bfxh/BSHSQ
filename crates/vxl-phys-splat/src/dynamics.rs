//! **高斯粒子域的动力学档**（②物理代理的**门面接入面**，2026-10-08）。
//!
//! 三片零件（运动步 / 自场压力 / 自场黏性）此前各自是 crate 级 API；本文件给它们一个**统一的
//! 开关 + 固定顺序**，挂在 [`GaussianSplatField::dynamics`] 上 ⇒ 门面每子步对"开了档的场"跑一遍，
//! **不需要给 `World` 加方法或字段**（god 门债务只准减：那时只能走扩展 trait/外部注册表）。
//!
//! 顺序（固定 ⇒ 确定性）：**候选表重建 → 自场压力 → 自场黏性 → 粒子积分 + 世界碰撞**（力先算、再积分）。
//! 重建的必要性见 `step_dynamics` 里那段注释（上一步积分末尾把网格置脏）。
//! 默认 `None` = 纯提供者（隐式场/渲染桥），**零代际**（门面首行短路）。
use crate::particles::ParticleStep;
use crate::pressure::SelfPressure;
use crate::viscosity::SelfViscosity;
use crate::GaussianSplatField;
use vxl_phys_core::interop::ProviderColliders;

/// 高斯粒子域的动力学档（`None` = 关）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SplatDynamics {
    /// 运动步（重力 + 每步衰减 + 世界碰撞的投影参数）。
    pub particles: ParticleStep,
    /// 自场压力（`None` = 不做压力）。
    pub pressure: Option<SelfPressure>,
    /// 自场黏性（`None` = 不做黏性）。
    pub viscosity: Option<SelfViscosity>,
}

impl GaussianSplatField {
    /// 开关动力学档（`None` = 纯提供者）。返回旧值（回放/判据用）。
    pub fn set_dynamics(&mut self, cfg: Option<SplatDynamics>) -> Option<SplatDynamics> {
        std::mem::replace(&mut self.dynamics, cfg)
    }

    /// 当前动力学档（`None` = 关）。
    pub fn dynamics(&self) -> Option<SplatDynamics> {
        self.dynamics
    }
}

/// 跑一步动力学（压力 → 黏性 → 粒子积分 + 世界碰撞）；返回接触投影次数。
/// `providers`/`ids` 与 [`crate::particles::step_particles`] 同口径（**调用方须把本场自己排除在
/// `ids` 之外**：自己的隐式场不参与自己的碰撞，那是自场压力的活）。
pub fn step_dynamics(
    field: &mut GaussianSplatField,
    dt: f32,
    providers: &dyn ProviderColliders,
    ids: &[u32],
    cfg: SplatDynamics,
) -> usize {
    // **每步重建候选表**：上一步的 `step_particles` 末尾已把网格置脏（核中心动了）⇒ 不在这里
    // 重建的话，下面的压力/黏性永远走全扫，`rebuild_grid` 在世界路径上从不被消费。重建只裁
    // 候选、不改求和序 ⇒ 与全扫逐位一致；核数 < `grid_min_splats`（默认 64）时内部短路 ⇒ 小场零成本。
    field.rebuild_grid();
    if let Some(p) = cfg.pressure {
        crate::pressure::apply_self_pressure(field, dt, p);
    }
    if let Some(v) = cfg.viscosity {
        crate::viscosity::apply_self_viscosity(field, dt, v);
    }
    crate::particles::step_particles(field, dt, providers, ids, cfg.particles)
}
