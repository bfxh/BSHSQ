//! world_struct：从 lib.rs 按域拆出（纯搬移，语义未改）。**域侧扩展 trait** 在 `ext/`
//! （11 个；显式再导出到 crate 根——glob-gate 禁 `pub use …::*`）。
use super::*;
pub mod ext;
pub use ext::{
    AeroExt, BodyBuildExt, DestructionOpsExt, DiagExt, FieldExt, FluidExt, HullExt, ImpactExt,
    MeshExt, NarrowTierExt, ProviderExt,
};
pub struct World {
    pub config: PhysConfig,
    pub bodies: BodySet,
    pub terrain: TerrainSet,
    /// 宽相（§2.3 主路径 = 增量 BVH；可用 `with_broadphase` 换网格等实现）。
    pub broad: Box<dyn BroadPhase>,
    pub narrow: crate::world_step::narrow_tier::NarrowSlot, // 宿主窄相 + 可选卡上档（§17.9）
    pub solver: ImpulseSolver,
    /// **关节约束族**（§2.5）：接触解算之后整帧求解一遍。
    pub joints: JointSet,
    pub fields: FieldRegistry,
    /// 任务调度（§6 依赖注入：SerialJobSystem / ScopedPool / 自定义实现）。
    pub jobs: Box<dyn JobSystem>,
    /// 帧级相位暂存（§0.1 #10）。
    pub arenas: FrameArenas,
    pub tick: u64,
    pub(crate) pairs: Vec<(u32, u32)>,
    pub(crate) manifolds: Vec<Manifold>,
    pub(crate) ccd_manifolds: Vec<Manifold>,
    /// **解算前的冲击记录**（破坏管线消费）：接触生成后、求解之前快照。
    pub(crate) impacts: Vec<ImpactRecord>,
    pub(crate) hf_bounds: Vec<Aabb>,
    /// 外部碰撞提供者集合与其 AABB。
    pub(crate) providers: Providers,
    pub(crate) provider_bounds: Vec<Aabb>,
    /// 已注册流体系统（第三槽 = 可选的**卡上步进后端**，见 §13.7）。
    pub(crate) fluids: Vec<crate::world_step::fluid_stepper::FluidSlot>,
    /// **2b 边界粒子那一族的成组状态**（开关 / 暂存 / 覆盖集）。
    pub(crate) fluid_boundary: FluidBoundary,
    /// **软体域**（绳索 + 布片；成组结构是为 god 门成员棘轮腾位）。
    pub(crate) soft: SoftDomain,
    /// **气动域**（T4）：`None` = 未开启 ⇒ `aero_pass` 首行短路 ⇒ 默认档逐位不变。
    pub(crate) aero: Option<vxl_phys_aero::AeroState>,
    pub(crate) timings: PhaseTimings,
}
