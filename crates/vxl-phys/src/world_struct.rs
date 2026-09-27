//! world_struct：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub struct World {
    pub config: PhysConfig,
    pub bodies: BodySet,
    pub terrain: TerrainSet,
    /// 宽相（§2.3 主路径 = 增量 BVH；可用 `with_broadphase` 换网格等实现）。
    pub broad: Box<dyn BroadPhase>,
    pub narrow: crate::world_step::narrow_tier::NarrowSlot, // 宿主窄相 + 可选卡上档（§17.9）
    pub solver: ImpulseSolver,
    /// **关节约束族**（§2.5）：接触解算之后、位置积分之前整帧求解一遍；
    /// 空集时零成本（`solve` 首行短路）。
    pub joints: JointSet,
    pub fields: FieldRegistry,
    /// 任务调度（§6 依赖注入：SerialJobSystem / ScopedPool / 自定义实现）。
    pub jobs: Box<dyn JobSystem>,
    /// 帧级相位暂存（§0.1 #10；M0 接入 = 哈希规范化缓冲）。
    pub arenas: FrameArenas,
    pub tick: u64,
    pub(crate) pairs: Vec<(u32, u32)>,
    pub(crate) manifolds: Vec<Manifold>,
    pub(crate) ccd_manifolds: Vec<Manifold>,
    /// **解算前的冲击记录**（破坏管线消费）：接触生成后、求解之前快照。
    /// 不能读"解算后"的速度——子步/迭代会把法向接近速度解得接近 0，管线
    /// 因此看不见这次冲击（实测：2 子步下 12 m/s 炮弹撞墙不再挖洞）。
    pub(crate) impacts: Vec<ImpactRecord>,
    pub(crate) hf_bounds: Vec<Aabb>,
    /// 外部碰撞提供者集合（体素/网格…；ROUTE §2.1 兼容轴）与其 AABB。
    pub(crate) providers: Providers,
    pub(crate) provider_bounds: Vec<Aabb>,
    /// 已注册流体系统（液体域；边界 provider id 随行存档；第三槽 = 可选的**卡上步进后端**，见 §13.7）。
    pub(crate) fluids: Vec<crate::world_step::fluid_stepper::FluidSlot>,
    /// **2b 边界粒子那一族的成组状态**（开关 / 暂存 / 覆盖集；定义见 `world_soft.rs`）——
    /// 收成一个结构是为给软体域腾成员位（本结构受 god 门成员棘轮，只准减）。
    pub(crate) fluid_boundary: FluidBoundary,
    /// **软体域：绳索**（`World::add_rope` 注册；`rope_pass` 每 tick 推进一次，空集零成本短路）。
    pub(crate) ropes: Vec<vxl_phys_soft::Rope>,
    /// 绳索的**刚体代理暂存**（每 tick 重建，复用免分配；`rope_pass` 里 `mem::take` 借出后归还）。
    pub(crate) rope_proxies: Vec<vxl_phys_soft::RigidProxy>,
    pub(crate) timings: PhaseTimings,
}
