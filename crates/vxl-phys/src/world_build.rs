//! world_build：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

mod config_guard;

/// §6 调度注入：`threads ≤ 1` ⇒ 串行（默认，回归对照基准）；并行 = 作用域分块池（结果与
/// 串行 bit 级一致）。**自由函数**（不是 `impl World` 的方法）：`World` 的方法数已在
/// god 门债务账上（76/76 顶格），加一个方法就红——见 `god.gate.json` 的 `god_pairs`。
fn jobs_for(threads: usize) -> Box<dyn JobSystem> {
    if threads > 1 {
        Box::new(ScopedPool::new(threads))
    } else {
        Box::new(SerialJobSystem)
    }
}

impl World {
    pub fn new(config: PhysConfig) -> Self {
        let broad = Box::new(BvhBroadPhase::new(config.contact_skin));
        Self::with_broadphase(config, broad)
    }

    /// 自定义宽相注入（依赖注入，§1；默认增量 BVH）。
    pub fn with_broadphase(mut config: PhysConfig, broad: Box<dyn BroadPhase>) -> Self {
        config_guard::sanitize(&mut config);
        let mut fields = FieldRegistry::new();
        fields.add(Box::new(GravityField { g: config.gravity }));
        let (skin, mut bodies) = (config.contact_skin, BodySet::new());
        bodies.materials[0] = Material::new(config.friction, config.restitution); // 默认材质（槽 0；逐材质用 add_material 覆盖）
        let jobs = jobs_for(config.threads);
        Self {
            broad,
            narrow: DefaultNarrowPhase::new(skin).into(),
            solver: ImpulseSolver::new(skin),
            joints: JointSet::default(),
            config,
            bodies,
            terrain: TerrainSet::new(),
            fields,
            jobs,
            arenas: FrameArenas::new(),
            tick: 0,
            pairs: Vec::new(),
            manifolds: Vec::new(),
            ccd_manifolds: Vec::new(),
            impacts: Vec::new(),
            hf_bounds: Vec::new(),
            providers: Providers::default(),
            provider_bounds: Vec::new(),
            fluids: Vec::new(),
            fluid_boundary: FluidBoundary::default(),
            soft: SoftDomain::default(),
            aero: None,
            timings: PhaseTimings::default(),
        }
    }

    /// 累计分相耗时（自上次 `reset_timings` 起）。
    pub fn timings(&self) -> PhaseTimings {
        self.timings
    }

    pub fn reset_timings(&mut self) {
        self.timings = PhaseTimings::default();
    }
}
