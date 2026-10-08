//! fluid_access：从 lib.rs 按域拆出（纯搬移，语义未改）。**公开**（`pub mod`）是为了让
//! `from_positions`（扫描点云建流体）可被外部消费方调用 —— 私有模块里的 `pub fn` 判 dead_code。
use super::*;

// 晶格初始化原语：单开文件让本文件**净缩**（`new` 从这里拿回 4 行写法的两行声明）。
#[path = "fluid_access/lattice.rs"]
mod lattice;
// **状态桥**（`StateBridge` 实现）搬到子模块：本文件受 god 门 file_lines 棘轮（只准减），
// 而"位置桥 → 状态桥"这次要加两个方法（速度/逐粒质量）⇒ 净账靠外迁保住。
#[path = "fluid_access/state.rs"]
mod state;
// **液面表面驱动**（风/水流 → 自由表面粒子；`ROUTE §4` 的「风 × 液」那一格）。
#[path = "fluid_access/wind.rs"]
pub mod wind;
use lattice::{lattice_positions, lattice_w_sum};

/// **按给定位置建流体系统**（`ROUTE.md` §4「扫描场景起步」的落点）：点云/扫描件先经
/// `StateBridge::import_positions` 或直接给位置，再由此建系统 —— 质量仍按**静止晶格**标定
/// （`ρ0 / Σ_lattice W`），所以"扫描点云当初始粒子"与"晶格块"用的是**同一套质量口径**。
///
/// **自由函数而非方法**：`FluidSystem` 的方法数是 god 门的**已登记债务**（只准减），
/// 新入口不占方法位（同 `for_each_neighbor` / `lattice_positions` 先例）。
pub fn from_positions(cfg: FluidConfig, pos: Vec<Vec3>, spacing: f32) -> FluidSystem {
    assemble(cfg, pos, spacing)
}

/// 组装本体（`new` 的晶格路径与 `from_positions` 的点云路径**共用**一条：核常数 + 质量标定 +
/// 逐段缓冲初始化）。抽出来的另一个理由：`new` 是本文件最长函数，god 门"合法交换"要求它严格下降。
fn assemble(cfg: FluidConfig, pos: Vec<Vec3>, spacing: f32) -> FluidSystem {
    let h = cfg.smoothing_radius.max(1e-5);
    let h2 = h * h;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
    let w0 = k6 * h2 * h2 * h2; // (h²)³
    let b_tait = cfg.sound_speed * cfg.sound_speed * cfg.rest_density / cfg.gamma_tait;
    let n = pos.len();
    // 质量标定：对晶格求 Σ W（间距 spacing 的无限晶格截断到核半径内）。
    let mass = cfg.rest_density / lattice_w_sum(h, h2, k6, spacing);
    FluidSystem {
        // 接触带 0.15h：粒子静置在 sdf = skin 处，带越薄壁邻密度亏越小
        // （镜像鬼影对贴壁层全额补回固体侧缺失的核质量）。带内最深穿透
        // 仍被推出（半空间无「另一侧」，薄带不引入穿隧）。
        skin: 0.15 * h,
        h,
        h2,
        k6,
        ks,
        w0,
        b_tait,
        mass,
        boundaries: Vec::new(),
        vel: vec![Vec3::ZERO; n],
        dens: vec![0.0; n],
        press: vec![0.0; n],
        acc: vec![Vec3::ZERO; n],
        xsph: vec![Vec3::ZERO; n],
        grid: UniformGrid::default(),
        contacts: Vec::new(),
        spacing,
        n_fluid: n,
        pmass: vec![mass; n],
        spans: Vec::new(),
        breact: Vec::new(),
        // 不变式：`bforce.len() == pos.len()`（`assemble` 给流体段；`set_boundary_particles` 随 `pos` 一起 resize；`truncate_to_fluid` 一起截断）。
        bforce: vec![Vec3::ZERO; n],
        lattice_cache: Vec::new(),
        phase_us: [0; 5],
        pos,
        cfg,
    }
}

impl FluidSystem {
    /// 晶格块初始化：粒子位于 `origin + (i + ½)·spacing`（三轴 `dims` 个），
    /// 质量按「静止晶格密度 = ρ0」反解：m = ρ0 / Σ_lattice W（含自身项）。
    pub fn new(cfg: FluidConfig, origin: Vec3, dims: [usize; 3], spacing: f32) -> Self {
        // 执行族：`FluidConfig::family` 目前未接线 ⇒ **显式拒绝** GPU 族（不静默跑 CPU SPH）。
        assert!(
            cfg.family == FluidFamily::CpuSph,
            "FluidConfig::family 只支持 CpuSph；GPU 族请走 World::set_fluid_stepper"
        );
        assemble(cfg, lattice_positions(origin, dims, spacing), spacing)
    }

    /// 流体粒子数（**不含**边界粒子）。
    ///
    /// 语义同旧 `len()`：渲染/导出/测试读到的永远是流体粒子，
    /// 边界粒子是 2b 的内部表示（`boundary_count()` 单独报）。
    pub fn len(&self) -> usize {
        self.n_fluid
    }

    pub fn is_empty(&self) -> bool {
        self.n_fluid == 0
    }

    pub fn config(&self) -> &FluidConfig {
        &self.cfg
    }

    /// 流体晶格间距（边界粒子的采样间距与体积标定同源）。
    pub fn particle_spacing(&self) -> f32 {
        self.spacing
    }

    /// 当前边界粒子数（2b；0 = 纯流体）。
    pub fn boundary_count(&self) -> usize {
        self.pos.len() - self.n_fluid
    }

    /// 当前边界粒子覆盖的体 id（按段序 = 生成序，确定性）。
    pub fn boundary_bodies(&self) -> impl Iterator<Item = u32> + '_ {
        self.spans.iter().map(|s| s.0)
    }

    /// **反作用**（`step` 后有效；**tick 平均** = dt_sub 时间加权，C2 口径）：每体 `(体 id, 力, 绕体原点的力矩)`。
    /// 体原点处的力与力矩都已是**力的量纲**（未乘 dt）；facade 按自己的子步施加。
    pub fn boundary_reactions(&self) -> &[(u32, Vec3, Vec3)] {
        &self.breact
    }

    /// 单粒质量（晶格标定结果；导出/审计用）。
    pub fn particle_mass(&self) -> f32 {
        self.mass
    }

    /// **邻域网格导出**（只读；GPU 后端与诊断用）。
    ///
    /// 语义：与 CPU 相位**同一张表**（计数排序：`start[c]..start[c+1]` = 格 c 的粒子、
    /// 格内按**索引升序**）⇒ GPU 侧按同一序枚举邻域即可与 CPU **同求和序**
    /// （这是 `docs/PLAN-gpu.md` §4 口径 A"能位级就位级"的前提）。
    pub fn neighbor_grid(&self) -> NeighborGrid<'_> {
        NeighborGrid {
            min: self.grid.min,
            inv: self.grid.inv,
            dims: (self.grid.nx, self.grid.ny, self.grid.nz),
            start: &self.grid.start,
            items: &self.grid.items,
        }
    }

    /// 流体粒子位置（前缀；不含边界粒子）。
    pub fn positions(&self) -> &[Vec3] {
        &self.pos[..self.n_fluid]
    }

    /// **全部粒子**（含 2b 边界粒子）的 `(pos, vel, pmass, n_fluid)`——**GPU 后端/耦合**用。
    /// 渲染与导出请走 `positions()`（只给流体前缀）。
    ///
    /// 语义（与 CPU 的 `substep` 逐条对应）：密度/压力/力跑**全部**粒子（邻域必须看得见边界粒子），
    /// 而**积分（含 XSPH/CFL）只跑 `0..n_fluid`**（边界粒子是运动学冻结的）。
    pub fn raw_particles(&self) -> (&[Vec3], &[Vec3], &[f32], usize) {
        (&self.pos, &self.vel, &self.pmass, self.n_fluid)
    }

    /// 流体粒子的**加速度**（前缀；不含重力以外的外力）——`Σ m·a` 与 `boundary_forces()` 一起
    /// 构成 2b 的"成对力等大反向"判据（见 `tests/boundary_reaction_balance.rs`）。
    pub fn accelerations(&self) -> &[Vec3] {
        &self.acc[..self.n_fluid]
    }

    /// **边界粒子受的反作用**（后缀；每子步由流体侧成对累加，见 `fluid_force.rs`）——
    /// 聚合到体上就是 `breact`（`boundary_reactions()`）。空 = 无边界粒子。
    pub fn boundary_forces(&self) -> &[Vec3] {
        &self.bforce[self.n_fluid..]
    }

    pub fn velocities(&self) -> &[Vec3] {
        &self.vel[..self.n_fluid]
    }

    pub fn densities(&self) -> &[f32] {
        &self.dens[..self.n_fluid]
    }

    pub fn pressures(&self) -> &[f32] {
        &self.press[..self.n_fluid]
    }

    /// 覆盖初速度（ dams break / 动量测试用；顺序 = 粒子索引序）。
    pub fn set_velocities(&mut self, vs: &[Vec3]) {
        let n = vs.len().min(self.n_fluid);
        self.vel[..n].copy_from_slice(&vs[..n]);
    }

    /// 边界提供者 id 列表（统一 id 空间）。
    pub fn boundaries(&self) -> &[u32] {
        &self.boundaries
    }

    pub fn set_boundaries(&mut self, ids: &[u32]) {
        self.boundaries = ids.to_vec();
    }
}
