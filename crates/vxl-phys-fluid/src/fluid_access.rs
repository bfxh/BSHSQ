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
use crate::system::domains::{Domains, FluidBoundary, KernelParams, ParticleState};
use lattice::lattice_positions;

/// **按给定位置建流体系统**（`ROUTE.md` §4「扫描场景起步」的落点）：点云/扫描件先经
/// `StateBridge::import_positions` 或直接给位置，再由此建系统 —— 质量仍按**静止晶格**标定
/// （`ρ0 / Σ_lattice W`），所以"扫描点云当初始粒子"与"晶格块"用的是**同一套质量口径**。
///
/// **自由函数而非方法**：`FluidSystem` 的方法数是 god 门的**已登记债务**（只准减），
/// 新入口不占方法位（同 `for_each_neighbor` / `lattice_positions` 先例）。
pub fn from_positions(cfg: FluidConfig, pos: Vec<Vec3>, spacing: f32) -> FluidSystem {
    assemble(cfg, pos, spacing)
}

/// 执行族守卫（**两条入口都受约束**）：本档只实现 CPU 三族（`CpuSph` WCSPH / `CpuPbf` PBF /
/// `CpuFlip` FLIP）⇒ 显式拒绝 GPU 族（不静默跑 CPU）。自由函数：内联进 `assemble` 会顶破 god 门的 fn 棘轮。
fn assert_cpu_family(cfg: &FluidConfig) {
    assert!(
        matches!(
            cfg.family,
            FluidFamily::CpuSph | FluidFamily::CpuPbf | FluidFamily::CpuFlip
        ),
        "FluidConfig::family 只支持 CpuSph / CpuPbf / CpuFlip；GPU 族请走 World::set_fluid_stepper"
    );
}

/// 组装本体（`new` 的晶格路径与 `from_positions` 的点云路径**共用**一条：核常数 + 质量标定 +
/// 逐段缓冲初始化）。抽出来的另一个理由：`new` 是本文件最长函数，god 门"合法交换"要求它严格下降。
///
/// **数据侧按域打包**（见 `system/domains.rs`）：四个域各建一份，交给 `Domains`。
fn assemble(cfg: FluidConfig, pos: Vec<Vec3>, spacing: f32) -> FluidSystem {
    assert_cpu_family(&cfg);
    // 核常数（域①）：从 `cfg` 一次性标定（含质量标定 `m = ρ0/Σ_lattice W`）。
    let kern = KernelParams::calibrate(&cfg, spacing);
    let mass = kern.mass;
    let n = pos.len();
    // 边界域（域③）：接触带 0.15h（见 `KernelParams::calibrate` 的注）。
    let bnd = FluidBoundary {
        skin: 0.15 * kern.h,
        ..FluidBoundary::default()
    };
    FluidSystem {
        dom: Domains {
            kern,
            ps: ParticleState::new(pos, mass),
            bnd,
            grid: UniformGrid::default(),
        },
        n_fluid: n,
        phase_us: [0; 5],
        cfg,
    }
}

impl FluidSystem {
    /// 晶格块初始化：粒子位于 `origin + (i + ½)·spacing`（三轴 `dims` 个），
    /// 质量按「静止晶格密度 = ρ0」反解：m = ρ0 / Σ_lattice W（含自身项）。
    pub fn new(cfg: FluidConfig, origin: Vec3, dims: [usize; 3], spacing: f32) -> Self {
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
        self.dom.kern.spacing
    }

    /// 当前边界粒子数（2b；0 = 纯流体）。
    pub fn boundary_count(&self) -> usize {
        self.dom.ps.pos.len() - self.n_fluid
    }

    /// 当前边界粒子覆盖的体 id（按段序 = 生成序，确定性）。
    pub fn boundary_bodies(&self) -> impl Iterator<Item = u32> + '_ {
        self.dom.bnd.spans.iter().map(|s| s.0)
    }

    /// **反作用**（`step` 后有效；**tick 平均** = dt_sub 时间加权，C2 口径）：每体 `(体 id, 力, 绕体原点的力矩)`。
    /// 体原点处的力与力矩都已是**力的量纲**（未乘 dt）；facade 按自己的子步施加。
    pub fn boundary_reactions(&self) -> &[(u32, Vec3, Vec3)] {
        &self.dom.bnd.breact
    }

    /// 单粒质量（晶格标定结果；导出/审计用）。
    pub fn particle_mass(&self) -> f32 {
        self.dom.kern.mass
    }

    /// **邻域网格导出**（只读；GPU 后端与诊断用）。
    ///
    /// 语义：与 CPU 相位**同一张表**（计数排序：`start[c]..start[c+1]` = 格 c 的粒子、
    /// 格内按**索引升序**）⇒ GPU 侧按同一序枚举邻域即可与 CPU **同求和序**
    /// （这是 `docs/PLAN-gpu.md` §4 口径 A"能位级就位级"的前提）。
    pub fn neighbor_grid(&self) -> NeighborGrid<'_> {
        let g = &self.dom.grid;
        NeighborGrid {
            min: g.min,
            inv: g.inv,
            dims: (g.nx, g.ny, g.nz),
            start: &g.start,
            items: &g.items,
        }
    }

    /// 流体粒子位置（前缀；不含边界粒子）。
    pub fn positions(&self) -> &[Vec3] {
        &self.dom.ps.pos[..self.n_fluid]
    }

    /// **全部粒子**（含 2b 边界粒子）的 `(pos, vel, pmass, n_fluid)`——**GPU 后端/耦合**用。
    /// 渲染与导出请走 `positions()`（只给流体前缀）。
    ///
    /// 语义（与 CPU 的 `substep` 逐条对应）：密度/压力/力跑**全部**粒子（邻域必须看得见边界粒子），
    /// 而**积分（含 XSPH/CFL）只跑 `0..n_fluid`**（边界粒子是运动学冻结的）。
    pub fn raw_particles(&self) -> (&[Vec3], &[Vec3], &[f32], usize) {
        (
            &self.dom.ps.pos,
            &self.dom.ps.vel,
            &self.dom.ps.pmass,
            self.n_fluid,
        )
    }

    /// 流体粒子的**加速度**（前缀；不含重力以外的外力）——`Σ m·a` 与 `boundary_forces()` 一起
    /// 构成 2b 的"成对力等大反向"判据（见 `tests/boundary_reaction_balance.rs`）。
    pub fn accelerations(&self) -> &[Vec3] {
        &self.dom.ps.acc[..self.n_fluid]
    }

    /// **边界粒子受的反作用**（后缀；每子步由流体侧成对累加，见 `fluid_force.rs`）——
    /// 聚合到体上就是 `breact`（`boundary_reactions()`）。空 = 无边界粒子。
    pub fn boundary_forces(&self) -> &[Vec3] {
        &self.dom.ps.bforce[self.n_fluid..]
    }

    pub fn velocities(&self) -> &[Vec3] {
        &self.dom.ps.vel[..self.n_fluid]
    }

    pub fn densities(&self) -> &[f32] {
        &self.dom.ps.dens[..self.n_fluid]
    }

    pub fn pressures(&self) -> &[f32] {
        &self.dom.ps.press[..self.n_fluid]
    }

    /// 覆盖初速度（ dams break / 动量测试用；顺序 = 粒子索引序）。
    pub fn set_velocities(&mut self, vs: &[Vec3]) {
        let n = vs.len().min(self.n_fluid);
        self.dom.ps.vel[..n].copy_from_slice(&vs[..n]);
    }

    /// 边界提供者 id 列表（统一 id 空间）。
    pub fn boundaries(&self) -> &[u32] {
        &self.dom.bnd.boundaries
    }

    pub fn set_boundaries(&mut self, ids: &[u32]) {
        self.dom.bnd.boundaries = ids.to_vec();
    }
}
