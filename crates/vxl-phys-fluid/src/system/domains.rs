//! **流体的域**（`system.rs` 的子模块）：把 `FluidSystem` 原来那 26 个字段
//! 按**职责域**打包成具名结构体。
//!
//! ## 为什么是这一刀（god 门「字段轴」债务的设计层处置）
//!
//! 登记的判据是：**「新加一个域不必再往本类型加字段」**。原来 `FluidSystem` 的
//! 字段是**一堆平行数组**（`pos/vel/dens/press/acc/xsph/pmass/bforce` +
//! `boundaries/spans/breact/lattice_cache/contacts` + 9 个标量），门眼里的
//! 「字段数」数的是**平行数组的条数**——于是**每加一个域就要往同一个类型里塞
//! 一条（乃至几条）数组**，字段数只涨不跌。
//!
//! 打包之后：
//!
//! - **一个域 = 一个具名结构体**；域的清单装在 [`Domains`] 里，`FluidSystem`
//!   只留 [`Domains`] 的**一个字段位**。⇒ **新加一个域 = 给 `Domains` 加一个字段，
//!   `FluidSystem` 的字段数不动**（这正是登记判据的字面形态）。
//! - **域内成员数不再计入 `FluidSystem` 的字段轴**（门按"类型名"归集：`FluidSystem`
//!   只数自己那几行；`KernelParams` 是另一个类型，有自己的 24 成员上限）。
//! - **打包是语义性的，不是数字游戏**：每个结构体回答一个独立的问题
//!   （"核有多宽" / "粒子现在是什么状态" / "边界这一侧记了什么"），
//!   且**内聚**：核参数永远不与粒子数组混在一个结构体里被单独改写。
//!
//! ## 四个域的边界（为什么这么分）
//!
//! ① [`KernelParams`]：**核与物理常数**（`h/h2/k6/ks/w0/b_tait/mass/spacing`）。
//!    这八个数是**从 `FluidConfig` 派生的一次性标定结果**（`assemble` 里算好，
//!    之后只读）——它们回答"这个流体的核有多宽、单粒多重"，与具体粒子无关
//!    ⇒ 换一套核参数 = 换**一个**结构体，而不是改 `FluidSystem` 的八个字段。
//!
//! ② [`ParticleState`]：**粒子的 SoA 状态**（`pos/vel/dens/press/acc/xsph/pmass/bforce`）。
//!    八个数组**同索引、同长度**（`n_fluid` 之前是流体、之后是 2b 边界粒子）
//!    ⇒ 它们是"一列粒子的八个属性"，本就该同生共死。打包后，
//!    "加一个粒子的属性" = 往 `ParticleState` 加一个数组
//!    （该类型的成员上限 24 内），而 `FluidSystem` 的字段数**不动**。
//!
//! ③ [`FluidBoundary`]：**边界（2b）这一域的账**（`skin/boundaries/spans/breact/
//!    lattice_cache/contacts`）：接触带宽度、提供者 id 列表、每体的段表、逐体反作用、
//!    形状→晶格缓存、接触缓冲。它们**只服务于边界耦合**，与流体粒子状态无关
//!    ⇒ 单独成域 ⇒ "再加一种边界口径"只动这个结构体。
//!
//! ④ **邻域网格**：`UniformGrid` 自已是具名类型（`grid.rs`），域位在
//!    [`Domains::grid`]；网格的第二个缓冲（若有）会进 `UniformGrid` 自己，不进
//!    `FluidSystem`。
//!
//! ## 不变式（打包不许改语义）
//!
//! - `ParticleState::bforce.len() == ParticleState::pos.len()`
//!   （`assemble` 给流体段；`set_boundary_particles` 随 `pos` 一起 resize；
//!   `truncate_to_fluid` 一起截断）。
//! - `FluidSystem::n_fluid` = 流体粒子数（**前缀**长度）⇒ 索引 ≥ `n_fluid` 的是边界粒子。
//! - 字段的**读出顺序与求和序**不变 ⇒ 数值行为逐位不变。
//!
//! **零 glob 导入**（glob-gate：新文件禁 `use …::*`）⇒ 全部显式列名。
use crate::boundary::BoundaryLattice;
use crate::config::FluidConfig;
use crate::grid::UniformGrid;
use vxl_phys_core::interop::InteropContact;
use vxl_phys_core::{Shape, Vec3};

/// **域的清单**（`FluidSystem` 的数据侧）：四个域各占一个字段位。
///
/// **新加一个域 = 给本结构体加一个字段**（而不是给 `FluidSystem` 加字段）——
/// 这就是 god 门登记的判据的字面形态。
pub(crate) struct Domains {
    /// 域①：核与物理常数。
    pub kern: KernelParams,
    /// 域②：粒子的 SoA 状态。
    pub ps: ParticleState,
    /// 域③：边界（2b）这一域的账。
    pub bnd: FluidBoundary,
    /// 域④：邻域网格（`UniformGrid` 自是具名类型）。
    pub grid: UniformGrid,
}

/// **核与物理常数**（域①）：从 `FluidConfig` 一次性派生的标定结果，之后只读。
///
/// 回答的问题是"**这个流体的核有多宽、单粒多重**"——与具体粒子无关，
/// 所以它与粒子数组**不同生共死**（换核参数不动粒子状态）。
#[derive(Clone, Copy, Debug)]
pub(crate) struct KernelParams {
    /// 核半径 h。
    pub h: f32,
    /// h²（`h*h`；单独存一份，**不重算**）。
    pub h2: f32,
    /// poly6 系数 315/(64πh⁹)。
    pub k6: f32,
    /// spiky 梯度幅系数 45/(πh⁶)。
    pub ks: f32,
    /// W(0) = poly6 自身项。
    pub w0: f32,
    /// Tait 刚度 B = c²ρ0/γ。
    pub b_tait: f32,
    /// 单粒质量（晶格标定：m = ρ0 / Σ_lattice W，见 `KernelParams::calibrate`）。
    pub mass: f32,
    /// 晶格间距（`new` 给定；边界粒子的采样间距与体积标定同源于它）。
    pub spacing: f32,
}

impl KernelParams {
    /// 一次标定（`assemble` 调用）：从 `cfg` + 晶格间距派生出八个核常数。
    ///
    /// **与旧口径逐位一致**：算式与顺序照抄旧 `assemble`（`h2 = h*h`、
    /// `k6 = 315/(64πh⁹)`、`ks = 45/(πh⁶)`、`w0 = k6*h2³`、
    /// `b_tait = c²ρ0/γ`、`mass = ρ0/Σ_lattice W`）。
    pub(crate) fn calibrate(cfg: &FluidConfig, spacing: f32) -> Self {
        let h = cfg.smoothing_radius.max(1e-5);
        let h2 = h * h;
        let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
        let ks = 45.0 / (std::f32::consts::PI * h.powi(6));
        let w0 = k6 * h2 * h2 * h2; // (h²)³
        let b_tait = cfg.sound_speed * cfg.sound_speed * cfg.rest_density / cfg.gamma_tait;
        // 质量标定：对晶格求 Σ W（间距 spacing 的无限晶格截断到核半径内）。
        let mass = cfg.rest_density / lattice_w_sum(h, h2, k6, spacing);
        Self {
            h,
            h2,
            k6,
            ks,
            w0,
            b_tait,
            mass,
            spacing,
        }
    }
}

/// **晶格核质量标定**（`KernelParams::calibrate` 的内核）：间距 `spacing` 的晶格按
/// poly6 截断求和 `Σ W`（含自身项）。
///
/// **为什么搬到这里**：它唯一的消费者是**核标定**（`mass = ρ0/ΣW`），原先住在
/// `fluid_access/lattice.rs`（"访问"域）里只是因为 `assemble` 在那儿——现在标定
/// 住在核域旁边，依赖方向与名字都对上了。
fn lattice_w_sum(h: f32, h2: f32, k6: f32, spacing: f32) -> f32 {
    let mut wsum = 0.0f32;
    let side = (h / spacing.max(1e-6)).ceil() as i32;
    for i in -side..=side {
        for j in -side..=side {
            for k in -side..=side {
                let d = Vec3::new(i as f32 * spacing, j as f32 * spacing, k as f32 * spacing);
                let r2 = d.length_squared();
                if r2 <= h2 {
                    let t = h2 - r2;
                    wsum += k6 * t * t * t;
                }
            }
        }
    }
    wsum
}

/// **粒子的 SoA 状态**（域②）：八个**同索引、同长度**的平行数组。
///
/// 不变式：八个数组长度恒等（`pos.len() == vel.len() == dens.len() == press.len()
/// == acc.len() == xsph.len() == pmass.len() == bforce.len()`）。
/// 索引 `0..n_fluid` 是流体粒子，`n_fluid..` 是 2b 边界粒子（**同数组**⇒
/// 既有核/式一字不改地作用于边界粒子）。
#[derive(Default)]
pub(crate) struct ParticleState {
    /// 位置（**全部**粒子：流体前缀 + 边界后缀）。
    pub pos: Vec<Vec3>,
    /// 速度（边界粒子的速度 = 体面速度 `v + ω×r`，运动学冻结）。
    pub vel: Vec<Vec3>,
    /// 密度（ρ）。
    pub dens: Vec<f32>,
    /// 压力（Tait EOS 的 p；FLIP 档 = 网格单元压力）。
    pub press: Vec<f32>,
    /// 加速度（每子步由力相位写）。
    pub acc: Vec<Vec3>,
    /// XSPH 速度平滑增量（每子步由力相位写）。
    pub xsph: Vec<Vec3>,
    /// 逐粒质量：流体 = `mass`，边界 = `ρ0·V_b`。流体段取同一个 f32 ⇒ 无边界时
    /// 与旧的常量乘法逐位等价。
    pub pmass: Vec<f32>,
    /// 每边界粒子的受力累加（每子步清零；`force_pass` 里借出以便写入）。
    pub bforce: Vec<Vec3>,
}

impl ParticleState {
    /// 建一份空状态（`assemble` 用）：各数组按 `pos.len()` 预分配，长度恒等。
    pub(crate) fn new(pos: Vec<Vec3>, mass: f32) -> Self {
        let n = pos.len();
        Self {
            pos,
            vel: vec![Vec3::ZERO; n],
            dens: vec![0.0; n],
            press: vec![0.0; n],
            acc: vec![Vec3::ZERO; n],
            xsph: vec![Vec3::ZERO; n],
            pmass: vec![mass; n],
            bforce: vec![Vec3::ZERO; n],
        }
    }

    /// **追加一个体的两层边界粒子**（2b）：位置 = 体面采样点经世界变换；速度 = 体面速度
    /// `v + ω×r`（`r` 自体原点量起）；质量 = `ρ0·V_b`；密度/压力/加速度/XSPH 归零。
    ///
    /// **为什么是 `ParticleState` 的方法而不是 `FluidSystem` 的**：这**只动粒子数组**
    /// （域②），不碰边界账（域③）⇒ 归到粒子域里最贴切；而且 `FluidSystem` 的方法数
    /// 是 god 门的**已登记债务**（只准减）⇒ 新能力不占它的方法位。
    /// **按 `keep` 取前缀、尾巴（边界粒子）原样接上**（carve 用）。
    ///
    /// 8 条平行数组必须**同序** ⇒ 一次改完，不给「只改了 5 条」留机会。
    /// 住在 `ParticleState` 而不是 `carve.rs` 里 ⇒ 这 8 条数组的**同步语义属于状态自己的事**。
    pub(crate) fn keep_prefix(&mut self, keep: &[usize], n: usize) {
        fn pick<T: Copy>(v: &[T], keep: &[usize], n: usize) -> Vec<T> {
            keep.iter()
                .map(|&i| v[i])
                .chain(v[n..].iter().copied())
                .collect()
        }
        self.pos = pick(&self.pos, keep, n);
        self.vel = pick(&self.vel, keep, n);
        self.acc = pick(&self.acc, keep, n);
        self.xsph = pick(&self.xsph, keep, n);
        self.bforce = pick(&self.bforce, keep, n);
        self.dens = pick(&self.dens, keep, n);
        self.press = pick(&self.press, keep, n);
        self.pmass = pick(&self.pmass, keep, n);
    }

    pub(crate) fn append_boundary(
        &mut self,
        rest_density: f32,
        lat: &crate::boundary::BoundaryLattice,
        pose: crate::types::BodyPose,
    ) {
        for &lp in &lat.pts {
            let wp = pose.pos + pose.rot.rotate_vec3(lp);
            self.pos.push(wp);
            // 体面速度：刚体速度场 v + ω×r（r 自体原点量起）。
            self.vel
                .push(pose.linvel + pose.angvel.cross(wp - pose.pos));
            self.pmass.push(rest_density * lat.volume);
            self.dens.push(0.0);
            self.press.push(0.0);
            self.acc.push(Vec3::ZERO);
            self.xsph.push(Vec3::ZERO);
        }
    }
}

/// **边界（2b）这一域的账**（域③）：只服务于边界耦合，与流体粒子状态无关。
///
/// 回答的问题是"**边界这一侧记了什么**"：接触带多宽（`skin`）、哪些体
/// （`boundaries`）、每体占哪段粒子（`spans`）、每体累计的反作用（`breact`）、
/// 形状→两层晶格的缓存（`lattice_cache`）、本子步的接触缓冲（`contacts`）。
#[derive(Default)]
pub(crate) struct FluidBoundary {
    /// 边界接触带（粒子中心距表面 < skin 触发投影）。
    pub skin: f32,
    /// 边界提供者 id（统一 id 空间；静态固体）。
    pub boundaries: Vec<u32>,
    /// 每体边界段 `(体 id, 体原点, start, end)`（`start..end` = 全局粒子索引区间）。
    pub spans: Vec<(u32, Vec3, u32, u32)>,
    /// 反作用输出：每体 `(体 id, 力, 力矩)`；**tick 级累加器**（step 清零→逐子步累加→÷子步数，C2）。
    pub breact: Vec<(u32, Vec3, Vec3)>,
    /// 形状 → 局部两层采样缓存（形状集小 ⇒ 线性查找；免逐 tick 重建）。
    pub lattice_cache: Vec<(Shape, BoundaryLattice)>,
    /// 接触缓冲（复用，免每粒分配）。
    pub contacts: Vec<InteropContact>,
}
