//! # vxl-phys-acoustics —— 声学传播（射线追踪音频）**A1 + A2 + A3 + A4 + A5**
//!
//! 设计与调研见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md`（技术全景 / 系统与论文清单 / A1–A5 分期）。
//! 本 crate 已落三期：
//!
//! - **A1**：静态场景的**直达 + 一次镜面反射（镜像源法）+ 遮挡**，材质表（吸声系数）；
//! - **A2**：**portal 穿孔传声**（开口当二级声源）+ **无 portal 的墙按透射系数泄漏**；
//! - **A3**：**介质体积场**（声速 + 沿程吸收，[`AcousticMedium`]）+ **频带**（[`BANDS`] =
//!   低/中/高：吸声、透射、介质吸收**逐带**，输出 [`SoundPath::bands`]）—— 每条路径的时延与
//!   逐带衰减按沿途介质积分（分层介质给[解析积分](AcousticMedium::segment_exact)，其它场退回
//!   固定步长中点法）。
//! - **A4**：**绕射**（[`Edge`] = 作者显式声明的衍射棱）—— 棱上取 **Fermat 点**（定次数二分，
//!   确定性），只在**阴影里**产出 `Diffracted` 路径，能量口径 `coeff_b · D(θ) · 介质衰减 / (d1·d2)`，
//!   `D(θ) = (1 + cosθ)/2`（掠射时与自由场连续）；**散射**（`scattering` 表）—— 镜面那条按
//!   `(1 - s_b)` 打折、剩下的按**漫反射二级声源**给出 `Scattered` 路径。**UTD 的 Fresnel 积分
//!   仍未做**（→ 见 A9a）。
//! - **A5**：**动态源/听者 + 预算与实时档** —— 声源本来就逐 tick 从 `World::impacts()` 来（会动），
//!   听者位置每次调用都由调用方给（会动）；本层补上**按优先级（能量）截断**与 [`AcousticTier`]
//!   （**确定性**的工作量上界，不是墙钟预算）；GPU 档落的是**基础件**（平铺布局 + 批处理入口 +
//!   等价契约），**设备内核仍未做**（见下面"GPU 档"一节）。
//! - **A6**：**离线音频渲染**（[`audio`] 模块）—— cue 流 → **能听的波形**：逐带分频
//!   （[`audio::SPLIT_HZ`]，完美重构）+ 按 `delay` 直叠（**幅度 = `sqrt(能量)`**）+
//!   最小 WAV 读写；单声道、不做 HRTF（双耳渲染仍是宿主的事）。判据 `tests/a6_audio.rs`。
//! - **A7**：**统计混响尾巴**（[`reverb`] 模块）—— 早期声仍由显式路径给，**晚段**用
//!   Sabine 逐带 `RT60 = 0.161·V/(S·ᾱ_b)` 补（体积/表面积/吸声全部从声学几何自己的表算），
//!   合成在 [`audio::mix_reverb_tail`]（**尾巴 = 干信号 ⊛ 稀疏衰减核**：velvet 噪声 × 逐带指数衰减 ⇒ 有音高的混响）。判据 `tests/a7_reverb.rs`。
//! - **A8**：**多次反射（二阶镜像展开）**（[`AcousticConfig::reflection_order`] = 2）——
//!   `声源 →(面 f₁) →(面 f₂) → 听者` 的展开构造 + 逐点内点/逐段遮挡检查（跳过链上的面），
//!   逐阶段预算、三角索引序 ⇒ 确定性。判据 `tests/a8_bounces.rs`。
//! - **A9a**：**刀刃真 Fresnel 绕射**（[`utd`]）—— 逐带**显式声明频率**（[`AcousticConfig::band_hz`]）
//!   时绕射「方向因子」换成真系数（Fresnel 积分 `C/S`、`ν = 2√(Δ/λ)`、`Δ` = Fermat 折线多余路程）；
//!   默认 `[0;3]`（不给绝对频率）⇒ 退回 A4 旧口径、**逐位不变**。判据 `tests/a9_utd.rs`。
//! - **A9b**：**一般楔角 UTD 修正**（[`Wedge`]）—— `Edge::wedge = Some(Ω)` 时按 KP 式算楔角系数，
//!   以刀刃修正因子 `fac` 乘到 A9a（`n = 2` ⇒ `1`，**逐位回到 A9a**；`n → 1` ⇒ `0`）。判据 `tests/a9b_wedge.rs`。
//! - **A10**：**体素空洞自动识别**（[`portal`]）—— 从**占据位图**（[`VoxelOccupancy`]）里用
//!   「腐蚀断颈 → 多源 BFS 分房 → 标签相接处取开口」认出**房间**与**开口**，产出 [`Portal`]
//!   （填 `regions`）。是开口那半的兜底路线（显式标仍是首选）。判据 `tests/a10_portals.rs`。
//!
//! ## 三条设计原则（文档 §4 的落实）
//!
//! 1. **声学几何 ≠ 碰撞几何**：[`AcousticGeometry`] 由调用方以**建模面**（顶点 + 三角 + 逐面
//!    吸声系数）交进来；本层**不读** `World` 的碰撞形状，也不要求被挡的墙存在于物理世界。
//!    （判据 `acoustic_geometry_is_independent_of_collision_shapes` 钉住这条。）
//! 2. **缝隙/开口是显式拓扑**（A2 已落）：开口由作者以 [`Portal`]（四边形 + 开启比例）交进来，
//!    本层**不看碰撞体**；传播时开口当二级声源产出 `Portal` 路径（开启比例线性衰减 ⇒ 缝宽→衰减
//!    单调），没登记 portal 的墙按逐面**透射系数**（[`AcousticGeometry::transmission`]）少量泄漏。
//! 3. **介质是体积参数场**（A3 已落）：[`AcousticGeometry::medium`] 由调用方给一个**只读**的
//!    [`AcousticMedium`]（`None` = 均匀空气 = A1/A2 行为）；吸收按 `exp(-α·len)` 沿途累乘、
//!    声速沿途积分；**折射（声线按 Snell 折转）**由介质自己给折线（[`AcousticMedium::refracted`]，
//!    分层介质已实现，含**全反射**），本层只做逐段遮挡检查 —— 产出 `Refracted` 路径。
//!
//! ## 输出格式（A3 起，**换代一次**）
//!
//! [`jsonl`] 每行多了 `"bands":[低,中,高]`（逐带能量）；`"gain"` 改成**中频带**
//! （[`SoundPath::gain`]）—— 单频带档（三带同参）下它与旧的标量口径**逐位相同**。旧基线在
//! `replay` 里会 FAIL 并指到第一个观测点（`record/replay` 的既有语义：格式换代要重新录）。
//!
//! ## 作者便利（文档 §8 的兜底路线）
//!
//! 开口（[`Portal`]）与衍射棱（[`Edge`]）都要**作者显式标**（凸凹/连通语义由人给最准）。衍射棱
//! 这一半另给了一条兜底：[`AcousticGeometry::detect_edges`] 能从网格里认出**边界棱与凸棱**
//! （前提：三角绕序**朝外**一致，见 [`EdgeDetect`]）。
//!
//! ## GPU 档（A5 的第二半）
//!
//! 几何与材质表本来就是**平铺切片**（`verts: &[Vec3]`、`tris: &[[u32; 3]]`、逐带表 `&[[f32; BANDS]]`）
//! ⇒ 上传给计算着色器不需要再打包。在此基础上加**批处理入口**
//! [`AcousticsExt::acoustic_cues_batch`]：对一批听者归约，每个听者一个**独立工作项**，
//! 输出里每个听者**连续**成段，预算是**逐听者**算的。
//!
//! ⚠️ **真正跑在设备上的计算内核仍未做**：它属于 `vxl-phys-gpu`（那里才有 wgpu 会话）；
//! 本层能先钉住的是**等价契约** —— 批处理必须与逐个标量调用**逐位相同**（判据
//! `tests/a5_gpu_batch.rs`）。GPU 后端只要通过这条契约，接进来就不会改数值。
//!
//! ## 三条契约（与 `vxl-phys-sense` 同款，CI 可判）
//!
//! 1. **零干扰**：[`AcousticsExt::acoustic_cues`] 只持有 `&self` ⇒ 开/关声学层，`state_hash`
//!    逐位不变；
//! 2. **确定性**：三角按**索引序**扫描、路径按（声源序 → 三角序）产出，无 HashMap 迭代序、
//!    无随机采样 ⇒ 同输入逐字相同的 cue 序列（可用 `vxl-phys-cli` 的 `record/replay` 对拍）；
//! 3. **预算**：每 tick 的**候选**（尝试的开口 / 衍射棱 / 散射面 / 反射面）都有上界；产出按
//!    **优先级（能量）**截断到 [`AcousticConfig::max_paths`]（稳定排序 ⇒ 平局仍按产出顺序，
//!    逐位可复现），两类丢弃都如实计入 [`AcousticStats::dropped_budget`]（同 `FragmentBudget`
//!    的口径）。档位预设见 [`AcousticTier`]。

#![forbid(unsafe_code)]

pub mod audio;
pub mod portal;
pub mod reverb;
pub mod utd;

mod auto_edge;
mod bounce;
mod diffract;
mod medium;
mod scatter;

pub use auto_edge::EdgeDetect;
pub use medium::{AcousticMedium, LayeredMedium, Medium, Slab};
pub use portal::{Portal, PortalDetect, VoxelOccupancy};
pub use utd::Wedge;

use bounce::push_second_order;
use diffract::push_diffraction;
use medium::seg_effect;
use scatter::push_scatter;
use vxl_phys::{Vec3, World};

/// **频带数**：低 / 中 / 高。本层**不给绝对频率** —— Hz 分组由宿主定，引擎只给三个标量
/// （判据只谈"高频更闷"这类**单调**性质，不绑具体 Hz）。
pub const BANDS: usize = 3;

/// **声学几何**（建模面：顶点 + 三角 + 逐面吸声 + 逐面透射 + 开口表 + 介质场）。
///
/// 两张材质表都是**逐频带**（`[BANDS]`）且是**外部表** —— 不往 `Material` 塞声学字段
/// （那是物理侧棘轮管的类型）。`absorption[i][b] ∈ [0,1]`：1 = 全吸（无声反射）、0 = 全反射。
pub struct AcousticGeometry<'a> {
    pub verts: &'a [Vec3],
    pub tris: &'a [[u32; 3]],
    pub absorption: &'a [[f32; BANDS]],
    /// 逐面**透射系数**（A2，逐频带）：直线被 `tris[i]` 挡住时按它泄漏，`0` = 完全不透、`1` = 全透。
    /// 与 `absorption` 同口径：**外部表**；短了 / 缺了按 `[0.0; BANDS]`（实墙）算。
    pub transmission: &'a [[f32; BANDS]],
    /// 逐面**散射系数**（A4 的散射那一半，逐频带）：`0` = 纯镜面（A1–A3 行为）、`1` = 全散射。
    /// 镜面那条按 `(1 - s_b)` 打折，剩下的能量由**漫反射二级声源**（`push_scatter`）接手。
    /// 与其它表同口径：**外部表**；短了 / 缺了按 `[0.0; BANDS]`（纯镜面）算。
    pub scattering: &'a [[f32; BANDS]],
    /// **显式开口拓扑**（A2）：作者标的开口；空 = 场景没有开口（A1 行为）。
    pub portals: &'a [Portal],
    /// **显式衍射棱拓扑**（A4）：作者标的棱；空 = 场景没有绕射（A1–A3 行为）。
    pub edges: &'a [Edge],
    /// **介质体积场**（A3）：`None` = 均匀空气（用 [`AcousticConfig::speed_of_sound`]、无吸收，
    /// 数值与 A1 / A2 逐位相同）。
    pub medium: Option<&'a dyn AcousticMedium>,
}

/// 一条**衍射棱**（A4）：两个端点 + 逐带**绕射系数**。
///
/// 与 [`Portal`] 同一条思路 —— **显式拓扑**：作者标哪条棱参与绕射（不从三角网格"自动认边"：
/// 那要先定凸凹与共面语义，留作兜底路线，见设计文档 §8）。棱上任取不了点：本层取
/// **Fermat 点**（让 `|P−声源| + |P−听者|` 最小的那个点，见 `fermat_point`）。
pub struct Edge {
    pub a: Vec3,
    pub b: Vec3,
    /// 逐带绕射系数（能量口径：`coeff_b · D(θ) · 介质衰减 / (d1·d2)`）；`0` = 该带不绕。
    pub coeff: [f32; BANDS],
    /// **一般楔角**（A9b）：`None` = 按刀口（`n = 2`）算 ⇒ **逐位回到 A9a**；`Some(Ω)` = 按楔角
    /// 内角 `Ω` 做系数修正（见 [`utd::Wedge`](crate::utd::Wedge)）。
    pub wedge: Option<crate::utd::Wedge>,
}

/// 听者（A1 只有一个点 + 半径：进入该球即可闻）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Listener {
    pub pos: Vec3,
    pub radius: f32,
}

/// 声学档（A1 的全部旋钮）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcousticConfig {
    /// 声速（m/s）；空气常温 ≈ 343。
    pub speed_of_sound: f32,
    /// **逐带中心频率**（Hz，A9 入口）：`> 0` = 该带显式声明了绝对频率 ⇒ 绕射走**真 Fresnel**
    /// 刀刃口径（[`utd`]）；`≤ 0` = 该带没有频率 ⇒ 退回 A4 旧口径。默认 `[0.0; BANDS]`
    /// （= crate 的「不给绝对频率」口径落实）⇒ **逐位不变**。
    pub band_hz: [f32; BANDS],
    /// 一次反射的最大候选面数（`0` = 只算直达；预算口径见模块头注）。
    pub max_reflections: u32,
    /// **反射阶数**：`1` = 只做一次镜面反射（A1–A7 的行为，默认）、`2` = 再加二阶镜像展开（A8）。
    ///
    /// ⚠️ 成本 ≈ `max_reflections^反射阶数` 条候选链（每条 3 段遮挡检查）⇒ **只建议离线档开**；
    /// 大于 2 的输入按 2 处理（三阶的候选是平方再乘一个因子，本层不做）。
    pub reflection_order: u32,
    /// 每个声源最多尝试几个开口（A2；`0` = 不算穿孔传声）。
    pub max_portals: u32,
    /// 每个声源最多尝试几条衍射棱（A4；`0` = 不算绕射）。
    pub max_edges: u32,
    /// 每个声源最多尝试几个散射面（A4 散射；`0` = 不算散射）。
    pub max_scatters: u32,
    /// 介质积分的**采样步长**（米；只在中点法回退时用到，下限 `1e-3`）。解析积分（分层介质）
    /// 不受它影响。
    pub medium_step: f32,
    /// **折射追迹的定次数**（只给数值解法当上界；无早退 ⇒ 逐位可复现）。
    pub refraction_iters: u32,
    /// 每 tick 最多产出多少条路径（硬预算）。
    pub max_paths: usize,
}

impl Default for AcousticConfig {
    fn default() -> Self {
        Self {
            speed_of_sound: 343.0,
            band_hz: [0.0; BANDS],
            max_reflections: 24,
            reflection_order: 1,
            max_portals: 8,
            max_edges: 8,
            max_scatters: 8,
            medium_step: 0.5,
            refraction_iters: 48,
            max_paths: 16,
        }
    }
}

/// **档位预设**（A5）：确定性的**工作量上界**组合 —— ⚠️ **不是墙钟预算**（本仓禁止时间相关的
/// 非确定性：同输入必须逐位同输出，跟机器快慢、负载、帧率无关）。
///
/// 实时档收的是四件事：候选面/开口/棱/散射面的上界、以及产出条数（`max_paths`）与介质采样步长。
/// 想做"帧率自适应"的话，请由**宿主**按自己的节奏选档（例如每 N 帧切一次 `Realtime`），
/// 而不是让本层去看时钟。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AcousticTier {
    /// 离线 / 烘焙：全量候选（= [`AcousticConfig::default`]）。
    Offline,
    /// 实时：每 tick 的候选与产出都收紧（游戏每帧都要出数）。
    Realtime,
    /// 极省：不算反射（`max_reflections = 0`），开口/棱/散射面各最多一个。
    Minimal,
}

impl AcousticTier {
    /// 本档的旋钮组合（**确定性**：与机器、负载、时间无关）。
    pub fn config(self) -> AcousticConfig {
        let base = AcousticConfig::default();
        match self {
            AcousticTier::Offline => base,
            AcousticTier::Realtime => AcousticConfig {
                max_reflections: 8,
                max_portals: 4,
                max_edges: 4,
                max_scatters: 4,
                medium_step: 1.0,
                max_paths: 8,
                ..base
            },
            AcousticTier::Minimal => AcousticConfig {
                max_reflections: 0,
                max_portals: 1,
                max_edges: 1,
                max_scatters: 1,
                medium_step: 2.0,
                max_paths: 4,
                ..base
            },
        }
    }
}

/// 路径种类（现在五档）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathKind {
    /// 直线可达。
    Direct,
    /// 直线被挡，但按挡它的面的**透射系数**泄漏过来（A2）。
    Transmitted,
    /// 经一个**开口**（portal）穿过来（A2）。
    Portal,
    /// 绕过一条**衍射棱**过来（A4；只在阴影里产出）。
    Diffracted,
    /// 由某个面的**漫反射**散射过来（A4 散射；面当二级声源）。
    Scattered,
    /// 经介质层界面**折射**过来（A3 第二半；折线由介质给，[`AcousticMedium::refracted`]）。
    Refracted,
    /// 一次镜面反射。
    Reflected,
}

/// 一条可闻路径（输出层的原子单位；上层把它卷成 SIR 再与干信号卷积）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoundPath {
    /// 声源序号（= `World::impacts()` 的记录序）。
    pub source: u32,
    /// 到达时延（秒）。
    pub delay: f32,
    /// **逐频带**能量增益（直达 = `1/d²`；反射再乘 `(1 - absorption[b])`；再乘介质衰减）。
    pub bands: [f32; BANDS],
    /// **宽带口径** = **中频带**（`bands[BANDS/2]`）—— **口径、不是物理量**：真宽带能量要按宿主的
    /// 频带权重求和，本层不替宿主定权重。单频带档（三带同参）下它与旧的标量口径**逐位相同**
    /// （`jsonl` 的 `"gain"` 与只想拿单值的宿主都读它）。
    pub gain: f32,
    /// 到达方向（单位；从听者指向来声）。
    pub dir: [f32; 3],
    pub kind: PathKind,
}

/// 一次归约的统计（"预算真的生效了没有"由它自证）。
///
/// ⚠️ **口径**：逐类计数是**产出**的路径（**截断之后**）—— 所以
/// [`paths`](AcousticStats::paths) 就是本次写进 `out` 的条数（批处理靠它切段）；
/// **候选**侧被丢多少看 [`dropped_budget`](AcousticStats::dropped_budget)。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AcousticStats {
    /// 声源数（= 本 tick 的冲击记录数）。
    pub sources: usize,
    /// 直达路径数。
    pub direct: usize,
    /// 透射泄漏路径数（A2：被挡、但墙不是完全不透）。
    pub leaked: usize,
    /// 穿孔路径数（A2）。
    pub portals: usize,
    /// 绕射路径数（A4）。
    pub diffracted: usize,
    /// 散射路径数（A4 散射）。
    pub scattered: usize,
    /// 折射路径数（A3 第二半）。
    pub refracted: usize,
    /// 反射路径数。
    pub reflected: usize,
    /// 因遮挡被丢的直达路径数。
    pub blocked: usize,
    /// 因超预算被丢的路径数。
    pub dropped_budget: usize,
}

impl AcousticStats {
    /// 本次归约**产出的路径条数**（五类路径之和）—— 批处理里用它切段。
    pub fn paths(&self) -> usize {
        self.direct
            + self.leaked
            + self.portals
            + self.diffracted
            + self.scattered
            + self.refracted
            + self.reflected
    }
}

/// **声学传播域的扩展 API**（`impl AcousticsExt for World`）——同感官域：**不往 `World` 加方法**。
pub trait AcousticsExt {
    /// 归约**本 tick**的可闻路径，**追加**到 `out`（返回 [`AcousticStats`]）。
    ///
    /// 声源取 `World::impacts()`（逐 tick 峰值冲击的接触点）——与破坏管线、感官域**同源**；
    /// 没有冲击的 tick 自然产出 0 条路径（事件驱动）。
    fn acoustic_cues(
        &self,
        geo: &AcousticGeometry<'_>,
        listener: &Listener,
        cfg: &AcousticConfig,
        out: &mut Vec<SoundPath>,
    ) -> AcousticStats;

    /// **批处理入口（GPU 档的基础件）**：对**一批听者**归约。
    ///
    /// 契约（判据 `tests/a5_gpu_batch.rs` 逐条钉住）：
    ///
    /// 1. **逐位等价**：每个听者在批里的结果与"单独调 [`acoustic_cues`](Self::acoustic_cues)"
    ///    完全相同（含顺序）⇒ GPU 后端只要通过这条，接进来不改数值；
    /// 2. **输出分段**：`out` 里每个听者的路径**连续**排在自己的段里，段长 = 对应
    ///    [`AcousticStats::paths`]；`stats_out` 与 `listeners` 一一对应（清空后重填）；
    /// 3. **逐听者预算**：`max_paths` **不跨听者共享**（每个听者各自最多 `max_paths` 条）。
    ///
    /// 默认实现就是"循环调 [`acoustic_cues`](Self::acoustic_cues)"—— 它天然满足前两条；
    /// GPU 后端可以覆盖它（平铺、并行），但必须仍然满足这三条。
    fn acoustic_cues_batch(
        &self,
        geo: &AcousticGeometry<'_>,
        listeners: &[Listener],
        cfg: &AcousticConfig,
        out: &mut Vec<SoundPath>,
        stats_out: &mut Vec<AcousticStats>,
    ) {
        stats_out.clear();
        for l in listeners {
            stats_out.push(self.acoustic_cues(geo, l, cfg, out));
        }
    }
}

impl AcousticsExt for World {
    fn acoustic_cues(
        &self,
        geo: &AcousticGeometry<'_>,
        listener: &Listener,
        cfg: &AcousticConfig,
        out: &mut Vec<SoundPath>,
    ) -> AcousticStats {
        let mut st = AcousticStats {
            sources: self.impacts().len(),
            ..AcousticStats::default()
        };
        let c = if cfg.speed_of_sound > 0.0 {
            cfg.speed_of_sound
        } else {
            343.0
        };
        // 本 tick 的**候选**先收在 `found`，最后统一按优先级截断（见下面 A5 那段）。
        let mut found: Vec<SoundPath> = Vec::new();
        // 声源按**记录序**、三角按**索引序**——确定性的两层（无排序、无哈希）。
        for (si, rec) in self.impacts().iter().enumerate() {
            let src = rec.point;
            let cx = Ctx {
                geo,
                listener,
                cfg,
                c,
                si: si as u32,
                src,
            };
            push_direct(&mut found, &mut st, &cx);
            push_refraction(&mut found, &mut st, &cx);
            push_portals(&mut found, &mut st, &cx);
            push_diffraction(&mut found, &mut st, &cx);
            push_reflections(&mut found, &mut st, &cx);
            // A8：二阶镜像展开（`reflection_order ≥ 2` 才做；默认 1 ⇒ 与 A1–A7 逐位相同）。
            if cfg.reflection_order >= 2 {
                push_second_order(&mut found, &mut st, &cx);
            }
            push_scatter(&mut found, &mut st, &cx);
        }
        // **A5：输出预算按优先级（能量）截断** —— 只留最响的 `room` 条。`sort_by` 是**稳定**排序
        // ⇒ 增益相同时仍按产出顺序（逐位可复现）；多出来的如实计入 `dropped_budget`。
        // 预算是**逐调用（逐听者）**算的：`out` 里已有的内容不吃本次的额度（批处理要逐听者独立）。
        let room = cfg.max_paths;
        if found.len() > room {
            found.sort_by(|a, b| b.gain.total_cmp(&a.gain));
            st.dropped_budget += found.len() - room;
            found.truncate(room);
        }
        // 逐类计数按**产出**重算（候选侧的丢弃已经记在 `dropped_budget` 里）
        st.direct = 0;
        st.leaked = 0;
        st.portals = 0;
        st.diffracted = 0;
        st.scattered = 0;
        st.refracted = 0;
        st.reflected = 0;
        for p in found.iter() {
            match p.kind {
                PathKind::Direct => st.direct += 1,
                PathKind::Transmitted => st.leaked += 1,
                PathKind::Portal => st.portals += 1,
                PathKind::Diffracted => st.diffracted += 1,
                PathKind::Scattered => st.scattered += 1,
                PathKind::Refracted => st.refracted += 1,
                PathKind::Reflected => st.reflected += 1,
            }
        }
        out.extend(found);
        st
    }
}

/// 一次归约的上下文（几何 + 听者 + 档 + 本次声源）——打包起来免得每个助手摊一长串形参
/// （`args-gate` 的 soft 阈值实测把 7 个形参也算违规）。
pub(crate) struct Ctx<'a> {
    pub(crate) geo: &'a AcousticGeometry<'a>,
    pub(crate) listener: &'a Listener,
    pub(crate) cfg: &'a AcousticConfig,
    pub(crate) c: f32,
    pub(crate) si: u32,
    pub(crate) src: Vec3,
}

/// 直达：连线被挡 ⇒ 若挡它的面**透射系数 > 0** 则按它泄漏成一条 `Transmitted`（A2：没有 portal
/// 的墙的那条路径），否则计入 `blocked` 丢掉。`leak == 1` 时就是纯直达（数值与 A1 逐位相同）。
fn push_direct(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let (src, c) = (cx.src, cx.c);
    let d = cx.listener.pos - src;
    let mut leak = [1.0f32; BANDS];
    if let Some(ti) = blocker(cx.geo, src, cx.listener.pos) {
        let tau = cx.geo.transmission.get(ti).copied().unwrap_or([0.0; BANDS]);
        if tau.iter().all(|t| *t <= 0.0) {
            st.blocked += 1;
            return;
        }
        for (l, t) in leak.iter_mut().zip(tau.iter()) {
            *l = t.clamp(0.0, 1.0);
        }
    }
    let dist = d.length().max(1e-3);
    // 听者半径只做**远场粗筛**（`radius` 的 1000 倍之外不产出；A3 起换成介质衰减与距离门）。
    if dist > cx.listener.radius.max(1e-3) * 1e3 {
        return;
    }
    // 约定：`dir` = **从听者指向来声**（空间化器要的就是入射方向，不是"声源→听者"）。
    let dir = ((src - cx.listener.pos) * (1.0 / dist)).normalize();
    // A3：沿途介质给的时延与逐带吸收（无介质场 ⇒ `len / c`、衰减因子全 1.0）。
    let (delay, atten) = seg_effect(cx.geo.medium, src, cx.listener.pos, c, cx.cfg.medium_step);
    let leaked = leak.iter().any(|t| *t < 1.0);
    let mut bands = [0.0f32; BANDS];
    for ((o, l), a) in bands.iter_mut().zip(leak.iter()).zip(atten.iter()) {
        *o = l * a / (dist * dist);
    }
    let kind = if leaked {
        PathKind::Transmitted
    } else {
        PathKind::Direct
    };
    out.push(SoundPath {
        source: cx.si,
        delay,
        bands,
        gain: mid_band(&bands),
        dir: [dir.x, dir.y, dir.z],
        kind,
    });
    if leaked {
        st.leaked += 1;
    } else {
        st.direct += 1;
    }
}

/// **宽带口径**：取 [`BANDS`] 的中频带 —— 单频带档（三带同参）下与旧的标量口径逐位相同。
pub(crate) fn mid_band(bands: &[f32; BANDS]) -> f32 {
    bands[BANDS / 2]
}

/// **A2：portal 穿孔传声** —— 开口当**二级声源**：`声源 → 开口中心 → 听者` 两段都不被挡就产出一条
/// `Portal` 路径，增益 = **开启比例** / (d1+d2)²（缝越窄增益越低 ⇒ 衰减随缝宽单调）。
/// 开启比例为 0 的开口不产出（等同实墙，剩下的泄漏走 [`push_direct`] 的透射那条）。
fn push_portals(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let mut tried = 0u32;
    for p in cx.geo.portals.iter() {
        if tried >= cx.cfg.max_portals {
            st.dropped_budget += 1;
            break;
        }
        let open = p.open.clamp(0.0, 1.0);
        if open <= 0.0 {
            continue;
        }
        tried += 1;
        let c = centroid(&p.quad);
        if occluded(cx.geo, cx.src, c) || occluded(cx.geo, c, cx.listener.pos) {
            continue;
        }
        let d1 = (c - cx.src).length().max(1e-3);
        let d2 = (cx.listener.pos - c).length().max(1e-3);
        let reach = d1 + d2;
        let dirv = (c - cx.listener.pos) * (1.0 / d2);
        // A3：两段各自的介质时延/逐带吸收（球面扩散仍用几何距离 `reach`）。
        let (t1, a1) = seg_effect(cx.geo.medium, cx.src, c, cx.c, cx.cfg.medium_step);
        let (t2, a2) = seg_effect(cx.geo.medium, c, cx.listener.pos, cx.c, cx.cfg.medium_step);
        let mut bands = [0.0f32; BANDS];
        for ((o, x), y) in bands.iter_mut().zip(a1.iter()).zip(a2.iter()) {
            *o = open * x * y / (reach * reach);
        }
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            bands,
            gain: mid_band(&bands),
            dir: [dirv.x, dirv.y, dirv.z],
            kind: PathKind::Portal,
        });
        st.portals += 1;
    }
}

/// 开口的中心（四个角的算术平均）——A2 把开口当点声源，取它的几何中心。
fn centroid(quad: &[Vec3; 4]) -> Vec3 {
    (quad[0] + quad[1] + quad[2] + quad[3]) * 0.25
}

/// **A3 第二半：折射** —— 声线在层界面上按 **Snell** 折转。折线由**介质自己**给
/// （[`AcousticMedium::refracted`]：只有它知道层界），本层只做**逐段遮挡检查**。
///
/// 球面扩散仍用**直线距离** `1/d²`（`d = |声源−听者|`）⇒ 介质均匀退化时它与直达那条**同值**
/// （而均匀时介质会直接返回 `None`，所以不会重复计能）。
fn push_refraction(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let Some(m) = cx.geo.medium else {
        return;
    };
    let l = cx.listener.pos;
    let Some((kinks, delay, atten)) = m.refracted(cx.src, l, cx.cfg.refraction_iters) else {
        return;
    };
    for w in kinks.windows(2) {
        if occluded(cx.geo, w[0], w[1]) {
            return; // 折线有一段被挡 ⇒ 整条不产出
        }
    }
    let Some(prev) = kinks.iter().rev().nth(1) else {
        return;
    };
    let dist = (l - cx.src).length().max(1e-3);
    let dirv = (l - *prev).normalize();
    let mut bands = [0.0f32; BANDS];
    for (b, a) in bands.iter_mut().zip(atten.iter()) {
        *b = a / (dist * dist);
    }
    out.push(SoundPath {
        source: cx.si,
        delay,
        bands,
        gain: mid_band(&bands),
        dir: [dirv.x, dirv.y, dirv.z],
        kind: PathKind::Refracted,
    });
    st.refracted += 1;
}

/// 一次镜面反射：**镜像源法**——镜像听者，与三角平面求交，命中点落在三角内且两段都不被挡 ⇒ 一条路径。
fn push_reflections(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let (src, c, geo, cfg) = (cx.src, cx.c, cx.geo, cx.cfg);
    let mut tried = 0u32;
    for (ti, tri) in geo.tris.iter().enumerate() {
        if tried >= cfg.max_reflections {
            break;
        }
        let Some((a, b, cc, absorb)) = tri_of(geo, ti, *tri) else {
            continue;
        };
        // A4 散射：镜面那条按 `(1 - s_b)` 打折（缺表 ⇒ 纯镜面，数值与 A1–A3 逐位相同）。
        let scatter = geo.scattering.get(ti).copied().unwrap_or([0.0; BANDS]);
        let n = (b - a).cross(cc - a);
        let ln = n.length();
        if ln <= 1e-9 {
            continue; // 退化面：不参与反射
        }
        tried += 1;
        let n = n * (1.0 / ln);
        // 镜像听者：`l' = l - 2·((l - a)·n)·n`
        let l = cx.listener.pos;
        let lp = l - n * (2.0 * (l - a).dot(n));
        let seg = lp - src;
        let denom = seg.dot(n);
        if denom.abs() <= 1e-9 {
            continue; // 与面平行：无镜面反射点
        }
        let t = (a - src).dot(n) / denom;
        if !(0.0..=1.0).contains(&t) {
            continue;
        }
        let hit = src + seg * t;
        if !inside_tri(a, b, cc, hit) {
            continue;
        }
        if occluded(geo, src, hit) || occluded(geo, hit, l) {
            continue;
        }
        let d1 = (hit - src).length().max(1e-3);
        let d2 = (l - hit).length().max(1e-3);
        let dirv = (hit - l) * (1.0 / d2);
        // A3：两段各自的介质时延/逐带吸收（吸声系数是面属性，逐带相乘）。
        let (t1, a1) = seg_effect(geo.medium, src, hit, c, cfg.medium_step);
        let (t2, a2) = seg_effect(geo.medium, hit, l, c, cfg.medium_step);
        let mut bands = [0.0f32; BANDS];
        for ((((o, r), x), y), sc) in bands
            .iter_mut()
            .zip(absorb.iter())
            .zip(a1.iter())
            .zip(a2.iter())
            .zip(scatter.iter())
        {
            let spec = 1.0 - sc.clamp(0.0, 1.0);
            *o = spec * (1.0 - r.clamp(0.0, 1.0)) * x * y / ((d1 + d2) * (d1 + d2));
        }
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            bands,
            gain: mid_band(&bands),
            dir: [dirv.x, dirv.y, dirv.z],
            kind: PathKind::Reflected,
        });
        st.reflected += 1;
    }
}

/// 取三角的三顶点与**逐带**吸声系数（索引越界 / 表短 ⇒ `None`，与 `TriMesh` 的"丢坏三角"同口径）。
pub(crate) fn tri_of(
    geo: &AcousticGeometry<'_>,
    ti: usize,
    tri: [u32; 3],
) -> Option<(Vec3, Vec3, Vec3, [f32; BANDS])> {
    let n = geo.verts.len();
    if tri.iter().any(|&k| k as usize >= n) {
        return None;
    }
    let absorb = *geo.absorption.get(ti)?;
    Some((
        geo.verts[tri[0] as usize],
        geo.verts[tri[1] as usize],
        geo.verts[tri[2] as usize],
        absorb,
    ))
}

/// 点是否在三角内（重心坐标，带 1e-5 的边界容差）。
fn inside_tri(a: Vec3, b: Vec3, c: Vec3, p: Vec3) -> bool {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let den = d00 * d11 - d01 * d01;
    if den.abs() <= 1e-12 {
        return false;
    }
    let u = (d11 * d20 - d01 * d21) / den;
    let v = (d00 * d21 - d01 * d20) / den;
    u >= -1e-5 && v >= -1e-5 && u + v <= 1.0 + 1e-5
}

/// 线段 `p→q` 撞上的**第一个**三角（Möller–Trumbore；端点共面不计）——`None` = 通视。
///
/// "第一个" = **索引序**（不是最近的那个）：只为确定性，A2 的透射系数取的就是它；判据里墙面
/// 不会与别的面重叠，所以"索引序"与"最近"在真实场景里一致。
pub(crate) fn blocker(geo: &AcousticGeometry<'_>, p: Vec3, q: Vec3) -> Option<usize> {
    blocker_skip(geo, p, q, (usize::MAX, usize::MAX))
}

/// 同 [`blocker`]，但可以**跳过至多两张面**（A8 的二阶链用：链上的面本来就在子段的端点上起落，
/// 不去看它们；跳过 `usize::MAX` 以外的值就等于没跳）。
pub(crate) fn blocker_skip(
    geo: &AcousticGeometry<'_>,
    p: Vec3,
    q: Vec3,
    skip: (usize, usize),
) -> Option<usize> {
    let dir = q - p;
    let len = dir.length();
    if len <= 1e-6 {
        return None;
    }
    let dir = dir * (1.0 / len);
    for (ti, tri) in geo.tris.iter().enumerate() {
        if ti == skip.0 || ti == skip.1 {
            continue;
        }
        let n = geo.verts.len();
        if tri.iter().any(|&k| k as usize >= n) {
            continue;
        }
        let (a, b, c) = (
            geo.verts[tri[0] as usize],
            geo.verts[tri[1] as usize],
            geo.verts[tri[2] as usize],
        );
        let e1 = b - a;
        let e2 = c - a;
        let pv = dir.cross(e2);
        let det = e1.dot(pv);
        if det.abs() <= 1e-9 {
            continue;
        }
        let inv = 1.0 / det;
        let tv = p - a;
        let u = tv.dot(pv) * inv;
        if !(-1e-6..=1.0 + 1e-6).contains(&u) {
            continue;
        }
        let qv = tv.cross(e1);
        let v = dir.dot(qv) * inv;
        if v < -1e-6 || u + v > 1.0 + 1e-6 {
            continue;
        }
        let t = e2.dot(qv) * inv;
        if t > 1e-4 && t < len - 1e-4 {
            return Some(ti);
        }
    }
    None
}

/// 线段是否被任何三角挡住（只看有没有，不看是谁）——A1 的旧口径，A2 起由 [`blocker`] 实现。
pub(crate) fn occluded(geo: &AcousticGeometry<'_>, p: Vec3, q: Vec3) -> bool {
    blocker(geo, p, q).is_some()
}

/// cue 流 → JSONL（一行一条；固定 6 位小数 ⇒ 可逐字对拍，接上 `vxl-phys-cli` 的 `record/replay`）。
pub fn jsonl(paths: &[SoundPath]) -> String {
    let mut s = String::new();
    for p in paths {
        s.push_str(&format!(
            "{{\"source\":{},\"kind\":\"{}\",\"delay\":{:.6},\"gain\":{:.6},\
             \"bands\":[{:.6},{:.6},{:.6}],\"dir\":[{:.6},{:.6},{:.6}]}}\n",
            p.source,
            match p.kind {
                PathKind::Direct => "direct",
                PathKind::Transmitted => "transmitted",
                PathKind::Portal => "portal",
                PathKind::Diffracted => "diffracted",
                PathKind::Scattered => "scattered",
                PathKind::Refracted => "refracted",
                PathKind::Reflected => "reflected",
            },
            p.delay,
            p.gain,
            p.bands[0],
            p.bands[1],
            p.bands[2],
            p.dir[0],
            p.dir[1],
            p.dir[2]
        ));
    }
    s
}
