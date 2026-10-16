//! **声学域的数据类型**（A1–A5 的公共面）：几何、衍射棱、听者、档、路径、统计。
//!
//! 从 crate 根拆出来（`god_gate` 的 800 行硬线只差两行）：这里只放**进出本层的数据**；
//! 归约入口留在 crate 根，产出族在 `paths` 模块。类型定义逐字未改。

use crate::BANDS;
use crate::{AcousticMedium, Portal};
use vxl_phys::Vec3;

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
