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
mod paths;
mod scatter;
mod types;

pub use auto_edge::EdgeDetect;
pub use medium::{AcousticMedium, LayeredMedium, Medium, Slab};
pub use portal::{Portal, PortalDetect, VoxelOccupancy};
pub use utd::Wedge;

use bounce::push_second_order;
use diffract::push_diffraction;
use scatter::push_scatter;
use vxl_phys::{ImpactExt, World};
// 拆出去的两块在根上 re-export：子模块照旧按 crate 根路径引用，外部按 `crate::Type` 引用。
pub(crate) use paths::{
    blocker, blocker_skip, inside_tri, mid_band, occluded, push_direct, push_portals,
    push_reflections, push_refraction, tri_of, Ctx,
};
pub use types::{
    AcousticConfig, AcousticGeometry, AcousticStats, AcousticTier, Edge, Listener, PathKind,
    SoundPath,
};

/// **频带数**：低 / 中 / 高。本层**不给绝对频率** —— Hz 分组由宿主定，引擎只给三个标量
/// （判据只谈"高频更闷"这类**单调**性质，不绑具体 Hz）。
pub const BANDS: usize = 3;

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
