//! # vxl-phys-acoustics —— 声学传播（射线追踪音频）**A1 + A2**
//!
//! 设计与调研见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md`（技术全景 / 系统与论文清单 / A1–A5 分期）。
//! 本 crate 已落三期：
//!
//! - **A1**：静态场景的**直达 + 一次镜面反射（镜像源法）+ 遮挡**，单频带材质表（吸声系数）；
//! - **A2**：**portal 穿孔传声**（开口当二级声源）+ **无 portal 的墙按透射系数泄漏**；
//! - **A3**：**介质体积场**（声速 + 沿程吸收，[`AcousticMedium`]）—— 每条路径的时延与衰减
//!   按沿途介质积分（分层介质给[解析积分](AcousticMedium::segment_exact)，其它场退回
//!   固定步长中点法）。
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
//!    声速沿途积分。**折射（声线折转）仍未做** —— A3 的直线几何是显式边界（文档 §7）。
//!
//! ## 三条契约（与 `vxl-phys-sense` 同款，CI 可判）
//!
//! 1. **零干扰**：[`AcousticsExt::acoustic_cues`] 只持有 `&self` ⇒ 开/关声学层，`state_hash`
//!    逐位不变；
//! 2. **确定性**：三角按**索引序**扫描、路径按（声源序 → 三角序）产出，无 HashMap 迭代序、
//!    无随机采样 ⇒ 同输入逐字相同的 cue 序列（可用 `vxl-phys-cli` 的 `record/replay` 对拍）；
//! 3. **预算**：每 tick 的路径数（[`AcousticConfig::max_paths`]）与尝试的开口数
//!    （[`AcousticConfig::max_portals`]）都有上界，超了按序截断并在
//!    [`AcousticStats::dropped_budget`] 里**上报**（同 `FragmentBudget` 的口径）。

#![forbid(unsafe_code)]

mod medium;

pub use medium::{AcousticMedium, LayeredMedium, Medium, Slab};

use medium::seg_effect;
use vxl_phys::{Vec3, World};

/// **声学几何**（建模面：顶点 + 三角 + 逐面吸声系数 + 逐面透射系数 + 开口表）。
///
/// `absorption[i] ∈ [0,1]`：1 = 全吸（无声反射），0 = 全反射。**外部表**——不往 `Material` 塞
/// 声学字段（那是物理侧棘轮管的类型）。
pub struct AcousticGeometry<'a> {
    pub verts: &'a [Vec3],
    pub tris: &'a [[u32; 3]],
    pub absorption: &'a [f32],
    /// 逐面**透射系数**（A2）：直线被 `tris[i]` 挡住时按它泄漏，取值 `0` = 完全不透、`1` = 全透。
    /// 与 `absorption` 同口径：**外部表**；短了 / 缺了按 `0`（实墙）算。
    pub transmission: &'a [f32],
    /// **显式开口拓扑**（A2）：作者标的开口；空 = 场景没有开口（A1 行为）。
    pub portals: &'a [Portal],
    /// **介质体积场**（A3）：`None` = 均匀空气（用 [`AcousticConfig::speed_of_sound`]、无吸收，
    /// 数值与 A1 / A2 逐位相同）。
    pub medium: Option<&'a dyn AcousticMedium>,
}

/// 一个**开口**（A2）：四边形（四个角，顶点顺序无要求）+ **开启比例**。
///
/// `open ∈ [0,1]`：`0` = 关死（这条开口不传声，只剩墙的透射泄漏）、`1` = 全开；中间值就是
/// "开了一条缝"——增益按 `open` 线性衰减 ⇒ **缝宽 → 衰减单调**（文档 §5 的 A2 判据）。
/// 开口推荐**叠在实心建模面上声明**（几何照常封闭，开口由本表给出）；几何里真挖了空洞也能用。
pub struct Portal {
    pub quad: [Vec3; 4],
    pub open: f32,
}

/// 听者（A1 只有一个点 + 半径：进入该球即可闻）。
pub struct Listener {
    pub pos: Vec3,
    pub radius: f32,
}

/// 声学档（A1 的全部旋钮）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcousticConfig {
    /// 声速（m/s）；空气常温 ≈ 343。
    pub speed_of_sound: f32,
    /// 一次反射的最大候选面数（`0` = 只算直达；预算口径见模块头注）。
    pub max_reflections: u32,
    /// 每个声源最多尝试几个开口（A2；`0` = 不算穿孔传声）。
    pub max_portals: u32,
    /// 介质积分的**采样步长**（米；只在中点法回退时用到，下限 `1e-3`）。解析积分（分层介质）
    /// 不受它影响。
    pub medium_step: f32,
    /// 每 tick 最多产出多少条路径（硬预算）。
    pub max_paths: usize,
}

impl Default for AcousticConfig {
    fn default() -> Self {
        Self {
            speed_of_sound: 343.0,
            max_reflections: 24,
            max_portals: 8,
            medium_step: 0.5,
            max_paths: 16,
        }
    }
}

/// 路径种类（A2 起四档；A4 再加 `Diffracted`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathKind {
    /// 直线可达。
    Direct,
    /// 直线被挡，但按挡它的面的**透射系数**泄漏过来（A2）。
    Transmitted,
    /// 经一个**开口**（portal）穿过来（A2）。
    Portal,
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
    /// 能量增益（直达 = `1/d²`；反射再乘 `(1 - absorption)`）。
    pub gain: f32,
    /// 到达方向（单位；从听者指向来声）。
    pub dir: [f32; 3],
    pub kind: PathKind,
}

/// 一次归约的统计（"预算真的生效了没有"由它自证）。
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
    /// 反射路径数。
    pub reflected: usize,
    /// 因遮挡被丢的直达路径数。
    pub blocked: usize,
    /// 因超预算被丢的路径数。
    pub dropped_budget: usize,
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
            push_direct(out, &mut st, &cx);
            push_portals(out, &mut st, &cx);
            push_reflections(out, &mut st, &cx);
        }
        st
    }
}

/// 一次归约的上下文（几何 + 听者 + 档 + 本次声源）——打包起来免得每个助手摊一长串形参
/// （`args-gate` 的 soft 阈值实测把 7 个形参也算违规）。
struct Ctx<'a> {
    geo: &'a AcousticGeometry<'a>,
    listener: &'a Listener,
    cfg: &'a AcousticConfig,
    c: f32,
    si: u32,
    src: Vec3,
}

/// 直达：连线被挡 ⇒ 若挡它的面**透射系数 > 0** 则按它泄漏成一条 `Transmitted`（A2：没有 portal
/// 的墙的那条路径），否则计入 `blocked` 丢掉。`leak == 1` 时就是纯直达（数值与 A1 逐位相同）。
fn push_direct(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let (src, c) = (cx.src, cx.c);
    let d = cx.listener.pos - src;
    let mut leak = 1.0f32;
    if let Some(ti) = blocker(cx.geo, src, cx.listener.pos) {
        let tau = cx.geo.transmission.get(ti).copied().unwrap_or(0.0);
        if tau <= 0.0 {
            st.blocked += 1;
            return;
        }
        leak = tau.clamp(0.0, 1.0);
    }
    let dist = d.length().max(1e-3);
    // 听者半径只做**远场粗筛**（`radius` 的 1000 倍之外不产出；A3 起换成介质衰减与距离门）。
    if dist > cx.listener.radius.max(1e-3) * 1e3 {
        return;
    }
    if out.len() >= cx.cfg.max_paths {
        st.dropped_budget += 1;
        return;
    }
    // 约定：`dir` = **从听者指向来声**（空间化器要的就是入射方向，不是"声源→听者"）。
    let dir = ((src - cx.listener.pos) * (1.0 / dist)).normalize();
    // A3：沿途介质给的时延与吸收（无介质场 ⇒ `len / c`、衰减因子 1.0，与 A1/A2 逐位相同）。
    let (delay, atten) = seg_effect(cx.geo.medium, src, cx.listener.pos, c, cx.cfg.medium_step);
    let kind = if leak < 1.0 {
        PathKind::Transmitted
    } else {
        PathKind::Direct
    };
    out.push(SoundPath {
        source: cx.si,
        delay,
        gain: leak * atten / (dist * dist),
        dir: [dir.x, dir.y, dir.z],
        kind,
    });
    if leak < 1.0 {
        st.leaked += 1;
    } else {
        st.direct += 1;
    }
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
        if out.len() >= cx.cfg.max_paths {
            st.dropped_budget += 1;
            break;
        }
        let d1 = (c - cx.src).length().max(1e-3);
        let d2 = (cx.listener.pos - c).length().max(1e-3);
        let reach = d1 + d2;
        let dirv = (c - cx.listener.pos) * (1.0 / d2);
        // A3：两段各自的介质时延/吸收（球面扩散仍用几何距离 `reach`）。
        let (t1, a1) = seg_effect(cx.geo.medium, cx.src, c, cx.c, cx.cfg.medium_step);
        let (t2, a2) = seg_effect(cx.geo.medium, c, cx.listener.pos, cx.c, cx.cfg.medium_step);
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            gain: open * a1 * a2 / (reach * reach),
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
        if out.len() >= cfg.max_paths {
            st.dropped_budget += 1;
            break;
        }
        let d1 = (hit - src).length().max(1e-3);
        let d2 = (l - hit).length().max(1e-3);
        let dirv = (hit - l) * (1.0 / d2);
        // A3：两段各自的介质时延/吸收（吸声系数仍是面属性，两者相乘）。
        let (t1, a1) = seg_effect(geo.medium, src, hit, c, cfg.medium_step);
        let (t2, a2) = seg_effect(geo.medium, hit, l, c, cfg.medium_step);
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            gain: (1.0 - absorb.clamp(0.0, 1.0)) * a1 * a2 / ((d1 + d2) * (d1 + d2)),
            dir: [dirv.x, dirv.y, dirv.z],
            kind: PathKind::Reflected,
        });
        st.reflected += 1;
    }
}

/// 取三角的三顶点与吸声系数（索引越界 / 表短 ⇒ `None`，与 `TriMesh` 的"丢坏三角"同口径）。
fn tri_of(geo: &AcousticGeometry<'_>, ti: usize, tri: [u32; 3]) -> Option<(Vec3, Vec3, Vec3, f32)> {
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
fn blocker(geo: &AcousticGeometry<'_>, p: Vec3, q: Vec3) -> Option<usize> {
    let dir = q - p;
    let len = dir.length();
    if len <= 1e-6 {
        return None;
    }
    let dir = dir * (1.0 / len);
    for (ti, tri) in geo.tris.iter().enumerate() {
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
fn occluded(geo: &AcousticGeometry<'_>, p: Vec3, q: Vec3) -> bool {
    blocker(geo, p, q).is_some()
}

/// cue 流 → JSONL（一行一条；固定 6 位小数 ⇒ 可逐字对拍，接上 `vxl-phys-cli` 的 `record/replay`）。
pub fn jsonl(paths: &[SoundPath]) -> String {
    let mut s = String::new();
    for p in paths {
        s.push_str(&format!(
            "{{\"source\":{},\"kind\":\"{}\",\"delay\":{:.6},\"gain\":{:.6},\"dir\":[{:.6},{:.6},{:.6}]}}\n",
            p.source,
            match p.kind {
                PathKind::Direct => "direct",
                PathKind::Transmitted => "transmitted",
                PathKind::Portal => "portal",
                PathKind::Reflected => "reflected",
            },
            p.delay,
            p.gain,
            p.dir[0],
            p.dir[1],
            p.dir[2]
        ));
    }
    s
}
