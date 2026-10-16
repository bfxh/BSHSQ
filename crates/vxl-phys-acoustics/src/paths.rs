//! **路径产出族**（A1–A8 的候选产线 + 几何/遮挡工具）。
//!
//! 从 crate 根拆出来（同一个 800 行硬线）：这里全是"给定一次归约的上下文，产出若干条
//! `SoundPath`"的助手，crate 根只留归约入口 `AcousticsExt`。函数体逐字未改；唯一动的是
//! **可见性** —— 原先靠"crate 根的私有项对后代模块可见"白拿，一搬模块就不成立，故提成
//! `pub(crate)`（crate 根的 `impl AcousticsExt for World` 与 `bounce`/`scatter` 要调）。

use crate::medium::seg_effect;
use crate::{
    AcousticConfig, AcousticGeometry, AcousticStats, Listener, PathKind, SoundPath, BANDS,
};
use vxl_phys::Vec3;

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
pub(crate) fn push_direct(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
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
pub(crate) fn push_portals(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
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
pub(crate) fn push_refraction(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
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
pub(crate) fn push_reflections(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
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
pub(crate) fn inside_tri(a: Vec3, b: Vec3, c: Vec3, p: Vec3) -> bool {
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
