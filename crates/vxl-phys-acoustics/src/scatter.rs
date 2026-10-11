//! **A4「散射」（漫反射二级声源）**：面当点源，按 Lambertian 往听者方向辐射。
//!
//! 抽成独立模块只是为了 `lib.rs` 留在 god 门的 800 行上限内；口径与判据见 crate 头注与
//! `tests/a4_scatter.rs`。

use crate::medium::seg_effect;
use crate::{mid_band, occluded, tri_of, AcousticStats, Ctx, PathKind, SoundPath, BANDS};

/// **A4「散射」** —— 面当**二级声源**（漫反射口径）：面中心按 Lambertian 往听者方向辐射
///
/// `gain_b = s_b · cos_in · cos_out · A / (d1² · d2²)`
///
/// ⚠️ **工程口径**（不承诺辐射度单位）：面积 × 入射余弦 × 出射余弦 × 散射系数 / 两段距离平方。
/// 要真辐射度得上离线烘焙（如 Project Acoustics 那类）。条件：正/背面要**同侧**、两个余弦 > 0、
/// 两条腿都不被挡。镜面那一半在 `push_reflections` 里按 `(1 - s_b)` 打折 ⇒ **能量守恒式分工**。
pub(crate) fn push_scatter(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    for (ti, tri) in cx.geo.tris.iter().enumerate() {
        if ti as u32 >= cx.cfg.max_scatters {
            st.dropped_budget += 1;
            break;
        }
        let s = cx.geo.scattering.get(ti).copied().unwrap_or([0.0; BANDS]);
        if s.iter().all(|v| *v <= 0.0) {
            continue;
        }
        let Some((a, b, cc, _)) = tri_of(cx.geo, ti, *tri) else {
            continue;
        };
        let cross = (b - a).cross(cc - a);
        let area2 = cross.length();
        if area2 <= 1e-9 {
            continue;
        }
        let p = (a + b + cc) * (1.0 / 3.0);
        let v1 = cx.src - p;
        let v2 = cx.listener.pos - p;
        let (d1, d2) = (v1.length(), v2.length());
        if d1 <= 1e-3 || d2 <= 1e-3 {
            continue;
        }
        // 法向朝"声源那一侧"：声与听者必须在面的同一侧，才谈得上反射式散射。
        let mut n = cross * (1.0 / area2);
        if n.dot(v1) < 0.0 {
            n *= -1.0;
        }
        let cos_in = n.dot(v1 * (1.0 / d1)).max(0.0);
        let cos_out = n.dot(v2 * (1.0 / d2)).max(0.0);
        if cos_in <= 0.0 || cos_out <= 0.0 {
            continue;
        }
        if occluded(cx.geo, cx.src, p) || occluded(cx.geo, p, cx.listener.pos) {
            continue;
        }
        let (t1, a1) = seg_effect(cx.geo.medium, cx.src, p, cx.c, cx.cfg.medium_step);
        let (t2, a2) = seg_effect(cx.geo.medium, p, cx.listener.pos, cx.c, cx.cfg.medium_step);
        let geom = 0.5 * area2 * cos_in * cos_out / (d1 * d1 * d2 * d2);
        let mut bands = [0.0f32; BANDS];
        for (((o, k), x), y) in bands.iter_mut().zip(s.iter()).zip(a1.iter()).zip(a2.iter()) {
            *o = k.clamp(0.0, 1.0) * geom * x * y;
        }
        let dirv = v2 * (1.0 / d2);
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            bands,
            gain: mid_band(&bands),
            dir: [dirv.x, dirv.y, dirv.z],
            kind: PathKind::Scattered,
        });
        st.scattered += 1;
    }
}
