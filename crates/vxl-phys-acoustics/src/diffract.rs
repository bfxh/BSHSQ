//! **A4「绕射」（工程近似）**：棱上取 **Fermat 点**、**只在阴影里**产出。
//!
//! 抽成独立模块只是为了 `lib.rs` 留在 god 门的 800 行上限内；口径与判据见 crate 头注与
//! `tests/a4_diffraction.rs`（含与**暴力扫棱**的对拍）。

use crate::medium::seg_effect;
use crate::{AcousticStats, Ctx, Edge, PathKind, SoundPath, BANDS};
use vxl_phys::Vec3;

/// Fermat 二分次数：定次数、无早退 ⇒ 逐位可复现（`f32` 尾数 24 位，48 次远超够用）。
const FERMAT_ITERS: u32 = 48;

/// **A4：绕射（工程近似）** —— `声源 → 棱上的 Fermat 点 → 听者`，**只在阴影里**产出
/// （直达没被挡就不算绕射，免得与直达重复计能；这也就是"拐角后仍可闻"的语义）。
///
/// 能量口径：`coeff_b · D(θ) · 介质衰减 / (d1·d2)` —— `1/(d1·d2)` 是 UTD 的**远场形状**，
/// `D(θ) = (1 + cosθ)/2`（`θ` = 两段折线的夹角）：**掠射**（刚被挡）时 `D → 1`（与自由场连续），
/// 越往阴影深处越小。⚠️ **不做** Fresnel 积分、也不做楔角相关的 UTD 系数（那是 A4 的另一半）。
pub(crate) fn push_diffraction(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    if crate::blocker(cx.geo, cx.src, cx.listener.pos).is_none() {
        return;
    }
    for (tried, e) in cx.geo.edges.iter().enumerate() {
        if tried as u32 >= cx.cfg.max_edges {
            st.dropped_budget += 1;
            break;
        }
        let p = fermat_point(e, cx.src, cx.listener.pos);
        let (v1, v2) = (p - cx.src, cx.listener.pos - p);
        let (d1, d2) = (v1.length(), v2.length());
        if d1 <= 1e-3 || d2 <= 1e-3 {
            continue;
        }
        if crate::occluded(cx.geo, cx.src, p) || crate::occluded(cx.geo, p, cx.listener.pos) {
            continue;
        }
        let cos_t = (v1.dot(v2) / (d1 * d2)).clamp(-1.0, 1.0);
        let dir_factor = 0.5 * (1.0 + cos_t);
        let (t1, a1) = seg_effect(cx.geo.medium, cx.src, p, cx.c, cx.cfg.medium_step);
        let (t2, a2) = seg_effect(cx.geo.medium, p, cx.listener.pos, cx.c, cx.cfg.medium_step);
        let mut bands = [0.0f32; BANDS];
        for (((o, k), x), y) in bands
            .iter_mut()
            .zip(e.coeff.iter())
            .zip(a1.iter())
            .zip(a2.iter())
        {
            *o = k * dir_factor * x * y / (d1 * d2);
        }
        let dirv = (p - cx.listener.pos) * (1.0 / d2);
        out.push(SoundPath {
            source: cx.si,
            delay: t1 + t2,
            bands,
            gain: crate::mid_band(&bands),
            dir: [dirv.x, dirv.y, dirv.z],
            kind: PathKind::Diffracted,
        });
        st.diffracted += 1;
    }
}

/// 棱上让 `|P−src| + |P−dst|` 最小的点（P 限定在**线段** `a→b` 上）。
///
/// `f(u) = |P(u)−src| + |P(u)−dst|` 是 `u` 的**凸**函数，且
/// `f'(u) = cos∠(dir, P−src) + cos∠(dir, P−dst)` 单调不减（每项导数 = `sin²/|P−·| ≥ 0`）
/// ⇒ 在「两个投影夹出的区间」上**定次数二分**求驻点（凸 ⇒ 驻点即全局最小），
/// 再把参数夹进 `[0, len]`：夹到端点就是"从棱的端点绕"，几何上正确（凸函数区间最小在端点）。
fn fermat_point(e: &Edge, src: Vec3, dst: Vec3) -> Vec3 {
    let seg = e.b - e.a;
    let len = seg.length();
    if len <= 1e-6 {
        return e.a;
    }
    let dir = seg * (1.0 / len);
    let ps = (src - e.a).dot(dir).clamp(0.0, len);
    let pd = (dst - e.a).dot(dir).clamp(0.0, len);
    let (mut lo, mut hi) = if ps <= pd { (ps, pd) } else { (pd, ps) };
    if hi - lo <= 1e-7 {
        return e.a + dir * (0.5 * (lo + hi));
    }
    for _ in 0..FERMAT_ITERS {
        let mid = 0.5 * (lo + hi);
        let p = e.a + dir * mid;
        let (va, vb) = (p - src, p - dst);
        let slope = va.dot(dir) / va.length().max(1e-6) + vb.dot(dir) / vb.length().max(1e-6);
        if slope < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    e.a + dir * (0.5 * (lo + hi))
}
