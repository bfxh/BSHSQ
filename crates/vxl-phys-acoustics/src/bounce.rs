//! **A8「多次反射」（二阶镜像展开）**：一次反射由 `push_reflections` 给（A1–A7），这里补上
//! `声源 →(面 f₁) →(面 f₂) → 听者` 的**二阶**那条。抽成独立模块只是为了 `lib.rs` 留在 god 门内。
//!
//! 口径（与一次反射同一套）：
//!
//! 1. **展开（unfolding）**：把听者沿链上各面**逐面镜像**得到 `L₁`，则折线长 = `|src − L₁|`、
//!    各反射点 = 逐段与对应面求交 —— 与一次反射的镜像源法是同一个构造；
//! 2. **有效性**：每个反射点必须落在那张三角**内部**、两两之间的子段都不能被挡
//!    （⚠️ 检查时**跳过链上的面**：子段本来就在那些面上起落 —— 这是低阶镜像展开的常用取舍，
//!    代价是"同一面被穿过两次"这类路径可能多算一条，2 阶下可接受，已在文档登记）；
//! 3. **能量**：`bands_b = Π_k (1 − α_{f_k,b}) · Π_k (1 − s_{f_k,b}) · 介质逐段衰减 / L²`
//!    （逐面吸声/散射相乘、球面扩散按**总程长** `L` 算 —— 与一次反射同口径）；
//! 4. **预算与确定性**：链上**每一阶段**最多试 `max_reflections` 张面（超了如实进
//!    `dropped_budget`）、按三角索引序扫描、平局按产出顺序 ⇒ 同输入逐位相同；
//!    **同一张面连续出现两次的链跳过**（`f→f` 等于原路返回，退化）。
//!
//! ⚠️ 成本：候选 ≈ `max_reflections²` 条链，每条 3 段遮挡检查 ⇒ 只建议**离线档**开
//! （[`AcousticConfig::reflection_order`] 默认 1 = 一次反射的行为，逐位不变）。

use crate::medium::seg_effect;
use crate::{
    blocker_skip, inside_tri, mid_band, tri_of, AcousticStats, Ctx, PathKind, SoundPath, BANDS,
};
use vxl_phys::Vec3;

/// 法向（长度 ≤ 1e-9 的退化面给 `None`）。
fn face_normal(geo: &crate::AcousticGeometry<'_>, ti: usize) -> Option<(Vec3, Vec3, Vec3, Vec3)> {
    let tri = *geo.tris.get(ti)?;
    let (a, b, c, _) = tri_of(geo, ti, tri)?;
    let n = (b - a).cross(c - a);
    let ln = n.length();
    if ln <= 1e-9 {
        return None;
    }
    Some((a, b, c, n * (1.0 / ln)))
}

/// 把 `p` 对平面（点 `a`、法向 `n`）镜像。
fn mirror(p: Vec3, a: Vec3, n: Vec3) -> Vec3 {
    p - n * (2.0 * (p - a).dot(n))
}

/// 线段 `p→q` 与平面（点 `a`、法向 `n`）的交点（平行 / 交在段外给 `None`）。
fn cross_plane(p: Vec3, q: Vec3, a: Vec3, n: Vec3) -> Option<Vec3> {
    let seg = q - p;
    let denom = seg.dot(n);
    if denom.abs() <= 1e-9 {
        return None;
    }
    let t = (a - p).dot(n) / denom;
    if !(0.0..=1.0).contains(&t) {
        return None;
    }
    Some(p + seg * t)
}

/// 展开一条链：返回 `(反射点表, 总程长)`；链不成立给 `None`。
pub(crate) fn unfold(
    geo: &crate::AcousticGeometry<'_>,
    chain: &[usize],
    src: Vec3,
    dst: Vec3,
) -> Option<(Vec<Vec3>, f32)> {
    // 1) 听者逐面镜像（链尾 → 链头）：`L_k` = 听者沿 `f_k..f_n` 镜像后的点。
    let mut images = vec![Vec3::ZERO; chain.len()];
    let mut cur = dst;
    for (k, ti) in chain.iter().enumerate().rev() {
        let (a, _, _, n) = face_normal(geo, *ti)?;
        cur = mirror(cur, a, n);
        images[k] = cur;
    }
    // 2) 逐段求交：`p_k` = 从上一个点走向 `L_k` 时与 `f_k` 的交点。
    let mut points = Vec::with_capacity(chain.len());
    let mut from = src;
    let mut len = 0.0f32;
    for (k, ti) in chain.iter().enumerate() {
        let (a, b, c, n) = face_normal(geo, *ti)?;
        let p = cross_plane(from, images[k], a, n)?;
        if !inside_tri(a, b, c, p) {
            return None;
        }
        len += (p - from).length();
        points.push(p);
        from = p;
    }
    len += (dst - from).length();
    Some((points, len))
}

/// **A8：二阶镜像展开** —— 逐对面 `(f₁, f₂)` 试一条链，成立就产出一条 `Reflected` 路径。
pub(crate) fn push_second_order(out: &mut Vec<SoundPath>, st: &mut AcousticStats, cx: &Ctx<'_>) {
    let (src, l, geo, cfg) = (cx.src, cx.listener.pos, cx.geo, cx.cfg);
    let mut tried1 = 0u32;
    for ti in 0..geo.tris.len() {
        if tried1 >= cfg.max_reflections {
            st.dropped_budget += 1;
            break;
        }
        let Some(_) = face_normal(geo, ti) else {
            continue;
        };
        tried1 += 1;
        let mut tried2 = 0u32;
        for tj in 0..geo.tris.len() {
            if tj == ti {
                continue; // 同一面连着两次 = 原路返回（退化）
            }
            if tried2 >= cfg.max_reflections {
                st.dropped_budget += 1;
                break;
            }
            let Some(_) = face_normal(geo, tj) else {
                continue;
            };
            tried2 += 1;
            let chain = [ti, tj];
            let Some((points, len)) = unfold(geo, &chain, src, l) else {
                continue;
            };
            // 子段两两不挡（**跳过链上的两张面**：子段本来就在它们上面起落）。
            let mut legs = Vec::with_capacity(3);
            legs.push(src);
            legs.extend(points.iter().copied());
            legs.push(l);
            let skip = (ti, tj);
            let blocked = legs
                .windows(2)
                .any(|w| blocker_skip(geo, w[0], w[1], skip).is_some());
            if blocked {
                continue;
            }
            let mut bands = [1.0f32; BANDS];
            let mut delay = 0.0f32;
            for (k, w) in legs.windows(2).enumerate() {
                let (dk, ak) = seg_effect(geo.medium, w[0], w[1], cx.c, cfg.medium_step);
                delay += dk;
                for (o, x) in bands.iter_mut().zip(ak.iter()) {
                    *o *= *x;
                }
                // 每落一次面：吸声与散射打折（与一次反射同口径）。
                if k + 1 < legs.len() - 1 {
                    let face = chain[k];
                    let abs = geo.absorption.get(face).copied().unwrap_or([0.0; BANDS]);
                    let sc = geo.scattering.get(face).copied().unwrap_or([0.0; BANDS]);
                    for (o, (r, s)) in bands.iter_mut().zip(abs.iter().zip(sc.iter())) {
                        *o *= (1.0 - r.clamp(0.0, 1.0)) * (1.0 - s.clamp(0.0, 1.0));
                    }
                }
            }
            let total = len.max(1e-3);
            for o in bands.iter_mut() {
                *o /= total * total;
            }
            let last = points[points.len() - 1];
            let dirv = (last - l) * (1.0 / (last - l).length().max(1e-3));
            out.push(SoundPath {
                source: cx.si,
                delay,
                bands,
                gain: mid_band(&bands),
                dir: [dirv.x, dirv.y, dirv.z],
                kind: PathKind::Reflected,
            });
            st.reflected += 1;
        }
    }
}
