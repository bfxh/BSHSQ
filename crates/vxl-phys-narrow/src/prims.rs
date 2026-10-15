//! prims：球-凸体 / 胶囊-凸体的**模块级自由函数**（接触点选点在 `select.rs`）
//! （2026-10-06 从 `impl DefaultNarrowPhase` 搬出；语义未改）。
use super::*;

/// 球-凸体：球心 center、半径 radius；凸体形状 convex（世界变换 cpos/crot）。
/// 返回 (球→凸体方向法线, 深度, 接触点)。
pub(crate) fn sphere_convex_ab(
    np: &mut DefaultNarrowPhase,
    center: Vec3,
    radius: f32,
    convex: &Shape,
    cpos: Vec3,
    crot: Quat,
) -> Option<(Vec3, f32, Vec3)> {
    let idx = crate::support::poly_for(np, convex)?;
    np.ws.poly_b.fill(&np.ws.polys[idx], cpos, crot);
    let (closest, d2, inside, _in_n) = closest_point_on_poly(&np.ws.poly_b, center);
    let dist = d2.sqrt();
    // 体内 **或**「球心落在体表面上」（`closest − center` 只剩浮点噪声 ⇒ 方向不可用）⇒ 同一个
    // **最近面**解析口径（口径与判据都在 `surface.rs`；穿透 = `radius − max_d`，不是 `+ max_d`）。
    if inside || dist < crate::surface::on_surface_eps(center) {
        return crate::surface::sphere_exit(&np.ws.poly_b, center, radius);
    }
    if dist >= radius {
        return None;
    }
    Some(((closest - center) * (1.0 / dist), radius - dist, closest))
}

/// **胶囊中心线 ↔ 凸体**：中心线取最近点后复用球的解析路径（`sphere_convex_ab`）。
/// 返回 `(中心线最近点, 对方表面点)`；无接触返回 `None`。
///
/// 为什么不用 GJK/EPA（`EXPERIMENTS.md` R.1/R.2 两次实测）：EPA 对**光滑**支撑不适定
/// （临界接触给 45° 假法线 + 42 m 假深度）；"收成芯再跑 GJK"又踩 GJK 的**面平局**退化
/// （盒顶面对 ±Y 时支撑解到角点 + 无进展提前退出，实测 `d=7.077 / p_other=(5,0,5)`）。
/// 解析最近点两条坑都不碰。
///
/// 采样点为什么就是段上最近点：点到**凸**集的距离沿线段是凸函数 ⇒ 迭代
/// `q ← 体上最近点(p)`、`p ← 段上最近点(q)` 的驻点即全局最近点对，且距离单调不增。
pub(crate) fn capsule_axis_reach(
    np: &mut DefaultNarrowPhase,
    s0: Vec3,
    s1: Vec3,
    radius: f32,
    other: &Shape,
    opos: Vec3,
    orot: Quat,
) -> Option<(Vec3, Vec3)> {
    let idx = crate::support::poly_for(np, other)?;
    np.ws.poly_b.fill(&np.ws.polys[idx], opos, orot);
    let seg = s1 - s0;
    let seg_len2 = seg.length_squared();
    let mut p = (s0 + s1) * 0.5;
    for _ in 0..4 {
        let (q, _d2, inside, in_n) = closest_point_on_poly(&np.ws.poly_b, p);
        // 内部时同样把 q 拉到"朝最近面"的表面上，迭代方向才有效。
        let q = if inside {
            p + in_n * -max_plane_d_of(&np.ws.poly_b, p)
        } else {
            q
        };
        let t = if seg_len2 > 1e-18 {
            ((q - s0).dot(seg) / seg_len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let p_new = s0 + seg * t;
        let done = (p_new - p).length_squared() <= 1e-12;
        p = p_new;
        if done {
            break;
        }
    }
    // 采样点交给球的解析路径（其内部会再填一次 `poly_b`，成本可接受）。
    let (_n, _depth, surf) = sphere_convex_ab(np, p, radius, other, opos, orot)?;
    Some((p, surf))
}

/// **胶囊体 × 凸体：GJK 距离**。返回 `(n_cap→other, dist, p_cap, p_other)`；
/// 线段与对方重叠（`Intersecting`）时改用 EPA 求穿透法线（返回 `dist = -depth`）。
///
/// 为什么不能用 SAT/裁剪：胶囊是**光滑**外形，SAT 需要有限面集。为什么不能直接用
/// EPA：EPA 只在**重叠**时可用，而"胶囊接触"的定义是**线段到对方的距离 < radius**
/// （此时线段本身可能完全没碰到对方）。GJK 距离恰好给出这个量。
pub(crate) fn capsule_reach(
    cap: &gjk::CapsuleSupport,
    other: &dyn gjk::Support,
) -> Option<(Vec3, f32, Vec3, Vec3)> {
    match gjk::gjk(cap, other) {
        gjk::Gjk::Separated {
            point_a,
            point_b,
            dist,
        } => {
            let n = if dist > 1e-9 {
                (point_b - point_a) * (1.0 / dist)
            } else {
                Vec3::Y
            };
            Some((n, dist, point_a, point_b))
        }
        gjk::Gjk::Intersecting => {
            let (n, depth, p) = gjk::epa(cap, other, 32)?;
            Some((n, -depth, p, p))
        }
    }
}
