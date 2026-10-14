//! **外壳 × 球 · 球心在体内那一档**（`OPEN-PROBLEMS.md` P22 ① 的修法）。
//!
//! 为什么单独一个文件：`support.rs` 受 god 门的**文件行数 + 最长函数**双棘轮（`hull_pair` 正卡在
//! 上限）⇒ 按 `mesh_mesh.rs` / `gyro.rs` 的先例另起文件、调用点只加两行。
//!
//! **缺口**（P22 ①，2026-10-14 实测）：`(球, 凸壳)` 原来一律走 `gjk::epa`，而 EPA 对**光滑支撑**
//! （球）在**深嵌**时给的是**最远面**的法线 —— 实测把球埋进 4 m 盒壳 0.5 m：报 `n = (−1,0,0)`、
//! 深度 **2.85**（正确的最小平移 **1.45**）⇒ 球被**横向**推走（本该从最近的顶面出去）。
//!
//! **修法**：球心在**凸壳内部**时改用**最近面**的解析口径 —— 与 `prims::sphere_convex_ab` 的
//! "球心在凸体内"分支**同一把尺**（`depth = radius + 到最近面的距离`、法线 = 该面外法线、
//! 接触点 = 球心在最近面平面上的投影）。凸壳只存**点云**（`gjk::ConvexHull` 没有面）⇒ 这里
//! **暴力枚举面**：三点定平面、其余点全在同侧即为一个面，按 (法线, 偏移) 去重、法线朝外
//! （与质心同侧的音）。
//!
//! **成本与护栏**：枚举是 `C(n,3)·n` 量级（n = 24 ⇒ ≈ 5 万次点积），且**只在"球心在体内"这一档**
//! 触发（外部/浅接触仍走 EPA ⇒ 那一档行为一字不动）。点数 > [`MAX_POINTS`] 或退化 ⇒ **如实放弃**
//! （返回 `None` ⇒ 回落原路径）。
use crate::{ContactPoint, DefaultNarrowPhase, Manifold};
use vxl_phys_core::{Shape, Vec3};

/// 暴力面枚举的点数上限（超过就回落 EPA：宁可保持旧行为，不在这里烧时间）。
pub(crate) const MAX_POINTS: usize = 40;

/// 若"对方是球且球心在壳内"则产出流形并返回 `true`（调用方 `return`）；否则 `false`（照旧走 EPA）。
///
/// `center` = **球的**世界球心（`hull_pair` 里 = 壳的对侧那个体的位姿）；顶点缓存取
/// `np.ws.hull_pts[side]`（`side` 由 `a_is_hull` 推出 —— `hull_pair` 刚填过它的世界点）。
pub(crate) fn pair(
    np: &DefaultNarrowPhase,
    ids: (u32, u32),
    a_is_hull: bool,
    center: Vec3,
    other: &Shape,
    out: &mut Vec<Manifold>,
) -> bool {
    let (a, b) = ids;
    let Shape::Sphere { radius } = *other else {
        return false;
    };
    let side = usize::from(!a_is_hull);
    let Some((n_out, depth, point)) = deep_exit(&np.ws.hull_pts[side], center, radius) else {
        return false;
    };
    // 流形法线 = a→b：外壳在 a ⇒ 球在 b ⇒ 球沿 `+n` 被推出 ⇒ `n = n_out`；反之取反。
    out.push(Manifold {
        a,
        b,
        normal: if a_is_hull { n_out } else { -n_out },
        points: [ContactPoint {
            point,
            depth,
            feature: 0,
        }]
        .into(),
    });
    true
}

/// **球心在凸壳内部**时的解析推出（全部在**世界系**）：返回 `(最近面外法线, depth, 最近面上的点)`；
/// 球心在外部 / 点数超限 / 退化 ⇒ `None`。
///
/// `depth = radius + 到最近面的距离`（最近面 = 平面带符号距离最大者；体内时该值 ≤ 0 ⇒ `depth ≥ radius`）。
pub(crate) fn deep_exit(points: &[Vec3], center: Vec3, radius: f32) -> Option<(Vec3, f32, Vec3)> {
    if points.len() < 4 || points.len() > MAX_POINTS {
        return None;
    }
    let facets = facets_of(points)?;
    // 球心在体内 ⟺ 对**所有**面都在内侧（`n·c ≤ d`）；最近面 = `n·c − d` 最大者。
    let mut best: Option<(Vec3, f32)> = None;
    let mut best_s = f32::MIN;
    for &(n, d) in &facets {
        let s = n.dot(center) - d;
        if s > 0.0 {
            return None; // 在某面外侧 ⇒ 不在体内（这一档不管，交回 EPA）
        }
        if s > best_s {
            best_s = s;
            best = Some((n, d));
        }
    }
    let (n_out, _) = best?;
    let depth = radius - best_s; // best_s ≤ 0 ⇒ depth ≥ radius
    let hit = center - n_out * best_s; // 投影到最近面
    Some((n_out, depth, hit))
}

/// **暴力枚举面**：三点定平面，其余点全在**同一侧** ⇒ 一个面。法线朝外（质心在内侧）、按 (法线, 偏移) 去重。
fn facets_of(points: &[Vec3]) -> Option<Vec<(Vec3, f32)>> {
    let n = points.len();
    let mut centroid = Vec3::ZERO;
    for p in points {
        centroid += *p;
    }
    centroid *= 1.0 / n as f32;
    const EPS: f32 = 1e-5;
    let mut out: Vec<(Vec3, f32)> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            for k in (j + 1)..n {
                let vn = (points[j] - points[i]).cross(points[k] - points[i]);
                let len = vn.length();
                if len < EPS {
                    continue; // 三点共线
                }
                let mut nrm = vn * (1.0 / len);
                let mut d = nrm.dot(points[i]);
                if nrm.dot(centroid) - d > 0.0 {
                    nrm = -nrm; // 让质心落在内侧
                    d = -d;
                }
                if points.iter().any(|p| nrm.dot(*p) - d > EPS) {
                    continue; // 有点在外侧 ⇒ 不是面
                }
                if out
                    .iter()
                    .any(|&(o, od)| o.dot(nrm) > 0.9999 && (od - d).abs() < 1e-4)
                {
                    continue; // 同一面的其它三点组合
                }
                out.push((nrm, d));
            }
        }
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use super::{deep_exit, facets_of};
    use vxl_phys_core::Vec3;

    fn box_hull(h: Vec3) -> Vec<Vec3> {
        let mut pts = Vec::new();
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    pts.push(Vec3::new(sx * h.x, sy * h.y, sz * h.z));
                }
            }
        }
        pts
    }

    /// 盒壳（8 点）：面数必须恰好 6；球心靠近**哪一面**就沿那一面推出 —— ±2 的盒里 y = −1.0
    /// 到底面 1.0、到顶面 3.0 ⇒ 沿 **−Y**；对称位置 y = +1.0 ⇒ 沿 **+Y**。深度都是 r + 1.0。
    #[test]
    fn box_hull_has_six_facets_and_exits_through_the_nearest() {
        let pts = box_hull(Vec3::splat(2.0));
        assert_eq!(
            facets_of(&pts).unwrap_or_default().len(),
            6,
            "盒壳应当恰好 6 个面"
        );
        let r = deep_exit(&pts, Vec3::new(0.4, -1.0, -0.4), 0.45);
        assert!(r.is_some(), "球心在体内 ⇒ 必须给出解析推出");
        let (n, depth, hit) = r.unwrap_or_default();
        assert!(
            (n + Vec3::Y).length() < 1e-3,
            "底面更近 ⇒ 应沿 −Y 推出：n = {n:?}"
        );
        assert!(
            (depth - 1.45).abs() < 1e-3,
            "深度应为 r + 1.0 = 1.45：{depth}"
        );
        assert!(
            (hit - Vec3::new(0.4, -2.0, -0.4)).length() < 1e-3,
            "接触点应落在 −Y 面：{hit:?}"
        );
        let r2 = deep_exit(&pts, Vec3::new(0.4, 1.0, -0.4), 0.45);
        assert!(r2.is_some(), "对称档也必须在体内");
        let (n2, _, hit2) = r2.unwrap_or_default();
        assert!(
            (n2 - Vec3::Y).length() < 1e-3,
            "对称档应沿 +Y 推出：n = {n2:?}"
        );
        assert!(
            (hit2 - Vec3::new(0.4, 2.0, -0.4)).length() < 1e-3,
            "接触点应落在 +Y 面：{hit2:?}"
        );
    }

    /// 球心在**外部** ⇒ 这一档不管（`None` ⇒ 交回 EPA）；点数超限同样 `None`。
    #[test]
    fn outside_or_too_many_points_defers_to_epa() {
        let pts = box_hull(Vec3::splat(1.0));
        assert!(deep_exit(&pts, Vec3::new(3.0, 0.0, 0.0), 0.45).is_none());
        let many: Vec<Vec3> = (0..super::MAX_POINTS + 1)
            .map(|i| Vec3::splat(i as f32))
            .collect();
        assert!(deep_exit(&many, Vec3::ZERO, 0.45).is_none());
    }
}
