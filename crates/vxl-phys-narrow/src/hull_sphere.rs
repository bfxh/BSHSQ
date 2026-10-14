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
//! 触发（外部/浅接触仍走 EPA ⇒ 那一档的法线/深度口径一字不动）。点数 > [`MAX_POINTS`] 或退化 ⇒
//! **如实放弃**（返回 `None` ⇒ 回落原路径）。
//!
//! ⚠️ **2026-10-14 第三刀（P22 ②）在本文件加了两处**，都不改上面这条护栏：
//! ① [`deep_exit`] 的"体内"判据加**尺寸量级**容差 ⇒「恰好落在面平面**外侧**一个浮点噪声量」也算体内
//! （否则那一 tick 交回 EPA ⇒ 深嵌时 EPA 给**顶点带**、接触点落到壳**角**上）；
//! ② [`band_points`] 把**对方是球**的顶点带收敛成**单点**（浅接触的**接触点**因此变了 —— 法线与
//! 深度口径不变；球是光滑体，接触本就只有一个点）。
use crate::{ContactPoint, ContactPoints, DefaultNarrowPhase, Manifold};
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

/// **球心在凸壳内部**（含「恰好落在面上」这一档，见 `tol`）时的解析推出（全部在**世界系**）：
/// 返回 `(最近面外法线, depth, 最近面上的点)`；球心在外部 / 点数超限 / 退化 ⇒ `None`。
///
/// `depth = radius − best_s`（`best_s` = 到最近面的**带符号**距离，最近面 = 该值最大者；体内时
/// `best_s ≤ 0` ⇒ `depth ≥ radius`；"恰好落在外侧"时 `best_s` 是个噪声量级的小正数 ⇒ 深度 ≈ radius）。
///
/// ⚠️ 「恰好落在面上」必须**算进来**：球心在面平面外侧一个浮点噪声量时（实测埋球顶出经过壳顶面那一
/// tick），旧写法退回 EPA，而 EPA 那一路给的是**壳顶点带**（`support.rs::hull_points_in_band`）——
/// 接触点落在 **4 m 壳的角上**（离真实接触点 2.8 m）⇒ 力臂错 ⇒ 顶出被塞进自旋（μ=0 也照样漂）。
/// 容差取壳**尺寸量级**的 `8·ε_f32`（2 m 半宽的壳 ≈ 1e-6 m），远小于任何真实浅接触。
pub(crate) fn deep_exit(points: &[Vec3], center: Vec3, radius: f32) -> Option<(Vec3, f32, Vec3)> {
    if points.len() < 4 || points.len() > MAX_POINTS {
        return None;
    }
    let facets = facets_of(points)?;
    // 壳的**尺寸**量级（半包围盒对角线；与平移无关）⇒ 容差随尺寸等比。
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for p in points {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let tol = 8.0 * f32::EPSILON * ((hi - lo).length() * 0.5).max(1.0);
    // 球心在体内 ⟺ 对**所有**面都在内侧（`n·c ≤ d`）；最近面 = `n·c − d` 最大者。
    let mut best: Option<(Vec3, f32)> = None;
    let mut best_s = f32::MIN;
    for &(n, d) in &facets {
        let s = n.dot(center) - d;
        if s > tol {
            return None; // 明显在某面外侧 ⇒ 不在体内（这一档不管，交回 EPA）
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

/// **顶点带 ⇒ 流形点**（`support.rs::hull_pair` 的 EPA 腿的收尾）：`pts` 为顶点带读数（深度降序，
/// 非空由调用方保证）。对方是**球**时收敛成**单点**、且落在「过球心的法线」上：
///
/// 顶点带取的是**壳顶点** —— 球落在 4 m 壳面上、球心离面心 0.57 m 时接触点会被放到壳**角**上
/// （力臂 2.8 m）⇒ 纯法向接触被塞进自旋（实测 μ = 0 顶出档 `max|ω| = 1.15`，修后 0）。
/// 深度沿用顶点带读数（面口径：球心在面外侧时为 `radius − 到面距离`），接触点
/// `center + n·(radius − depth)` ⇒ 力臂 ∥ 法线 ⇒ 不产生自旋（与 [`deep_exit`] 的投影口径一致）。
/// 其它形状照旧走顶点带。`pts` 为空 ⇒ `None`（调用方早退）。
pub(crate) fn band_points(
    pts: Vec<ContactPoint>,
    other: &Shape,
    center: Vec3,
    n: Vec3,
) -> Option<ContactPoints> {
    let deep = pts.first()?;
    if let Shape::Sphere { radius } = *other {
        let p = ContactPoint {
            point: center + n * (radius - deep.depth),
            depth: deep.depth,
            feature: 0,
        };
        return Some(ContactPoints::from_slice(&[p]));
    }
    Some(ContactPoints::from_slice(&pts))
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
    use crate::{DefaultNarrowPhase, NarrowPhase, Shape};
    use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Vec3};

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

    /// **「恰好落在面上」这一档必须算体内**：球心在面平面**外侧** `1e-6 m`（浮点噪声量级）——
    /// 旧写法（`s > 0.0 ⇒ None`）会把它交给 EPA，而 EPA 在退化构形上不可靠（P22 ① 实测给过**最远面**）。
    /// 判据：仍给出最近面解析推出（±Y、深度 ≈ r、接触点在球心正下方）。
    #[test]
    fn almost_on_face_still_uses_the_analytic_exit() {
        // 顶面落到 y = 0（与 `tests/deep_penetration.rs` 壳档的实测构形一致）
        let mut pts = box_hull(Vec3::splat(2.0));
        for p in &mut pts {
            p.y -= 2.0;
        }
        let r = deep_exit(&pts, Vec3::new(0.4, 1e-6, -0.4), 0.45);
        assert!(
            r.is_some(),
            "面外侧一个噪声量 ⇒ 必须仍走解析口径（不许交回 EPA）"
        );
        let (n, depth, hit) = r.unwrap_or_default();
        assert!(
            (n - Vec3::Y).length() < 1e-3,
            "最近面是 +Y 面 ⇒ 外法线应是 +Y：{n:?}"
        );
        assert!(
            (depth - 0.45).abs() < 1e-3,
            "深度应 ≈ 半径（球心几乎就在面平面上）：{depth}"
        );
        assert!(
            (hit - Vec3::new(0.4, 0.0, -0.4)).length() < 1e-3,
            "接触点应落在球心正下方的 +Y 面上：{hit:?}"
        );
    }

    /// **球心落在壳顶面平面外侧一个浮点噪声量**（`1e-7 m`，`deep_exit` 的容差之内）⇒ 一路走完
    /// 流形构造：对面是**球**时接触必须是**单点**、且落在「过球心的法线」上（接触点 ≈ 球心在面平面上的
    /// 投影）。旧口径（顶点带原样）给的是**壳角** `(−2, 0, −2)`（离真实接触点 2.8 m）⇒ 力臂错
    /// ⇒ 法向顶出被塞进自旋（`tests/deep_penetration.rs` ⑤ 的壳档：μ=0 时 `max|ω| = 1.15`）。
    #[test]
    fn sphere_on_hull_face_plane_gets_single_point_under_center() {
        let mut np = DefaultNarrowPhase::new(0.01);
        let mut pts: Vec<Vec3> = Vec::with_capacity(8);
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    pts.push(Vec3::new(sx * 2.0, sy * 2.0, sz * 2.0));
                }
            }
        }
        let hid = np.add_hull(pts);
        let mut b = BodySet::new();
        b.push_static(
            Shape::ConvexHull {
                hull: hid,
                half: Vec3::splat(2.0),
            },
            Vec3::new(0.0, -2.0, 0.0), // 顶面 y = 0
            Quat::IDENTITY,
        );
        let c = Vec3::new(0.4, 1e-7, -0.4); // 球心在顶面平面**外侧**一个噪声量
        b.push_dynamic(Shape::Sphere { radius: 0.45 }, c, Quat::IDENTITY, 1000.0);
        let mut pairs = Vec::new();
        for i in 0..b.len() as u32 {
            for j in (i + 1)..b.len() as u32 {
                pairs.push((i, j));
            }
        }
        let mut out = Vec::new();
        np.collide(
            &b,
            &pairs,
            &[],
            &vxl_phys_core::interop::NoProviders,
            &mut out,
            &SerialJobSystem,
        );
        assert_eq!(out.len(), 1, "应有 1 个流形（外壳 × 球）");
        let m = &out[0];
        let want = if m.a == 0 { 1.0 } else { -1.0 }; // 法线约定 a→b
        assert!(
            m.normal.y * want > 0.999,
            "法线必须是面法线（±Y），实得 {:?}",
            m.normal
        );
        assert_eq!(m.points.len(), 1, "球的接触必须是单点");
        let p = m.points[0];
        let proj = Vec3::new(c.x, 0.0, c.z);
        assert!(
            (p.point - proj).length() < 0.05,
            "接触点应是球心在面平面上的投影 {proj:?}，实得 {:?}（旧口径落在壳角上：2.8 m 力臂）",
            p.point
        );
        assert!(
            (p.depth - 0.45).abs() < 1e-3,
            "深度应 ≈ 半径（球心就在面平面上），实得 {}",
            p.depth
        );
    }
}
