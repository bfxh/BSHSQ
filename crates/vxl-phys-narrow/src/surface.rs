//! surface：**球心落在凸体表面上**（含恰好落在面平面那一 tick）时的接触口径（`OPEN-PROBLEMS.md` P22 ②）。
//!
//! 为什么单列一个文件：`support.rs` / `prims.rs` / `entry.rs` 都受 god 门的**文件行数棘轮**
//! （只准减）⇒ 按 `hull_sphere.rs` / `mesh_mesh.rs` / `gyro.rs` 的先例另起文件、调用点只加 1~2 行。
//!
//! **缺口**（2026-10-14 实测）：`closest_point_on_poly` 的"内/外"判据是**严格** `d > 0.0`（无容差）
//! ⇒ 球心落在面平面**外侧**一个浮点噪声量（顶出**恰好经过**面平面那一 tick 就是这种状态）时判"外"，
//! 随即用 `(closest − center)/dist` 取法线 —— 而那时 `dist` 只剩 `~1e-8`，**方向被最近点重算的舍入
//! 噪声主导**（实测盒顶档报 `n ≈ (−0.943, +0.106, −0.314)`）⇒ 纯法向的顶出被塞进 `vx ≈ −2.6 m/s`
//! + 自旋 ⇒ 球随后从平台边缘滚走（`tests/deep_penetration.rs` ⑤ 的判据）。
//!
//! **修法**：这一档与"球心在凸体内"**同用最近面解析口径** —— 法线 = 最近面外法线、深度 = `radius − max_d`、
//! 接触点 = 球心在最近面平面上的**投影**（力臂 ∥ 法线 ⇒ 不产生自旋）。退化档的判据是"到最近点的距离
//! 小到只剩噪声"，见 [`on_surface_eps`]。
use crate::WorldPoly;
use vxl_phys_core::Vec3;

/// 「球心落在体表面上」的容差：`closest − center` 的长度小到这个量级时，它**只剩浮点噪声**
/// —— 归一化出来的是**噪声方向**而不是几何方向。取坐标量级的 `8·ε_f32`（`ε_f32 ≈ 1.2e-7`）：
/// 1 m 场景 ≈ 1e-6 m，且随坐标等比放大。
pub(crate) fn on_surface_eps(p: Vec3) -> f32 {
    let a = p.abs().max(Vec3::ONE);
    8.0 * f32::EPSILON * a.x.max(a.y).max(a.z)
}

/// **最近面**：对**所有**面取「平面带符号距离」最大者 ⇒ `(该距离, 该面外法线)`。
/// 体内时该距离 ≤ 0、即最近面的**负**距离；体外时 > 0（≤ 到体的距离）。
pub(crate) fn nearest_face_of(poly: &WorldPoly, p: Vec3) -> (f32, Vec3) {
    let mut best = f32::MIN;
    let mut best_n = Vec3::Y; // 空面集（不该出现）时的占位
    for f in 0..poly.face_normal.len() {
        let n = poly.face_normal[f];
        let d = (p - poly.verts[poly.face_start[f] as usize]).dot(n);
        if d > best {
            best = d;
            best_n = n;
        }
    }
    (best, best_n)
}

/// **球心在体内 / 落在体表面上**的解析推出：返回 `(球→凸体方向法线, 深度, 接触点)`。
/// `depth = radius − max_d`（`max_d` ≤ 0 时越埋越深）；`depth ≤ 0`（球还没碰到）⇒ `None`。
pub(crate) fn sphere_exit(
    poly: &WorldPoly,
    center: Vec3,
    radius: f32,
) -> Option<(Vec3, f32, Vec3)> {
    let (max_d, face_n) = nearest_face_of(poly, center);
    let depth = radius - max_d;
    if depth <= 0.0 {
        return None;
    }
    // 接触点 = 球心在最近面平面上的**投影**（力臂 ∥ 法线 ⇒ 不产生自旋）。
    Some((-face_n, depth, center - face_n * max_d))
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use crate::tests::manifolds_for;
    use vxl_phys_core::{BodySet, Quat, Shape, Vec3};

    /// **球心落在盒顶面平面外侧一个浮点噪声量**（`y = +1e-8`；顶面在 `y = 0`）：
    /// 法线必须是**面法线**（竖直），不许是「噪声方向」—— 此时 `closest − center` 的 y 分量只有 1e-8，
    /// 方向被最近点重算的 x/z 舍入噪声（≈1e-7）**主导**。实测旧口径报 `n ≈ (−0.943, +0.106, −0.314)`
    /// （斜法线）⇒ 纯法向的顶出被塞进 `vx ≈ −2.6 m/s`（埋球恢复的侧滑，`tests/deep_penetration.rs` ⑤）。
    /// ⚠️ `y = 0.0` **恰好**时反而是对的（`d = 0` 不算"外侧" ⇒ 走体内分支）—— 缺口只在"恰好外侧"这一档。
    #[test]
    fn sphere_center_on_box_face_plane_keeps_face_normal() {
        let mut b = BodySet::new();
        b.push_static(
            Shape::Box {
                half: Vec3::new(2.0, 2.0, 2.0),
            },
            Vec3::new(0.0, -2.0, 0.0), // 顶面 y = 0
            Quat::IDENTITY,
        );
        b.push_dynamic(
            Shape::Sphere { radius: 0.45 },
            Vec3::new(0.4, 1e-8, -0.4), // 球心在顶面平面**外侧**一个噪声量
            Quat::IDENTITY,
            1000.0,
        );
        let out = manifolds_for(&b, &[]);
        assert_eq!(out.len(), 1, "应有 1 个流形（盒 × 球）");
        let m = &out[0];
        let want = if m.a == 0 { 1.0 } else { -1.0 }; // 法线约定 a→b
        assert!(
            m.normal.y * want > 0.999,
            "球心落在顶面平面外侧 ⇒ 法线必须是面法线（±Y），实得 {:?}（a={} b={}）",
            m.normal,
            m.a,
            m.b
        );
        let d = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!(
            (d - 0.45).abs() < 1e-3,
            "球心就在面平面上 ⇒ 深度应 ≈ 半径 0.45，实得 {d}"
        );
    }
}
