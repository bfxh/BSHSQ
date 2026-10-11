//! **越界三角索引的入口处置**（安全审计 F-04）：`ClothSheet::new` 此前对索引 ≥ 点数的三角形
//! 直接索引 `points` ⇒ **构造期 panic**（`len is 3 but the index is 7`）；同仓窄相 `TriMesh::new`
//! 只跳过坏三角 ⇒ 两处口径不一致。本判据固化统一后的口径：**丢弃坏三角，不 panic**。
//!
//! 三条：① 坏三角不进 `tris` / 约束；② 好三角的拓扑**逐字不变**（丢弃是局部行为）；
//! ③ 只剩坏三角时退化成"无约束点集"（零应变、零力），不是 panic 也不是 NaN。

use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 四边形板（4 点、2 三角）——最小可分辨拓扑。
fn quad() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let pts = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 1.0),
    ];
    (pts, vec![[0, 1, 2], [1, 3, 2]])
}

#[test]
fn out_of_range_triangle_is_dropped_not_panicking() {
    let (pts, mut tris) = quad();
    tris.push([0, 1, 7]); // 索引 7 ≥ 点数 4：旧实现在此 panic
    let sheet = ClothSheet::new(pts, tris, 1.0, 1e-3, Stiffness::Hard);
    assert_eq!(sheet.tris.len(), 2, "坏三角不得留在拓扑里");
    // 两个好三角的唯一边：{0,1} {1,2} {0,2} {1,3} {2,3} = 5
    assert_eq!(sheet.edge_count(), 5, "好三角的唯一边集不受坏三角影响");
}

#[test]
fn all_bad_triangles_degrade_to_free_points() {
    let (pts, _) = quad();
    let tris = vec![[0, 1, 9], [4, 5, 6]];
    let mut sheet = ClothSheet::new(pts, tris, 1.0, 1e-3, Stiffness::Hard);
    assert_eq!(sheet.tris.len(), 0);
    assert_eq!(sheet.edge_count(), 0);
    // 无约束 ⇒ 全体零质量 ⇒ inv_mass 全 0（等同钉住）；哪怕真的全部动起来也不该出 NaN。
    assert!(sheet.inv_mass.iter().all(|m| *m == 0.0));
    for _ in 0..30 {
        sheet.step(
            1.0 / 60.0,
            Vec3::new(0.0, -9.8, 0.0),
            &vxl_phys_core::interop::NoProviders,
            0,
            &[],
        );
    }
    assert!(sheet
        .pos
        .iter()
        .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()));
}

#[test]
fn bad_index_boundary_is_not_off_by_one() {
    let (pts, _) = quad();
    // 索引恰好 = 点数 - 1（合法）与 = 点数（非法）各一个。
    let tris = vec![[0, 1, 3], [2, 3, 4]];
    let sheet = ClothSheet::new(pts, tris, 1.0, 1e-3, Stiffness::Hard);
    assert_eq!(sheet.tris.len(), 1, "n-1 合法、n 非法");
    assert_eq!(sheet.tris[0], [0, 1, 3]);
}
