//! surface：体素占据位图 → 三角网格的**表面提取**（surface nets 最小面）。
//!
//! 这是「体素 ↔ 多边形转换」四件里的第 ① 件（`docs/PLAN-CONVERSION.md` §3.1，
//! 2026-10-02 拍板 P1 = surface nets 起步）——**复制式**：只读位图、不消耗占据格
//! （与挖格提取 `extract_*` 一族的消耗语义相反），输出可直接喂窄相 `MeshStore::add` /
//! 门面 `add_trimesh` / `spawn_trimesh_body`。
//!
//! 场与样点：样点 = **格中心**处的 [`VoxelVolume::sdf`]。占据格中心恰为 −半格、
//! 紧邻空格中心恰为 +半格（`voxel.rs::sdf_signs_and_surface` 的符号锚）⇒ 零交点
//! 恰落在占据/空格的**共享面中心**上，内部平面精确重建格边界平面；盒的棱/角处有
//! 半格级倒角（分辨率本性，体积/面积判据按此设阈）。样点格点阵比占据网格外扩
//! 1 格**幽灵环**（幽灵样点恒正，只作符号上下文、不产几何）。
//!
//! 算法（surface nets）：① 每个「8 样点既有负又有正」的样点立方产一个顶点 =
//! 其被穿越棱上零交点的平均；② 每条被穿越样点棱把环绕它的 ≤4 个活跃立方的顶点
//! 按环绕序连成面（缺环退化成三角 / 不足三个跳过）；③ 朝向 = 面法线与
//! 「占据侧 → 空侧」的棱方向同侧 ⇒ 法线指向空侧（外）。
//!
//! 确定性：样点/立方/棱全按固定下标序扫描；顶点是固定序的有限次加平（无归约序
//! 问题）；输出路径无哈希迭代。环绕棱的 4 个立方下标必在界内：穿越棱必有**占据
//! 端点**（幽灵样点恒正），占据格下标 ∈ [0, n)，据此各偏移分量 ≥ 0 且 ≤ 上界
//! （幽灵—幽灵棱不穿越，进不了环绕分支）。区域裁剪留转换窗口（V2）一并设计——
//! 裁剪面要不要封口是窗口语义的一部分（PLAN-CONVERSION §3.2），本函数只做
//! **整个体积**。

use super::voxel_volume::VoxelVolume;
use vxl_phys_core::Vec3;

/// 表面提取结果：顶点 + 三角（顶点索引；三角法线指向**空侧**）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceMesh {
    /// 顶点（世界系；序 = 立方扫描序）。
    pub points: Vec<Vec3>,
    /// 三角（索引指向 [`SurfaceMesh::points`]）。
    pub tris: Vec<[u32; 3]>,
}

/// 样点立方的 12 条棱（样点偏移对；固定序 ⇒ 确定性）。
const CUBE_EDGES: [([u32; 3], [u32; 3]); 12] = [
    ([0, 0, 0], [1, 0, 0]),
    ([0, 1, 0], [1, 1, 0]),
    ([0, 0, 1], [1, 0, 1]),
    ([0, 1, 1], [1, 1, 1]),
    ([0, 0, 0], [0, 1, 0]),
    ([1, 0, 0], [1, 1, 0]),
    ([0, 0, 1], [0, 1, 1]),
    ([1, 0, 1], [1, 1, 1]),
    ([0, 0, 0], [0, 0, 1]),
    ([1, 0, 0], [1, 0, 1]),
    ([0, 1, 0], [0, 1, 1]),
    ([1, 1, 0], [1, 1, 1]),
];

/// 采样场：格中心样点的 sdf 值 + 格点阵尺寸（样点下标 s ↔ 格下标 s−1）。
struct Field {
    f: Vec<f32>,
    snx: usize,
    sny: usize,
    snz: usize,
    org: Vec3,
    h: f32,
}

impl Field {
    /// 样点世界坐标（格中心；幽灵环样点在网格外半格步长处）。
    fn pos(&self, i: usize, j: usize, k: usize) -> Vec3 {
        Vec3::new(
            self.org.x + (i as f32 - 0.5) * self.h,
            self.org.y + (j as f32 - 0.5) * self.h,
            self.org.z + (k as f32 - 0.5) * self.h,
        )
    }

    /// 样点值。
    fn val(&self, i: usize, j: usize, k: usize) -> f32 {
        self.f[i + self.snx * (j + self.sny * k)]
    }
}

/// 采整个体积的格中心场（外扩 1 格幽灵环；样点序 = 固定 (k, j, i) 扫描序）。
fn sample_field(vol: &VoxelVolume) -> Field {
    let (nx, ny, nz) = vol.dims();
    let snx = (nx + 2) as usize;
    let sny = (ny + 2) as usize;
    let snz = (nz + 2) as usize;
    let mut field = Field {
        f: vec![0f32; snx * sny * snz],
        snx,
        sny,
        snz,
        org: vol.origin(),
        h: vol.step(),
    };
    for k in 0..snz {
        for j in 0..sny {
            for i in 0..snx {
                field.f[i + snx * (j + sny * k)] = vol.sdf(field.pos(i, j, k));
            }
        }
    }
    field
}

/// ① 顶点腿：逐样点立方放顶点（被穿越棱零交点的平均）+ 立方→顶点索引表。
fn vertex_pass(field: &Field) -> (Vec<Vec3>, Vec<i32>) {
    let (ncx, ncy, ncz) = (field.snx - 1, field.sny - 1, field.snz - 1);
    let mut vid = vec![-1i32; ncx * ncy * ncz];
    let mut points: Vec<Vec3> = Vec::new();
    for ck in 0..ncz {
        for cj in 0..ncy {
            for ci in 0..ncx {
                let mut has_neg = false;
                let mut has_pos = false;
                for dk in 0..2usize {
                    for dj in 0..2usize {
                        for di in 0..2usize {
                            let v = field.val(ci + di, cj + dj, ck + dk);
                            has_neg |= v < 0.0;
                            has_pos |= v > 0.0;
                        }
                    }
                }
                if !(has_neg && has_pos) {
                    continue;
                }
                // 12 棱零交点平均（固定序；穿越棱必有零交点）
                let mut acc = Vec3::ZERO;
                let mut cnt = 0u32;
                for (a, b) in CUBE_EDGES {
                    let fa = field.val(ci + a[0] as usize, cj + a[1] as usize, ck + a[2] as usize);
                    let fb = field.val(ci + b[0] as usize, cj + b[1] as usize, ck + b[2] as usize);
                    if (fa < 0.0) == (fb < 0.0) {
                        continue;
                    }
                    let t = fa / (fa - fb);
                    let pa = field.pos(ci + a[0] as usize, cj + a[1] as usize, ck + a[2] as usize);
                    let pb = field.pos(ci + b[0] as usize, cj + b[1] as usize, ck + b[2] as usize);
                    acc += pa + (pb - pa) * t;
                    cnt += 1;
                }
                let cidx = ci + ncx * (cj + ncy * ck);
                vid[cidx] = points.len() as i32;
                points.push(acc * (1.0 / cnt as f32));
            }
        }
    }
    (points, vid)
}

/// ②③ 连面腿：每条被穿越样点棱，把环绕立方的顶点按环绕序连面
/// （朝向 = 面法线与「占据侧 → 空侧」同侧；缺环退化成三角 / 不足三个跳过）。
fn face_pass(field: &Field, points: &[Vec3], vid: &[i32]) -> Vec<[u32; 3]> {
    let (ncx, ncy, _) = (field.snx - 1, field.sny - 1, field.snz - 1);
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let mut skipped_flat = 0usize;
    let axis_vec = [Vec3::X, Vec3::Y, Vec3::Z];
    let area_min = 1e-4 * field.h * field.h * field.h * field.h; // 退化四边形闸
    for k in 0..field.snz {
        for j in 0..field.sny {
            for i in 0..field.snx {
                for (axis, &axis_dir) in axis_vec.iter().enumerate() {
                    let (di, dj, dk) = match axis {
                        0 => (1usize, 0, 0),
                        1 => (0, 1, 0),
                        _ => (0, 0, 1),
                    };
                    let (i2, j2, k2) = (i + di, j + dj, k + dk);
                    if i2 >= field.snx || j2 >= field.sny || k2 >= field.snz {
                        continue;
                    }
                    let fa = field.val(i, j, k);
                    let fb = field.val(i2, j2, k2);
                    // 外向 = 从占据样点指向空样点（幽灵恒正 ⇒ 两侧至少一端占据）
                    let outward = match (fa < 0.0, fb < 0.0) {
                        (true, false) => axis_dir,
                        (false, true) => -axis_dir,
                        _ => continue,
                    };
                    // 环绕棱的 4 个立方（环绕序固定；界内性见模块注释）
                    let ring: [(usize, usize, usize); 4] = match axis {
                        // x 棱：立方 (i, j−1+ay, k−1+az)
                        0 => [(i, j - 1, k - 1), (i, j, k - 1), (i, j, k), (i, j - 1, k)],
                        // y 棱：立方 (i−1+ax, j, k−1+az)
                        1 => [(i - 1, j, k - 1), (i, j, k - 1), (i, j, k), (i - 1, j, k)],
                        // z 棱：立方 (i−1+ax, j−1+ay, k)
                        _ => [(i - 1, j - 1, k), (i, j - 1, k), (i, j, k), (i - 1, j, k)],
                    };
                    let mut vs = [0u32; 4];
                    let mut n_vs = 0usize;
                    for (cx, cy, cz) in ring {
                        let id = vid[cx + ncx * (cy + ncy * cz)];
                        if id >= 0 {
                            vs[n_vs] = id as u32;
                            n_vs += 1;
                        }
                    }
                    if n_vs < 3 {
                        continue;
                    }
                    let (p0, p1, p2) = (
                        points[vs[0] as usize],
                        points[vs[1] as usize],
                        points[vs[2] as usize],
                    );
                    let nrm = (p1 - p0).cross(p2 - p0);
                    if nrm.length_squared() <= area_min {
                        skipped_flat += 1;
                        continue;
                    }
                    let forward = nrm.dot(outward) >= 0.0;
                    if n_vs == 4 {
                        let q: [u32; 4] = if forward {
                            [vs[0], vs[1], vs[2], vs[3]]
                        } else {
                            [vs[0], vs[3], vs[2], vs[1]]
                        };
                        tris.push([q[0], q[1], q[2]]);
                        tris.push([q[0], q[2], q[3]]);
                    } else {
                        let q: [u32; 3] = if forward {
                            [vs[0], vs[1], vs[2]]
                        } else {
                            [vs[0], vs[2], vs[1]]
                        };
                        tris.push(q);
                    }
                }
            }
        }
    }
    debug_assert_eq!(skipped_flat, 0, "本实现的测试形状不应产退化四边形");
    tris
}

/// 从占据位图提取**整个体积**的零等值面（三角法线指向空侧）。
pub fn surface_mesh(vol: &VoxelVolume) -> SurfaceMesh {
    let field = sample_field(vol);
    let (points, vid) = vertex_pass(&field);
    let tris = face_pass(&field, &points, &vid);
    SurfaceMesh { points, tris }
}

#[cfg(test)]
mod tests {
    use super::{surface_mesh, SurfaceMesh};
    use crate::voxel::VoxelVolume;
    use std::collections::HashMap;
    use vxl_phys_core::Vec3;

    fn floor() -> VoxelVolume {
        // 与 voxel.rs 判据同源的地板：8×2×8、边长 0.5、顶面 y = 1.0
        let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
        v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
        v
    }

    fn sphere_blob() -> VoxelVolume {
        // 16³ 格、边长 0.25：格心距原点 ≤ 1.5 的填充（解析球 r=1.5）
        let mut v = VoxelVolume::new(Vec3::new(-2.0, -2.0, -2.0), 0.25, 16, 16, 16);
        for iz in 0..16u32 {
            for iy in 0..16u32 {
                for ix in 0..16u32 {
                    let c = v.grid_center(ix, iy, iz) - Vec3::ZERO;
                    if c.length() <= 1.5 {
                        v.set(ix, iy, iz, true);
                    }
                }
            }
        }
        v
    }

    /// 无向棱流形计数（封闭 ⇔ 每条棱恰两张三角）。
    fn edge_counts(tris: &[[u32; 3]]) -> HashMap<(u32, u32), u32> {
        let mut m: HashMap<(u32, u32), u32> = HashMap::new();
        for t in tris {
            for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if e.0 < e.1 { e } else { (e.1, e.0) };
                *m.entry(key).or_insert(0) += 1;
            }
        }
        m
    }

    fn assert_closed(tris: &[[u32; 3]]) {
        for (e, c) in edge_counts(tris) {
            assert_eq!(c, 2, "棱 {e:?} 出现 {c} 次（封闭流形要求恰 2）");
        }
    }

    /// 每张三角的法线应指向空侧（sdf 沿法线增大）。
    fn assert_outward(vol: &VoxelVolume, mesh: &SurfaceMesh) {
        let eps = vol.step() * 0.125;
        for t in &mesh.tris {
            let a = mesh.points[t[0] as usize];
            let b = mesh.points[t[1] as usize];
            let c = mesh.points[t[2] as usize];
            let n = (b - a).cross(c - a);
            let dir = n * (1.0 / n.length());
            let ctr = (a + b + c) * (1.0 / 3.0);
            let out = vol.sdf(ctr + dir * eps);
            let inn = vol.sdf(ctr - dir * eps);
            assert!(out > inn, "三角法线应指向空侧：out={out} inn={inn}");
        }
    }

    /// 三角合法性（即窄相 MeshStore 的丢弃判据：界内 / 无重复顶点 / 非零面积）。
    fn assert_legal(mesh: &SurfaceMesh) {
        let n = mesh.points.len() as u32;
        for t in &mesh.tris {
            assert!(t[0] < n && t[1] < n && t[2] < n, "索引越界：{t:?}");
            assert!(
                t[0] != t[1] && t[1] != t[2] && t[0] != t[2],
                "重复顶点：{t:?}"
            );
            let a = mesh.points[t[0] as usize];
            let b = mesh.points[t[1] as usize];
            let c = mesh.points[t[2] as usize];
            assert!((b - a).cross(c - a).length_squared() > 0.0, "零面积：{t:?}");
        }
    }

    /// 散度定理体积（一致朝向 ⇒ 正）。
    fn mesh_volume(mesh: &SurfaceMesh) -> f32 {
        let mut acc = 0.0f32;
        for t in &mesh.tris {
            let a = mesh.points[t[0] as usize];
            let b = mesh.points[t[1] as usize];
            let c = mesh.points[t[2] as usize];
            acc += a.dot(b.cross(c));
        }
        acc / 6.0
    }

    fn mesh_area(mesh: &SurfaceMesh) -> f32 {
        let mut acc = 0.0f32;
        for t in &mesh.tris {
            let a = mesh.points[t[0] as usize];
            let b = mesh.points[t[1] as usize];
            let c = mesh.points[t[2] as usize];
            acc += (b - a).cross(c - a).length() * 0.5;
        }
        acc
    }

    #[test]
    pub(crate) fn floor_box_closed_exact_planes_and_conserved() {
        let vol = floor();
        let mesh = surface_mesh(&vol);
        assert!(!mesh.tris.is_empty(), "地板应提出非空网格");
        assert_closed(&mesh.tris);
        assert_outward(&vol, &mesh);
        assert_legal(&mesh);
        // 零等值面一致性（I1，先量后写）：首测 max|sdf(v)| = h/3（盒/球两形同值，
        // 2026-10-02）⇒ 钉半格带（= 倒角拉离量的上界口径）
        let band = vol.step() * 0.5;
        for p in &mesh.points {
            assert!(vol.sdf(*p).abs() <= band, "|sdf| 超带：p={p:?}");
        }
        // 内部顶面**精确**落在 y = 1.0（非倒角区；棱角倒角顶点 y ≤ 0.875 < 0.9）
        for p in &mesh.points {
            if p.y > 0.9 {
                assert_eq!(p.y, 1.0, "内部顶面顶点应精确在 y=1.0：p={p:?}");
            }
        }
        // 守恒（I2，先量后写）：首测 V=13.984（缺口 2.016）、A=40.177（2026-10-02）。
        // 缺口 = surface nets 对二值场的棱/角倒角（特征线处顶点被拉离盒棱 ~h/4 级）；
        // 本地板 128 格**全是表面格** ⇒ PLAN I2 先验上界 = 0.5·h³·128 = 8 m³，实测 2.0。
        let v = mesh_volume(&mesh);
        let a = mesh_area(&mesh);
        assert!(
            (v - 16.0).abs() <= 2.5 && (a - 40.0).abs() <= 1.0,
            "体积 {v} 应在 16±2.5（首测 13.984）、面积 {a} 应在 40±1（首测 40.177）"
        );
    }

    #[test]
    pub(crate) fn extraction_is_bitwise_deterministic() {
        let vol = floor();
        let m1 = surface_mesh(&vol);
        let m2 = surface_mesh(&vol);
        assert_eq!(m1, m2, "两次提取必须逐位相同");
        let blob = sphere_blob();
        assert_eq!(surface_mesh(&blob), surface_mesh(&blob));
    }

    #[test]
    pub(crate) fn sphere_blob_closed_and_plausible() {
        let vol = sphere_blob();
        let mesh = surface_mesh(&vol);
        assert_closed(&mesh.tris);
        assert_outward(&vol, &mesh);
        assert_legal(&mesh);
        // 解析对照：r=1.5 球 V≈14.137、A≈28.274；占据近似 + 表面带 ⇒ 宽容差
        let v = mesh_volume(&mesh);
        let a = mesh_area(&mesh);
        assert!(
            (v - 14.137).abs() < 2.0 && (a - 28.274).abs() < 8.0,
            "球团体积 {v} 应≈14.1、面积 {a} 应≈28.3"
        );
    }

    #[test]
    pub(crate) fn dug_tunnel_surface_stays_closed() {
        // 挖穿地板的竖井（同 voxel.rs 的 dug_hole 判据场景）：表面仍须封闭
        let mut vol = floor();
        vol.set(4, 0, 4, false);
        vol.set(4, 1, 4, false);
        let mesh = surface_mesh(&vol);
        assert_closed(&mesh.tris);
        assert_outward(&vol, &mesh);
        // 与未挖版不同（井壁必须产几何）
        let plain = surface_mesh(&floor());
        assert_ne!(mesh.tris.len(), plain.tris.len());
    }
}
