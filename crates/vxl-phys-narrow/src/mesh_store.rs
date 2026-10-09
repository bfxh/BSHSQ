//! **三角网仓库**（薄壳：布片 / 薄板 / 毁坏碎片）——`Shape::TriMesh { mesh, half }` 的句柄后端。
//!
//! 表示法与 `HullStore`（点云）/`CompoundStore`（子形状表）同款：几何本体在窄相自持 ⇒
//! `vxl-phys-core` 保持**零依赖**、`Shape` 保持 `Copy`（`bodies.shape[i]` 的按值 match 是承重前提）。
//!
//! ⚠️ **三角网非凸** ⇒ 不进 GJK/EPA（`support.rs::support_of` 如实返回 `None`，是语义决定）；
//! 接触走**逐顶点采样**。逐域支持矩阵（受理 / 已知缺口）见 `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md`。
use super::*;

/// 一张三角网（**局部坐标**）：顶点 + 三角（索引指向顶点）+ 顶点 AABB 半长（宽相用）。
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub points: Vec<Vec3>,
    /// 三角（索引三元组）。**注册期已过滤非法项**（越界 / 重复顶点 / 零面积）。
    pub tris: Vec<[u32; 3]>,
    /// 顶点局部 AABB 半长（空网 = ZERO）。
    pub half: Vec3,
}

/// **三角网仓库**：注册后由 `Shape::TriMesh { mesh, .. }` 按 id 引用（id = 注册序 ⇒ 确定性）。
#[derive(Clone, Default)]
pub struct MeshStore {
    pub(crate) meshes: Vec<Mesh>,
}

impl MeshStore {
    /// 注册一张三角网（局部坐标）；返回 id。
    ///
    /// **注册期丢弃非法三角**（索引越界 / 重复顶点 / 零面积）：它们对接触与质量都无贡献，
    /// 留着只会让面积积分与面法线在退化处出 NaN。过滤按原序 ⇒ 确定性。
    pub fn add(&mut self, points: Vec<Vec3>, tris: Vec<[u32; 3]>) -> u32 {
        let tris = keep_legal_tris(&points, tris);
        let half = points_half(&points);
        let id = self.meshes.len() as u32;
        self.meshes.push(Mesh { points, tris, half });
        id
    }

    #[inline]
    pub fn get(&self, id: u32) -> Option<&Mesh> {
        self.meshes.get(id as usize)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.meshes.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.meshes.is_empty()
    }

    /// 顶点（局部；空切片 = id 无效）。
    pub fn points(&self, id: u32) -> &[Vec3] {
        self.get(id).map(|m| m.points.as_slice()).unwrap_or(&[])
    }

    /// 三角（局部；空切片 = id 无效）。
    pub fn tris(&self, id: u32) -> &[[u32; 3]] {
        self.get(id).map(|m| m.tris.as_slice()).unwrap_or(&[])
    }

    /// 顶点局部 AABB 半长（宽相 / 惯量兜底用；空网 = ZERO）。
    pub fn half_extents(&self, id: u32) -> Vec3 {
        self.get(id).map(|m| m.half).unwrap_or(Vec3::ZERO)
    }

    /// 表面积 `Σ A_tri`（判据仪器：薄壳质量 = `ρ·t·A`）。
    pub fn surface_area(&self, id: u32) -> f32 {
        let Some(m) = self.get(id) else {
            return 0.0;
        };
        let mut area = 0.0f32;
        for tri in &m.tris {
            area += tri_area(
                m.points[tri[0] as usize],
                m.points[tri[1] as usize],
                m.points[tri[2] as usize],
            );
        }
        area
    }

    /// **薄壳质量属性**：`(inv_mass, local_inv_inertia)`（对角；与 `core::mass::MassProps` 同表示）。
    ///
    /// `m = ρ·t·ΣA`（均匀薄壳**精确**）；惯量 = 薄板**面内**二阶矩，关于**体原点**、**离对角项丢弃**
    /// —— 与 `compound_mass_props` 同款两条取舍；`t²/12` 项忽略（薄壳 `t ≪ 边长`）。
    ///
    /// 非法输入按既有惯例兜底（`density ≤ 0` ⇒ 1.0，同 `core::mass`；`thickness ≤ 0` ⇒ 1e-3）。
    /// **无有效三角**（面积 0）⇒ `None`（调用方保留 core 的 AABB 兜底，与空复合体同款）。
    pub fn shell_props(&self, id: u32, density: f32, thickness: f32) -> Option<(f32, Vec3)> {
        let m = self.get(id)?;
        let rho = if density > 0.0 { density } else { 1.0 };
        let t = if thickness > 0.0 { thickness } else { 1e-3 };
        let mut area = 0.0f32;
        let mut j_diag = Vec3::ZERO;
        let mut j_trace = 0.0f32;
        for tri in &m.tris {
            let a = m.points[tri[0] as usize];
            let b = m.points[tri[1] as usize];
            let c = m.points[tri[2] as usize];
            let ar = tri_area(a, b, c);
            let (diag, tr) = tri_area_moment(a, b, c, ar);
            area += ar;
            j_diag += diag;
            j_trace += tr;
        }
        let mass = rho * t * area;
        if mass <= 0.0 || !mass.is_finite() {
            return None;
        }
        // I = ρt·(tr J·E − J) ⇒ 对角分量 = ρt·(tr J − J_ii)。
        let k = rho * t;
        let i_diag = Vec3::new(
            k * (j_trace - j_diag.x),
            k * (j_trace - j_diag.y),
            k * (j_trace - j_diag.z),
        );
        let inv = |v: f32| if v > 0.0 { 1.0 / v } else { 0.0 };
        Some((
            1.0 / mass,
            Vec3::new(inv(i_diag.x), inv(i_diag.y), inv(i_diag.z)),
        ))
    }
}

impl DefaultNarrowPhase {
    /// 注册**三角网**（顶点 + 三角，局部坐标）→ mesh id；配 `Shape::TriMesh { mesh, .. }` 使用。
    pub fn add_mesh(&mut self, points: Vec<Vec3>, tris: Vec<[u32; 3]>) -> u32 {
        std::sync::Arc::make_mut(&mut self.meshes).add(points, tris)
    }

    /// 三角网顶点（局部坐标；空切片 = id 无效）。
    pub fn mesh_points(&self, id: u32) -> &[Vec3] {
        self.meshes.points(id)
    }

    /// 三角网三角（局部坐标；空切片 = id 无效）。
    pub fn mesh_tris(&self, id: u32) -> &[[u32; 3]] {
        self.meshes.tris(id)
    }

    /// 顶点局部 AABB 半长（宽相 / 惯量兜底用；空网返回 ZERO）。
    pub fn mesh_half_extents(&self, id: u32) -> Vec3 {
        self.meshes.half_extents(id)
    }

    /// 三角网表面积（判据/渲染用）。
    pub fn mesh_surface_area(&self, id: u32) -> f32 {
        self.meshes.surface_area(id)
    }

    /// **薄壳质量属性**（口径见 `MeshStore::shell_props`）。
    pub fn mesh_shell_props(&self, id: u32, density: f32, thickness: f32) -> Option<(f32, Vec3)> {
        self.meshes.shell_props(id, density, thickness)
    }
}

/// 三角形面积 `0.5·|(b−a)×(c−a)|`。
fn tri_area(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    (b - a).cross(c - a).length() * 0.5
}

/// 三角形的**面积二阶矩**（关于**原点**、局部坐标）：返回 `(diag J, tr J)`，`J = ∫_T x xᵀ dA`。
///
/// 参数化 `x = a + u·e1 + v·e2`（`e1 = b−a`、`e2 = c−a`；`u,v ≥ 0`、`u+v ≤ 1`；`dA = 2A·du·dv`），
/// 用 `∫∫1 = 1/2`、`∫∫u = ∫∫v = 1/6`、`∫∫u² = ∫∫v² = 1/12`、`∫∫uv = 1/24` 得
/// `J = 2A·[a aᵀ/2 + (a e1ᵀ + e1 aᵀ)/6 + (a e2ᵀ + e2 aᵀ)/6 + e1 e1ᵀ/12 + e2 e2ᵀ/12 + (e1 e2ᵀ + e2 e1ᵀ)/24]`
/// ⇒ 对角 `J_ii = 2A·[a_i²/2 + (a_i·e1_i + a_i·e2_i)/3 + (e1_i² + e2_i² + e1_i·e2_i)/12]`；
/// 迹 `tr J = 2A·[a·a/2 + (a·e1 + a·e2)/3 + (e1·e1 + e2·e2 + e1·e2)/12]`。
///
/// ⚠️ **`a aᵀ` 前的 1/2 别漏**（首版漏了）：漏它只在**三角形起点恰在原点**时看不出来
/// ⇒ 判据必须用**平移过**的三角（`shell_props_match_closed_form_for_flat_plate` 就是靠这条抓的）。
///
/// 平面区域的三角剖分下，逐三角求和**等于**该区域的连续积分（同一被积函数、区域恰好被覆盖）
/// ⇒ 判据可与解析式**精确对拍**（误差只剩 f32 舍入）。
fn tri_area_moment(a: Vec3, b: Vec3, c: Vec3, area: f32) -> (Vec3, f32) {
    let e1 = b - a;
    let e2 = c - a;
    let k = 2.0 * area;
    let pair = a.mul_per_elem(e1) + a.mul_per_elem(e2);
    let self_dot = e1.mul_per_elem(e1) + e2.mul_per_elem(e2) + e1.mul_per_elem(e2);
    let diag = (a.mul_per_elem(a) * 0.5 + pair * (1.0 / 3.0) + self_dot * (1.0 / 12.0)) * k;
    let tr = k
        * (a.dot(a) * 0.5
            + (a.dot(e1) + a.dot(e2)) / 3.0
            + (e1.dot(e1) + e2.dot(e2) + e1.dot(e2)) / 12.0);
    (diag, tr)
}

/// 顶点集局部 AABB 半长（空集 ⇒ ZERO）。
fn points_half(points: &[Vec3]) -> Vec3 {
    let Some(first) = points.first() else {
        return Vec3::ZERO;
    };
    let (mut lo, mut hi) = (*first, *first);
    for p in &points[1..] {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    (hi - lo) * 0.5
}

/// 过滤非法三角（索引越界 / 重复顶点 / 零面积）；按原序保留合法项（确定性）。
fn keep_legal_tris(points: &[Vec3], tris: Vec<[u32; 3]>) -> Vec<[u32; 3]> {
    let n = points.len() as u32;
    tris.into_iter()
        .filter(|t| {
            let in_range = t.iter().all(|&i| i < n);
            let distinct = t[0] != t[1] && t[1] != t[2] && t[0] != t[2];
            if !(in_range && distinct) {
                return false;
            }
            let (a, b, c) = (
                points[t[0] as usize],
                points[t[1] as usize],
                points[t[2] as usize],
            );
            (b - a).cross(c - a).length_squared() > 0.0
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 平铺网格（`n×n` 格、跨度 `±size`、落在 **y = 0 平面**）：`(顶点, 三角)`。
    fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
        let mut pts = Vec::new();
        let mut tris = Vec::new();
        for iz in 0..=n {
            for ix in 0..=n {
                let s = 2.0 * size / n as f32;
                pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
            }
        }
        for iz in 0..n as u32 {
            for ix in 0..n as u32 {
                let a = iz * (n as u32 + 1) + ix;
                let (c, d) = (a + 1, a + n as u32 + 1);
                tris.push([a, d, c]);
                tris.push([c, d, d + 1]);
            }
        }
        (pts, tris)
    }

    /// **薄壳判据（解析对拍）**：`±1` 方板（面积 4、`y ≡ 0`）的解析解
    /// `m = ρ·t·A`、`I = (m/3, 2m/3, m/3)`（绕 x 面内轴 = `ρt·∫z²`、绕 y 法向 = `ρt·∫(x²+z²)`、绕 z 同 x）。
    ///
    /// 为什么可以**精确**对拍：平面区域的三角剖分下，逐三角求和 = 该区域连续积分
    /// （同一被积函数、区域恰好被覆盖）⇒ 误差只剩 f32 舍入（不用放离散化容差）。
    #[test]
    fn shell_props_match_closed_form_for_flat_plate() {
        let (pts, tris) = plate(2, 1.0);
        let mut store = MeshStore::default();
        let id = store.add(pts, tris);
        let (rho, t) = (7.0f32, 0.02f32);
        assert!((store.surface_area(id) - 4.0).abs() < 1e-5, "面积应为 4");
        let (inv_mass, inv_i) = store.shell_props(id, rho, t).expect("非退化网");
        let m = rho * t * 4.0;
        assert!((1.0 / inv_mass - m).abs() / m < 1e-5, "m = ρtA");
        for (got, want, axis) in [
            (1.0 / inv_i.x, m / 3.0, "x（面内）"),
            (1.0 / inv_i.y, 2.0 * m / 3.0, "y（法向）"),
            (1.0 / inv_i.z, m / 3.0, "z（面内）"),
        ] {
            assert!(
                (got - want).abs() / want < 1e-5,
                "I_{axis} = {got} ≠ {want}"
            );
        }
    }

    /// **零厚板极限**（`t → 0`）与**法向轴最大**：薄壳绕法向轴最难转（`I_yy > I_xx`）。
    /// 同时钉住"非法厚度兜底"（`t ≤ 0` ⇒ 1e-3，与 `density ≤ 0` 同款惯例）。
    #[test]
    fn shell_props_extremes_and_fallbacks() {
        let (pts, tris) = plate(2, 1.0);
        let mut store = MeshStore::default();
        let id = store.add(pts, tris);
        let thin = store.shell_props(id, 1.0, 0.0).expect("兜底厚度");
        let legal = store.shell_props(id, 1.0, 1e-3).expect("正常");
        assert!(
            (1.0 / thin.0 - 1.0 / legal.0).abs() < 1e-9,
            "t ≤ 0 应兜底成 1e-3"
        );
        let (_, i) = store.shell_props(id, 1000.0, 0.01).expect("正常");
        assert!(i.y < i.x, "法向轴惯量最大 ⇒ 逆惯量最小");
        assert!((i.x - i.z).abs() < 1e-6 * i.x, "x/z 对称（不要求逐位相等）");
    }

    /// **注册期过滤**：索引越界 / 重复顶点 / 零面积三角被丢弃（按原序保留合法项）；
    /// **空网 / 全退化 ⇒ `shell_props` 返回 `None`**（调用方保留 core 的 AABB 兜底）。
    #[test]
    fn invalid_tris_are_dropped_and_degenerate_mesh_has_no_props() {
        let (mut pts, tris) = plate(1, 1.0);
        let mid = pts.len() as u32;
        pts.push(Vec3::new(0.0, 0.0, -1.0)); // 落在 0/1 连线上 ⇒ 与 [0,1,mid] 共线（零面积）
        let mut bad = tris.clone();
        bad.push([0, 1, 99]); // 越界
        bad.push([0, 0, 1]); // 重复顶点
        bad.push([0, 1, mid]); // 零面积（共线）
        let mut store = MeshStore::default();
        let id = store.add(pts, bad);
        assert_eq!(store.tris(id).len(), tris.len(), "非法三角应被丢弃");
        assert!(store.shell_props(id, 1.0, 0.01).is_some());

        let empty = store.add(Vec::new(), tris);
        assert_eq!(store.half_extents(empty), Vec3::ZERO);
        assert!(store.shell_props(empty, 1.0, 0.01).is_none());
    }
}
