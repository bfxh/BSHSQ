//! world_mesh：从 lib.rs 按域拆出（三角网域：**动态薄壳**的注册与生成）。
//!
//! 与 `world_build.rs` 的 [`World::add_mesh`]（**静态**提供者）分工写清：
//! - `add_mesh`：地形/关卡几何 ⇒ 进 `Providers`（静态 marker 体，接触走提供者通道）；
//! - 本文件：**动态体的一等几何**（薄壳：布片/薄板/碎片）⇒ 进窄相 `MeshStore`，
//!   由 `Shape::TriMesh { mesh, half }` 引用，带位姿、有质量、进宽相/窄相。
use super::*;

impl World {
    /// 注册**三角网**（局部坐标顶点 + 三角）→ mesh id；配 [`Self::spawn_trimesh_body`] 使用。
    ///
    /// 注册期丢弃非法三角（越界 / 重复顶点 / 零面积）⇒ 面积积分与面法线不会在退化处出 NaN
    /// （口径见窄相 `MeshStore::add`）。
    pub fn add_trimesh(&mut self, points: Vec<Vec3>, tris: Vec<[u32; 3]>) -> u32 {
        self.narrow.add_mesh(points, tris)
    }

    /// 三角网顶点（局部；渲染/转储用）。
    pub fn trimesh_points(&self, mesh: u32) -> &[Vec3] {
        self.narrow.mesh_points(mesh)
    }

    /// 三角网三角（局部）。
    pub fn trimesh_tris(&self, mesh: u32) -> &[[u32; 3]] {
        self.narrow.mesh_tris(mesh)
    }

    /// 三角网表面积（`Σ A_tri`；判据/渲染用）。
    pub fn trimesh_area(&self, mesh: u32) -> f32 {
        self.narrow.mesh_surface_area(mesh)
    }

    /// 生成**三角网动态体**（薄壳）：顶点/三角已在 [`Self::add_trimesh`] 注册；`thickness` = 壳厚（m）。
    ///
    /// 半长取**顶点局部 AABB**（宽相保守）；质量/惯量按**薄壳**覆写
    /// （`m = ρ·t·ΣA` + 薄板面内二阶矩，口径见窄相 `MeshStore::shell_props`）——
    /// `push_dynamic` 内部只有 AABB 盒兜底近似。退化网（无有效三角）保留兜底 ⇒ 质量 0
    /// （等效静态），与空复合体同款。
    ///
    /// ⚠️ **已知缺口（T1b 关闭）**：三角网 × 任何形状**暂不产生接触**（逐顶点采样未落地）——
    /// 半长/质量/惯量已是可用的一等几何，但"撞得上"要等 T1b。该缺口由
    /// `crates/vxl-phys/tests/shape_support_matrix.rs` **钉住**（不是静默）。
    pub fn spawn_trimesh_body(
        &mut self,
        mesh: u32,
        pos: Vec3,
        rot: Quat,
        density: f32,
        thickness: f32,
    ) -> u32 {
        let half = self.narrow.mesh_half_extents(mesh);
        let density = density.max(1e-3);
        let id = self
            .bodies
            .push_dynamic(Shape::TriMesh { mesh, half }, pos, rot, density);
        let i = id as usize;
        if let Some((inv_mass, inv_inertia)) =
            self.narrow.mesh_shell_props(mesh, density, thickness)
        {
            self.bodies.inv_mass[i] = inv_mass;
            self.bodies.local_inv_inertia[i] = inv_inertia;
        }
        id
    }
}
