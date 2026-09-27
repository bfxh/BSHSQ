//! **三角网参与的对**（薄壳 × 静态/凸体）：`Shape::TriMesh` 的接触采样。
//!
//! 为什么单独一个文件：`pair_shaped.rs` / `support.rs` 都是**受 god 门棘轮管**的文件
//! （只准减）⇒ "还会继续长的采样代码"按 `provider.rs` 的先例另起一个域文件，那边只留分发。
//!
//! 口径（**先写清再实现**）：
//! - **非凸 ⇒ 不进 GJK/EPA**（`support_of` 如实返回 `None`）⇒ 一律走**逐顶点采样**；
//! - **顶点即样本**：一个顶点 = 一个候选接触点（与外壳的点云采样同构）；法线取**对方几何**在该点
//!   的外法线（提供者点查询/解析最近点给），深度取**沿该法线的穿透**（正 = 穿透，负 = 预期接触）；
//! - **流形法线 = a→b**：对方给的是"从它表面朝外"的法线 ⇒ 只当**对方是 a 侧**时才需要翻转
//!   （`sgn` 口径与 `provider_pair` 一致）；
//! - `feature` = **顶点序号 + 1**（顶点序稳定 ⇒ 跨帧可续接 warm 缓存；0 保留给"无特征"）。
//!
//! ⚠️ 本片（T1b）只接**提供者（体素/三角网/平面集/喷溅）**；凸体对（盒/球/外壳）与高度场
//! 在后续片，逐条登记在 `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md` 的矩阵表里（**不是静默**：
//! `crates/vxl-phys/tests/shape_support_matrix.rs` 的缺口金丝雀钉住现状）。
use super::*;

impl DefaultNarrowPhase {
    /// 填充**三角网的世界点缓存**（side：0 = 对侧 a、1 = 侧 b）。返回 `true` = 该形状是三角网
    /// 且缓存已就绪（顶点列在 `self.hull_pts[side]`）。
    ///
    /// **与 `fill_hull_world` 共用 `hull_pts` 缓冲**（不新增字段 ⇒ 不碰 god 门成员棘轮）：
    /// 一个体的 `shape` 只有一种 ⇒ 同一体不可能同时以两种来源填同一槽；键含
    /// `(体号, 姿态指纹)` ⇒ 同体同帧多对只做一次 O(n) 世界变换（与外壳同机制）。
    pub(crate) fn fill_mesh_world(
        &mut self,
        side: usize,
        body: u32,
        shape: &Shape,
        pos: Vec3,
        rot: Quat,
    ) -> bool {
        let Shape::TriMesh { mesh, .. } = *shape else {
            return false;
        };
        let fp = rot_fp(rot);
        if self.cached_hull[side] != (body, fp) {
            let m = Mat3::from_quat(rot);
            let out = &mut self.hull_pts[side];
            out.clear();
            for p in self.meshes.points(mesh) {
                out.push(pos + m.mul_vec3(*p));
            }
            self.cached_hull[side] = (body, fp);
        }
        true
    }

    /// **三角网 vs 提供者**（体素 / 三角网 / 平面集 / 喷溅）：**逐顶点**点查询。
    ///
    /// 与 `hull_provider_contacts` 同构（只换点源）：提供者的 `contacts_point` 给
    /// `(表面点, 外法线, 穿透深度)` ⇒ 压进 `buf`，由 `provider_pair` 做**主导面选择 + 取点**
    /// （那条路的 `feature % 16` 约定对顶点采样不适用，但其"无面心组 ⇒ 用全部组"的兜底覆盖了
    /// 这种情形 ⇒ 不产流形的风险由 `shape_support_matrix` 的落定判据守）。
    ///
    /// 返回值语义与外壳臂一致：`true` = **本通路受理**（含"网无顶点/全部不在带内"）。
    #[allow(clippy::too_many_arguments)] // 形状/位姿/提供者/出参 + 带符号朝向（与外壳臂同形）
    pub(crate) fn mesh_provider_contacts(
        &mut self,
        body_shape: &Shape,
        body: u32,
        bpos: Vec3,
        brot: Quat,
        id: u32,
        pr_is_a: bool,
        band: f32,
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        buf: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        let side = if pr_is_a { 1 } else { 0 };
        if !self.fill_mesh_world(side, body, body_shape, bpos, brot) {
            return true;
        }
        let mut supported = false;
        for k in 0..self.hull_pts[side].len() {
            supported |= providers.contacts_point(id, self.hull_pts[side][k], band, buf);
        }
        supported
    }
}
