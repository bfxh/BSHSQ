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

    /// **三角网 × 凸体**（T1b-2；只接**盒 / 球**）：逐顶点**解析**最近点采样。
    ///
    /// 其余形状（胶囊/圆柱/圆锥/外壳/复合体/另一个三角网）**如实不受理**（直接返回、不产接触），
    /// 逐条登记在 `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md`（**不是静默**：由金丝雀判据钉住现状）。
    ///
    /// **法线口径**（本片唯一容易搞反的地方，从求解器定义反推）：求解器把 `+n·λ` 给 b、`−n·λ` 给 a，
    /// 而 `n_o` = "把顶点推出去"的方向（对方在该点的外法线）⇒
    /// **三角网在 a 侧 ⇒ `n(a→b) = −n_o`；在 b 侧 ⇒ `n(a→b) = +n_o`**。
    ///
    /// **取点**：两遍——先取**最深候选所属的法线**作主导面（并列取先出现者 ⇒ 确定性；点-盒的角点
    /// 会同时落在两张面上），再过 `select_contacts` 去重；一条流形最多 4 点（槽数所致）。
    #[allow(clippy::too_many_arguments)] // 两侧体号/形状/位姿 + 出参（与 `process_pair` 同形）
    pub(crate) fn mesh_pair(
        &mut self,
        a: u32,
        b: u32,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        let mesh_is_a = matches!(*sa, Shape::TriMesh { .. });
        let (mesh_shape, mpos, mrot, opos, orot, oshape) = if mesh_is_a {
            (sa, pa, ra, pb, rb, sb)
        } else {
            (sb, pb, rb, pa, ra, sa)
        };
        // 受理面：只接盒 / 球（其余 = 不受理 ⇒ 本对不产接触）
        if !matches!(*oshape, Shape::Box { .. } | Shape::Sphere { .. }) {
            return;
        }
        let side = if mesh_is_a { 0 } else { 1 };
        let body = if mesh_is_a { a } else { b };
        if !self.fill_mesh_world(side, body, mesh_shape, mpos, mrot) {
            return;
        }
        let n_pts = self.hull_pts[side].len();
        // ① 主导面 = 最深候选的法线
        let mut dom: Option<(Vec3, f32)> = None;
        for k in 0..n_pts {
            let Some((n_o, depth, _)) = point_shape(self.hull_pts[side][k], oshape, opos, orot)
            else {
                continue;
            };
            if depth > -self.skin && dom.is_none_or(|(_, d)| depth > d) {
                dom = Some((n_o, depth));
            }
        }
        let Some((n_dom, _)) = dom else {
            return;
        };
        // ② 同面候选（点-盒角点可能同时落在两张面上 ⇒ 只留主导面，免得多法线混进同一流形）
        self.cand.clear();
        for k in 0..n_pts {
            let Some((n_o, depth, hit)) = point_shape(self.hull_pts[side][k], oshape, opos, orot)
            else {
                continue;
            };
            if depth > -self.skin && n_o.dot(n_dom) > 0.9 {
                self.cand.push(ContactPoint {
                    point: hit,
                    depth,
                    feature: k as u32 + 1, // 顶点序稳定 ⇒ 跨帧可续接 warm
                });
            }
        }
        if self.cand.is_empty() || !self.select_contacts(self.min_point_sep) {
            return;
        }
        out.push(Manifold {
            a,
            b,
            normal: if mesh_is_a { -n_dom } else { n_dom },
            points: ContactPoints::from_slice(&self.cand),
        });
    }

    /// **三角网 × 高度场**：逐顶点采样（与 `hull_heightfield` **同构**，只换点源）。
    ///
    /// 逐个**顶点**取世界点 → `hf.sample(x, z)` ⇒ `depth = h − v.y`（正 = 顶点在地形之下 = 穿透），
    /// 接触点落在地形面上（`(x, h, z)`）；法线与 sign 由调用方 `heightfield_pair` 按 a/b 侧决定
    /// （那里是**形状无关**的：取最深样本的地形法线）⇒ 本函数只负责**填 `self.cand`**。
    pub(crate) fn mesh_heightfield(
        &mut self,
        mesh: u32,
        pos: Vec3,
        rot: Quat,
        hf: &HeightField,
    ) -> bool {
        self.cand.clear();
        let r = Mat3::from_quat(rot);
        let pts = self.meshes.points(mesh);
        for (idx, &p) in pts.iter().enumerate() {
            let v = pos + r.mul_vec3(p);
            if let Some((h, _)) = hf.sample(v.x, v.z) {
                let depth = h - v.y;
                if depth > -self.skin {
                    self.cand.push(ContactPoint {
                        point: Vec3::new(v.x, h, v.z),
                        depth,
                        feature: idx as u32 + 1,
                    });
                }
            }
        }
        if self.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }
}

/// 点 × {盒, 球} 的**解析最近点**：返回 `(对方外法线 n_o, 深度, 对方表面上的接触点)`。
///
/// 深度口径与全仓一致（`SPEC §4.3`）：**正 = 穿透**、负 = 分离（分离时 `n_o` 仍指"把点推出去"）。
/// `None` = 该形状不在本片受理面内（**不猜**：调用方按"不受理"处理）。
fn point_shape(p: Vec3, shape: &Shape, c: Vec3, rot: Quat) -> Option<(Vec3, f32, Vec3)> {
    match *shape {
        Shape::Sphere { radius } => {
            let d = p - c;
            let len = d.length();
            if len <= 1e-6 {
                return Some((Vec3::Y, radius, c + Vec3::Y * radius)); // 球心重合：任取轴向（确定性）
            }
            let n_o = d * (1.0 / len);
            Some((n_o, radius - len, c + n_o * radius))
        }
        Shape::Box { half } => {
            let r = Mat3::from_quat(rot);
            let q = r.transpose_mul_vec3(p - c); // 局部坐标
            let gap = Vec3::new(half.x - q.x.abs(), half.y - q.y.abs(), half.z - q.z.abs());
            // 最近面 = 间隙最小的轴（并列按 x → y → z ⇒ 确定性）；盒外时 `gap` 有负分量
            let (axis, min_gap, comp) = if gap.x <= gap.y && gap.x <= gap.z {
                (Vec3::X, gap.x, q.x)
            } else if gap.y <= gap.z {
                (Vec3::Y, gap.y, q.y)
            } else {
                (Vec3::Z, gap.z, q.z)
            };
            if min_gap >= 0.0 {
                // 点在盒内：穿透深度 = 到最近面的距离；外法线 = 该轴（按点在这侧定符号）
                let n_l = if comp < 0.0 { -axis } else { axis };
                let n_o = r.mul_vec3(n_l);
                let hit = c + r.mul_vec3(q - n_l * min_gap); // 投到该面
                return Some((n_o, min_gap, hit));
            }
            // 点在盒外：最近点 = 逐轴夹取
            let k = Vec3::new(
                q.x.clamp(-half.x, half.x),
                q.y.clamp(-half.y, half.y),
                q.z.clamp(-half.z, half.z),
            );
            let delta = q - k;
            let len = delta.length();
            let n_o = if len > 1e-6 {
                r.mul_vec3(delta * (1.0 / len))
            } else {
                r.mul_vec3(axis)
            };
            Some((n_o, -len, c + r.mul_vec3(k)))
        }
        _ => None,
    }
}
