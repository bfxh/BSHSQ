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

    /// **三角网 × 凸体**（受理 **盒 / 球 / 胶囊 / 圆柱 / 锥**；T1b-2 起分片扩到五族）：
    /// 逐顶点**解析**最近点采样。
    ///
    /// **外壳 / 另一个三角网如实不受理**（要面数据 ⇒ T2 领地）；**复合体无需此处受理**——
    /// 它在 `process_pair_shaped` 最前部就展开成子对，子形状各走本函数。逐条登记在
    /// `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md`（**不是静默**：由矩阵判据钉住现状）。
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
        // 受理面：盒 / 球 / 胶囊 / 圆柱 / 锥（其余 = 不受理 ⇒ 本对不产接触）
        if !matches!(
            *oshape,
            Shape::Box { .. }
                | Shape::Sphere { .. }
                | Shape::Capsule { .. }
                | Shape::Cylinder { .. }
                | Shape::Cone { .. }
        ) {
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

    /// **外壳 × 三角网**（T2 第一片）：**外壳顶点** × **网面三角形**（解析点-三角）。
    ///
    /// 分发在 `support.rs::hull_pair`（外壳对在那里被截走，进不了 `pair_non_heightfield`）；
    /// 外壳世界点已由调用方填好（`hull_pts[hull_side]`），本函数再填**网顶点**世界缓存
    /// （`fill_mesh_world`，另一侧槽 ⇒ 不冲突）。采样方向只有一路：**外壳顶点查网面**
    /// ——外壳是凸的点云、没有"面"可供反向查询（点-凸最近点要走 GJK 距离，另立片）；
    /// 反作用由求解器经同一条流形回给两体 ⇒ 双向耦合不丢。
    ///
    /// **法线口径**：取**网面三角的环绕法线**（`plate` 类网格 = +Y 朝上）——顶点从环绕侧压入
    /// ⇒ `depth > 0`、`n_o` = 环绕法线（把外壳顶点推出去）⇒ 流形 a→b 按外壳在哪侧定号
    /// （与 `mesh_pair` 同款）。⚠️ **单面**语义：从反面压入也算穿透、沿同一法线推出
    /// （薄壳无厚度，"哪面朝外"由环绕序决定；双面/厚度口径属 T3 自碰撞那片）。
    ///
    /// **成本**：暴力 O(V_壳 × T_网)（石块 8 点 × 8 三角 = 64 对/帧，探针档无压力）；
    /// 大网大壳要空间加速（与 T3 自碰撞的空间哈希同族，另立片）。
    #[allow(clippy::too_many_arguments)] // 两侧体号/形状/位姿 + 接触带 + 出参（与 `process_pair` 同形）
    pub(crate) fn hull_vs_mesh(
        &mut self,
        a: u32,
        b: u32,
        hull_is_a: bool,
        hull_side: usize,
        mesh: u32,
        mesh_body: u32,
        mesh_shape: &Shape,
        mesh_pos: Vec3,
        mesh_rot: Quat,
        band: f32,
        out: &mut Vec<Manifold>,
    ) {
        let mesh_side = 1 - hull_side;
        if !self.fill_mesh_world(mesh_side, mesh_body, mesh_shape, mesh_pos, mesh_rot) {
            return;
        }
        let n_hull = self.hull_pts[hull_side].len();
        let n_tris = self.meshes.tris(mesh).len();
        if n_hull == 0 || n_tris == 0 {
            return;
        }
        // ① 主导面 = 最深候选的法线（平面薄网上全部三角同法线 ⇒ 实际只有一组）
        let mut dom: Option<(Vec3, f32)> = None;
        for hi in 0..n_hull {
            let p = self.hull_pts[hull_side][hi];
            for ti in 0..n_tris {
                let tri = self.meshes.tris(mesh)[ti];
                let (w0, w1, w2) = (
                    self.hull_pts[mesh_side][tri[0] as usize],
                    self.hull_pts[mesh_side][tri[1] as usize],
                    self.hull_pts[mesh_side][tri[2] as usize],
                );
                let Some((n_o, depth, _)) = point_triangle(p, w0, w1, w2) else {
                    continue;
                };
                if depth > -band && dom.is_none_or(|(_, d)| depth > d) {
                    dom = Some((n_o, depth));
                }
            }
        }
        let Some((n_dom, _)) = dom else {
            return;
        };
        // ② 同面候选（顶点可能落在网格对角线等共享边上 ⇒ 邻三角法线相同，取其一即可）
        self.cand.clear();
        for hi in 0..n_hull {
            let p = self.hull_pts[hull_side][hi];
            for ti in 0..n_tris {
                let tri = self.meshes.tris(mesh)[ti];
                let (w0, w1, w2) = (
                    self.hull_pts[mesh_side][tri[0] as usize],
                    self.hull_pts[mesh_side][tri[1] as usize],
                    self.hull_pts[mesh_side][tri[2] as usize],
                );
                let Some((n_o, depth, hit)) = point_triangle(p, w0, w1, w2) else {
                    continue;
                };
                if depth > -band && n_o.dot(n_dom) > 0.9 {
                    self.cand.push(ContactPoint {
                        point: hit,
                        depth,
                        feature: hi as u32 + 1, // 外壳顶点序稳定 ⇒ 跨帧可续接 warm
                    });
                }
            }
        }
        if self.cand.is_empty() || !self.select_contacts(self.min_point_sep) {
            return;
        }
        out.push(Manifold {
            a,
            b,
            normal: if hull_is_a { -n_dom } else { n_dom },
            points: ContactPoints::from_slice(&self.cand),
        });
    }
}

/// **点 × 三角**的解析最近点（Ericson《Real-Time Collision Detection》`ClosestPtPointTriangle`）：
/// 返回 `(环绕法线 n_o, 深度, 三角上的最近点)`。
///
/// 深度 = **沿环绕法线的平面距离取负**（正 = 点在环绕背面 = 穿透）；平面上的投影落在
/// 三角**内部**时这是精确距离，落在**边/顶点区**时是近似（真距离是到边/点的距离）——
/// 平面薄网（全部三角共面）下两种区域给出的深度口径一致 ⇒ 判据不受影响。
/// `None` = 退化三角（注册期已滤，此处兜底）。
fn point_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<(Vec3, f32, Vec3)> {
    let vn = (b - a).cross(c - a);
    let len2 = vn.length_squared();
    if len2 <= 1e-12 {
        return None;
    }
    let n = vn * (1.0 / len2.sqrt());
    let depth = -n.dot(p - a); // 正 = 点在环绕背面
                               // —— 最近点（Voronoi 区判别；区分子式见 Ericson 5.1.5）——
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    let hit = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                a + ab * (d1 / (d1 - d3))
            } else {
                let cp = p - c;
                let d5 = ab.dot(cp);
                let d6 = ac.dot(cp);
                if d6 >= 0.0 && d5 <= d6 {
                    c
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        a + ac * (d2 / (d2 - d6))
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))
                        } else {
                            let denom = 1.0 / (va + vb + vc);
                            let v = vb * denom;
                            let w = vc * denom;
                            a + ab * v + ac * w
                        }
                    }
                }
            }
        }
    };
    Some((n, depth, hit))
}

/// 点 × {盒, 球, 胶囊, 圆柱, 锥} 的**解析最近点**：返回 `(对方外法线 n_o, 深度, 对方表面上的接触点)`。
///
/// 深度口径与全仓一致（`SPEC §4.3`）：**正 = 穿透**、负 = 分离（分离时 `n_o` 仍指"把点推出去"）。
/// `None` = 该形状不在本片受理面内（**不猜**：调用方按"不受理"处理）。外壳 / 复合体 /
/// 另一个三角网仍未受理（要面数据 ⇒ 另立片；**复合体 × 三角网无需本函数**——复合体在窄相
/// 最前部就展开成子对，子形状走本函数，见支持矩阵）。
///
/// 注：圆柱/胶囊/锥走**解析面**（真圆、真柱/锥面）——与本仓其它路径对这三族用的**多面化**近似
/// 不同，对本片（顶点采样）反而是更准的接触面。
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
        // 胶囊 = "线段 + 半径" ⇒ 先求轴段最近点，再按球处理（与 `vxl-phys-soft::rigid` 同款）。
        Shape::Capsule {
            half_height,
            radius,
        } => {
            let axis = Mat3::from_quat(rot).mul_vec3(Vec3::Y);
            let t = (p - c).dot(axis).clamp(-half_height, half_height);
            let q = c + axis * t;
            let d = p - q;
            let len = d.length();
            if len <= 1e-6 {
                return Some((Vec3::Y, radius, q + Vec3::Y * radius));
            }
            let n_o = d * (1.0 / len);
            Some((n_o, radius - len, q + n_o * radius))
        }
        // 有限圆柱（局部 +Y）：侧面 / 端面 / **边圈** 三分支（含柱内"最近面"选择）。
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let r3 = Mat3::from_quat(rot);
            let q = r3.transpose_mul_vec3(p - c); // 局部坐标
            let rho = (q.x * q.x + q.z * q.z).sqrt();
            let ay = q.y.abs();
            // 径向单位向量（局部；落在轴上时任取 +X ⇒ 确定性）
            let rdir = if rho > 1e-6 {
                Vec3::new(q.x / rho, 0.0, q.z / rho)
            } else {
                Vec3::X
            };
            let side = |y: f32| rdir * radius + Vec3::Y * y; // 侧面上的点
            let cap = |s: f32| Vec3::new(q.x, s * half_height, q.z); // 端面上的点
            let (n_l, depth, surf) = if rho <= radius && ay <= half_height {
                // 柱内：最近的是侧面还是端面（并列取侧面 ⇒ 确定性）
                let (d_side, d_cap) = (radius - rho, half_height - ay);
                if d_side <= d_cap {
                    (rdir, d_side, side(q.y))
                } else {
                    let s = if q.y < 0.0 { -1.0 } else { 1.0 };
                    (Vec3::Y * s, d_cap, cap(s))
                }
            } else if rho > radius && ay <= half_height {
                (rdir, radius - rho, side(q.y)) // 侧面外侧（depth ≤ 0）
            } else if rho <= radius {
                let s = if q.y < 0.0 { -1.0 } else { 1.0 };
                (Vec3::Y * s, half_height - ay, cap(s)) // 端面外侧
            } else {
                // 侧面外侧 + 端面外 ⇒ 最近点是**边圈**
                let s = if q.y < 0.0 { -1.0 } else { 1.0 };
                let rim = Vec3::new(rdir.x * radius, s * half_height, rdir.z * radius);
                let d = q - rim;
                let len = d.length();
                let n_l = if len > 1e-6 { d * (1.0 / len) } else { rdir };
                (n_l, -len, rim)
            };
            Some((r3.mul_vec3(n_l), depth, c + r3.mul_vec3(surf)))
        }
        // 有限圆锥（局部 +Y：底面 `y = −h` 半径 `r`，顶点 `y = +h`）：**解析锥面**。
        // 口径与圆柱同款——本仓其它路径把锥**多面化**（16 边棱锥）进 GJK/EPA，这里用真锥面
        // （对顶点采样更准，不引入棱面误差）。做法：把查询点投到**过轴的 2D 剖面** `(ρ, y)`
        // （ρ = 径向距），锥的剖面是三角形（顶点 (0,h)、底圈 (r,−h)、轴底 (0,−h)），
        // 最近点只会在**侧边**或**底边**上（轴边不是表面；"顶点正上方"退化为同一解）。
        Shape::Cone {
            half_height,
            radius,
        } => {
            let r3 = Mat3::from_quat(rot);
            let q = r3.transpose_mul_vec3(p - c);
            let rho = (q.x * q.x + q.z * q.z).sqrt();
            let rdir = if rho > 1e-6 {
                Vec3::new(q.x / rho, 0.0, q.z / rho)
            } else {
                Vec3::X
            };
            // 剖面几何：侧边 P0=(0,h)→P1=(r,−h)，方向 d=(r,−2h)、|d|² = r²+4h²；外法线 (2h,r)/|d|。
            let h2 = 2.0 * half_height;
            let len2 = radius * radius + h2 * h2;
            let len = len2.sqrt();
            // 侧边最近点（t ∈ [0,1]）与法线：光滑段用解析外法线；夹到**顶点/底圈**（退化特征，
            // 无唯一法线）用"表面点 → 查询点"方向（并列/退化都取确定分支）。
            let t = ((rho * radius + (q.y - half_height) * (-h2)) / len2).clamp(0.0, 1.0);
            let side_pt = rdir * (t * radius) + Vec3::Y * (half_height - h2 * t);
            let side_n = if t > 0.0 && t < 1.0 {
                (rdir * h2 + Vec3::Y * radius) * (1.0 / len)
            } else {
                let d = q - side_pt;
                if d.length_squared() > 1e-12 {
                    d.normalize()
                } else {
                    rdir
                }
            };
            // 底边最近点：(min(ρ, r), −h)；法线 −Y。
            let base_pt = rdir * rho.min(radius) + Vec3::Y * (-half_height);
            let s_side = h2 * rho + radius * (q.y - half_height); // ×(1/|d|) 即带符号距离
            let (n_l, depth, surf) = if s_side <= 0.0 && q.y >= -half_height {
                // 锥内：侧面 / 底面取近（并列取侧面 ⇒ 确定性）
                let d_side = -s_side / len;
                let d_base = q.y + half_height;
                if d_side <= d_base {
                    (side_n, d_side, side_pt)
                } else {
                    (-Vec3::Y, d_base, base_pt)
                }
            } else {
                // 锥外：两个候选取近者（并列取侧边 ⇒ 确定性）；深度取负 = 分离
                let dy_s = q.y - (half_height - h2 * t);
                let d2_side = (rho - t * radius) * (rho - t * radius) + dy_s * dy_s;
                let dr_b = rho - rho.min(radius);
                let d2_base = dr_b * dr_b + (q.y + half_height) * (q.y + half_height);
                if d2_side <= d2_base {
                    (side_n, -d2_side.sqrt(), side_pt)
                } else {
                    (-Vec3::Y, -d2_base.sqrt(), base_pt)
                }
            };
            Some((r3.mul_vec3(n_l), depth, c + r3.mul_vec3(surf)))
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
