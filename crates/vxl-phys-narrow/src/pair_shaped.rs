//! pair_shaped：从 lib.rs 按域拆出（纯搬移，语义未改）。
//!
//! `process_pair_shaped` 是形状对的总分发：按"复合体展开 → 提供者 → 外壳 → 高度场 →
//! 通用 match"分段编排，各段各一 helper（纯搬移，`return` 语义逐条保持——段内 `return;`
//! 原来就是"整对结束"，段 helper 改成返回 `true` 由调用方 `return`）。
use super::*;

// 胶囊族拆成**子模块**（理由见 `pair_shaped/capsule.rs` 头注：`narrow/lib.rs` 只剩 1 行预算，
// crate 级新文件要加 `mod` 声明；子模块还能看见父模块的私有项）。
#[path = "pair_shaped/capsule.rs"]
mod capsule;

/// **速度自适应接触带**（`provider` 通道口径；`OPEN-PROBLEMS.md` #6 把它一并带给地形）。
///
/// `band = max(skin, |v_rel|·dt·1.5 + skin)`，`dt` = **固定基础步 1/60**（子步更小 ⇒
/// 该带偏保守、安全）。由来（2026-09-15 实测修复）：固定 `skin`（0.02 m）小于每 tick
/// 位移（8 m/s ⇒ 0.13 m）时，体**跨过皮肤带** ⇒ 进入体内才建接触，此时接近速度已≈0
/// ⇒ 冲击判据不触发、且无 spec 减速（实测：8 m/s 弹体无声停在墙前 0.04 m、零破坏）。
///
/// 三处共用：`provider_pair`、`support.rs::hull_vs_mesh`、以及**直连高度场**
/// （`heightfield_pair` 把结果存进 `ws.inflate`，高度场族采样读 `skin + inflate`）。
/// 放**模块级**而非 `DefaultNarrowPhase` 的方法：类型账对它的方法数已登记为「只准减」。
pub(crate) fn velocity_band(skin: f32, vrel: Vec3) -> f32 {
    skin.max(vrel.length() * (1.0 / 60.0) * 1.5 + skin)
}

impl DefaultNarrowPhase {
    /// **配对主入口**（形状 + 世界位姿已给定）：复合体在此按子形状展开并**递归**本函数。
    #[allow(clippy::too_many_arguments)] // 两侧形状 + 位姿 + 上下文 + 出参
    pub(crate) fn process_pair_shaped(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        heightfields: &[HeightField],
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
    ) {
        // 盒对专用路径开关：每对先复位（非盒对 / 圆柱对一律走通用路径）。
        self.ws.box_axes_a = None;
        self.ws.box_axes_b = None;

        // —— 复合体：子形状展开为**子对**并递归本函数 ——
        //
        // 放在**最前**（先于地形/提供者分支）⇒ 子形状各自走完整配对路径（含地形）。
        // 同一体对会**同时**存在多条流形（每个子形状各一条）：`feature` 是暖启动缓存键的
        // 一部分，不并入子序号会让不同子形状的接触点互相顶替（`0` 是"无特征"哨兵，保持 0）。
        if let Shape::Compound { compound, .. } = *sa {
            let Some(kids) = self.kids_take(compound) else {
                return;
            };
            for (ci, kid) in kids.iter().enumerate() {
                let before = out.len();
                let cpos = pa + Mat3::from_quat(ra).mul_vec3(kid.offset);
                let crot = ra * kid.rot;
                self.process_pair_shaped(
                    a,
                    b,
                    bodies,
                    &kid.shape,
                    sb,
                    cpos,
                    crot,
                    pb,
                    rb,
                    heightfields,
                    providers,
                    out,
                );
                tag_child_features(&mut out[before..], ci);
            }
            self.kids_put(kids);
            return;
        }
        if let Shape::Compound { compound, .. } = *sb {
            let Some(kids) = self.kids_take(compound) else {
                return;
            };
            for (ci, kid) in kids.iter().enumerate() {
                let before = out.len();
                let cpos = pb + Mat3::from_quat(rb).mul_vec3(kid.offset);
                let crot = rb * kid.rot;
                self.process_pair_shaped(
                    a,
                    b,
                    bodies,
                    sa,
                    &kid.shape,
                    pa,
                    ra,
                    cpos,
                    crot,
                    heightfields,
                    providers,
                    out,
                );
                tag_child_features(&mut out[before..], ci);
            }
            self.kids_put(kids);
            return;
        }

        // 提供者参与的对（见 `provider_pair`）。
        if self.provider_pair(a, b, bodies, sa, sb, pa, ra, pb, rb, providers, out) {
            return;
        }

        // **凸体外壳参与的对**（多边形域）：外壳 × {盒|球|外壳} → GJK/EPA。
        // 与提供者的组合已在上面的 provider 分支处理；与高度场暂不受理。
        if matches!(*sa, Shape::ConvexHull { .. }) || matches!(*sb, Shape::ConvexHull { .. }) {
            self.hull_pair(a, b, bodies, sa, sb, pa, ra, pb, rb, heightfields, out);
            return;
        }

        // 高度场参与的对（见 `heightfield_pair`）。
        if self.heightfield_pair(a, b, bodies, sa, sb, pa, ra, pb, rb, heightfields, out) {
            return;
        }

        // 非 heightfield 对（见 `pair_non_heightfield` 的各臂 helper）。
        self.pair_non_heightfield(a, b, bodies, sa, sb, pa, ra, pb, rb, out);
    }

    /// **外部碰撞提供者参与的对**（体素/网格/喷溅场…；ROUTE §2.1 兼容轴）。
    /// 受理的形状面：`Box` / `Sphere` / `ConvexHull` / `Capsule`（其余形状待专用解法，
    /// 见 `provider.rs::provider_shape_contacts`）；
    /// 法线约定与高度场一致：提供者在 a → +n_s（外向）；在 b → −n_s。
    ///
    /// 返回 `true` = 本段已处理（**进入本段后所有路径都结束整对**，与原实现一致）。
    #[allow(clippy::too_many_arguments)]
    fn provider_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        out: &mut Vec<Manifold>,
    ) -> bool {
        let pr_a = match sa {
            Shape::Provider(id) => Some(*id),
            _ => None,
        };
        let pr_b = match sb {
            Shape::Provider(id) => Some(*id),
            _ => None,
        };
        if pr_a.is_some() || pr_b.is_some() {
            if pr_a.is_some() && pr_b.is_some() {
                return true; // provider-provider 暂不支持（需要 provider 对偶解法）
            }
            let (body_shape, bpos, brot, pr_is_a) = if pr_a.is_some() {
                (sb, pb, rb, true)
            } else {
                (sa, pa, ra, false)
            };
            let id = pr_a.or(pr_b).unwrap();
            let mut buf: Vec<vxl_phys_core::interop::InteropContact> = Vec::new();
            // **接触带按相对速度自适应**（实测修复）：固定 skin（0.02 m）小于每 tick
            // 位移（8 m/s ⇒ 0.13 m）时，体**跨过皮肤带** ⇒ 进入体内才建接触，此时
            // 接近速度已≈0 ⇒ 冲击判据不触发、且无 spec 减速（实测：8 m/s 弹体无声
            // 停在墙前 0.04 m、零破坏）。带 = max(skin, |v_rel|·dt·1.5)；dt 取引擎
            // 固定基础步 1/60（子步更小 ⇒ 该带偏保守、安全）。
            let vrel = if pr_is_a {
                bodies.linvel[b as usize] - bodies.linvel[a as usize]
            } else {
                bodies.linvel[a as usize] - bodies.linvel[b as usize]
            };
            // **+ skin 余量**（2026-09-15 修复）：带恰等于「样点到表面距离」时
            // 接触的出现与否只在速度上差 0.5%（实测基准 wall_provider：2 子步下
            // 0.1398 vs 0.14 的刀锋 ⇒ 墙面接触整整晚一个子步出现，期间水平动量
            // 被倾斜的地板法线吸收、撞墙事件不登记）。加一个 skin 把刀锋推开，
            // 使「带内即建（预测）接触」在速度上连续；预测接触由求解器按
            // distance/dt 限接近速度，不会造成假制动。
            let band = velocity_band(self.skin, vrel);
            // 体形状 → 候选接触：**分发与采样在 `provider.rs`**（本文件受尺寸棘轮，
            // 只准减不许胖 ⇒ 会继续长的采样代码不放这里）。受理面见该文件。
            let ok = self.provider_shape_contacts(
                body_shape,
                if pr_is_a { b } else { a },
                bpos,
                brot,
                id,
                pr_is_a,
                band,
                providers,
                &mut buf,
            );
            if !ok || buf.is_empty() {
                return true; // 不支持 / 全部顶点都不在接触带内
            }
            let sgn = if pr_is_a { 1.0 } else { -1.0 };
            // 流形法线 = 多面候选里选**主导接触面**。候选来自 provider 的
            // **逐面发射**（每张面各自发带内样本；共享角点会在相邻面里重复出现
            // ——只有「面心样本」（feature % 16 == 0）能证明该面真的贴着）。
            // 选择次序（2026-09-15 修复，取代旧的"点数最多、并列取最深"单一规则）：
            //   ① 有**闭合速度**的面（法线逆着相对速度 = 正在撞上去）优先，取最大者；
            //   ② 否则只考虑**带面心样本**的组（排除只有角点的"伪面"）；
            //   ③ 组内仍按点数最多、并列取最深。
            // 实测：旧的单一规则在墙角按深度选中**地板面**、丢掉墙面 ⇒ 体的水平
            // 动量被倾斜地板法线吸收、撞墙事件不登记（bench `wall_provider`：
            // 12 m/s 弹体停在墙前 0.14 m、vx 11.4→−0.02）。
            let Some((best_key, best_normal)) = pick_dominant_normal(&buf, vrel, sgn) else {
                return true;
            };
            let normal = best_normal;
            // **选点：四角优先，面心补位**（2026-09-22，P10 修复）。
            // 提供者每面发 5 点：`feature % 16 == 0` = **面心**，1..4 = **四角**；而流形只有
            // 4 槽。旧的 `take(4)` 按**生成序**取 ⇒ 恰好丢掉**第 4 个角** ⇒ 接触力偶不对称
            // ⇒ 每 tick 注入净力矩 ⇒ **静置单盒持续自旋**（实测 `|ω| ≈ 1.6 rad/s`、永不如入睡；
            // 见 `OPEN-PROBLEMS.md` P10 的逐 tick 轨迹：补丁恰为「中心 + 三角」、缺 (+x,+z)）。
            // ⇒ 角点定义力偶、面心只是冗余：**先把角点取满**，面心仅在角点不足时补位。
            // 稳定性：`sort_by_key` 是稳定排序 ⇒ 同类内保持生成序（确定性不变）。
            let pts = four_corner_points(&buf, best_key);
            out.push(Manifold {
                a,
                b,
                normal,
                points: ContactPoints::from_slice(&pts),
            });
            return true;
        }
        false
    }

    /// 高度场参与的对（地形法线取最深接触的采样法线）。返回 `true` = 本段已处理。
    #[allow(clippy::too_many_arguments)]
    fn heightfield_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        heightfields: &[HeightField],
        out: &mut Vec<Manifold>,
    ) -> bool {
        let hf_a = match sa {
            Shape::HeightField(id) => Some(*id as usize),
            _ => None,
        };
        let hf_b = match sb {
            Shape::HeightField(id) => Some(*id as usize),
            _ => None,
        };
        if hf_a.is_some() && hf_b.is_some() {
            return true;
        }
        if hf_a.is_some() || hf_b.is_some() {
            let (body_shape, bpos, brot, hf_is_a) = if hf_a.is_some() {
                (sb, pb, rb, true)
            } else {
                (sa, pa, ra, false)
            };
            let hf = match heightfields.get(hf_a.or(hf_b).unwrap()) {
                Some(h) => h,
                None => return true,
            };
            // **速度自适应接触带**（`OPEN-PROBLEMS.md` #6）：直连高度场路径原先**恒用**
            // 固定 `self.skin`，而 provider 通道早已按 `velocity_band` 把带加宽
            // （`heightfield.rs` 的 `CollisionProvider` 实现只是把这个带当 `skin` 参数
            // 收下 ⇒ 两条路对同一对应当给出同一条带）。这里补上同一条口径。
            //
            // 存进 `ws.inflate`（本对 scratch：`collide` 每帧先清零、各路径各自置位）
            // ⇒ 高度场族三个采样函数统一读 `self.skin + self.ws.inflate`。
            let vrel = bodies.linvel[b as usize] - bodies.linvel[a as usize];
            self.ws.inflate = velocity_band(self.skin, vrel) - self.skin;
            let ok = match *body_shape {
                Shape::Sphere { radius } => self.sphere_heightfield(bpos, radius, hf),
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. } => {
                    let idx = match self.poly_for(body_shape) {
                        Some(i) => i,
                        None => return true,
                    };
                    self.poly_heightfield(idx, bpos, brot, hf)
                }
                // 三角网 × 高度场：**逐顶点采样**（`mesh_pair.rs::mesh_heightfield`；法线与 sign
                // 由本函数的"最深样本地形法线"那段形状无关地处理）。
                Shape::TriMesh { mesh, .. } => self.mesh_heightfield(mesh, bpos, brot, hf),
                // 下列各臂都不可达：复合体已在上面展开；外壳 × 高度场由 `hull_pair` 先行受理；
                // 高度场 × 提供者由 `provider_pair` 先行受理（原 `hull_heightfield` 死代码已删）。
                Shape::Compound { .. } | Shape::HeightField(_) => return true,
                Shape::ConvexHull { .. } | Shape::Provider(_) => return true,
                // 胶囊体 × 地形：沿中心线取 N 个样本（每个样本按球处理）。
                Shape::Capsule {
                    half_height,
                    radius,
                } => {
                    let axis = Mat3::from_quat(brot).mul_vec3(Vec3::Y);
                    self.capsule_heightfield(
                        bpos - axis * half_height,
                        bpos + axis * half_height,
                        radius,
                        hf,
                    )
                }
            };
            if !ok {
                return true;
            }
            // 地形法线（取最深接触的采样法线）。
            let deepest = self.ws.cand[0];
            let n_t = hf
                .sample(deepest.point.x, deepest.point.z)
                .map(|(_, n)| n)
                .unwrap_or(Vec3::Y);
            // 流形法线 a→b：a=地形 → +n_t（推向 b）；b=地形 → -n_t。
            let normal = if hf_is_a { n_t } else { -n_t };
            out.push(Manifold {
                a,
                b,
                normal,
                points: ContactPoints::from_slice(&self.ws.cand),
            });
            return true;
        }
        false
    }

    /// 非 heightfield 对的形状分发（各臂见对应 helper）。
    #[allow(clippy::too_many_arguments)]
    fn pair_non_heightfield(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        // 三角网 × 其它：**逐顶点采样**（非凸 ⇒ 不进 GJK/EPA；实现在 `mesh_pair.rs`，
        // 受理面 = 盒/球/胶囊/圆柱/锥 + **另一个三角网**（T2 续，双面口径），其余如实不受理）。
        if matches!(*sa, Shape::TriMesh { .. }) || matches!(*sb, Shape::TriMesh { .. }) {
            self.mesh_dispatch(a, b, sa, sb, pa, ra, pb, rb, out);
            return;
        }
        match (*sa, *sb) {
            (Shape::Sphere { radius: ra_ }, Shape::Sphere { radius: rb_ }) => {
                sphere_sphere(a, b, pa, ra_, pb, rb_, out);
            }
            (Shape::Sphere { radius }, convex) => {
                self.sphere_convex_pair(a, b, pa, radius, &convex, pb, rb, out);
            }
            (convex, Shape::Sphere { radius }) => {
                // 球在 b：convex = a。用球-凸路径后翻转法线。
                self.convex_sphere_pair(a, b, pa, ra, &convex, pb, radius, out);
            }
            (Shape::Box { half: ha }, Shape::Box { half: hb }) => {
                self.box_pair(a, b, bodies, pa, ra, pb, rb, ha, hb, out);
            }
            (
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. },
                Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Cone { .. },
            ) => {
                self.poly_pair(a, b, bodies, sa, sb, pa, ra, pb, rb, out);
            }
            (
                Shape::Capsule {
                    half_height,
                    radius,
                },
                _,
            ) => {
                self.capsule_ab(a, b, pa, ra, pb, rb, sb, half_height, radius, out);
            }
            (
                _,
                Shape::Capsule {
                    half_height,
                    radius,
                },
            ) => {
                self.capsule_ba(a, b, pa, ra, pb, rb, sa, half_height, radius, out);
            }
            _ => {}
        }
    }

    /// （球, 凸体）：用球-凸路径，法线为 a→b。
    #[allow(clippy::too_many_arguments)]
    fn sphere_convex_pair(
        &mut self,
        a: u32,
        b: u32,
        pa: Vec3,
        radius: f32,
        convex: &Shape,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        if let Some((n, depth, point)) = self.sphere_convex_ab(pa, radius, convex, pb, rb) {
            out.push(Manifold {
                a,
                b,
                normal: n,
                points: [ContactPoint {
                    point,
                    depth,
                    feature: 0,
                }]
                .into(),
            });
        }
    }

    /// （凸体, 球）：球在 b，用球-凸路径后**翻转法线**。
    #[allow(clippy::too_many_arguments)]
    fn convex_sphere_pair(
        &mut self,
        a: u32,
        b: u32,
        pa: Vec3,
        ra: Quat,
        convex: &Shape,
        pb: Vec3,
        radius: f32,
        out: &mut Vec<Manifold>,
    ) {
        if let Some((n_ba, depth, point)) = self.sphere_convex_ab(pb, radius, convex, pa, ra) {
            out.push(Manifold {
                a,
                b,
                normal: -n_ba,
                points: [ContactPoint {
                    point,
                    depth,
                    feature: 0,
                }]
                .into(),
            });
        }
    }

    /// （盒, 盒）：T3 盒对专用路径（轴缓存 → SAT → clip）。
    #[allow(clippy::too_many_arguments)]
    fn box_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        ha: Vec3,
        hb: Vec3,
        out: &mut Vec<Manifold>,
    ) {
        // ===== T3 盒对专用路径 =====
        // 轴由 (rot, half) 直生（每体 3 次旋转），供 SAT/clip 直接消费。
        // 【实验】不再调用分离预筛：其 15 轴是 SAT 21 轴的子集（面轴 ± 同解），
        // 预筛不拒的对必然要再做一遍同样的 15 轴测试 ⇒ 对真接触对是纯重复。
        self.ws.box_a = Some((ha, pa));
        self.ws.box_b = Some((hb, pb));
        // 体轴缓存（T3）：同体连续出现 ⇒ 每体每帧只算一次 3 次旋转。
        let fp_a = rot_fp(ra);
        self.ws.box_axes_a = Some(
            if self.ws.cached_ax_a.0 == a && self.ws.cached_ax_a.1 == fp_a {
                self.ws.cached_ax_a.2
            } else {
                let ax = box_axes(ra);
                self.ws.cached_ax_a = (a, fp_a, ax);
                ax
            },
        );
        let fp_b = rot_fp(rb);
        self.ws.box_axes_b = Some(
            if self.ws.cached_ax_b.0 == b && self.ws.cached_ax_b.1 == fp_b {
                self.ws.cached_ax_b.2
            } else {
                let ax = box_axes(rb);
                self.ws.cached_ax_b = (b, fp_b, ax);
                ax
            },
        );
        if let Some((sep, n, src)) = self.sat(pb - pa) {
            // 速度充气视野（见 `predict_dt` 字段注）：`0` ⇒ 逐位同现行。
            self.ws.inflate = self.predict_inflate(a, b, bodies, n);
            if sep > self.skin + self.ws.inflate {
                return;
            }
            if self.clip(n, src) {
                out.push(Manifold {
                    a,
                    b,
                    normal: n,
                    points: ContactPoints::from_slice(&self.ws.cand),
                });
            }
        }
    }

    /// （盒|圆柱|圆锥, 盒|圆柱|圆锥）：通用路径（多面体填充 + 逐顶点/通用 SAT）。
    #[allow(clippy::too_many_arguments)]
    fn poly_pair(
        &mut self,
        a: u32,
        b: u32,
        bodies: &vxl_phys_core::BodySet,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        // 圆柱/圆锥参与的对：走通用路径（多面体填充 + 逐顶点/通用 SAT）。
        let ia = match self.poly_for(sa) {
            Some(i) => i,
            None => return,
        };
        let ib = match self.poly_for(sb) {
            Some(i) => i,
            None => return,
        };
        // 世界多面体填充缓存（T3）：键 = (体号, 多面体序号)；pair 按
        // (a,b) 排序 ⇒ 同一体连续命中，每体每帧只填一次（纯函数）。
        if self.ws.cached_a != (a, ia as u64) {
            self.ws.poly_a.fill(&self.ws.polys[ia], pa, ra);
            self.ws.cached_a = (a, ia as u64);
            self.probe.poly_fills += 1;
            self.probe.poly_fill_verts += self.ws.poly_a.verts.len() as u64;
        }
        if self.ws.cached_b != (b, ib as u64) {
            self.ws.poly_b.fill(&self.ws.polys[ib], pb, rb);
            self.ws.cached_b = (b, ib as u64);
            self.probe.poly_fills += 1;
            self.probe.poly_fill_verts += self.ws.poly_b.verts.len() as u64;
        }
        // 盒对 SAT 快路径参数（圆柱 → None，走通用逐顶点路径）。
        self.ws.box_a = match sa {
            Shape::Box { half } => Some((*half, pa)),
            _ => None,
        };
        self.ws.box_b = match sb {
            Shape::Box { half } => Some((*half, pb)),
            _ => None,
        };
        if let Some((sep, n, src)) = self.sat(pb - pa) {
            // 速度充气视野（同盒对路径；见 `predict_dt` 字段注）。
            self.ws.inflate = self.predict_inflate(a, b, bodies, n);
            if sep > self.skin + self.ws.inflate {
                return;
            }
            if self.clip(n, src) {
                out.push(Manifold {
                    a,
                    b,
                    normal: n,
                    points: ContactPoints::from_slice(&self.ws.cand),
                });
            }
        }
    }
}

/// 法向量化键（0.1 精度）：同面法线落进同一小组（确定性、组数 ≤ 10）。
fn quant_normal(v: Vec3) -> (i32, i32, i32) {
    (
        (v.x * 10.0).round() as i32,
        (v.y * 10.0).round() as i32,
        (v.z * 10.0).round() as i32,
    )
}

/// 从提供者候选里选**主导接触面**并给出带符号法线；`None` = 无可用面（整对结束）。
///
/// 选择次序（2026-09-15 修复，取代旧的"点数最多、并列取最深"单一规则）：
/// ① 有**闭合速度**的面优先（取最大者）；② 否则只考虑**带面心样本**的组；
/// ③ 组内仍按点数最多、并列取最深。理由与实测见 `provider_pair` 处注。
fn pick_dominant_normal(
    buf: &[vxl_phys_core::interop::InteropContact],
    vrel: Vec3,
    sgn: f32,
) -> Option<((i32, i32, i32), Vec3)> {
    // 小组统计（确定性；组数 ≤ 10）
    pub(crate) struct Group {
        pub(crate) key: (i32, i32, i32),
        pub(crate) count: usize,
        pub(crate) deepest: f32,
        pub(crate) closing: f32,
        pub(crate) has_center: bool,
    }
    let mut groups: Vec<Group> = Vec::with_capacity(8);
    for c in buf {
        let key = quant_normal(c.normal);
        let idx = match groups.iter().position(|g| g.key == key) {
            Some(i) => i,
            None => {
                groups.push(Group {
                    key,
                    count: 0,
                    deepest: f32::NEG_INFINITY,
                    closing: f32::NEG_INFINITY,
                    has_center: false,
                });
                groups.len() - 1
            }
        };
        let g = &mut groups[idx];
        g.count += 1;
        g.deepest = g.deepest.max(c.depth);
        g.closing = g.closing.max(-(c.normal * sgn).dot(vrel));
        if c.feature % 16 == 0 {
            g.has_center = true;
        }
    }
    let closing_group = groups
        .iter()
        .filter(|g| g.closing > CLOSING_MIN)
        .max_by(|x, y| x.closing.total_cmp(&y.closing));
    let pick = closing_group.or_else(|| {
        let with_center: Vec<&Group> = groups.iter().filter(|g| g.has_center).collect();
        let pool: Vec<&Group> = if with_center.is_empty() {
            groups.iter().collect()
        } else {
            with_center
        };
        pool.into_iter()
            .max_by(|x, y| x.count.cmp(&y.count).then(x.deepest.total_cmp(&y.deepest)))
    });
    pick.map(|g| {
        (
            g.key,
            buf.iter()
                .find(|c| quant_normal(c.normal) == g.key)
                .map(|c| c.normal * sgn)
                .unwrap_or(Vec3::Y),
        )
    })
}

/// **选点：四角优先，面心补位**（2026-09-22，P10 修复）。
/// 提供者每面发 5 点：`feature % 16 == 0` = **面心**，1..4 = **四角**；而流形只有
/// 4 槽。旧的 `take(4)` 按**生成序**取 ⇒ 恰好丢掉**第 4 个角** ⇒ 接触力偶不对称
/// ⇒ 每 tick 注入净力矩 ⇒ **静置单盒持续自旋**（实测 `|ω| ≈ 1.6 rad/s`、永不如入睡；
/// 见 `OPEN-PROBLEMS.md` P10 的逐 tick 轨迹：补丁恰为「中心 + 三角」、缺 (+x,+z)）。
/// ⇒ 角点定义力偶、面心只是冗余：**先把角点取满**，面心仅在角点不足时补位。
/// 稳定性：`sort_by_key` 是稳定排序 ⇒ 同类内保持生成序（确定性不变）。
fn four_corner_points(
    buf: &[vxl_phys_core::interop::InteropContact],
    best_key: (i32, i32, i32),
) -> Vec<ContactPoint> {
    let mut group_pts: Vec<&vxl_phys_core::interop::InteropContact> = buf
        .iter()
        .filter(|c| quant_normal(c.normal) == best_key)
        .collect();
    if group_pts.len() > 4 {
        group_pts.sort_by_key(|c| u32::from(c.feature % 16 == 0));
    }
    group_pts
        .iter()
        .take(4)
        .map(|c| ContactPoint {
            point: c.point,
            depth: c.depth,
            feature: c.feature,
        })
        .collect()
}

/// （球, 球）：中心距判据 + 单点流形（`dist < 1e-9` 时给轴向兜底点）。
fn sphere_sphere(a: u32, b: u32, pa: Vec3, ra_: f32, pb: Vec3, rb_: f32, out: &mut Vec<Manifold>) {
    let d = pb - pa;
    let dist = d.length();
    let rr = ra_ + rb_;
    if dist >= rr || dist < 1e-9 {
        if dist < 1e-9 {
            out.push(Manifold {
                a,
                b,
                normal: Vec3::Y,
                points: [ContactPoint {
                    point: pa,
                    depth: rr,
                    feature: 0,
                }]
                .into(),
            });
        }
        return;
    }
    let n = d * (1.0 / dist);
    let point = pa + n * (ra_ - (rr - dist) * 0.5);
    out.push(Manifold {
        a,
        b,
        normal: n,
        points: [ContactPoint {
            point,
            depth: rr - dist,
            feature: 0,
        }]
        .into(),
    });
}
