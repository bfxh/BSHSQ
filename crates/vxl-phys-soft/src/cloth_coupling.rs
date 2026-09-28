//! **布片 ↔ 刚体耦合**（软体切片 2b-i / 2b-ii）——`ClothSheet` 的刚体接触段（从 `cloth.rs` 拆出）。
//!
//! **为什么独立成文件**：`cloth.rs` 受 god 门**文件行数棘轮**（只准减）——反作用两腿一上来就把
//! 它从 425 行推到 648 行（超出 10% 交换窗）⇒ 按先例（`world_mesh.rs`：`impl World` 方法数顶 god 门
//! 24 成员，故按域拆新文件）把"刚体接触"这**一个域**整块搬走。
//!
//! **两条路径**：
//! - **静态/睡眠代理 = 墙**（2b-i）：逐粒子解析穿透 + 库仑锥，**无反作用**（`inv_mass = 0`）。
//!   逐字保留（`tests/cloth_body.rs` 的两条读数**逐位不变**）。
//! - **动态代理 = 两体约束 + 反作用两腿**（2b-ii）：口径与 `rope::apply_body_hit` 同款，
//!   §8.4.20 三条使能条件一条不缺。**唯一偏离**见 [`ClothSheet::body_contacts_dynamic`] 的注。
use crate::cloth::ClothSheet;
use crate::rigid::RigidProxy;
use vxl_phys_core::Vec3;

/// **一次体接触的收集项**（切片 2b-ii；与 `rope::BodyHit` 同款）：
/// 先收全部命中、按**几何序**排、再施加 ⇒ 施加顺序是**几何的函数**。
///
/// 为什么必须排（§8.4.24 的教训）：`body.dv` 是**就地推进**的顺序 Gauss-Seidel ⇒ "谁先被处理"
/// 会改变结果。实测（rope，§8.4.23）：把粒子循环反向，常规载重下的横向漂移**符号翻转**
/// （0.5 kg：`+0.098 → −0.124`）⇒ 那是**顺序偏差**、不是物理。
#[derive(Clone, Copy)]
pub(crate) struct BodyHit {
    /// 粒子号。
    i: u32,
    /// 排序主键：粒子自己的 `x`（世界横坐标，升序）——**物理量** ⇒ 与数组方向无关。
    ord: f32,
    /// 排序次键：`z`（同 `x` 不可能出现 ⇒ 仅作确定性兜底）。
    z: f32,
    /// 该次接触的法线 / 穿透 / 接触点（`shape_penetration` 的读数）。
    n: Vec3,
    depth: f32,
    q: Vec3,
}

/// **粒子↔刚体耦合状态**（与 `step` 传入的 `bodies` 同序；每 tick 开头重置）。
///
/// **为什么收成一个结构**：`ClothSheet` 受 god 门**成员棘轮**（21 成员顶格，阈值 24）——
/// 这四样一上来就超阈 ⇒ 按先例（`rope::VirtRot` / `world_soft::SoftDomain`）合并成一个
/// （`ClothSheet` 成员 21 → 22）。
///
/// **`disp` 是"虚拟位移"**：体在本 tick 内自己走的 + 被接触推开的（**两条腿都算**），
/// **逐子步增量累加**。
/// ⚠️ **不可写成 `(linvel + Δv)·(sub_idx·h)`**（rope 实测过的公式错误）：那会把"当前累计的
/// `Δv`"乘上**整段时间**（末子步乘 8h）⇒ 虚拟位姿被放大 ⇒ 接触几何算错。
#[derive(Clone, Default)]
pub struct BodyCoupling {
    /// **体的速度增量**（门面 `bodies.linvel[j] += dv[j]`）：引擎在体解算**之后**施加
    /// ⇒ **下一 tick 生效**。每 tick 开头清零。
    pub dv: Vec<Vec3>,
    /// **位置口径的补足量**（门面 `bodies.position[j] += dx[j]`，§8.4.9）：被速度钳位
    /// `λ = min(λ_geom, λ_vel)` 压掉的那一份**只补位置、不补速度**。
    ///
    /// **为什么必须有**：静载下 `approach ≈ 0 ⇒ λ ≈ 0` ⇒ 接触**只回速度、不回位置** ⇒
    /// 体每 tick 先按 `v·dt` 走过的 `g·dt² = 2.72 mm` **一去不回**（rope 实测下沉 = 整漏的 83%）。
    /// **为什么用"差值"而不是 `λ_geom` 本身**：`disp`（虚拟位姿）已含钳位后的那一份 ⇒
    /// 补足 `λ_geom − λ` 之后，体在本 tick 的**总位置效应恰好 = `λ_geom`**（不重复计账），
    /// 且**钳位生效时（运动/冲击工况）恒为 0 ⇒ 既有工况逐位不变**。
    pub dx: Vec<Vec3>,
    /// 子步内**虚拟位移**（体自己走的 + 被推开的，两条腿都算；增量累加，见类型注）。
    pub(crate) disp: Vec<Vec3>,
    /// 命中收集暂存（复用免分配）。
    hits: Vec<BodyHit>,
}

impl ClothSheet {
    /// **反作用状态每 tick 重置**（与 `bodies` 同序；`n = 0` ⇒ 三个 Vec 皆空 ⇒ 零成本）。
    pub(crate) fn reset_body_state(&mut self, n: usize) {
        for v in [&mut self.body.dv, &mut self.body.disp, &mut self.body.dx] {
            v.clear();
            v.resize(n, Vec3::ZERO);
        }
    }

    /// ③.5 **布片 ↔ 刚体**：逐代理分派。
    ///
    /// **静态/睡眠代理 = 墙**（切片 2b-i，逐字保留 ⇒ 那两条判据的读数**逐位不变**）；
    /// **动态代理**走两体约束 + 反作用两腿（切片 2b-ii，§8.4.20 三条使能条件同款）。
    pub(crate) fn body_contacts(&mut self, bodies: &[RigidProxy], h: f32) {
        for (j, b) in bodies.iter().enumerate() {
            if b.inv_mass > 0.0 {
                self.body_contacts_dynamic(j, b, h);
            } else {
                self.body_contacts_static(b, h);
            }
        }
    }

    /// **静态/睡眠代理 = 墙**（切片 2b-i）：逐粒子解析穿透（`rigid::shape_penetration`，
    /// Sphere/Box/Capsule 受理）+ 法向推出 + 库仑锥（滑移量取**相对体**的：`dp − v_body·h`
    /// —— 提供者是静态地形这一项恒为零，体在动时布才不会被"粘"在原地；与 rope 同款口径）。
    /// **无反作用**（`inv_mass = 0`）。本函数**逐字保留**切片 2b-i 的路径。
    fn body_contacts_static(&mut self, b: &RigidProxy, h: f32) {
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            let Some((n, depth, _)) =
                crate::rigid::shape_penetration(&b.shape, b.pos, b.rot, self.pos[i], self.radius)
            else {
                continue;
            };
            if depth <= 0.0 {
                continue; // 带内预判不推（无恢复系数的位置口径）
            }
            self.pos[i] += n * depth;
            if self.friction > 0.0 {
                let dp = self.pos[i] - self.prev[i] - b.linvel * h;
                let t = dp - n * dp.dot(n);
                let slip = t.length();
                if slip > 0.0 {
                    let budget = self.friction * depth;
                    let removed = if slip < budget { slip } else { budget };
                    self.pos[i] -= t * (removed / slip);
                }
            }
        }
    }

    /// **动态代理**（切片 2b-ii）：**两体按逆质量分担的约束**、**反作用两腿**（`dv`/`dx`）、
    /// **虚拟位姿**（`disp`）。口径与 `rope::project_body_contacts` / `apply_body_hit`
    /// **同款**（计划原文："动量守恒口径与 §8.4.20 同款，**不再另发明一套**"）。
    ///
    /// **顺序**（§8.4.24 同款）：先**收集**本子步全部命中、按**几何序**（`x` 升序、`z` 升序）排，
    /// 再施加 —— `body.dv` 是就地推进的顺序 Gauss-Seidel，施加顺序必须是**几何的函数**。
    /// **与 rope 的两处差别（如实登记）**：
    /// 1. rope 的"按 `x` 升序"与它的数组序（左→右）**逐位等价** ⇒ 既有读数不动；布片的数组序是
    ///    **行主序（`z` 外层、`x` 内层）** ⇒ 本序与数组序**不等价**（这是一条新的 GS 路径）。
    ///    取它是因为"几何的函数"这条性质比"沿用数组序"更承重。
    /// 2. ⭐ **位置腿也进虚拟位姿**（见 [`ClothSheet::apply_body_hit`] 的 `disp` 那两行）——
    ///    rope 没有这一步，实测差 15 cm 级的单 tick 上抛（下面是实测留档）。
    ///
    /// **⭐ 实测（2026-09-29，同一场景只差那一行）**：让盒落到四周钉住的布片上（网格 8×8、
    /// 间距 0.125 m、盒底面 0.4×0.4 ⇒ **9 个接触点**、8 子步 ⇒ 一份穿透要被 **72** 个
    /// `(接触, 子步)` 项各算一遍）：
    /// - **不把位置腿放进 `disp`** ⇒ 1.8 cm 的穿透被放大成**单 tick 12.2 cm 的上抛**
    ///   ⇒ 盒在布上**弹跳**（稳态 y 波动 **0.1486 m**、接触集反复丢失、末速 −0.21 m/s）；
    /// - **放进去** ⇒ 稳态 y 波动 **0.0000**、每 tick `dv.y` 恒为 `g·dt`（真正的静平衡）、
    ///   净漂移 ≈ 0、|x|max 0.018。
    ///
    /// 机制：`vpos = b.pos + disp` 是"体在本子步的真实位置"的模拟；位置腿是这条轨迹的一部分，
    /// 不进 `disp` ⇒ **下一子步重新测到同一份穿透** ⇒ 体的位置修正被重复计账
    /// （× 接触数 × 子步数）。速度腿没有这个问题：它经 `body.dv` **就地反馈**（自限幅）。
    fn body_contacts_dynamic(&mut self, j: usize, b: &RigidProxy, h: f32) {
        // 虚拟位姿：体自己走的（含本子步已累积的反作用）。**增量累加**（见 [`BodyCoupling::disp`]）。
        self.body.disp[j] += (b.linvel + self.body.dv[j]) * h;
        let vpos = b.pos + self.body.disp[j];
        // ① 收集：命中判定只读粒子与体，且 `vpos` 在本子步内固定 ⇒ 与施加顺序无关。
        self.body.hits.clear();
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            let Some((n, depth, q)) =
                crate::rigid::shape_penetration(&b.shape, vpos, b.rot, self.pos[i], self.radius)
            else {
                continue;
            };
            if depth <= 0.0 {
                continue;
            }
            let p = self.pos[i];
            self.body.hits.push(BodyHit {
                i: i as u32,
                ord: p.x,
                z: p.z,
                n,
                depth,
                q,
            });
        }
        // ② 按**几何序**排（§8.4.24）。
        self.body
            .hits
            .sort_by(|a, c| a.ord.total_cmp(&c.ord).then(a.z.total_cmp(&c.z)));
        let hits = std::mem::take(&mut self.body.hits);
        // ③ 施加（顺序 Gauss-Seidel：`body.dv` 就地推进 ⇒ 后面的粒子看到"已被推走的体"）。
        for hit in &hits {
            self.apply_body_hit(hit, j, b, vpos, h);
        }
        self.body.hits = hits; // 归还暂存（`mem::take` 的配对）
    }

    /// **施加一次体接触的响应**（从 `body_contacts_dynamic` 抽出 —— god 门最长函数约束；
    /// rope 的 `apply_body_hit` 同款）：两体按逆质量分担解 `λ_geom` → 速度钳位 `λ_vel`
    /// （= 无恢复系数）→ **位置口径补足** `body.dx`（只补位置不补速度，§8.4.9）→ 法向推出、
    /// 库仑锥摩擦（**冲量口径** `μ·w_p·λ`，§8.4.25；相对速度与法向**同口径**，§8.4.19）
    /// → **反作用两腿**（含就地推进虚拟状态 = Gauss-Seidel）。
    ///
    /// **接触柔度 `α_c = 0`**（硬接触）：与 rope 的**默认值**同口径 —— rope 那个旋钮实测
    /// `α_c = 1e-4` 反而太软（60 tick 就沉到 `y ≈ 0.26`）。本片**不引入**该旋钮：布片接触点多，
    /// 柔度应当连同"多点如何分担"一起作为独立切片量。
    fn apply_body_hit(&mut self, hit: &BodyHit, j: usize, b: &RigidProxy, vpos: Vec3, h: f32) {
        let i = hit.i as usize;
        let n = hit.n;
        let inv_h = 1.0 / h;
        let w_p = self.inv_mass[i];
        let w_b = b.inv_mass;
        // **按逆质量分担的两体约束**（α=0）：`λ = depth/(w_p + w_b)`，粒子沿 +n 让 `w_p·λ`、
        // 体沿 −n 让 `w_b·λ` —— 相对位移正好合上穿透量。
        // ⚠️ **不能让粒子吃满穿透量**（rope 首版就是这么写的）：那样体的位置永远不被挡，
        // 只吃到与穿透量成正比的巨大冲量 ⇒ 盒子"被踹飞着掉下去"（实测 y 比自由落体还低）。
        // 静态代理（`w_b = 0`）下 `λ = depth/w_p` ⇒ 与 2b-i 的路径同口径。
        let v_p = (self.pos[i] - self.prev[i]) * inv_h;
        let v_b = b.linvel + self.body.dv[j];
        // **接触点速度**（§8.4.30）：体在转时**材料点**的速度是 `v + ω × r`（`r = q − vpos`）。
        // **法向的 `approach` 与摩擦的滑移量共用这一个口径** —— §8.4.19 与 §8.4.25 两次都栽在
        // "两支用了不同的相对速度"上，所以这里一次改齐。
        let v_point = v_b + b.angvel.cross(hit.q - vpos);
        let approach = (v_point - v_p).dot(n);
        let lam_geom = hit.depth / (w_p + w_b);
        // **冲量上限 = 消掉本次接近速度所需**（顺序冲量的标准钳位；物理上 = 无恢复系数）。
        // 为什么必须有：位置口径按几何穿透量解会让"一子步内冒出的深穿透"反射出巨大速度
        // （rope 实测 0.035 m ⇒ 8.4 m/s/粒子 ⇒ 十几颗粒子把盒子踹成 +38 m/s）。
        let lam_vel = if approach > 0.0 {
            approach * h / (w_p + w_b)
        } else {
            0.0
        };
        let lam = lam_geom.min(lam_vel);
        // **位置口径的补足**（§8.4.9）：被速度钳位压掉的那一份**只补位置、不补速度**
        // （静载下钳位恒 0 ⇒ 不补就是"只回速度不回位置"的位置漏）。
        // ⭐ **同时进虚拟位姿**（`disp`）——这是本片对 rope 口径唯一的偏离，理由与实测见
        // [`ClothSheet::body_contacts_dynamic`] 的注：不进 `disp` ⇒ 同一份穿透被下一子步重新测到
        // ⇒ 体的位置修正被"接触数 × 子步数"重复计账。`disp` **只用于接触检测**（不落盘到体的
        // 真实位置）⇒ 与门面的 `position += dx` 不重复施加。
        if lam_geom > lam {
            let make_up = n * (w_b * (lam_geom - lam));
            self.body.dx[j] -= make_up;
            self.body.disp[j] -= make_up;
        }
        if lam <= 0.0 {
            return;
        }
        let before = self.pos[i];
        // 法向推出（位移 = `w_p·λ`）。
        self.pos[i] += n * (w_p * lam);
        // 切向：库仑锥（锥内整段吃掉 = 静摩擦；超出按动摩擦滑）。**冲量口径** `μ·w_p·λ`：
        // 库仑锥本是 `|J_t| ≤ μ·J_n`，而本接触**实际**施加的法向冲量 ∝ `λ`、**不是**几何穿透量
        // `depth`（`μ·depth` 是偏大的预算，rope 实测**偏大不是承重而是棘轮源** ⇒ 轻载巡航逃逸）。
        if self.friction > 0.0 {
            let dp = self.pos[i] - self.prev[i] - v_point * h;
            let t = dp - n * dp.dot(n);
            let slip = t.length();
            if slip > 0.0 {
                let budget = self.friction * (w_p * lam);
                let removed = if slip < budget { slip } else { budget };
                self.pos[i] -= t * (removed / slip);
            }
        }
        // **反作用两腿**（§8.4.20 条件①③）：粒子的动量变化取反 = 体所受冲量。
        // **角反作用停用**（条件②）：接触模型看不见转动（`disp` 只跟平移、`crossed_face` 用冻结
        // 朝向）⇒ 回填角动量后体转起来的运动会完全落在模型之外（rope 实测 1 kg 薄盒 `|ω|` 一 tick
        // 就到 5~8 rad/s ⇒ 立刻丢失接触 ⇒ 被甩下去）。本片**不引入**该开关（默认关的那套也不带）。
        let d = self.pos[i] - before;
        let impulse = d * (-inv_h / w_p);
        self.body.dv[j] += impulse * w_b;
        self.body.disp[j] += impulse * (w_b * h);
    }
}
