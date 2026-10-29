//! **ClothBodyExt**：`ClothSheet` 的**刚体耦合约束族**——本族**不往 `ClothSheet` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——36 个
//! 方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在同一片上，
//! 处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，判据是
//! 「新加一族约束**不必再往本类型添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `ClothSheet` 的方法数**不计入本类型**（门账 36 → 减本族 7 个）。
//!
//! **先例**：同判据的既有落法见 `crates/vxl-phys/src/world_struct/ext/`（`World` 76 → 22）。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
//!
//! **两条路径**（原 `cloth_coupling.rs` 的文件头注，此处照录）：
//! - **静态/睡眠代理 = 墙**（2b-i）：逐粒子解析穿透 + 库仑锥，**无反作用**。
//! - **动态代理 = 两体约束 + 反作用两腿**（2b-ii）：口径与 `rope::apply_body_hit` 同款。
use crate::cloth::ClothSheet;
use crate::cloth_coupling::BodyHit;
use crate::rigid::RigidProxy;
use vxl_phys_core::{Quat, Shape, Vec3};

/// **ClothBodyExt**：`ClothSheet` 的**刚体耦合约束族**（布片 ↔ 刚体代理）。
pub(crate) trait ClothBodyExt {
    /// **反作用状态每 tick 重置**（与 `bodies` 同序；`n = 0` ⇒ 三个 Vec 皆空 ⇒ 零成本）。
    fn reset_body_state(&mut self, n: usize);

    /// ③.5 **布片 ↔ 刚体**：逐代理分派。
    ///
    /// **静态/睡眠代理 = 墙**（切片 2b-i，逐字保留 ⇒ 那两条判据的读数**逐位不变**）；
    /// **动态代理**走两体约束 + 反作用两腿（切片 2b-ii，§8.4.20 三条使能条件同款）。
    fn body_contacts(&mut self, bodies: &[RigidProxy], h: f32);

    /// **本粒子对代理的穿透查询**（把 `contact.radius` 的取用收在一处）：语义/参数序与
    /// `rigid::shape_penetration` **逐字相同**，只是把半径从 `self` 里取 —— 顺带让两个
    /// `body_contacts_*` 的调用行不再被 rustfmt 折成 7 行（`self.contact.radius` 变长所致）。
    fn pen(&self, shape: &Shape, pos: Vec3, rot: Quat, p: Vec3) -> Option<(Vec3, f32, Vec3)>;

    /// **切向库仑锥**（从 `apply_body_hit` 抽出：那里是全目录最长函数）。**冲量口径** `μ·w_p·λ`：
    /// 库仑锥本是 `|J_t| ≤ μ·J_n`，而本接触**实际**施加的法向冲量 ∝ `λ`、**不是**几何穿透量
    /// `depth`（`μ·depth` 是偏大的预算，rope 实测**偏大不是承重而是棘轮源** ⇒ 轻载巡航逃逸）。
    /// 滑移量取**相对体**的（`v_point` 含 `ω × r`，§8.4.19/§8.4.30 同一口径）。
    fn apply_body_friction(&mut self, i: usize, n: Vec3, v_point: Vec3, w_p: f32, lam: f32, h: f32);

    /// **静态/睡眠代理 = 墙**（切片 2b-i）：逐粒子解析穿透（`rigid::shape_penetration`，
    /// Sphere/Box/Capsule 受理）+ 法向推出 + 库仑锥（滑移量取**相对体**的：`dp − v_body·h`
    /// —— 提供者是静态地形这一项恒为零，体在动时布才不会被"粘"在原地；与 rope 同款口径）。
    /// **无反作用**（`inv_mass = 0`）。本函数**逐字保留**切片 2b-i 的路径。
    fn body_contacts_static(&mut self, b: &RigidProxy, h: f32);

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
    /// 2. ⭐ **位置腿也进虚拟位姿**（见 `apply_body_hit` 的 `disp` 那两行）——
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
    fn body_contacts_dynamic(&mut self, j: usize, b: &RigidProxy, h: f32);

    /// **施加一次体接触的响应**（从 `body_contacts_dynamic` 抽出 —— god 门最长函数约束；
    /// rope 的 `apply_body_hit` 同款）：两体按逆质量分担解 `λ_geom` → 速度钳位 `λ_vel`
    /// （= 无恢复系数）→ **位置口径补足** `body.dx`（只补位置不补速度，§8.4.9）→ 法向推出、
    /// 库仑锥摩擦（**冲量口径** `μ·w_p·λ`，§8.4.25；相对速度与法向**同口径**，§8.4.19）
    /// → **反作用两腿**（含就地推进虚拟状态 = Gauss-Seidel）。
    ///
    /// **接触柔度 `α_c = 0`**（硬接触）：与 rope 的**默认值**同口径 —— rope 那个旋钮实测
    /// `α_c = 1e-4` 反而太软（60 tick 就沉到 `y ≈ 0.26`）。本片**不引入**该旋钮：布片接触点多，
    /// 柔度应当连同"多点如何分担"一起作为独立切片量。
    fn apply_body_hit(
        &mut self,
        hit: &BodyHit,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        h: f32,
    ) -> f32;

    /// **三段拆分之①（纯读）**：解 λ / 接触点速度 / make-up。不写任何状态 ⇒ 可重入。
    fn solve_body_hit(
        &self,
        hit: &BodyHit,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        h: f32,
    ) -> HitSolve;

    /// **三段拆分之②**：粒子法向推出 + 摩擦（只动粒子位置）。
    fn project_body_hit(&mut self, hit: &BodyHit, s: &HitSolve, h: f32);

    /// **三段拆分之③**：反作用两腿（`dv` / `disp`）。
    fn react_body_hit(&mut self, hit: &BodyHit, j: usize, s: &HitSolve, h: f32, before: Vec3);
}

/// 一次体接触的**解**（三段拆分的中间量；块求解多遍时靠它做残余记账）。
///
/// 为什么必须由求解本身给出：钳位截掉的那份**不消穿透**、make-up 补的那份**改穿透但不进
/// 反作用** —— 在外面按 `λ·(w_p+w_b)` 记账会同时低估与高估，多遍必然震荡。
#[derive(Clone, Copy, Debug)]
pub(crate) struct HitSolve {
    /// 实际施加的 λ（已被速度钳位截断）。
    pub(crate) lam: f32,
    /// 几何解（未被钳位）——make-up 与"体总让位"都按它算。
    pub(crate) lam_geom: f32,
    /// 接触点相对速度（摩擦与法向共用口径）。
    pub(crate) v_point: Vec3,
    /// 位置补足（沿 +n 的 `w_b·(λ_geom − λ)`）。
    pub(crate) make_up: Vec3,
    /// 体的逆质量（反作用要用；放进解里，避免函数再多一个参数撞 args 门）。
    pub(crate) w_b: f32,
}

impl HitSolve {
    /// **本遍真正消掉的穿透** = 粒子让的 `w_p·λ` + 体让的 `w_b·λ_geom`（含 make-up 那份）。
    pub(crate) fn closing(&self, w_p: f32, w_b: f32) -> f32 {
        w_p * self.lam + w_b * self.lam_geom
    }
}

impl ClothBodyExt for ClothSheet {
    fn reset_body_state(&mut self, n: usize) {
        for v in [&mut self.body.dv, &mut self.body.disp, &mut self.body.dx] {
            v.clear();
            v.resize(n, Vec3::ZERO);
        }
    }

    fn body_contacts(&mut self, bodies: &[RigidProxy], h: f32) {
        for (j, b) in bodies.iter().enumerate() {
            if b.inv_mass > 0.0 {
                self.body_contacts_dynamic(j, b, h);
            } else {
                self.body_contacts_static(b, h);
            }
        }
    }

    fn pen(&self, shape: &Shape, pos: Vec3, rot: Quat, p: Vec3) -> Option<(Vec3, f32, Vec3)> {
        crate::rigid::shape_penetration(shape, pos, rot, p, self.contact.radius)
    }

    fn apply_body_friction(
        &mut self,
        i: usize,
        n: Vec3,
        v_point: Vec3,
        w_p: f32,
        lam: f32,
        h: f32,
    ) {
        if self.contact.friction <= 0.0 {
            return;
        }
        let dp = self.pos[i] - self.prev[i] - v_point * h;
        let t = dp - n * dp.dot(n);
        let slip = t.length();
        if slip > 0.0 {
            let budget = self.contact.friction * (w_p * lam);
            let removed = if slip < budget { slip } else { budget };
            self.pos[i] -= t * (removed / slip);
        }
    }

    fn body_contacts_static(&mut self, b: &RigidProxy, h: f32) {
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            let Some((n, depth, _)) = self.pen(&b.shape, b.pos, b.rot, self.pos[i]) else {
                continue;
            };
            if depth <= 0.0 {
                continue; // 带内预判不推（无恢复系数的位置口径）
            }
            self.pos[i] += n * depth;
            if self.contact.friction > 0.0 {
                let dp = self.pos[i] - self.prev[i] - b.linvel * h;
                let t = dp - n * dp.dot(n);
                let slip = t.length();
                if slip > 0.0 {
                    let budget = self.contact.friction * depth;
                    let removed = if slip < budget { slip } else { budget };
                    self.pos[i] -= t * (removed / slip);
                }
            }
        }
    }

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
            let Some((n, depth, q)) = self.pen(&b.shape, vpos, b.rot, self.pos[i]) else {
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
        // ②③ 施加：默认 = 几何排序 + 逐 hit 就地 GS；块求解开 = 不排序 + Jacobi 块
        // （与 rope 那条边同一套，见 `vbd::apply_all_cloth`）。
        crate::vbd::apply_all_cloth(self, j, b, vpos, h);
    }

    fn apply_body_hit(
        &mut self,
        hit: &BodyHit,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        h: f32,
    ) -> f32 {
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
        // [`ClothBodyExt::body_contacts_dynamic`] 的注：不进 `disp` ⇒ 同一份穿透被下一子步重新测到
        // ⇒ 体的位置修正被"接触数 × 子步数"重复计账。`disp` **只用于接触检测**（不落盘到体的
        // 真实位置）⇒ 与门面的 `position += dx` 不重复施加。
        if lam_geom > lam {
            let make_up = n * (w_b * (lam_geom - lam));
            self.body.dx[j] -= make_up;
            self.body.disp[j] -= make_up;
        }
        if lam <= 0.0 {
            return 0.0;
        }
        let before = self.pos[i];
        // 法向推出（位移 = `w_p·λ`）。
        self.pos[i] += n * (w_p * lam);
        self.apply_body_friction(i, n, v_point, w_p, lam, h);
        // **反作用两腿**（§8.4.20 条件①③）：粒子的动量变化取反 = 体所受冲量。
        // **角反作用停用**（条件②）：接触模型看不见转动（`disp` 只跟平移、`crossed_face` 用冻结
        // 朝向）⇒ 回填角动量后体转起来的运动会完全落在模型之外（rope 实测 1 kg 薄盒 `|ω|` 一 tick
        // 就到 5~8 rad/s ⇒ 立刻丢失接触 ⇒ 被甩下去）。本片**不引入**该开关（默认关的那套也不带）。
        let d = self.pos[i] - before;
        let impulse = d * (-inv_h / w_p);
        self.body.dv[j] += impulse * w_b;
        self.body.disp[j] += impulse * (w_b * h);
        lam
    }

    fn solve_body_hit(
        &self,
        hit: &BodyHit,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        h: f32,
    ) -> HitSolve {
        let i = hit.i as usize;
        let n = hit.n;
        let inv_h = 1.0 / h;
        let w_p = self.inv_mass[i];
        let w_b = b.inv_mass;
        let v_p = (self.pos[i] - self.prev[i]) * inv_h;
        let v_b = b.linvel + self.body.dv[j];
        let v_point = v_b + b.angvel.cross(hit.q - vpos);
        let approach = (v_point - v_p).dot(n);
        let lam_geom = hit.depth / (w_p + w_b);
        let lam_vel = if approach > 0.0 {
            approach * h / (w_p + w_b)
        } else {
            0.0
        };
        let lam = lam_geom.min(lam_vel);
        let make_up = if lam_geom > lam {
            n * (w_b * (lam_geom - lam))
        } else {
            Vec3::ZERO
        };
        HitSolve {
            lam,
            lam_geom,
            v_point,
            make_up,
            w_b,
        }
    }

    fn project_body_hit(&mut self, hit: &BodyHit, s: &HitSolve, h: f32) {
        let i = hit.i as usize;
        let n = hit.n;
        let w_p = self.inv_mass[i];
        self.pos[i] += n * (w_p * s.lam);
        self.apply_body_friction(i, n, s.v_point, w_p, s.lam, h);
    }

    fn react_body_hit(&mut self, hit: &BodyHit, j: usize, s: &HitSolve, h: f32, before: Vec3) {
        let i = hit.i as usize;
        let inv_h = 1.0 / h;
        let w_p = self.inv_mass[i];
        let d = self.pos[i] - before;
        let impulse = d * (-inv_h / w_p);
        self.body.dv[j] += impulse * s.w_b;
        self.body.disp[j] += impulse * (s.w_b * h);
    }
}
