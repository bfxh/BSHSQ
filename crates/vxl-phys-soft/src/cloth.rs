//! cloth：**布料最小闭环**（三角网 + XPBD 约束）——软体域第二块（绳索之后的切片）。
//!
//! 与 [`crate::rope::Rope`] 同一套 XPBD 机械（预测 → 逐子步 Gauss-Seidel 投影 → 速度回写），
//! 差别只在**拓扑**：绳 = 链式邻居对，布 = **三角网的唯一边集**（结构 + 剪切：四边形网格的
//! 对角线本就是三角形的共享边 ⇒ 唯一边集已覆盖两组）；**弯曲**由二环对补齐（见 `bend`）。
//!
//! **质量 = 薄壳均分**：`m_i = ρ·t·Σ(incident A_tri)/3`——与窄相 `MeshStore::shell_props`
//! 的薄壳总质量同口径（两者对同一张网给出的总质量一致）。
//!
//! **已有**：三组距离约束（结构/剪切/弯曲）、提供者接触、**刚体接触**——静态/睡眠代理 = 墙
//! （切片 2b-i），动态代理 = 两体约束 + **反作用两腿** `body.dv`/`body.dx`（切片 2b-ii，
//! 口径与 `rope::apply_body_hit` 同款，见 [`BodyCoupling`]）。
//! **边界（写清，不是漏）**：无自碰撞、无撕裂、无塑性、无气动消费（`vxl-phys-aero` 的
//! `face_force` 已备，接线属后续切片）；接触几何走"最近面"（`rigid::shape_penetration`），
//! **没有** rope 那套"入口面"缓存 —— 布片命中点多、面法线不像绳那样在底面↔侧面↔顶面之间翻转。
use crate::cloth_coupling::BodyCoupling;
use crate::cloth_self_collision::SelfContacts;
use crate::params::Stiffness;
use crate::rigid::RigidProxy;
use std::collections::HashSet;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::Vec3;

/// **球采样接触投影 + 库仑锥**（从 `rope.rs::project_contacts` **纯搬移**——rope 现委托本函数
/// ⇒ 那边净缩、这边新增，口径逐字不变）：法向推出（只有真穿透才推 ⇒ 无恢复系数）+
/// 切向锥（`budget = μ·depth`，锥内整段吃掉 = 黏住）。平面/缓坡判据同 rope 的
/// 15° 黏 / 35° 滑（`tanθ` 对 μ）。
#[allow(clippy::too_many_arguments)] // 提取自方法（self 摊平成参数）：pos/prev/inv_mass/半径/带/μ + 提供者 + 出参
pub(crate) fn sphere_contacts_project(
    pos: &mut [Vec3],
    prev: &[Vec3],
    inv_mass: &[f32],
    radius: f32,
    skin: f32,
    friction: f32,
    providers: &dyn ProviderColliders,
    provider_count: u32,
    buf: &mut Vec<InteropContact>,
) {
    for i in 0..pos.len() {
        if inv_mass[i] == 0.0 {
            continue;
        }
        for id in 0..provider_count {
            buf.clear();
            if !providers.contacts_sphere(id, pos[i], radius, skin, buf) {
                continue;
            }
            for c in buf.iter() {
                if c.depth <= 0.0 {
                    continue; // 带内预判不推（无恢复系数的位置口径）
                }
                let n = c.normal;
                // ① 法向：推到面上（位移 = 穿透量）。
                pos[i] += n * c.depth;
                // ② 切向：库仑锥（锥内整段吃掉 ⇒ 静摩擦黏住；只扣超出部分会恒定蠕变——rope 首版教训）。
                if friction > 0.0 {
                    let dp = pos[i] - prev[i];
                    let t = dp - n * dp.dot(n);
                    let slip = t.length();
                    if slip > 0.0 {
                        let budget = friction * c.depth;
                        let removed = if slip < budget { slip } else { budget };
                        pos[i] -= t * (removed / slip);
                    }
                }
            }
        }
    }
}

/// **布片**（XPBD 三角网）：粒子 + 唯一边距离约束 + 逐子步投影。
#[derive(Clone)]
pub struct ClothSheet {
    pub pos: Vec<Vec3>,
    pub(crate) prev: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    /// `1/m`；钉住/退化粒子 = 0。
    pub inv_mass: Vec<f32>,
    /// 粒子质量（薄壳均分；判据/渲染用）。
    pub mass: Vec<f32>,
    /// 三角（索引指向 `pos`；注册序 = 确定性）。
    pub tris: Vec<[u32; 3]>,
    /// 唯一边（`[min, max]` 有序对；插入序 = 三角扫描序 ⇒ 确定性）。
    pub(crate) cons: Vec<[u32; 2]>,
    pub(crate) rest: Vec<f32>,
    pub(crate) lambda: Vec<f32>,
    /// **弯曲约束**（二环对；`[min, max]` 有序，插入序见 `new`）。
    pub(crate) bend: Vec<[u32; 2]>,
    pub(crate) bend_rest: Vec<f32>,
    pub(crate) bend_lambda: Vec<f32>,
    /// 弯曲 compliance（默认 `Soft` 档；`f32::INFINITY` ⇒ 关弯曲）。
    pub bend_compliance: f32,
    /// 距离约束 compliance（m/N；`Stiffness::alpha`）。
    pub compliance: f32,
    /// 每 tick 子步数（判据收敛旋钮）。
    pub substeps: u32,
    /// 每子步投影遍数（Gauss-Seidel）。
    pub iterations: u32,
    /// 速度回写阻尼（1.0 = 无阻尼；同 [`crate::rope::Rope::damping`] 口径）。
    pub damping: f32,
    /// 粒子**球采样半径**（提供者接触用；`< 0` = 关接触。默认 = 壳厚一半）。
    pub radius: f32,
    /// 库仑锥摩擦系数（`budget = μ·depth`；默认 0.5，与 rope 一致）。
    pub friction: f32,
    /// 接触带（预测接触用；默认 0.02）。
    pub skin: f32,
    pub(crate) buf: Vec<InteropContact>,
    /// **粒子↔刚体耦合状态**（反作用两腿 `dv`/`dx`；与 `step` 的 `bodies` 同序）。
    pub body: BodyCoupling,
    /// **自碰撞状态**（空间哈希 + 禁止对；`cfg.enabled` 默认 `false` ⇒ 零成本）。
    pub self_contacts: SelfContacts,
}

impl ClothSheet {
    /// 由顶点 + 三角构一张布片：**唯一边** = 距离约束（rest = 当前长度 ⇒ 注册态即零应变态）；
    /// 粒子质量按薄壳均分。`thickness ≤ 0` 兜底 1e-3（与 `MeshStore::shell_props` 同惯例）。
    pub fn new(
        points: Vec<Vec3>,
        tris: Vec<[u32; 3]>,
        density: f32,
        thickness: f32,
        stiffness: Stiffness,
    ) -> Self {
        let n = points.len();
        let rho = if density > 0.0 { density } else { 1.0 };
        let t = if thickness > 0.0 { thickness } else { 1e-3 };
        // 唯一边（结构+剪切）、弯曲二环对、薄壳均分质量 —— 三块各自抽成纯函数
        // （`new` 原 100 行，顶 god 门最长函数棘轮；2026-09-29 切片 T3 抽出的）。
        let cons = unique_edges(&tris);
        let rest = cons
            .iter()
            .map(|[a, b]| (points[*b as usize] - points[*a as usize]).length())
            .collect();
        let bend = bend_pairs(&cons, n);
        let bend_rest = bend
            .iter()
            .map(|[a, b]| (points[*b as usize] - points[*a as usize]).length())
            .collect();
        let mass = shell_mass(&tris, &points, rho, t);
        let inv_mass = mass
            .iter()
            .map(|&m| if m > 0.0 { 1.0 / m } else { 0.0 })
            .collect();
        let mut self_contacts = SelfContacts::new(t * 0.5);
        self_contacts.set_forbidden(&cons, &bend);
        Self {
            pos: points,
            prev: Vec::new(),
            vel: vec![Vec3::ZERO; n],
            inv_mass,
            mass,
            tris,
            cons,
            rest,
            lambda: Vec::new(),
            compliance: stiffness.alpha(),
            bend,
            bend_rest,
            bend_lambda: Vec::new(),
            bend_compliance: Stiffness::Soft.alpha(),
            substeps: 8,
            iterations: 1,
            damping: 1.0,
            radius: t * 0.5,
            friction: 0.5,
            skin: 0.02,
            buf: Vec::new(),
            body: BodyCoupling::default(),
            self_contacts,
        }
    }

    pub fn particle_count(&self) -> usize {
        self.pos.len()
    }

    pub fn edge_count(&self) -> usize {
        self.cons.len()
    }

    /// 钉住/释放一个粒子（钉住 = `inv_mass = 0`，同 [`crate::rope::Rope::set_pinned`]）。
    pub fn set_pinned(&mut self, i: usize, pinned: bool) {
        if i >= self.pos.len() {
            return;
        }
        self.inv_mass[i] = if pinned || self.mass[i] <= 0.0 {
            0.0
        } else {
            1.0 / self.mass[i]
        };
    }

    /// 推进一个 `dt`（内部按 `substeps` 细分）。接触 = **提供者**球采样 + 库仑锥
    /// （`provider_count = 0` 或 `radius < 0` ⇒ 零成本跳过）+ **刚体代理**
    /// （静态/睡眠代理 = 墙；**动态代理 = 两体约束 + 反作用两腿 `body.dv`/`body.dx`**，切片 2b-ii）。
    /// 反作用累加在 [`BodyCoupling`] 里，由门面在**体解算之后**取用（⇒ 下一 tick 生效）。
    pub fn step(
        &mut self,
        dt: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        provider_count: u32,
        bodies: &[RigidProxy],
    ) {
        if self.prev.len() != self.pos.len() {
            self.prev = self.pos.clone();
        }
        if self.lambda.len() != self.cons.len() {
            self.lambda = vec![0.0; self.cons.len()];
        }
        if self.bend_lambda.len() != self.bend.len() {
            self.bend_lambda = vec![0.0; self.bend.len()];
        }
        self.reset_body_state(bodies.len());
        let h = dt / self.substeps.max(1) as f32;
        for _ in 0..self.substeps.max(1) {
            self.substep(h, gravity, providers, provider_count, bodies);
        }
    }

    /// 单个子步：预测 → 距离约束（XPBD）→ 接触 → 速度回写 + 阻尼。
    ///
    /// **为什么切成四段**（2026-09-29，切片 2b-ii）：① 本函数原本 149 行（god 门**最长函数**
    /// 棘轮顶格）⇒ 动态代理那条腿（两体约束 + 反作用两腿 + 收集/排序）**放不下**；
    /// ② 四段的边界就是原注释里的 ①②③④ —— 切法与注释同构，不是为拆而拆。
    fn substep(
        &mut self,
        h: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        provider_count: u32,
        bodies: &[RigidProxy],
    ) {
        self.predict(h, gravity);
        self.project_constraints(h);
        self.project_contacts(h, providers, provider_count, bodies);
        self.write_back(h);
    }

    /// ① 预测（钉住粒子原地不动、速度清零；与 rope 同款）。
    fn predict(&mut self, h: f32, gravity: Vec3) {
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                self.prev[i] = self.pos[i];
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] += gravity * h;
            self.prev[i] = self.pos[i];
            self.pos[i] += self.vel[i] * h;
        }
    }

    /// ② 距离约束（Gauss-Seidel；`λ` 每子步清零、子步内按迭代累加 = XPBD 口径）。
    /// **结构/剪切**（唯一边，`compliance`）+ **弯曲**（二环对，`bend_compliance`）——
    /// 两组同段位求解：弯曲列在后（先满足强约束再让软约束让位，与"硬→软"的
    /// Gauss-Seidel 顺序惯例一致）。`bend_compliance = ∞` ⇒ 该组自然退化为不动。
    fn project_constraints(&mut self, h: f32) {
        for l in self.lambda.iter_mut() {
            *l = 0.0;
        }
        for l in self.bend_lambda.iter_mut() {
            *l = 0.0;
        }
        let a_tilde = self.compliance / (h * h);
        // **关弯曲**（非有限 compliance）⇒ `a_tilde_bend = inf`。`inf/inf = NaN` 会毒化整场
        // （弯曲金丝雀判据实测抓到过）⇒ 下方**显式短路**，不是"靠数学自然退化"。
        let a_tilde_bend = self.bend_compliance / (h * h);
        for _ in 0..self.iterations.max(1) {
            for (k, [i, j]) in self.cons.iter().enumerate() {
                let (i, j) = (*i as usize, *j as usize);
                let w = self.inv_mass[i] + self.inv_mass[j];
                if w <= 0.0 {
                    continue;
                }
                let d = self.pos[j] - self.pos[i];
                let len = d.length();
                if len < 1e-9 {
                    continue;
                }
                let dir = d * (1.0 / len);
                let c = len - self.rest[k];
                let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
                self.lambda[k] += dl;
                self.pos[i] -= dir * (self.inv_mass[i] * dl);
                self.pos[j] += dir * (self.inv_mass[j] * dl);
            }
            for (k, [i, j]) in self.bend.iter().enumerate() {
                if !a_tilde_bend.is_finite() {
                    break; // **关弯曲**（`bend_compliance` 非有限）：`inf/inf = NaN` ⇒ 必须显式短路
                }
                let (i, j) = (*i as usize, *j as usize);
                let w = self.inv_mass[i] + self.inv_mass[j];
                if w <= 0.0 {
                    continue;
                }
                let d = self.pos[j] - self.pos[i];
                let len = d.length();
                if len < 1e-9 {
                    continue;
                }
                let dir = d * (1.0 / len);
                let c = len - self.bend_rest[k];
                let dl = (-c - a_tilde_bend * self.bend_lambda[k]) / (w + a_tilde_bend);
                self.bend_lambda[k] += dl;
                self.pos[i] -= dir * (self.inv_mass[i] * dl);
                self.pos[j] += dir * (self.inv_mass[j] * dl);
            }
        }
    }

    /// ③ **接触**（`radius < 0` ⇒ 两类接触一起关、零成本）：**提供者**球采样 + 库仑锥
    /// （`sphere_contacts_project`，与 rope 同款纯搬移）→ ③.5 **刚体代理**
    /// （静态/睡眠 = 墙；动态 = 反作用两腿，见 [`BodyCoupling`]）。
    fn project_contacts(
        &mut self,
        h: f32,
        providers: &dyn ProviderColliders,
        provider_count: u32,
        bodies: &[RigidProxy],
    ) {
        // **自碰撞**（切片 T3；默认关 ⇒ 本行即短路）。列在 `radius` 早退**之前**：它是
        // 布片**自己对自己**的接触，与"世界接触半径"（`radius < 0` = 关世界接触）无关。
        if self.self_contacts.cfg.enabled {
            self.project_self_contacts();
        }
        if self.radius < 0.0 {
            return;
        }
        if provider_count > 0 {
            let Self {
                pos,
                prev,
                inv_mass,
                radius,
                skin,
                friction,
                buf,
                ..
            } = self;
            sphere_contacts_project(
                pos,
                prev,
                inv_mass,
                *radius,
                *skin,
                *friction,
                providers,
                provider_count,
                buf,
            );
        }
        self.body_contacts(bodies, h);
    }

    /// ④ 速度回写 + 阻尼（`v = (x − x_prev)/h`；钉住粒子恒 0）。
    fn write_back(&mut self, h: f32) {
        let inv_h = 1.0 / h;
        for i in 0..self.pos.len() {
            self.vel[i] = if self.inv_mass[i] == 0.0 {
                Vec3::ZERO
            } else {
                (self.pos[i] - self.prev[i]) * inv_h * self.damping
            };
        }
    }

    /// 当前**最大边应变** `max |len − rest| / rest`（判据仪器：收敛/悬垂判据用）。
    /// **只统计结构/剪切**（唯一边）——弯曲约束按设计更软、应变天然更大，混进来会让
    /// 既有的收敛判据失去分辨力（两条曲线的口径不同）。
    pub fn max_strain(&self) -> f32 {
        let mut worst = 0.0f32;
        for (k, [a, b]) in self.cons.iter().enumerate() {
            let len = (self.pos[*b as usize] - self.pos[*a as usize]).length();
            let s = ((len - self.rest[k]) / self.rest[k]).abs();
            worst = worst.max(s);
        }
        worst
    }

    /// 弯曲对的条数（判据/诊断用）。
    pub fn bend_count(&self) -> usize {
        self.bend.len()
    }
}

/// **唯一边集**（结构 + 剪切）：四边形网格的对角线本就是三角形的共享边 ⇒ 一组覆盖两组。
/// 插入序 = 三角扫描序（`HashSet` 只做去重）⇒ **确定性**。
fn unique_edges(tris: &[[u32; 3]]) -> Vec<[u32; 2]> {
    let mut cons: Vec<[u32; 2]> = Vec::new();
    let mut seen: HashSet<[u32; 2]> = HashSet::new();
    for tri in tris {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let key = if a < b { [a, b] } else { [b, a] };
            if seen.insert(key) {
                cons.push(key);
            }
        }
    }
    cons
}

/// **弯曲约束**（规格 `ClothConstraints.bending` 的落点）：取**二环对**（邻居的邻居，排除直连与
/// 自身）—— 四边形网格上即"隔一格"的跨格距离约束，是最省的弯曲代理（与 XPBD 布料同族做法）。
/// 默认 compliance = `Soft` 档（弯曲通常比拉伸软一个量级）；置 `f32::INFINITY` ⇒ 关弯曲。
/// 插入序 = `i` 升序 → 邻居表序 → `k` 升序（**确定性**）。
fn bend_pairs(cons: &[[u32; 2]], n: usize) -> Vec<[u32; 2]> {
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    for [a, b] in cons {
        adj[*a as usize].push(*b);
        adj[*b as usize].push(*a);
    }
    for v in adj.iter_mut() {
        v.sort_unstable();
    }
    let mut bend: Vec<[u32; 2]> = Vec::new();
    let mut seen: HashSet<[u32; 2]> = HashSet::new();
    for i in 0..n as u32 {
        for &j in &adj[i as usize] {
            for &k in &adj[j as usize] {
                if k == i || adj[i as usize].binary_search(&k).is_ok() {
                    continue; // 自身 / 直连（直连已由结构+剪切覆盖）
                }
                let key = if i < k { [i, k] } else { [k, i] };
                if seen.insert(key) {
                    bend.push(key);
                }
            }
        }
    }
    bend
}

/// **薄壳均分质量**：每三角面积 × `ρt/3` 记到三个角上（与窄相 `MeshStore::shell_props` 同口径 ⇒
/// 两者对同一张网给出的总质量一致）。
fn shell_mass(tris: &[[u32; 3]], points: &[Vec3], rho: f32, t: f32) -> Vec<f32> {
    let mut mass = vec![0.0f32; points.len()];
    for tri in tris {
        let (a, b, c) = (
            points[tri[0] as usize],
            points[tri[1] as usize],
            points[tri[2] as usize],
        );
        let area = (b - a).cross(c - a).length() * 0.5;
        let share = rho * t * area / 3.0;
        for v in tri {
            mass[*v as usize] += share;
        }
    }
    mass
}
