//! **绳索最小闭环**（T1，见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §T1）：
//! 1D 粒子链 + **XPBD 距离约束** + **点-形状接触**（走 `interop::ProviderColliders`，与窄相同一个提供者通道）。
//!
//! **为什么先做绳索**（调研档的口径）：布料那条线最硬的缺口不是 XPBD 本身，而是"三角形作为一等几何"
//! （`Shape` 无三角网/无自碰撞/无面元气动）。而绳索只需要"距离约束 + 点-形状接触"⇒ 能在**不碰三角网
//! 自碰撞**的前提下把 XPBD 的核心循环（子步 × 约束投影 × 拉格朗日乘子/柔度）打通，并立起机器无关判据。
//!
//! **口径**（对齐 `SPEC.md` §4.6 与 Müller et al. 2020 XPBD）：
//! - **子步 × 每子步 1 次迭代**（"Small Steps"口径）⇒ 子步数是**刚度**旋钮，不是精度旋钮；
//! - 距离约束：`C = |Δx| − L`、`α̃ = α/h²`、`Δλ = (−C − α̃·λ)/(w₁+w₂+α̃)`、`Δx = ±w·Δλ·∇C`
//!   （`λ` **每子步清零** —— 它是子步内的量）；`α = 0` ⇒ 不可伸长；
//! - 接触：**位置级投影**（= XPBD 接触 `α = 0` 的特例）⇒ **无恢复系数**（XPBD 接触口的天然行为），
//!   且只有**真穿透**（`depth > 0`）才推、带内（预判接触）不推；
//! - 速度由 `v = (x − x_prev)/h` 反推 ⇒ 位置级修正自动进入速度，**不需要冲量求解器**。
//!
//! **本片已有**：最小闭环、**摩擦（切向库仑锥）**、**粒子↔刚体耦合（Akinci 式最小实现）**、
//! 门面接线（`World::add_rope`）；**待落地**：体积/弯曲约束、自碰撞、GPU 档。
//! 判据在 `tests/rope_minimal.rs` 与 `crates/vxl-phys/tests/rope_scene.rs`。
use crate::rigid::{crossed_face, shape_penetration, RigidProxy, RigidReaction};
use vxl_phys_core::{interop::ProviderColliders, Mat3, Quat, Shape, Vec3};

/// 入口面缓存里的"无"（见 `Rope::entry`）。
const FACE_NONE: u8 = 255;

/// **虚拟转动状态**（与 `bodies` 同序，每 tick 重置；计划 2c-1 / 2c-3 引入）。
///
/// **为什么收成一个结构**：`Rope` 受 god 门**成员棘轮**（只增即红，阈值 24 成员）—— 这两步一上来就加了三个
/// 字段（25 > 24）⇒ 按先例（`world_soft.rs` 的 `FluidBoundary`：把散字段收起来腾成员位）合并成一个。
#[derive(Default)]
struct VirtRot {
    /// 当前子步的虚拟朝向：每 tick 从 `b.rot` 起，按 `b.angvel + dw` 逐子步 `integrate_angular` 推进。
    cur: Vec<Quat>,
    /// 上一子步的虚拟朝向（`crossed_face` 要"两个位姿各带朝向"）。
    prev: Vec<Quat>,
    /// 子步内累积的**角速度反作用** `Σ I⁻¹·(r × J)`（角反作用开关关时恒 0）。
    dw: Vec<Vec3>,
}

/// **一次体接触的收集项**（§8.4.24）：先收全部命中、再按**物理序**施加 ⇒ 结果与**数组方向无关**。
///
/// 为什么必须这么做：`project_body_contacts` 是**顺序 Gauss-Seidel**（体速度 `body_dv` 就地推进），
/// 所以"谁先被处理"会改变结果。实测（§8.4.23）：把粒子循环反向，常规载重下的横向漂移**符号翻转**
/// （0.5 kg：`+0.098 → −0.124`）⇒ 那是**顺序偏差**，不是物理。把施加顺序改成**物理量的函数**
/// （`|Δx|` 降序 = 两端先、向中间收；次级键取粒子自己的 `x`）⇒ 翻转数组不再改变任何东西。
#[derive(Clone, Copy)]
struct BodyHit {
    /// 粒子号。
    i: u32,
    /// 排序主键：粒子自己的 `x`（**物理量** ⇒ 施加顺序与**数组方向无关**）。
    ord: f32,
    /// 排序次键：`−|Δx|`（同 `x` 不可能出现 ⇒ 仅作确定性兜底）。
    x: f32,
    /// 该次接触的法线 / 穿透 / 接触点（`box_hit` 的读数）。
    n: Vec3,
    depth: f32,
    q: Vec3,
}

/// 绳索：等距粒子链 + XPBD 距离约束 + 点-形状接触。
pub struct Rope {
    /// 粒子位置（世界系）。
    pub pos: Vec<Vec3>,
    /// 子步起点位置：速度由 `(pos − prev)/h` 反推。
    prev: Vec<Vec3>,
    /// 粒子速度（`m/s`）。
    pub vel: Vec<Vec3>,
    /// 逆质量（`0` = **钉住**：约束与接触都不会移动它，速度恒为 0）。
    pub inv_mass: Vec<f32>,
    /// 每条距离约束的拉格朗日乘子（每子步清零）。
    lambda: Vec<f32>,
    /// 段长（rest length，`m`）。
    pub rest_len: f32,
    /// compliance α（`m/N`）：`0` = 不可伸长；档位见 [`crate::Stiffness::alpha`]。
    pub compliance: f32,
    /// 粒子半径（接触用，`m`）：`0` = 质点（贴在面上）。
    pub radius: f32,
    /// 子步数（每子步内再跑 `chain_iterations` 遍距离约束）。
    pub substeps: u32,
    /// **链的迭代数**（每子步对距离约束扫几遍）：`1` = 现状（**默认，逐位不变**）；`>1` ⇒ 链更"硬"。
    ///
    /// **为什么需要这个旋钮**（§8.4.8.1 的结论）：`compliance = 0` 只表示"无穷刚度"的**连续**口径，
    /// 离散下每子步 1 遍 Gauss-Seidel 会让张力**每子步只传约一颗粒子** ⇒ 绳在静载下**蠕变**。
    /// 实测（2026-09-27）：紧绳两端钉住、1.0 m 跨距，自己先垂到 **0.13 m** ⇒ 它是个**软弹簧**；
    /// 1 kg 盒压上去后长窗以 ~0.1 m/s **匀速下沉**（≈1.7 mm/tick）。提高本值 = 约束收敛更充分。
    /// **默认必须留在 1**：默认档冻结值不得动（`rope_scene` 的冻结哈希守）。
    pub chain_iterations: u32,
    /// 接触皮肤带（预判接触宽度，`m`）。
    pub skin: f32,
    /// **接触摩擦系数 μ**（库仑，位置口径）：每个接触把本子步的**切向滑移**限制到
    /// `μ·法向修正量`。`0` = 无摩擦（纯法向投影）。
    ///
    /// 为什么用"位移"而不是"力"：XPBD 的位置级接触里没有显式冲量，**法向修正量就是法向
    /// 冲量的位置代理**——静止在斜面上时它 ≈ `|g_n|·h²`（重力在法向的分量），而切向位移
    /// ≈ `|g_t|·h²` ⇒ 这条判据恰好退化成解析的静摩擦阈值 **`tanθ ≤ μ`**（判据里有双向实测）。
    pub friction: f32,
    /// **刚体接触的柔度 α_c**（`m/N`，XPBD 口径）：`0` = 硬接触 —— 一个子步内解掉穿透 ⇒
    /// 会把深穿透**反射成巨大速度**（0.035 m ⇒ 8.4 m/s/粒子），且与 XPBD 的**软链**刚度错配
    /// ⇒ **抖动、十几 tick 后把体弹飞**（§8.3 的已知缺口）。正数 ⇒ 接触像弹簧：
    /// 法向修正 `= depth/(w_p + w_b + α̃)`、`α̃ = α_c/h²`（与链的 `compliance` 同一套 XPBD 语义，
    /// 只是分母换成两体逆质量之和）。物理上 `α_c = 1/k`，`k` = 接触刚度（N/m）。
    pub contact_compliance: f32,
    /// 速度阻尼（每子步乘一次）：`1.0` = 无阻尼。**不是** XPBD 的组成部分，只为把"悬垂形状"
    /// 做成**稳态读数**（否则绳永远在摆，读数只能取窗口均值，见测量协议 §5）。
    pub damping: f32,
    /// 接触查询缓冲（复用，免每粒子每子步一次堆分配）。
    buf: Vec<vxl_phys_core::interop::InteropContact>,
    /// **体接触的收集暂存**（§8.4.24；复用免分配）：先收全部命中、排序、再施加 ⇒ 与数组方向无关。
    hits: Vec<BodyHit>,
    /// **粒子↔刚体反作用**（累加**冲量**，每 tick 开头清空）：门面 `÷dt` 后作为力/力矩加到体上
    /// （与 2b 流体反作用同段位、同量纲口径）。
    pub reactions: Vec<RigidReaction>,
    /// **刚体的速度增量**（与 `step` 传入的 `bodies` 同序，每 tick 开头清零）：
    /// 门面做 `bodies.linvel[i] += body_dv[i]`（在体解算之前施加 ⇒ 下一 tick 生效）。
    pub body_dv: Vec<Vec3>,
    /// 子步内**虚拟位移**（同上序）：体自己走的 + 我们推开的，**增量累加** —— 见 `project_body_contacts`
    /// 里那条"不能乘整段时间"的注（那是所有"参数怎么调都逃逸"的真凶）。
    body_disp: Vec<Vec3>,
    /// **虚拟转动**（朝向 / 上一朝向 / 角速度反作用；见 [`VirtRot`]）。
    vrot: VirtRot,
    /// **角反作用开关**（计划 2c-3，**默认关**）：开 ⇒ 按库仑/冲量口径把 `r × J` 回填成体的角冲量
    /// （门面按 `angular_impulse_contract` 钉住的契约注入），并把它折进"虚拟角速度"。
    /// **默认关**是刻意的：打开会改变动态盒在绳上的读数（那是**换代级**），先按"0=关字段"的先例落地，
    /// 让"打开会怎样"由实测登记（§8.4.31）。静态体/睡眠体不受影响（`inv_mass`/`awake` 已为 0）。
    pub angular_reaction: bool,
    /// **位置口径的补足量**（同上序，每 tick 开头清零；**门面读它做 `bodies.position += dx`**）：
    /// `Σ −n·w_b·(λ_geom − λ)` —— `λ = min(λ_geom, λ_vel)` 被**速度钳位**压掉的那一部分，
    /// **只补位置、不补速度**。
    ///
    /// **为什么必须有**（§8.4.9 实测）：静载下 `approach ≈ 0 ⇒ λ_vel ≈ 0 ⇒ λ ≈ 0` ⇒ 接触
    /// **只回速度、不回位置** ⇒ 体每 tick 先按 `v·dt` 走过的 `g·dt² = 2.72 mm` **一去不回**
    /// （整漏等效速率 `g·dt = 0.1635 m/s`，实测下沉 −0.1354 = **83%**）。
    /// **为什么用"差值"而不是 `λ_geom` 本身**：`body_disp`（虚拟位姿）已经含了钳位后的那一份
    /// ⇒ 补足 `λ_geom − λ` 之后，体在本 tick 的**总位置效应恰好 = `λ_geom`**（不重复计账），
    /// 且**钳位生效时（运动/冲击工况）恒为 0 ⇒ 既有工况逐位不变**。
    /// 门面做 `bodies.position[p.body] += dx`（速度侧不动 ⇒ 不会把位置修正反射成速度）。
    pub body_dx: Vec<Vec3>,
    /// **入口面缓存**（与粒子同序）：`(体号, 面号)` —— 该粒子**从哪张面**进的盒（§8.4.5.1）。
    /// 为什么必须缓存：穿越判据只在**进来的那一子步**成立（实测：只做穿越 ⇒ 静止接触 = 零接触、
    /// 盒子自由落体 −3.7868）⇒ 接触是**一段状态**：只要还在该面内侧带内就继续按它推，出去了或
    /// 横向滑出面范围就释放。只用 `(粒子, 体, 位姿历史)` 决定 ⇒ 逐位可复现。
    /// `体号 == u32::MAX` = 无缓存。
    entry: Vec<(u32, u8)>,
}

impl Rope {
    /// **直线投放**：`nodes` 个粒子均布在 `a → b` 上，**两端钉住**；段长 = 弦长/(nodes−1)（紧绳）。
    pub fn line(a: Vec3, b: Vec3, nodes: usize, radius: f32) -> Self {
        Self::span(a, b, nodes, (b - a).length(), radius)
    }

    /// 同上，但**段长按给定总长**：`total_len > 弦长` ⇒ 松绳（初始被压缩，会自己垂下来）。
    pub fn span(a: Vec3, b: Vec3, nodes: usize, total_len: f32, radius: f32) -> Self {
        let n = nodes.max(2);
        let denom = n as f32 - 1.0;
        let mut pos = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as f32 / denom;
            pos.push(a + (b - a) * t);
        }
        let mut inv_mass = vec![1.0f32; n];
        inv_mass[0] = 0.0;
        inv_mass[n - 1] = 0.0;
        Rope {
            prev: pos.clone(),
            vel: vec![Vec3::ZERO; n],
            lambda: vec![0.0f32; n - 1],
            pos,
            inv_mass,
            rest_len: total_len / denom,
            compliance: 0.0,
            radius,
            substeps: 8,
            chain_iterations: 1,
            skin: 0.01,
            friction: 0.5,
            // **默认 0（硬接触）**：实测柔度 α_c = 1e-4 反而**太软**（盒子 60 tick 就沉到 y≈0.26），
            // 而硬接触是"接得住、留不住"（见 `tests/rigid_coupling_gap.rs` 的钉住判据）。
            // 旋钮保留：等"入口法线/扫掠"那条结构修法落地后再回来重扫（那才是真因）。
            contact_compliance: 0.0,
            damping: 1.0,
            buf: Vec::new(),
            hits: Vec::new(),
            reactions: Vec::new(),
            body_dv: Vec::new(),
            body_disp: Vec::new(),
            vrot: VirtRot::default(),
            body_dx: Vec::new(),
            angular_reaction: false,
            entry: vec![(u32::MAX, FACE_NONE); n],
        }
    }

    /// 粒子数。
    pub fn nodes(&self) -> usize {
        self.pos.len()
    }

    /// **入口面缓存**（只读，判据/调试用）：`(体号, 面号)`；`体号 == u32::MAX` = 该粒子当前无缓存。
    pub fn entry_faces(&self) -> &[(u32, u8)] {
        &self.entry
    }

    /// **Box 的接触判定**（"入口面" + 缓存；从 `project_body_contacts` 抽出，god 门最长函数约束）。
    ///
    /// 为什么不用"当前最近面"：体相对绳线**下沉**时最近面会在底面↔侧面↔顶面之间**翻转** ⇒ 推力
    /// 方向突变 ⇒ 踢击把体送走（柔度/入口法线/质量比/虚拟位姿共 4 次否定都栽在这上面）；而"它**穿过**
    /// 的那个面"只要还在从下面顶就一直是底面 ⇒ 不翻转。**穿越判据只在一子步成立** ⇒ 接触按
    /// "一段状态"维护：记住面号，直到出去（或横向滑出面范围）。非 Box 走点式（无"面"歧义）。
    #[allow(clippy::too_many_arguments)] // 接触包：i/j/代理/虚拟位姿（位置+朝向）/上一朝向/h/半径
    fn box_hit(
        &mut self,
        i: usize,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        qrot: Quat,
        qrot_prev: Quat,
        h: f32,
        radius: f32,
    ) -> Option<(Vec3, f32, Vec3)> {
        let comp = |v: Vec3, k: usize| match k {
            0 => v.x,
            1 => v.y,
            _ => v.z,
        };
        if let Shape::Box { half } = b.shape {
            // **几何一律走"虚拟朝向"**（§8.4.29 / 2c-1）：体的转动由 `qrot` 带进来；
            // `ω = 0` 时 `qrot == qrot_prev == b.rot` ⇒ 与既有行为逐位一致（零换代门靠这条）。
            let m = Mat3::from_quat(qrot);
            let local = m.transpose_mul_vec3(self.pos[i] - vpos);
            let mut face = FACE_NONE;
            if self.entry[i].0 == b.body && self.entry[i].1 != FACE_NONE {
                let f = self.entry[i].1;
                let (k, s) = (
                    (f / 2) as usize,
                    if f.is_multiple_of(2) { -1.0 } else { 1.0 },
                );
                let (j1, j2) = ((k + 1) % 3, (k + 2) % 3);
                let in_face = comp(local, j1).abs() <= comp(half, j1) + radius
                    && comp(local, j2).abs() <= comp(half, j2) + radius;
                // **滞回**：进入要求 `out < radius`（穿越判据同口径），**保持**放宽到 `1.5·radius`
                // —— 静止接触恰好坐在 `out = radius` 的刀锋上，不放宽会反复释放/重建
                // （实测：命中只有 2~3 颗、盒子缓慢下沉）。
                if in_face && s * comp(local, k) - comp(half, k) < radius * 1.5 {
                    face = f; // 仍在该面内侧带内 ⇒ 继续用它
                } else {
                    self.entry[i] = (u32::MAX, FACE_NONE); // 释放
                }
            }
            if face == FACE_NONE {
                // 无缓存 ⇒ 跑一次穿越测试；命中就**记下这张面**。
                // **两个位姿都要给**：动的是盒子（粒子几乎不动）⇒ 只用一个位姿永远测不到穿越
                // （实测：整场"缓存命中 = 0"、盒子直接穿过绳线）。
                let vpos_prev = vpos - (b.linvel + self.body_dv[j]) * h;
                crossed_face(
                    qrot_prev,
                    qrot,
                    half,
                    vpos_prev,
                    vpos,
                    self.prev[i],
                    self.pos[i],
                    radius,
                )
                .map(|(n, depth, q, f)| {
                    self.entry[i] = (b.body, f);
                    (n, depth, q)
                })
            } else {
                let (k, s) = (
                    (face / 2) as usize,
                    if face.is_multiple_of(2) { -1.0 } else { 1.0 },
                );
                let out = s * comp(local, k) - comp(half, k);
                let n_local = match k {
                    0 => Vec3::new(s, 0.0, 0.0),
                    1 => Vec3::new(0.0, s, 0.0),
                    _ => Vec3::new(0.0, 0.0, s),
                };
                // 接触点：球心投影到该面平面上（第 k 轴钉到 ±half，另两轴取粒子值）。
                let q_local = match k {
                    0 => Vec3::new(s * half.x, local.y, local.z),
                    1 => Vec3::new(local.x, s * half.y, local.z),
                    _ => Vec3::new(local.x, local.y, s * half.z),
                };
                Some((
                    m.mul_vec3(n_local),
                    radius - out,
                    vpos + m.mul_vec3(q_local),
                ))
            }
        } else {
            shape_penetration(&b.shape, vpos, b.rot, self.pos[i], radius)
        }
    }

    /// 钉住 / 松开第 `i` 个粒子。
    pub fn set_pinned(&mut self, i: usize, pinned: bool) {
        if i < self.inv_mass.len() {
            self.inv_mass[i] = if pinned { 0.0 } else { 1.0 };
        }
    }

    /// 第 `k` 段的当前长度（判据用：不可伸长性）。
    pub fn segment_len(&self, k: usize) -> f32 {
        (self.pos[k + 1] - self.pos[k]).length()
    }

    /// 推进一个 `dt`（内部再切 `substeps` 个子步）；接触走 `providers` 的 **id `0..provider_count`**
    /// （门面注册的提供者就是这段连续 id；用"个数"而不是 id 列表 ⇒ 门面不必每 tick 造一个 Vec），
    /// 以及 `bodies` 里的**刚体代理**（粒子↔刚体，Akinci 式最小实现；`&[]` = 不耦合）。
    pub fn step(
        &mut self,
        dt: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        provider_count: u32,
        bodies: &[RigidProxy],
    ) {
        self.reactions.clear();
        self.body_dv.clear();
        self.body_dv.resize(bodies.len(), Vec3::ZERO);
        self.body_disp.clear();
        self.body_disp.resize(bodies.len(), Vec3::ZERO);
        self.vrot.cur.clear();
        self.vrot.cur.extend(bodies.iter().map(|b| b.rot));
        self.vrot.prev.clear();
        self.vrot.prev.extend(bodies.iter().map(|b| b.rot));
        self.body_dx.clear();
        self.body_dx.resize(bodies.len(), Vec3::ZERO);
        self.vrot.dw.clear();
        self.vrot.dw.resize(bodies.len(), Vec3::ZERO);
        let h = dt / self.substeps.max(1) as f32;
        for _ in 0..self.substeps {
            self.substep(h, gravity, providers, provider_count, bodies);
        }
    }

    /// 单个子步：预测 → 距离约束 → 接触（提供者 / 刚体）→ 速度回写。
    fn substep(
        &mut self,
        h: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        provider_count: u32,
        bodies: &[RigidProxy],
    ) {
        let n = self.pos.len();
        // ① 预测：`v ← v + g·h`、`x_prev ← x`、`x ← x + v·h`（钉住粒子原地不动、速度清零）。
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                self.prev[i] = self.pos[i];
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] += gravity * h;
            self.prev[i] = self.pos[i];
            self.pos[i] += self.vel[i] * h;
        }
        // ② 距离约束（Gauss-Seidel 顺序推进；`λ` **每子步**清零、子步内按迭代累加 = XPBD 口径）。
        //    `chain_iterations` 遍/子步是**链的"硬"旋钮**（默认 1 = 现状、逐位不变；见字段注）。
        for l in self.lambda.iter_mut() {
            *l = 0.0;
        }
        let a_tilde = self.compliance / (h * h);
        for _ in 0..self.chain_iterations.max(1) {
            for k in 0..n.saturating_sub(1) {
                let (i, j) = (k, k + 1);
                let w = self.inv_mass[i] + self.inv_mass[j];
                if w <= 0.0 {
                    continue; // 两粒子都钉住 ⇒ 该约束无自由度
                }
                let d = self.pos[j] - self.pos[i];
                let len = d.length();
                if len < 1e-9 {
                    continue; // 退化（两端重合）：方向无定义，跳过（下一子步自会分开）
                }
                let dir = d * (1.0 / len);
                let c = len - self.rest_len;
                let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
                self.lambda[k] += dl;
                self.pos[i] -= dir * (self.inv_mass[i] * dl);
                self.pos[j] += dir * (self.inv_mass[j] * dl);
            }
        }
        // ③ 接触：位置级投影（只有真穿透才推 ⇒ 无恢复系数）。
        if self.radius >= 0.0 && provider_count > 0 {
            self.project_contacts(providers, provider_count);
        }
        // ③.5 粒子 ↔ 刚体：同款投影 + 库仑锥 + **反作用回填**（`&[]` ⇒ 零成本跳过）。
        if !bodies.is_empty() {
            self.project_body_contacts(bodies, h);
        }
        // ④ 速度回写 + 阻尼。
        let inv_h = 1.0 / h;
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] = (self.pos[i] - self.prev[i]) * inv_h * self.damping;
        }
    }

    /// 逐粒子把穿透推到面上（多接触按 Gauss-Seidel 顺序逐个推；带内预判不推），
    /// 并按**库仑锥**限制切向滑移（位置口径，见 [`Rope::friction`]）。
    fn project_contacts(&mut self, providers: &dyn ProviderColliders, provider_count: u32) {
        // `buf` 借出去才能再借 `self.pos`（同窄相 `prims.rs` 的 `mem::take` 手法）。
        let mut buf = std::mem::take(&mut self.buf);
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            for id in 0..provider_count {
                buf.clear();
                if !providers.contacts_sphere(id, self.pos[i], self.radius, self.skin, &mut buf) {
                    continue;
                }
                for c in &buf {
                    if c.depth <= 0.0 {
                        continue; // 带内预判不推（无恢复系数的位置口径）
                    }
                    let n = c.normal;
                    // ① 法向：推到面上（位移 = 穿透量）。
                    self.pos[i] += n * c.depth;
                    // ② 切向：库仑锥 —— 摩擦能"吃掉"的切向位移上限 = `μ·法向修正量`。
                    //    （力的等效：切向位移 `d` 对应 `F = m·d/h²`，锥内 `F ≤ μN` ⇔ `d ≤ μ·法向位移`。）
                    //    **锥内整段吃掉 ⇒ 完全黏住（静摩擦）**；超出 ⇒ 吃掉 `μ·法向`、余下按动摩擦滑掉。
                    //    ⚠️ 只扣"超出部分"是错的（首版就这么写）：锥内不修正 ⇒ 每子步照落一格
                    //    `g_t·h²` ⇒ **恒定蠕变**（实测率精确 ∝ h：子步 1/2/4/8/16 ⇒ 1.271/0.636/0.318/
                    //    0.159/0.079 m/千步），且**两粒子（无链张力）情形同速率** ⇒ 模型本身的问题，不是张力。
                    if self.friction > 0.0 {
                        let dp = self.pos[i] - self.prev[i];
                        let t = dp - n * dp.dot(n);
                        let slip = t.length();
                        if slip > 0.0 {
                            let budget = self.friction * c.depth;
                            let removed = if slip < budget { slip } else { budget };
                            self.pos[i] -= t * (removed / slip);
                        }
                    }
                }
            }
        }
        self.buf = buf;
    }

    /// **粒子 ↔ 刚体**：逐粒子对每个代理做投影（与提供者那套同款：法向推出 + 库仑锥），
    /// 并把**粒子的动量变化取反**累加为体的冲量/角冲量（静态体收不到反作用）。
    ///
    /// **两处与提供者路径的差别**（写清）：
    /// - 摩擦的滑移量取**相对体**的（`dp − v_body·h`）—— 提供者是静态地形，这一项恒为零；
    /// - 反作用口径：粒子动量变化 `m·Δ/h`（`m = 1/inv_mass`）取反，作用在**体表接触点** ⇒
    ///   绕体原点的角冲量 `r × impulse`；门面 `÷dt` 后按力施加（账：引擎每子步施加一次 ⇒ `F·dt`）。
    fn project_body_contacts(&mut self, bodies: &[RigidProxy], h: f32) {
        // `inv_h`/`friction` 现在归 `apply_body_hit`（响应体已抽出）；这里只留 `radius`（命中判定用）。
        let radius = self.radius;
        for (j, b) in bodies.iter().enumerate() {
            // **子步内虚拟位姿**（增量推进）：本子步它自己走 `(v + Δv)·h`，再加上我们推它的修正位移。
            // ⚠️ **不要写成 `(linvel + Δv)·(sub_idx·h)`**：那会把"当前累计的 Δv"乘上**整段时间**
            // （最后一个子步乘 8h）⇒ 虚拟位姿被放大 ⇒ 接触几何算错。这是**公式错误**（已修），
            // 但**实测修完仍然逃逸**（质量比 m ∈ {1,5,20,100,200} 全部逃逸）⇒ 它**不是**逃逸的成因。
            self.body_disp[j] += (b.linvel + self.body_dv[j]) * h;
            // **虚拟朝向**（§8.4.29 / 2c-1）：上一子步的朝向先存档（`crossed_face` 要两个位姿各带朝向），
            // 再用 `b.angvel` 按本子步时长推进。**静态/睡眠体的 `angvel` 为 0**（门面填 0，§8.4.27）
            // ⇒ 与既有行为逐位一致。
            self.vrot.prev[j] = self.vrot.cur[j];
            // 角速度 = 代理给的外部 ω + **本子步累积的反作用**（后者只在开关打开时非零）。
            self.vrot.cur[j] = self.vrot.cur[j].integrate_angular(b.angvel + self.vrot.dw[j], h);
            let vpos = b.pos + self.body_disp[j];
            // **① 收集**（先不施加）：把本子步的全部命中收进 `hits`，命中判定与原来的逐粒子调用等价
            // （`box_hit` 只读 `self.pos[i]`/`prev[i]` 与体位姿，而响应只改**别的**粒子 ⇒ 顺序无关）。
            self.hits.clear();
            for i in 0..self.pos.len() {
                if self.inv_mass[i] == 0.0 {
                    continue;
                }
                // **接触面 = "入口面"（必须缓存）**（§8.4.5）：判定抽到 `box_hit`
                // —— 本函数受 god 门"最长函数 ≤ 120 行"约束，那段不留在原地。
                let Some((nn, depth, q)) = self.box_hit(
                    i,
                    j,
                    b,
                    vpos,
                    self.vrot.cur[j],
                    self.vrot.prev[j],
                    h,
                    radius,
                ) else {
                    continue;
                };
                if depth <= 0.0 {
                    continue;
                }
                let dx = self.pos[i].x - vpos.x;
                self.hits.push(BodyHit {
                    i: i as u32,
                    ord: self.pos[i].x,
                    x: -dx.abs(),
                    n: nn,
                    depth,
                    q,
                });
            }
            // **② 按**物理量**排**（§8.4.24）：主键取**粒子自己的 `x`**（世界横坐标，升序）。
            // 为什么不是 `|Δx|` 降序：那一版实测**修好了 1 kg 却把 0.5 kg 从"托住"打成"逃逸"**
            // （§8.4.24.1）⇒ 施加顺序变了就是换了一条 GS 路径。**取 `x` 升序的好处是**：
            // 对"绳沿 `x` 铺开、索引即左→右"的常规场景，它与**原来的数组顺序逐位等价**（既有读数全不动），
            // 而它**只依赖几何、不依赖数组方向** ⇒ 把数组反转不再改变任何结果（那正是 §8.4.23 的病灶）。
            self.hits
                .sort_by(|a, c| a.ord.total_cmp(&c.ord).then(a.x.total_cmp(&c.x)));
            let hits = std::mem::take(&mut self.hits);
            // **③ 施加**（顺序 Gauss-Seidel：`body_dv` 就地推进 —— 只是顺序变成了物理序）。
            for hit in &hits {
                self.apply_body_hit(hit.i as usize, j, b, vpos, h, hit.n, hit.depth, hit.q);
            }
            self.hits = hits; // 归还暂存（`mem::take` 的配对；`hits` 的借用到此结束）
        }
    }

    /// **施加一次体接触的响应**（§8.4.24 从 `project_body_contacts` 抽出 —— 那里加了"收集/排序"
    /// 两段后到 125 行、越 god 门 120）。内容：两体按逆质量分担的几何解 `λ_geom` → 速度钳位 `λ_vel`
    /// （= 无恢复系数）→ **位置口径补足** `body_dx`（只补位置不补速度，§8.4.9）→ 法向推出 + 库仑锥摩擦
    /// （相对速度与法向同口径，§8.4.19）→ 反作用回填（含**就地推进**虚拟状态 = Gauss-Seidel）。
    #[allow(clippy::too_many_arguments)] // 接触包：i/j/体/位姿/步长/法线/深度/接触点
    fn apply_body_hit(
        &mut self,
        i: usize,
        j: usize,
        b: &RigidProxy,
        vpos: Vec3,
        h: f32,
        n: Vec3,
        depth: f32,
        q: Vec3,
    ) {
        let inv_h = 1.0 / h;
        let friction = self.friction;
        // **按逆质量分担的两体约束**（α=0）：`λ = depth/(w_p + w_b)`，
        // 粒子沿 +n 让 `w_p·λ`、体（若有质量）沿 −n 让 `w_b·λ` —— 相对位移正好合上穿透量。
        // ⚠️ **不能让粒子吃满穿透量**（首版就是这么写的）：那样体的位置永远不被挡，只吃到
        // 与穿透量成正比的巨大冲量 ⇒ 盒子"被踹飞着掉下去"（实测 y 比自由落体还低）。
        // 静态提供者/静态体（`w_b = 0`）下 `λ = depth/w_p` ⇒ 与既有行为**逐位一致**。
        let w_p = self.inv_mass[i];
        let w_b = b.inv_mass;
        // **冲量上限 = 消掉本次接近速度所需**（顺序冲量的标准钳位；物理上 = 无恢复系数）。
        // 为什么必须有：位置口径按几何穿透量解会让"一子步内冒出的深穿透"反射出巨大速度
        // ——实测 0.035 m 的穿透 ⇒ 8.4 m/s/粒子 ⇒ 十几颗粒子把盒子踹成 **+38 m/s**。
        // 钳位后：体不会被推得超过"刚好不再接近"，穿透量本身不强行愈合（不再增长即可）。
        let v_p = (self.pos[i] - self.prev[i]) * inv_h;
        let v_b = b.linvel + self.body_dv[j];
        // **接触点速度**（§8.4.30 / 计划 2c-2）：体在转时，**材料点**的速度是 `v + ω × r`
        // （`r = q − vpos`），不是体心速度。**法向的 `approach` 与摩擦的滑移量共用这一个口径** ——
        // §8.4.19 与 §8.4.25 两次都栽在"两支用了不同的相对速度"上（法向用新值、摩擦用旧值），
        // 所以这里**一次改齐**。`ω = 0` ⇒ `v_point == v_b` ⇒ 与既有行为逐位一致（零换代门）。
        let v_point = v_b + b.angvel.cross(q - vpos);
        let approach = (v_point - v_p).dot(n);
        let lam_geom = depth / (w_p + w_b + self.contact_compliance / (h * h));
        let lam_vel = if approach > 0.0 {
            approach * h / (w_p + w_b)
        } else {
            0.0
        };
        let lam = lam_geom.min(lam_vel);
        // **位置口径的补足**（§8.4.9）：被速度钳位压掉的那一份**只补位置、不补速度**
        //（静载下钳位恒 0 ⇒ 不补就是"只回速度不回位置"的位置漏）。速度侧不动 ⇒
        // 不会把位置修正反射成速度；钳位生效时（运动/冲击）该项恒 0 ⇒ 既有工况逐位不变。
        if b.inv_mass > 0.0 && lam_geom > lam {
            self.body_dx[j] -= n * (b.inv_mass * (lam_geom - lam));
        }
        if lam <= 0.0 {
            return;
        }
        let before = self.pos[i];
        // 法向推出（位移 = `w_p·λ`）。
        self.pos[i] += n * (w_p * lam);
        // 切向：库仑锥（锥内整段吃掉 = 静摩擦；超出按动摩擦滑）——同提供者路径，
        // 但滑移量取**相对体**的（体在动时绳不该被"粘"在原地）。
        if friction > 0.0 {
            // **相对速度与法向同口径**（§8.4.19）：法向那支用 `v_b = b.linvel + body_dv[j]`
            // （见上面的 `approach`），摩擦这里原来只用 `b.linvel` ⇒ 子步内 `body_dv` 一涨，
            // 滑移估计就**过期** ⇒ 系统性切向偏置（实测：`μ=0` 时盒子 3 维下托得住，
            // `μ` 越大横向蠕变越猛 ⇒ ~1 mm/tick、400 tick 滑出绳端）。
            let dp = self.pos[i] - self.prev[i] - v_point * h;
            let t = dp - n * dp.dot(n);
            let slip = t.length();
            if slip > 0.0 {
                // **候选①（§8.4.25 实验）：库仑锥的冲量口径** `|J_t| ≤ μ·J_n`。
                // 本接触**实际**施加的法向冲量 ∝ `λ`（`J_n = m_p·w_p·λ/h`），**不是**几何穿透量
                // `depth`（后者只在几何支被选中时等于 λ 的口径）⇒ 位置预算取 `μ·w_p·λ`。
                // ⚠️ 这一版**预期更小**（静载下 λ 被速度钳位到远小于 λ_geom）⇒ 若它把托住打坏，
                // 说明"现在的摩擦预算偏大"本身是**承重**的（`μ·depth` 在静载下远大于真实 `μ·J_n`）。
                let budget = friction * (w_p * lam);
                let removed = if slip < budget { slip } else { budget };
                self.pos[i] -= t * (removed / slip);
            }
        }
        // 反作用（只回填给能动的体），并**就地推进虚拟状态** ⇒ 同一子步里后续粒子
        // 看到的是"已经被推开、且已被推走"的体（Gauss-Seidel；不做这层就是 Jacobi 叠加）。
        if b.inv_mass > 0.0 {
            let d = self.pos[i] - before;
            let impulse = d * (-inv_h / self.inv_mass[i]);
            let r = q - vpos;
            let torque = Vec3::new(
                r.y * impulse.z - r.z * impulse.y,
                r.z * impulse.x - r.x * impulse.z,
                r.x * impulse.y - r.y * impulse.x,
            );
            self.body_dv[j] += impulse * b.inv_mass;
            self.body_disp[j] += impulse * (b.inv_mass * h);
            // **角反作用**（计划 2c-3，开关默认关）：`r × J` 折成角速度增量，与 `body_dv` 对平移对称
            // ⇒ 同一子步里后续粒子看到"已被转过一点"的体。量级由法向那支的钳位
            // `min(λ_geom, λ_vel)` 限制（`r×J` 与 `J` 同源）⇒ 不必另设钳位。
            // `I⁻¹` 在本体系逐轴相乘、再转回世界（与积分器 `apply_world_inv_inertia` 同口径）。
            if self.angular_reaction {
                let m = Mat3::from_quat(self.vrot.cur[j]);
                let tau_local = m.transpose_mul_vec3(torque);
                let dw = m.mul_vec3(tau_local.mul_per_elem(b.local_inv_inertia));
                self.vrot.dw[j] += dw;
            }
            match self.reactions.iter_mut().find(|e| e.body == b.body) {
                Some(e) => {
                    e.impulse += impulse;
                    e.torque += torque;
                }
                None => self.reactions.push(RigidReaction {
                    body: b.body,
                    impulse,
                    torque,
                }),
            }
        }
    }
}
