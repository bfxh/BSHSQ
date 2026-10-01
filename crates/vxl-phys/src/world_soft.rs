//! **软体域（绳索）通道** + 2b 边界的**成组状态**。
//!
//! **为什么 `FluidBoundary` 定义在这里**：`World` 是 god 门棘轮下的**记录型**结构（成员数只准减），
//! 加一个域就得腾一个成员位 ⇒ 把 2b 那一族的 3 个散字段收成一个结构（成员 23 → 22，再 +1 给绳索）。
//! 定义随域走、`World` 只留一个字段。
//!
//! **绳索通道**（T1，`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8）：与液体域同款——每 tick **一次**
//! （体子步全部完成之后）推进；接触走**统一提供者通道**（`Providers` 实现 `ProviderColliders`，
//! 与窄相同一个 id 空间，见 `world_step.rs` 的 `fluid_pass` 同款段位）。**没有绳索的场景逐位不变**
//! （`ropes` 空 ⇒ `rope_pass` 首行短路）⇒ 默认档判据不受影响。
//!
//! **本片边界**：绳索的接触走**提供者**（地形）+ **刚体代理**（形状：Sphere/Box/Capsule）；
//! 自碰撞、`Compound`/`Cylinder`/`Cone`/`ConvexHull` 代理、体积/弯曲约束都属后续切片。
use super::*;
use crate::world_step::coupling as cpl; // 耦合契约短别名（用 add_force / add_tick_torque / apply_two_leg_reactions）

/// 2b（Akinci 边界粒子）那一族的成组状态（原为 `World` 的 3 个散字段：开关 / 暂存 / 覆盖集）。
#[derive(Default)]
pub struct FluidBoundary {
    /// 与 `fluids` 同序的 2b 开关：`true` = 该流体每 tick 重建边界粒子并回流反作用。
    pub two_b: Vec<bool>,
    /// 边界粒子生成的暂存 `(体 id, 形状, 位姿)`（复用免每 tick 分配）。
    pub scratch: Vec<(u32, Shape, vxl_phys_fluid::BodyPose)>,
    /// **覆盖集**（**按流体**各一份：`covered[fi]` 与 `bodies` 同序，每 tick 由该流体的
    /// `refresh_fluid_boundary` 重建）= 2a 对该流体的让位集。修 §2 C4：原先跨流体单数组
    /// ⇒ 只有最后一个 2b 流体的覆盖集有效 ⇒ 2a/2b 双重计账（判据 `fluid_covered_multi`：
    /// 修前体被 2a 浮力从 y 1.27 弹到 5.85）。
    pub covered: Vec<Vec<bool>>,
}

/// **软体域的成组状态**（原 `World` 的 `ropes`/`rope_proxies` 两个散字段收成一组，
/// 给布片腾成员位——`FluidBoundary` 同款先例；god 门成员棘轮只准减）。
#[derive(Default)]
pub struct SoftDomain {
    /// 绳索（`World::add_rope` 注册；`rope_pass` 每 tick 推进一次，空集零成本短路）。
    pub ropes: Vec<vxl_phys_soft::Rope>,
    /// 绳索的**刚体代理暂存**（每 tick 重建，复用免分配；`rope_pass` 里 `mem::take` 借出后归还）。
    pub rope_proxies: Vec<vxl_phys_soft::RigidProxy>,
    /// 布片（`World::add_cloth` 注册；`cloth_pass` 每 tick 推进一次，空集零成本短路）。
    pub cloths: Vec<vxl_phys_soft::ClothSheet>,
}

impl FluidBoundary {
    /// 该流体是否开了 2b（越界一律 `false`）。**写成方法而不是在调用点展开链**：
    /// 展开式在原地超 `chain_width` 会被 rustfmt 折成 5 行 —— 而 `world_body.rs` 受尺寸棘轮
    /// （只准减），折行会让它"变胖"。
    pub fn is_two_b(&self, fluid: usize) -> bool {
        self.two_b.get(fluid).copied().unwrap_or(false)
    }
}

impl World {
    /// 注册一条绳索（`vxl_phys_soft::Rope`），返回其索引。
    pub fn add_rope(&mut self, rope: vxl_phys_soft::Rope) -> usize {
        self.soft.ropes.push(rope);
        self.soft.ropes.len() - 1
    }

    /// 已注册绳索（判据/渲染读 `pos` / `vel`）。
    pub fn ropes(&self) -> &[vxl_phys_soft::Rope] {
        &self.soft.ropes
    }

    /// 第 `i` 条绳索。
    pub fn rope(&self, i: usize) -> Option<&vxl_phys_soft::Rope> {
        self.soft.ropes.get(i)
    }

    /// 注册一张布片（`vxl_phys_soft::ClothSheet`），返回其索引。
    pub fn add_cloth(&mut self, cloth: vxl_phys_soft::ClothSheet) -> usize {
        self.soft.cloths.push(cloth);
        self.soft.cloths.len() - 1
    }

    /// 已注册布片（判据/渲染读 `pos`）。
    pub fn cloths(&self) -> &[vxl_phys_soft::ClothSheet] {
        &self.soft.cloths
    }

    /// 第 `i` 张布片。
    pub fn cloth(&self, i: usize) -> Option<&vxl_phys_soft::ClothSheet> {
        self.soft.cloths.get(i)
    }

    /// **域通道**（每 tick 一次、体子步全部完成之后）：液体域 → 软体域（绳 → 布）。
    /// 顺序固定 ⇒ 确定性不受影响（各条通道互不读对方状态）。
    /// 刚体代理在**进域前重建一次**（绳/布共用同一份快照；任一域非空才建）。
    pub(crate) fn domain_pass(&mut self) {
        self.fluid_pass();
        // 介质域推进（D3 切片 1/2）：喷溅场逐核平流 + 衰减一格（two_way 关 ⇒ 每场首行短路、
        // 逐位不变）。放在域轮次其余格之前——各格互不读对方状态，顺序只为确定性固定。
        self.providers.advance_medium(self.config.dt);
        if !self.soft.ropes.is_empty() || !self.soft.cloths.is_empty() {
            self.rebuild_soft_proxies();
        }
        self.rope_pass();
        self.cloth_pass();
    }

    /// **重建刚体代理快照**（每 tick 一次：体在动；从 `rope_pass` 提取，绳/布共用）。
    fn rebuild_soft_proxies(&mut self) {
        self.soft.rope_proxies.clear();
        for i in 0..self.bodies.len() {
            let shape = self.bodies.shape[i];
            if matches!(
                shape,
                Shape::Provider(_) | Shape::HeightField(_) | Shape::Compound { .. }
            ) {
                continue;
            }
            self.soft.rope_proxies.push(vxl_phys_soft::RigidProxy {
                body: i as u32,
                shape,
                pos: self.bodies.position[i],
                rot: self.bodies.rot(i),
                linvel: self.bodies.linvel[i],
                // **角速度**（§8.4.29 / 2c-1）：接触几何要跟着转动走。**静态体与睡眠体一律 0**
                // （睡眠体对软体域呈现静态，§8.4.27）。
                angvel: if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                    self.bodies.angvel(i)
                } else {
                    Vec3::ZERO
                },
                local_inv_inertia: if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                    self.bodies.local_inv_inertia[i]
                } else {
                    Vec3::ZERO
                },
                // **睡眠体对软体域呈现为"静态"**（§8.4.27）：`inv_mass = 0` ⇒ 软体把它当墙
                // （接触照做、反作用自动跳过）。不这么做的话反作用会**静默累进睡眠体的
                // `linvel`** ⇒ 醒来瞬间被弹出（判据 `rope_scene::rope_does_not_disturb_a_sleeping_body`）。
                inv_mass: if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                    self.bodies.inv_mass[i]
                } else {
                    0.0
                },
            });
        }
    }

    /// **软体域通道**：每条绳索按自身 `substeps` 推进一个 `config.dt`；接触走统一提供者通道
    /// （`0..providers.len()` 全量 id）与**刚体代理**（粒子↔刚体，Akinci 式最小实现）。
    /// 反作用回填：`bodies.linvel += body_dv`（速度增量）、`bodies.position += body_dx`（位置补足）；
    /// 角反作用（`angular_reaction`，默认开）经 `cpl::add_tick_torque`（`÷dt_sub` 契约）注入。
    /// **空集 ⇒ 零成本短路**。
    pub(crate) fn rope_pass(&mut self) {
        if self.soft.ropes.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        // 刚体代理已在 `domain_pass` 重建（`rebuild_soft_proxies`，绳/布共用同一份快照）。
        // **睡眠体对软体域呈现为"静态"**（§8.4.27）：代理里 `inv_mass = 0` ⇒ 软体把它当墙
        // （接触照做、反作用自动跳过）；判据 `rope_scene::rope_does_not_disturb_a_sleeping_body`。
        let proxies = std::mem::take(&mut self.soft.rope_proxies);
        let providers = &self.providers;
        for rope in &mut self.soft.ropes {
            rope.step(dt, gravity, providers, count, &proxies);
            // **反作用两腿**（`Rope::body_dv` 速度口径 + `Rope::body_dx` 位置口径，§8.4.9）：
            // 与布料域**共用同一段实现**（切片 2b-ii 提取 —— 两处逐字相同，见 `apply_two_leg_reactions`）。
            cpl::apply_two_leg_reactions(&mut self.bodies, &proxies, &rope.body_dv, &rope.body_dx);
            // **角反作用**（计划 2c-3，**默认开** since 2026-10-01；此前默认关）：关闭的理由不是
            // "力矩算错了"，而是**接触模型看不见转动**：`body_disp` 只跟踪平移、摩擦的滑移用
            // `b.linvel` 而非 `linvel + ω×r`、`crossed_face` 用的是**冻结的** `rot` ⇒ 回填角动量
            // 等于注入模型看不见的运动（实测 1 kg 薄盒：`hit = 1` 时 `|ω|` 一 tick 就到 5~8 rad/s
            // ⇒ 接触立刻丢 ⇒ 盒子被甩下去，门面 1800 tick y = −2420）。**翻默认的依据**（换代级，
            // 与 C2 同批，`PLAN-COUPLING.md` §5 P1）：§8.4.31/§8.4.32 实测打开后中心场景仍托住，
            // 且 `angular_reaction_holds`（带转动的自扮引擎）与 `rope_scene` 偏置判据守着符号与量级；
            // 残留局限 = 上面的"模型看不见转动"三条（转动感知代理，见 `PLAN-COUPLING.md` §9）。
            //
            // ⚠️ **口径**（`crates/vxl-phys/tests/angular_impulse_contract.rs` 钉住，2026-09-28 §8.4.28）：
            // `torque` 是**每子步消费并清零**的累加器，而本处注入发生在**所有子步之后**
            // ⇒ 只被**一个**子步消费 ⇒ 交付"整 tick 的角冲量 `r.torque`"必须 `÷ dt_sub`
            // （= `× substeps / dt`）；写成 `÷ dt` 只会交付 `1/substeps`（默认 2 ⇒ **差 2×**，
            // 那条判据的金丝雀里实测比值正好 `0.500000`）。
            if rope.angular_reaction {
                let substeps = self.config.substeps.max(1) as f32;
                for r in &rope.reactions {
                    // 经受体门 + tick 末注入契约（`PLAN-COUPLING.md` §2 A3 / §3.5：原先无 awake 检查）。
                    cpl::add_tick_torque(&mut self.bodies, r.body, r.torque, substeps, dt);
                }
            }
        }
        self.soft.rope_proxies = proxies;
    }

    /// **布料域通道**（软体切片 1/2/2b-ii）：每张布片按自身 `substeps` 推进一个 `config.dt`；
    /// 接触走**统一提供者通道**（与 rope 同款：`0..providers.len()` 全量 id + 球采样 + 库仑锥）
    /// 与**刚体代理**（静态/睡眠 = 墙；动态 = 两体约束 + 反作用两腿）。
    /// 反作用回填与 `rope_pass` **同口径**：`bodies.linvel += dv`（速度增量）+
    /// `bodies.position += dx`（位置口径补足，§8.4.9）。**空集 ⇒ 零成本短路**。
    pub(crate) fn cloth_pass(&mut self) {
        if self.soft.cloths.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        let providers = &self.providers;
        // 刚体代理：与 rope 同一份快照（`domain_pass` 已重建；借出免借用冲突）。
        let proxies = std::mem::take(&mut self.soft.rope_proxies);
        for cloth in &mut self.soft.cloths {
            cloth.step(dt, gravity, providers, count, &proxies);
            // **反作用两腿**（切片 2b-ii，与 `rope_pass` 同段位、同口径 —— 共用同一段实现）：
            // 静态/睡眠体（代理里 `inv_mass = 0`）不收反作用；角反作用**本片不引入**
            // （§8.4.20 条件②：接触模型看不见转动）。
            cpl::apply_two_leg_reactions(
                &mut self.bodies,
                &proxies,
                &cloth.body.dv,
                &cloth.body.dx,
            );
        }
        self.soft.rope_proxies = proxies;
    }
}

// **反作用两腿**的施加本体已收口进 `world_step/coupling.rs`（`cpl::apply_two_leg_reactions`，
// C3 片 2 第 1 步纯搬移）——本文件不再直接写 `bodies.linvel` / `bodies.position`。
