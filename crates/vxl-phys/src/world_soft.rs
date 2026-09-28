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

/// 2b（Akinci 边界粒子）那一族的成组状态（原为 `World` 的 3 个散字段：开关 / 暂存 / 覆盖集）。
#[derive(Default)]
pub struct FluidBoundary {
    /// 与 `fluids` 同序的 2b 开关：`true` = 该流体每 tick 重建边界粒子并回流反作用。
    pub two_b: Vec<bool>,
    /// 边界粒子生成的暂存 `(体 id, 形状, 位姿)`（复用免每 tick 分配）。
    pub scratch: Vec<(u32, Shape, vxl_phys_fluid::BodyPose)>,
    /// **覆盖集**（与 `bodies` 同序，每 tick 重建）：上次进了边界粒子集的体。
    pub covered: Vec<bool>,
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

    /// **域通道**（每 tick 一次、体子步全部完成之后）：液体域 → 软体域。
    /// 顺序固定 ⇒ 确定性不受影响（各条通道互不读对方状态）。
    pub(crate) fn domain_pass(&mut self) {
        self.fluid_pass();
        self.rope_pass();
        self.cloth_pass();
    }

    /// **软体域通道**：每条绳索按自身 `substeps` 推进一个 `config.dt`；接触走统一提供者通道
    /// （`0..providers.len()` 全量 id）与**刚体代理**（粒子↔刚体，Akinci 式最小实现）。
    /// 反作用回填：`bodies.linvel += body_dv`（速度增量）、`bodies.torque += τ/dt`（角冲量 → 力矩口径，
    /// 与 2b 流体反作用同段位）。**空集 ⇒ 零成本短路**。
    pub(crate) fn rope_pass(&mut self) {
        if self.soft.ropes.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        // 刚体代理（每 tick 重建：体在动）。**地形类形状跳过**（`Provider`/`HeightField` 走提供者
        // 通道；`Compound` 本片不支持 ⇒ 直接跳过，别让它悄悄不清碰）。
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
                // 本体系逆惯量（开角反作用时绳要用它推进"虚拟角速度"；关时不用，填 0 也行，但保持一致更好）。
                local_inv_inertia: if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                    self.bodies.local_inv_inertia[i]
                } else {
                    Vec3::ZERO
                },
                // **睡眠体对软体域呈现为"静态"**（§8.4.27）：与 2b 流体同口径——流体那边睡眠体
                // 照样生成边界粒子（"让静态几何可感"）但**不接收反作用**（`fluid_reaction_pass`
                // 判 `awake`）。这里用更省的等价写法：`inv_mass = 0` ⇒ 绳索把它当墙（接触照做），
                // 而门面既有的"静态体不收反作用"那条自动跳过回填。
                // ⚠️ 不这么做的话：反作用会**静默累进睡眠体的 `linvel`**（体在睡、位置不积分）
                // ⇒ 醒来瞬间被弹出。判据：`rope_scene::rope_does_not_disturb_a_sleeping_body`。
                inv_mass: if self.bodies.is_dynamic(i) && self.bodies.awake[i] {
                    self.bodies.inv_mass[i]
                } else {
                    0.0
                },
            });
        }
        let proxies = std::mem::take(&mut self.soft.rope_proxies);
        let providers = &self.providers;
        for rope in &mut self.soft.ropes {
            rope.step(dt, gravity, providers, count, &proxies);
            for (j, p) in proxies.iter().enumerate() {
                if p.inv_mass <= 0.0 {
                    continue; // 静态体不收反作用
                }
                if let Some(dv) = rope.body_dv.get(j) {
                    self.bodies.linvel[p.body as usize] += *dv;
                }
                // **位置口径回填**（`Rope::body_dx`，§8.4.9）：只回速度会让体每 tick 按 `v·dt` 走过的
                // `g·dt²` 一去不回（实测下沉 ≈ 整漏的 83%）⇒ 把钳位压掉的那一份位置补上。
                // **速度侧不动**（`body_dx` 是从位置口径算出来的）⇒ 不会把修正反射成速度。
                if let Some(dx) = rope.body_dx.get(j) {
                    self.bodies.position[p.body as usize] += *dx;
                }
            }
            // **角反作用：本片不施加**（§8.4.9/§8.4.10 实测）。理由不是"力矩算错了"，而是
            // **接触模型看不见转动**：`body_disp` 只跟踪平移、摩擦的滑移用 `b.linvel` 而非
            // `linvel + ω×r`、`crossed_face` 用的是**冻结的** `rot` ⇒ 把角动量回填给体以后，
            // 体转起来的运动会**完全落在模型之外**。实测（1 kg 薄盒压在绳上）：`hit = 1`
            // （单点接触、力臂 ≈ 0.3 m、`I_zz ≈ 0.031`）⇒ `|ω|` 一 tick 就到 **5~8 rad/s**
            // ⇒ 接触立刻丢失（`hit = 0`）⇒ 盒子被甩下去（门面 1800 tick y = −2420）。
            // **自扮引擎侧一直不读 `reactions`**（只吃 `body_dv`）⇒ 它托得住（y@1800 = +0.995），
            // 这正是两侧差异的最后一块。**要恢复本行**必须先做"转动感知的代理"（见 §8.4.10）。
            //
            // ⚠️ **恢复时口径必须用下面这一行**（已由 `crates/vxl-phys/tests/angular_impulse_contract.rs`
            // 钉住，2026-09-28 §8.4.28）：`torque` 是**每子步消费并清零**的累加器
            // （`Integrate`：`ω += I⁻¹·τ·dt_sub` 之后清零），而本处注入发生在**所有子步之后**
            // ⇒ 只被**一个**子步消费 ⇒ 想交付"整 tick 的角冲量 `r.torque`"就必须 `÷ dt_sub`
            // （= `× substeps / dt`）；写成 `÷ dt` 只会交付 `1/substeps`（默认 2 ⇒ **差 2×**，
            // 那条判据的金丝雀里实测比值正好 `0.500000`）。
            // for r in &rope.reactions {
            //     let b = r.body as usize;
            //     if b < self.bodies.len() {
            //         let substeps = self.config.substeps.max(1) as f32;
            //         self.bodies.torque[b] += r.torque * (substeps / dt);
            //     }
            // }
            if rope.angular_reaction {
                // **角反作用注入**（计划 2c-3）：口径按 `crates/vxl-phys/tests/angular_impulse_contract.rs`
                // 钉住的契约 —— 本处注入发生在**所有子步之后** ⇒ 只被**一个**子步消费 ⇒ 必须 `÷ dt_sub`
                // （= `× substeps / dt`）才交付"整 tick 的角冲量 `r.torque`"。
                let substeps = self.config.substeps.max(1) as f32;
                for r in &rope.reactions {
                    let b = r.body as usize;
                    if b < self.bodies.len() {
                        self.bodies.torque[b] += r.torque * (substeps / dt);
                    }
                }
            }
        }
        self.soft.rope_proxies = proxies;
    }

    /// **布料域通道**（软体切片 1/2）：每张布片按自身 `substeps` 推进一个 `config.dt`；
    /// 接触走**统一提供者通道**（与 rope 同款：`0..providers.len()` 全量 id + 球采样 + 库仑锥）。
    /// 空集 ⇒ 零成本短路 ⇒ 默认档逐位不变。
    pub(crate) fn cloth_pass(&mut self) {
        if self.soft.cloths.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        let providers = &self.providers;
        for cloth in &mut self.soft.cloths {
            cloth.step(dt, gravity, providers, count);
        }
    }
}
