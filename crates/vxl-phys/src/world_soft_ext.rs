//! **软体域的扩展 API**（`impl SoftExt for World`）——本域**不往 `World` 加方法**。
//!
//! **为什么**：god 门把 `World` 的方法数登记为债务（`god.gate.json::god_pairs`），
//! 处置写的是「**域侧 API 拆成扩展 trait**（`impl FluidExt for World` 等），门面只留管线组装」，
//! 判据是「**新加一域不必再往 `World` 添方法**」。先例：`vxl-phys-sense` 的 `SenseExt`。
//!
//! **为什么 trait 住在 `vxl-phys` 自己家里**：`vxl-phys` 依赖 `vxl-phys-soft`（不是反之）
//! ⇒ trait 放 `vxl-phys-soft` 会成**依赖环** ⇒ 只能放本 crate 的**域模块**里。
//!
//! **公开面不变**：原 `World::add_rope` 等是 `World` 的**固有方法**；
//! 搬进 trait 后调用点要 `use vxl_phys::world_soft_ext::SoftExt;`（或经 `lib.rs` 再导出）。
//! ⇒ **这是 API 位置的变更**（不是行为变更）：数值与顺序一字不改。
use crate::*;
// 软体域的**私有**反作用自由函数（`world_soft.rs` 的 `#[path]` 子模块）——
// 本文件搬走了用到它们的方法 ⇒ 导入路径要跟着走。
use super::{apply_rope_angular_reactions, cloth_medium_reaction};
use crate::world_step::coupling as cpl; // 耦合契约短别名（与 world_soft.rs 同款）

pub trait SoftExt {
    fn add_rope(&mut self, rope: vxl_phys_soft::Rope) -> usize;
    fn ropes(&self) -> &[vxl_phys_soft::Rope];
    fn rope(&self, i: usize) -> Option<&vxl_phys_soft::Rope>;
    fn add_cloth(&mut self, cloth: vxl_phys_soft::ClothSheet) -> usize;
    fn cloths(&self) -> &[vxl_phys_soft::ClothSheet];
    fn cloth(&self, i: usize) -> Option<&vxl_phys_soft::ClothSheet>;
}
impl SoftExt for World {
    /// 注册一条绳索（`vxl_phys_soft::Rope`），返回其索引。
    fn add_rope(&mut self, rope: vxl_phys_soft::Rope) -> usize {
        self.soft.ropes.push(rope);
        self.soft.ropes.len() - 1
    }

    /// 已注册绳索（判据/渲染读 `pos` / `vel`）。
    fn ropes(&self) -> &[vxl_phys_soft::Rope] {
        &self.soft.ropes
    }

    /// 第 `i` 条绳索。
    fn rope(&self, i: usize) -> Option<&vxl_phys_soft::Rope> {
        self.soft.ropes.get(i)
    }

    /// 注册一张布片（`vxl_phys_soft::ClothSheet`），返回其索引。
    fn add_cloth(&mut self, cloth: vxl_phys_soft::ClothSheet) -> usize {
        self.soft.cloths.push(cloth);
        self.soft.cloths.len() - 1
    }

    /// 已注册布片（判据/渲染读 `pos`）。
    fn cloths(&self) -> &[vxl_phys_soft::ClothSheet] {
        &self.soft.cloths
    }

    /// 第 `i` 张布片。
    fn cloth(&self, i: usize) -> Option<&vxl_phys_soft::ClothSheet> {
        self.soft.cloths.get(i)
    }
}

/// **软体域的管线组装**（`World` 门面自己留的四个内部推进方法）——
/// 它们是**每 tick 的调度**，不是域的公开 API ⇒ 留在门面上（不搬进 trait）。
impl World {
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
        cloth_medium_reaction(self);
    }
    pub(crate) fn rebuild_soft_proxies(&mut self) {
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
                // （睡眠体对软体域呈现静态，§8.4.27）；**运动学体算**（转盘要带着布料走）。
                angvel: if self.bodies.is_dynamic_or_kinematic(i) && self.bodies.awake[i] {
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
            // **角反作用**（默认开；为什么/怎么算，长注已随实现搬进 `apply_rope_angular_reactions`）。
            if rope.angular_reaction {
                let substeps = self.config.substeps.max(1) as f32;
                apply_rope_angular_reactions(&mut self.bodies, rope, substeps, dt);
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
