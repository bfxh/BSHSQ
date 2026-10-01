//! world_step/medium：**介质通道段①（喷溅场）**——从 `world_step.rs` 按域拆出
//! （同 `aero.rs` 先例；段自成一域，`world_step.rs` 净瘦身）。
//!
//! 段①物理 = 二次阻力 `F = −½·ρ·Cd·A·|v_rel|·v_rel`（密度 0 的场直接跳过）；
//! **施加式是冻结行为**（PhysArena/喷溅场景的哈希以它为基准）。
//!
//! **D3 切片 1（双向，默认关；`PLAN-COUPLING.md` §4.3）**：场 `two_way` 开时
//! ① `MediumField::sample` 的速度项读核速度插值（介质→体，`vxl_phys_splat::flow`）；
//! ② 本段把**反作用** `−F·dt` 按核权重沉积回场（体→介质，`MediumField::deposit`）。
//! 收集段只读场与体（保持原遍历序/浮点序），提交段才取可变场视图——两段是
//! `&self.providers` 与 `&mut self.providers` 的借出纪律所决定，与 C4 的
//! "收集→提交"同型。**关档不收集/不提交 ⇒ 与旧单向逐位一致**。
//! （显式导入，不用 `use super::*`——glob-gate：新文件零通配。）
use crate::props::cross_section_area;
use crate::world_step::coupling;
use crate::{Aabb, ProviderEntry, Providers, Vec3, World};

/// 段①阻力系数（与 `medium_pass` 段②同值；分开声明以免跨函数依赖）。
const DRAG_CD: f32 = 1.0;

impl World {
    /// 介质通道段①：每场一遍（体按索引序、采样点 = 体心）。
    pub(crate) fn splat_medium_pass(&mut self, dt: f32) {
        for id in 0..self.providers.len() as u32 {
            let two_b = match self.providers.splat(id) {
                Some(f) if f.medium_density > 0.0 => f.two_way(),
                _ => continue, // 密度 0 的场 = 只作碰撞提供者（零成本短路）
            };
            // 体→介质待沉积项（采样点、反作用、体质量、阻力功率·dt）。
            let mut pending: Vec<(Vec3, Vec3, f32, f32)> = Vec::new();
            for i in 0..self.bodies.len() {
                let Some((force, sp, mass)) = splat_body_drag(self, id, i) else {
                    continue;
                };
                // 经门写力（力通道的唯一收口，`coupling::add_force`）。
                coupling::add_force(&mut self.bodies, i, force, Vec3::ZERO);
                if two_b {
                    // 反作用（牛顿第三定律）：介质获得**体失去的动量** `−F·dt`
                    // ⇒ 介质被带向体运动方向 ⇒ 下一 tick 的相对速度变小、阻力更小。
                    let j = force * (-dt);
                    let work = force.length() * sp * dt;
                    pending.push((self.bodies.position[i], j, mass, work));
                }
            }
            if !pending.is_empty() {
                if let Some(fm) = self.providers.splat_mut(id) {
                    for (p, j, mass, work) in pending {
                        use vxl_phys_core::interop::MediumField as _;
                        fm.deposit(p, j, mass, work);
                    }
                }
            }
        }
    }
}

/// 段①的单体一遍：受体门 → 场外包围盒 → 采样（真空/零速率/零截面短路）→ 二次阻力。
/// 返回 `(阻力, 相对速率, 体质量)`；`None` = 本场对此体无作用（与原 `continue` 条件逐一对应）。
/// **自由函数**：`World` 的方法数是已登记债务（god 门只准减），helper 不挂 `impl`。
fn splat_body_drag(w: &World, id: u32, i: usize) -> Option<(Vec3, f32, f32)> {
    // 受体门（`PLAN-COUPLING.md` §3.2）：静态/睡眠/零质量体不收跨域作用——原先此处
    // 只查 `is_dynamic`（写进去后被积分器静默丢弃），C1 起统一为提前不写。
    if !coupling::is_receptor(&w.bodies, i) {
        return None;
    }
    let bb = w.provider_bounds[id as usize];
    if !in_aabb(w.bodies.position[i], &bb) {
        return None; // 场外 = 真空
    }
    let p = w.bodies.position[i];
    use vxl_phys_core::interop::MediumField as _;
    let f = w.providers.splat(id)?;
    let m = f.sample(p);
    if m.density <= 0.0 {
        return None;
    }
    let v_rel = w.bodies.linvel[i] - m.velocity;
    let sp = v_rel.length();
    let a = cross_section_area(&w.bodies.shape[i]);
    if sp < 1e-6 || a <= 0.0 {
        return None;
    }
    let im = w.bodies.inv_mass[i];
    let mass = if im > 0.0 { 1.0 / im } else { 0.0 };
    Some((v_rel * (-0.5 * m.density * DRAG_CD * a * sp), sp, mass))
}

/// 点是否在场包围盒内（含边界；与原逐轴比较同义）。
fn in_aabb(p: Vec3, bb: &Aabb) -> bool {
    p.x >= bb.min.x
        && p.x <= bb.max.x
        && p.y >= bb.min.y
        && p.y <= bb.max.y
        && p.z >= bb.min.z
        && p.z <= bb.max.z
}

impl Providers {
    /// 喷溅场可变视图（D3 双向沉积用；同 `voxel_mut` 口径）。
    pub fn splat_mut(&mut self, id: u32) -> Option<&mut vxl_phys_splat::GaussianSplatField> {
        match self.entries.get_mut(id as usize)? {
            ProviderEntry::Splat(f) => Some(f),
            _ => None,
        }
    }

    /// **介质域推进**（每 tick 一次，域轮次的一格；D3 切片 1）：逐喷溅场 `advance`。
    /// `two_way` 关的场在 `advance` 首行布尔短路 ⇒ 未开启时零额外成本。
    pub fn advance_medium(&mut self, dt: f32) {
        for e in &mut self.entries {
            if let ProviderEntry::Splat(f) = e {
                f.advance(dt);
            }
        }
    }
}
