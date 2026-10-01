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
            let Some(f) = self.providers.splat(id) else {
                continue;
            };
            let two_b = f.two_way();
            // **D3 切片 3：世界 AABB 随核漂移刷新**——双向场的核每 tick 被 `advance` 平流，
            // 注册时的 `provider_bounds` 快照会过期（介质采样门与宽相 AABB 都读它）。
            // 位置放在本 pass（每子步首段）⇒ 恰好在本子步宽相之前拿最新几何。单向场
            // （默认）不进去 ⇒ 默认档逐位不变、零额外成本。
            if two_b {
                use vxl_phys_core::interop::ProviderColliders;
                if let Some(nb) = f.bounds(id) {
                    self.provider_bounds[id as usize] = nb;
                }
            }
            if f.medium_density <= 0.0 {
                continue; // 密度 0 的场 = 只作碰撞提供者（零成本短路）
            }
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
    Some((drag_force(f, p, v_rel, sp, a, m.density), sp, mass))
}

/// 段①阻力（D3 切片 4）：未分离式 = 二次阻力 `F = −½ρCdA|v|·v`（**冻结式**，默认档逐位不变）；
/// `drag_split` 开且 `cd_n ≠ cd_t` ⇒ 按**等值面法线** `n̂ = −∇σ/|∇σ|`（复用 `density_grad`）分离：
/// `F = −½ρA|v|·(cd_n·v_n + cd_t·v_t)`；**零梯度回落**为未分离式（系数取 `cd_n`）。
/// **自由函数**：`World` 方法数在 god 门只准减。
fn drag_force(
    f: &vxl_phys_splat::GaussianSplatField,
    p: Vec3,
    v_rel: Vec3,
    sp: f32,
    a: f32,
    density: f32,
) -> Vec3 {
    if !f.drag_split {
        return v_rel * (-0.5 * density * DRAG_CD * a * sp);
    }
    if f.cd_normal == f.cd_tangent {
        // 同系数 ⇒ 与未分离式**同式**（只换系数）⇒ 逐位退化（供对拍）。
        return v_rel * (-0.5 * density * f.cd_normal * a * sp);
    }
    let (_, g) = f.density_grad(p);
    let gl = g.length();
    if gl <= 1e-6 {
        return v_rel * (-0.5 * density * f.cd_normal * a * sp); // 零梯度：回落（方向 = v_rel）
    }
    let n = (-g) * (1.0 / gl);
    let vn = n * v_rel.dot(n);
    let vt = v_rel - vn;
    (vn * f.cd_normal + vt * f.cd_tangent) * (-0.5 * density * a * sp)
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

#[cfg(test)]
mod tests {
    use super::{cross_section_area, splat_body_drag, Vec3, World, DRAG_CD};
    use vxl_phys_core::{PhysConfig, Quat, Shape};

    /// 单核场（原点、σ=0.5）+ 一枚动态盒（`(0, 0.2, 0)`）：`p` 处 ∇σ 精确 ∥ −y（轴上）。
    fn scene() -> (World, usize) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0);
        f.push(vxl_phys_splat::Splat::isotropic(Vec3::ZERO, 0.5, 1.0));
        f.medium_density = 1.0;
        w.add_splat_field(f); // marker 体 = 0
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.1),
            },
            Vec3::new(0.0, 0.2, 0.0),
            Quat::IDENTITY,
            1.0,
        ) as usize;
        (w, b)
    }

    /// 从世界现状**独立复算**参照量：(v_rel, |v_rel|, 截面 a, 介质密度)。
    fn refs(w: &World, b: usize, v: Vec3) -> (Vec3, f32, f32, f32) {
        let p = w.bodies.position[b];
        let (m_vel, m_den) = match w.providers.splat(0) {
            Some(f) => {
                use vxl_phys_core::interop::MediumField as _;
                let m = f.sample(p);
                (m.velocity, m.density)
            }
            None => (Vec3::ZERO, f32::NAN),
        };
        let v_rel = v - m_vel;
        (
            v_rel,
            v_rel.length(),
            cross_section_area(&w.bodies.shape[b]),
            m_den,
        )
    }

    fn force_of(w: &World, b: usize) -> Vec3 {
        match splat_body_drag(w, 0, b) {
            Some((frc, _, _)) => frc,
            None => Vec3::splat(f32::NAN), // 不该发生；由 is_finite 断言兜底
        }
    }

    /// ① **默认关逐位**：`drag_split = false` ⇒ 与现行公式（`DRAG_CD`）**逐位**一致。
    #[test]
    fn drag_default_path_is_bitwise_unchanged() {
        let (mut w, b) = scene();
        let v = Vec3::new(1.0, 2.0, 0.2);
        w.bodies.linvel[b] = v;
        let (v_rel, sp, a, density) = refs(&w, b, v);
        let expected = v_rel * (-0.5 * density * DRAG_CD * a * sp);
        let force = force_of(&w, b);
        assert!(force.is_finite(), "应有阻力（用例非平凡）");
        assert_eq!(force.x.to_bits(), expected.x.to_bits(), "关档 x 逐位");
        assert_eq!(force.y.to_bits(), expected.y.to_bits(), "关档 y 逐位");
        assert_eq!(force.z.to_bits(), expected.z.to_bits(), "关档 z 逐位");
    }

    /// ② **同系数逐位退化**：`cd_n == cd_t` ⇒ 走未分离式（同式换系数）⇒ 逐位。
    #[test]
    fn drag_split_equal_coeffs_degenerates_bitwise() {
        let (mut w, b) = scene();
        let v = Vec3::new(1.0, 2.0, 0.2);
        w.bodies.linvel[b] = v;
        if let Some(fm) = w.providers.splat_mut(0) {
            fm.drag_split = true;
            fm.cd_normal = 2.5;
            fm.cd_tangent = 2.5;
        }
        let (v_rel, sp, a, density) = refs(&w, b, v);
        let expected = v_rel * (-0.5 * density * 2.5 * a * sp);
        let force = force_of(&w, b);
        assert_eq!(force.x.to_bits(), expected.x.to_bits(), "同系数 x 逐位");
        assert_eq!(force.y.to_bits(), expected.y.to_bits(), "同系数 y 逐位");
        assert_eq!(force.z.to_bits(), expected.z.to_bits(), "同系数 z 逐位");
    }

    /// ③ **轴上解析对拍**：`p = (0, 0.2, 0)` ⇒ `n̂ = +y` 精确 ⇒ `vn = (0, v.y, 0)`、
    /// `vt = (v.x, 0, 0)`；与闭式 `−½ρA|v|·(cd_n·vn + cd_t·vt)` 相对误差 ≤ 1e-5。
    #[test]
    fn drag_split_matches_closed_form_on_axis() {
        let (mut w, b) = scene();
        let v = Vec3::new(1.0, 2.0, 0.0);
        w.bodies.linvel[b] = v;
        if let Some(fm) = w.providers.splat_mut(0) {
            fm.drag_split = true;
            fm.cd_normal = 2.0;
            fm.cd_tangent = 0.5;
        }
        let (_, sp, a, density) = refs(&w, b, v);
        // n̂ = +y（轴上精确）⇒ vn = 2ŷ、vt = x̂ ⇒ 括号内 = (0.5, 4.0, 0)
        let expected = Vec3::new(0.5, 4.0, 0.0) * (-0.5 * density * a * sp);
        let force = force_of(&w, b);
        let err = (force - expected).length();
        assert!(
            err <= 1e-5 * expected.length().max(1e-6),
            "解析对拍 err={err:.3e}（force={force:?} expected={expected:?}）"
        );
    }

    /// ④ **零梯度回落**：体在核心（∇σ = 0）⇒ 回落未分离式（方向 = `v_rel`、系数 = `cd_n`）。
    #[test]
    fn drag_split_falls_back_at_zero_gradient() {
        let (mut w, b) = scene();
        w.bodies.position[b] = Vec3::ZERO; // 核心
        let v = Vec3::new(1.0, 2.0, 0.2);
        w.bodies.linvel[b] = v;
        if let Some(fm) = w.providers.splat_mut(0) {
            fm.drag_split = true;
            fm.cd_normal = 2.0;
            fm.cd_tangent = 0.5;
        }
        let (v_rel, sp, a, density) = refs(&w, b, v);
        let expected = v_rel * (-0.5 * density * 2.0 * a * sp);
        let force = force_of(&w, b);
        assert!(force.is_finite(), "核心处应有限（回落路径）");
        assert_eq!(force.x.to_bits(), expected.x.to_bits(), "回落 x 逐位");
        assert_eq!(force.y.to_bits(), expected.y.to_bits(), "回落 y 逐位");
        assert_eq!(force.z.to_bits(), expected.z.to_bits(), "回落 z 逐位");
    }
}
