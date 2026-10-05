//! fluid_medium：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;
use scan::{for_each_neighbor, poly6_scan};

mod scan;

/// **介质场：`MediumField` 的第一个真实现**（ADR 0008/0009 的交互通道；`ROUTE.md` §4
/// 「刚体↔液体」格的第一刀 2a）。
///
/// 语义（只读采样；**不改流场、不改哈希**）：
/// - `sample(x)`：在 `x` 处按 **poly6 核**对 27 邻域求和，给出
///   `density = m·Σ_j W(r_ij)`（与流体内部的密度定义**同核同式**，只是不含自身项与鬼影项）、
///   `velocity = Σ w_j v_j / Σ w_j`（Shepard 平均 ⇒ 均匀流场下逐位精确）、
///   `occupied = clamp(ρ/ρ0, 0, 1)`（自由表面判据）。无近邻 ⇒ [`MediumSample::VACUUM`]。
/// - `deposit(…)`：**点式反作用沉积**（2026-10-05 落地）——把 `momentum` 按**同一** poly6
///   权重分摊到 27 邻域的流体粒子（`Δv_j = momentum·w_j / (Σw·m)`）⇒ 逐粒求和**严格守恒**
///   `Σ m_j·Δv_j = momentum`。**刚体**仍走 2b（Akinci 边界粒子逐对反对称，不需要点式沉积）；
///   本通道是给**软体/布**这类"真点式"受体的（`world_step` 的布×液反作用段）。
///   `mass`/`pressure_work` 沿用 trait 的审计参数（本实现不消费）。
/// - `viscosity`/`temperature` 报 0：XSPh 的 `ε` 是**无量纲**系数、不是 Pa·s，
///   本实现**不编造换算常数**（耦合侧用"密度 + 流速"算阻力即可，别依赖这个字段）。
impl MediumField for FluidSystem {
    fn sample(&self, x: Vec3) -> MediumSample {
        if self.pos.is_empty() || self.grid.items.is_empty() || self.grid.nz == 0 {
            return MediumSample::VACUUM;
        }
        let (wsum, rho, vsum) = poly6_scan(self, x);
        if wsum <= 0.0 {
            return MediumSample::VACUUM;
        }
        MediumSample {
            density: rho,
            velocity: vsum * (1.0 / wsum),
            viscosity: 0.0,
            temperature: 0.0,
            occupied: (rho / self.cfg.rest_density).clamp(0.0, 1.0),
        }
    }

    fn deposit(&mut self, x: Vec3, momentum: Vec3, _mass: f32, _pressure_work: f32) {
        if momentum == Vec3::ZERO {
            return;
        }
        let wsum = poly6_scan(self, x).0;
        if wsum <= 1e-12 {
            return; // 无近粒：动量不打空气（与 splat 场同口径）
        }
        // **严格守恒的分摊**：`k = 1/(Σw·m)` ⇒ `Σ_j m·Δv_j = momentum·(Σw_j)/Σw = momentum`。
        let k = 1.0 / (wsum * self.mass);
        let vel = &mut self.vel;
        for_each_neighbor(
            &self.grid,
            &self.pos,
            x,
            self.h2,
            self.k6,
            self.n_fluid,
            |j, w| vel[j] += momentum * (w * k),
        );
    }
}
