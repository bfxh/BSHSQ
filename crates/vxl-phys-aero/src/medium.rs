//! **空气 = 介质场**（`MediumField`）：本 crate 的同一个域有**两种用法** ——
//! ① 面元施力（`face.rs` + 门面 `aero_pass`，刚体/布）；② 作为**介质**被任何消费者按统一通道
//! 采样（液面表面驱动、布、喷溅…）。第二种用法此前缺失 ⇒ 「风 × 液」那一格只能"同帧共存"。
//!
//! 语义（与 `vxl_phys_core::interop::MediumField` 的约定逐条对齐）：
//! - `sample(x)`：均匀、稳定、无黏的空气 —— `density = air_density`、`velocity = wind`、
//!   `occupied = 1`（开放空气处处可用）、`temperature = 0`；**与 `x` 无关**（均匀风场）
//!   ⇒ 同输入同输出（确定性）。`air_density <= 0` ⇒ [`MediumSample::VACUUM`]（显式关档）。
//! - `deposit(x, momentum, …)`：动量记进 [`AeroState::absorbed`]（受体拿到 `+J`、大气记 `−J`
//!   ⇒ 两者之和为 0）。**风速是否因此改变**由 [`AeroConfig::air_mass`] 决定：
//!   `air_mass = 0`（默认）= **运动学背景**（无限大气库，吸收不改风速）；
//!   有限 `air_mass` ⇒ `v = wind + absorbed/air_mass`（动量槽，双向那一半）——
//!   判据见 `tests/medium_field.rs`（动量账 `M·Δv = J` 与能量账 `ΔKE ≤ 0`）。
use crate::AeroState;
use vxl_phys_core::interop::{MediumField, MediumSample};
use vxl_phys_core::Vec3;

impl MediumField for AeroState {
    fn sample(&self, _x: Vec3) -> MediumSample {
        let cfg = self.cfg;
        if cfg.air_density <= 0.0 {
            return MediumSample::VACUUM;
        }
        let mut velocity = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
        if cfg.air_mass > 0.0 {
            velocity += self.absorbed * (1.0 / cfg.air_mass);
        }
        MediumSample {
            density: cfg.air_density,
            velocity,
            viscosity: 0.0,
            temperature: 0.0,
            occupied: 1.0,
        }
    }

    fn deposit(&mut self, _x: Vec3, momentum: Vec3, _mass: f32, _pressure_work: f32) {
        self.absorbed += momentum;
    }
}
