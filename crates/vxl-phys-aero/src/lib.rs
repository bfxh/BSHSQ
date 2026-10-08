//! # vxl-phys-aero
//!
//! 气动域（面元气动力）—— **T4 落地（2026-09-28）**：把本 crate 从纯配置变成**消费方**。
//! 每三角面（Bridson 线化气动力，与布料 §4.7 同通道）：
//! `F_face = ½·ρ·Cd·A·u·|u|`，`u = v_wind − v_face_center`（**全相对速度**——含切向；
//! 面元 ⊥ 风时退化为平板阻力 `½ρv²A·Cd`，判据 ③ 与解析式**精确对拍**）。
//! **升力**（2026-09-29）：`face_force_with_lift`（阻力 + 线性升力）—— `lift_slope`
//! 从此有了消费方；三角面助手 `face_force_tri`（面积/法线/退化面口径）同在 `face.rs`。
//! 力/力矩由门面的 `aero_pass` **逐子步**施加（`bodies.force/torque` 是逐子步累加器，
//! 口径同 `angular_impulse_contract` 的已钉契约）；`AeroState.forces/torques` 留**逐体快照**
//! 作判据仪器（读到的就是施加的那份，不是另算的一份）。
//!
//! **开关语义**：门面 `World::set_aero` 显式开启 ⇒ `Option` 槽默认 `None` ⇒ **默认档逐位不变**
//! （零成本短路，与 `angular_reaction` / 卡上窄相同款先例）。

#![forbid(unsafe_code)]

use vxl_phys_core::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct AeroConfig {
    /// 空气密度 kg/m³。
    pub air_density: f32,
    /// 面法向阻力系数。
    pub drag_coefficient: f32,
    /// 升力线斜率（简化薄翼）。
    pub lift_slope: f32,
    pub wind: [f32; 3],
}

// 紧凑写法（god 门 file_lines / max_fn 棘轮）：`#[rustfmt::skip]` 保持单行 impl，本仓既有先例。
#[rustfmt::skip]
impl Default for AeroConfig {
    fn default() -> Self { Self { air_density: 1.225, drag_coefficient: 1.0, lift_slope: 5.0, wind: [0.0; 3] } }
}

mod face;
/// **空气作为 `MediumField`**（`medium.rs`）：气动域既能对面元施力，也能被介质消费者采样。
mod medium;

pub use face::{face_force, face_force_tri, face_force_with_lift};

/// **气动域的运行态**（门面 `World` 的 `Option` 槽内容）：配置 + **逐体力/力矩快照**
/// （`aero_pass` 每子步重写；判据经 `World::aero_force/aero_torque` 读到的就是
/// 本子步施加的那份 ⇒ 仪器与被测代码同源）。
#[derive(Clone, Debug)]
pub struct AeroState {
    pub cfg: AeroConfig,
    /// 逐体力快照（与 `bodies` 同序；本子步施加的那份）。
    pub forces: Vec<Vec3>,
    /// 逐体力矩快照（关于**体原点**；与 `bodies.torque` 同口径）。
    pub torques: Vec<Vec3>,
    /// **被大气吸收的累计动量**（N·s；`MediumField::deposit` 的审计量）。
    /// 风是运动学背景（无限大气库）⇒ 吸收不改风速，只记账（见 `medium.rs`）。
    pub absorbed: Vec3,
}

#[rustfmt::skip]
impl AeroState {
    pub fn new(cfg: AeroConfig) -> Self { Self { cfg, forces: Vec::new(), torques: Vec::new(), absorbed: Vec3::ZERO } }
}
