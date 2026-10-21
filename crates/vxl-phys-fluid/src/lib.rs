//! # vxl-phys-fluid
//!
//! 流体（§4.8，按技术族分档）—— M3 CPU 档 / M4 GPU 档。
//!
//! - CPU SPH：WCSPH + XSPH 黏度（Monaghan 核）；
//! - PBF：Müller 2013 poly6/spiky 核，密度约束迭代 3~4（**CPU 口径档** = `CpuPbf`；GPU 1000 万粒目标档 = M4）；
//! - GPU FLIP/APIC：PIC/FLIP 传输 + APIC 仿射速度，稀疏哈希网格（512³ 活跃格）；
//! - 刚体耦合：Akinci 边界粒子两层（体积/压力）。
//!
//! 首切片（docs/PLAN-0.3.md）：`CpuSph` 的 WCSPH 路径完整落地——
//! poly6 密度（含自身项）、spiky 压力梯度（中心不减益 ⇒ 张力不稳定抑制）、
//! XSPH 速度平滑、Tait 状态方程（γ=7）、晶格质量标定、半隐式欧拉 + CFL 限速；
//! 邻居 = 均匀网格（格边 = h，27 邻域，遍历序确定）；边界 = 统一提供者通道
//! `ProviderColliders::contacts_point`（体素/三角网/喷溅即插即用，非弹性投影）。
//! 确定性：全 f32、同输入两次运行逐位一致（测试守门）。
//!
//! **刚体→流体（2b，Akinci 两层边界粒子）**：边界粒子与流体粒子**同数组、同核、
//! 同式**（逐粒质量 `pmass`：流体 = `mass`、边界 = `ρ0·V_b`），只是**运动学冻结**
//! （不积分、由 facade 每 tick 按体重建，速度 = 体面速度 `v + ω×r`）。反作用 =
//! 边界粒子上压力梯度力之和 → 每体（力 + 绕体原点的力矩）。
//! 几何（表面采样/两层内移/体积）在 [`boundary`] 模块。**无边界粒子时逐位不变**
//! （`sum_b` 恒 0 ⇒ `mass·sum + 0.0` 与原式同值同序）。
//!
//! **数据侧按域打包**（见 `system/domains.rs`）：`FluidSystem` 的 SoA 数组按**职责域**
//! 装进具名结构体（核参数 / 粒子状态 / 边界账 / 邻域网格）⇒ **新加一个域不必再往
//! `FluidSystem` 加字段**。

#![forbid(unsafe_code)]

pub mod boundary;
pub use boundary::{lattice, BoundaryLattice, SurfaceLattice};

use vxl_phys_core::interop::{InteropContact, MediumField, MediumSample, ProviderColliders};
use vxl_phys_core::{Quat, Shape, Vec3};
// ── 按域拆出的子模块（子目录 src/）
mod carve;
mod config;
mod flip;
pub mod fluid_access;
mod fluid_boundary;
mod fluid_density;
mod fluid_force;
mod fluid_medium;
mod fluid_step;
mod grid;
mod laminar;
mod pbf;
pub mod probe_micro;
mod system;
mod types;
pub use self::{carve::*, config::*, grid::*, system::*, types::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests;
