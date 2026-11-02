//! # vxl-phys-solver
//!
//! 刚体约束求解（§2.5/§2.6）：
//! - M0：顺序冲量（warm starting + Baumgarte 位置修正 + 摩擦锥）——即 M0 里程碑
//!   明确要求的「顺序冲量」；TGS-Soft（time-of-impact 分级 + 软约束）在 M1 升级，
//!   本文件求解循环骨架与岛/休眠/缓存层保持不变。
//! - 岛：并查集分岛；岛内约束按 (a, b, 点序) 固定排序（§4.14 确定性模式）。
//! - 休眠（§4.11）：线性 0.04 / 角速 0.05 rad/s、计时 0.5 s，岛级判定。
//! - CCD（§4.12/§2.7）：`ccd` 模块——选择性判据 + 保守推进扫描（M1）。
//! - 并行（§6）：清醒岛分组并行（组内串行、组间体集合不相交），
//!   gather→solve→scatter 走组内 scratch 速度缓冲，与串行 bit 级一致（§5）。

#![forbid(unsafe_code)]

pub mod ccd;
pub mod joints;

use std::collections::HashMap;

use vxl_phys_core::{BodySet, JobSystem, Mat3, PhysConfig, Vec3};
use vxl_phys_narrow::Manifold;

// ── 按域拆出的子模块（子目录 src/）
pub mod build_seg;
mod cfg;
mod constraint;
mod helpers;
mod island;
mod shock_budget;
mod solver_impl;
pub mod stats;
pub mod tier_phase;
mod types;
mod warm;
mod warm_data;
pub mod warm_match;
pub(crate) use self::shock_budget::island_shock_budget;
pub use self::{cfg::*, stats::*, types::*};
pub(crate) use self::{constraint::*, helpers::*, island::*, warm::*, warm_data::*};
#[cfg(test)]
#[path = "lib_tests.rs"] // 迷你闭环单测（本文件行数棘轮零余量 ⇒ 搬出）
mod lib_tests;
