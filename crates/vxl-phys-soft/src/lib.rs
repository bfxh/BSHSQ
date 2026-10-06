//! # vxl-phys-soft
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地；参数骨架在 [`params`]（刚度档 α / 撕裂阈值 / 自碰撞，数值来自规格书）。
//! - [`cloth`]：**布料闭环**（三角网 + XPBD 距离/弯曲约束 + 薄壳质量 + 提供者/刚体接触与反作用 + 自碰撞/自摩擦 + 撕裂/塑性 + 气动）——判据见 crate 内 `tests/cloth_*.rs`（13 个）；
//! - [`rope`]：**绳索最小闭环**（XPBD 链 + 点-形状接触 + 摩擦）——判据在 `tests/rope_minimal.rs` 与 `crates/vxl-phys/tests/rope_scene.rs`；
//! - [`rigid`]：**粒子↔刚体耦合**（Akinci 式最小实现：代理视图 + 穿透查询 + 反作用回填）。
//!
//! **仍待落地**（2026-10-05 更正：原列的「自碰撞进阶/自摩擦/升力/撕裂/塑性」多已落地）：体积约束、点-边对自摩擦、面元力矩（升力已接入）、GPU 档；逐条见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md`。

#![forbid(unsafe_code)]

pub mod cloth;
pub mod cloth_access;
pub mod cloth_aero;
mod cloth_coupling;
mod cloth_self_collision;
mod cloth_self_friction;
pub mod cloth_tear;
pub mod params;
pub mod rigid;
pub mod rope;
pub use params::{ClothConstraints, SelfCollision, Stiffness, TearStrain};
pub use rigid::{RigidProxy, RigidReaction};
pub use rope::Rope;
pub use {cloth::ClothSheet, cloth_coupling::BodyCoupling, cloth_self_collision::SelfContacts};
