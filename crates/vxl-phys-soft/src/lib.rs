//! # vxl-phys-soft
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地；参数骨架在 [`params`]（刚度档 α / 撕裂阈值 / 自碰撞，数值来自规格书）。
//! - [`cloth`]：**布料最小闭环**（三角网 + XPBD 约束 + 薄壳质量 + 提供者/刚体接触与反作用两腿 + 自碰撞开关 + 气动消费）——判据在 `tests/cloth_{minimal,contact,body,bending,reaction,self_collision,aero}.rs`；
//! - [`rope`]：**绳索最小闭环**（XPBD 链 + 点-形状接触 + 摩擦）——判据在 `tests/rope_minimal.rs` 与 `crates/vxl-phys/tests/rope_scene.rs`；
//! - [`rigid`]：**粒子↔刚体耦合**（Akinci 式最小实现：代理视图 + 穿透查询 + 反作用回填）。
//!
//! 待落地（各自是后续切片）：体积约束、**自碰撞的进阶档**（点-边对/自摩擦）、**面元力矩与升力**、撕裂/塑性、GPU 档。

#![forbid(unsafe_code)]

pub mod cloth;
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
