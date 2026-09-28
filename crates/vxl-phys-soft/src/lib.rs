//! # vxl-phys-soft
//!
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地。
//!
//! - [`params`]：参数骨架（刚度档 α / 撕裂阈值 / 自碰撞），数值全部来自规格书；
//! - [`cloth`]：**布料最小闭环**（三角网 + XPBD 约束 + 薄壳均分质量 + 提供者/刚体接触与反作用两腿）
//!   ——判据在 `tests/cloth_{minimal,contact,body,bending,reaction}.rs`；刚体耦合段在 `cloth_coupling`；
//! - [`rope`]：**绳索最小闭环**（XPBD 链 + 点-形状接触 + 摩擦）——判据在 `tests/rope_minimal.rs` 与 `crates/vxl-phys/tests/rope_scene.rs`；
//! - [`rigid`]：**粒子↔刚体耦合**（Akinci 式最小实现：代理视图 + 穿透查询 + 反作用回填）。
//!
//! 待落地（各自是后续切片）：体积约束、**自碰撞**、Bridson 面元气动消费、撕裂/塑性、GPU 档。

#![forbid(unsafe_code)]

pub mod cloth;
mod cloth_coupling;
pub mod params;
pub mod rigid;
pub mod rope;

pub use params::{ClothConstraints, SelfCollision, Stiffness, TearStrain};
pub use rigid::{RigidProxy, RigidReaction};
pub use rope::Rope;
pub use {cloth::ClothSheet, cloth_coupling::BodyCoupling};
