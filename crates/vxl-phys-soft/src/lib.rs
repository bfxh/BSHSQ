//! # vxl-phys-soft
//!
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地。
//!
//! - [`params`]：参数骨架（刚度档 α / 撕裂阈值 / 自碰撞），数值全部来自规格书；
//! - [`rope`]：**绳索最小闭环**（1D 粒子链 + XPBD 距离约束 + 点-形状接触 + 切向摩擦）——
//!   判据在 `tests/rope_minimal.rs`（悬垂形状对**同长度解析悬链线** / 二阶收敛 / 落在真实三角网上 /
//!   斜面静摩擦阈值）与 `crates/vxl-phys/tests/rope_scene.rs`（门面级）；
//! - [`rigid`]：**粒子↔刚体耦合**（Akinci 式边界处理的最小实现：代理视图 + 穿透查询 + 反作用回填）。
//!
//! 待落地（各自是后续切片）：布料三组约束 + Bridson 面元气动、体积约束、自碰撞、GPU 档。

#![forbid(unsafe_code)]

pub mod params;
pub mod rigid;
pub mod rope;

pub use params::{ClothConstraints, SelfCollision, Stiffness, TearStrain};
pub use rigid::{RigidProxy, RigidReaction};
pub use rope::Rope;
