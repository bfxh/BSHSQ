//! # vxl-phys-soft
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地。
//!
//! - [`cloth`]：规则三角网 + 三组 XPBD 距离约束 + 静态提供者接触 + 可选面元风力；
//! - [`rope`]：绳索最小闭环（XPBD 距离约束 + 点-形状接触 + 摩擦），
//!   判据在 `tests/rope_minimal.rs`（悬垂形状 / 二阶收敛 / 真实三角网 / 斜面摩擦）
//!   与 `crates/vxl-phys/tests/rope_scene.rs`（门面级）；
//! - [`rigid`]：**粒子↔刚体耦合**（Akinci 式边界处理的最小实现：代理视图 + 穿透查询 + 反作用回填）。
#![forbid(unsafe_code)]
pub mod cloth;
mod cloth_body;
pub mod cloth_wind;
pub mod params;
pub mod rigid;
pub mod rope;
pub use cloth::ClothSheet;
pub use cloth_wind::ClothWind;
pub use params::{ClothConstraints, SelfCollision, Stiffness, TearStrain};
pub use rigid::{RigidProxy, RigidReaction};
pub use rope::Rope;
