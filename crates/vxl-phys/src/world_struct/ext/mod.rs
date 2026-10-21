//! **world_struct/ext**：`World` 的**域侧扩展 trait**（god 门处置方案的执行面）。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys::World"]` 写明——`World` 是
//! **已登记债务（非豁免）**，处置是「**域侧 API 拆成扩展 trait**（`impl FluidExt for World`
//! 等），门面只留管线组装」，判据是「新加一域**不必再往 `World` 添方法**」。
//!
//! **口径**：门禁 `scripts/god_gate.py::brace_metrics` 跳过 `impl Trait for Type`
//! （`god_gate.py:216`）⇒ 域侧 API 不计入 `World` 的方法数 ⇒ 门账上的 76 个方法
//! 在本目录落地后降到门面自身的数量（管线组装面）。
//!
//! **挂载路径**：本目录挂在 `world_struct` 下 ⇒ `world_struct.rs` 里 `pub use ext::*;`
//! ⇒ `lib.rs` 现有的 `pub use self::{…, world_struct::*};` 自动把各 trait 带到 **crate 根**
//! ⇒ 外部 crate 照旧 `use vxl_phys::HullExt;`（**lib.rs 零改动**，棘轮不动）。
mod aero;
mod bodybuild;
mod destruction;
mod diag;
mod field;
mod fluid;
mod hull;
mod impact;
mod mesh;
mod narrowtier;
mod provider;

pub use aero::AeroExt;
pub use bodybuild::BodyBuildExt;
pub use destruction::DestructionOpsExt;
pub use diag::DiagExt;
pub use field::FieldExt;
pub use fluid::FluidExt;
pub use hull::HullExt;
pub use impact::ImpactExt;
pub use mesh::MeshExt;
pub use narrowtier::NarrowTierExt;
pub use provider::ProviderExt;
