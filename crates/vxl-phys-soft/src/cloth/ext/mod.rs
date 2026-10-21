//! **cloth/ext**：`ClothSheet` 的**约束族扩展 trait**（god 门处置方案的执行面）。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——
//! 36 个方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在
//! 同一片上，处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，
//! 判据是「新加一族约束**不必再往本类型添方法**」。
//!
//! **口径**：门禁 `scripts/god_gate.py::brace_metrics` 跳过 `impl Trait for Type`
//! ⇒ 约束族的方法不计入 `ClothSheet` 的方法数 ⇒ 门账上的 36 个方法在本目录落地后
//! 降到表示面（构造/索引/推进）的数量。
//!
//! **本轮落的五个族**：
//! - `ClothTearExt`：撕裂域（8 个方法）—— `cloth_tear.rs`。
//! - `ClothPlasticExt`：塑性域（4 个方法）—— `cloth/plastic.rs`。
//! - `ClothBodyExt`：刚体耦合域（7 个方法）—— `cloth_coupling.rs`。
//! - `ClothSelfCollideExt`：自碰撞域（2 个方法）—— 原 `cloth_self_collide_pass.rs` +
//!   `cloth_self_collision/edge.rs`。
//! - `ClothDamageExt`：损伤域汇总步（1 个方法）—— `cloth/damage.rs`。
//!
//! **可见性分两档**（**不扩大公开面**）：
//! - **`pub`**：`ClothTearExt` 与 `ClothPlasticExt`——它们的方法原本就是 `pub`
//!   （判据仪器：`is_torn` / `torn_count` / `edge_strain` / `rest_of` / `plastic_of` …），
//!   外部 crate 在用 ⇒ 搬进 trait 后由 **trait 自身的 `pub`** 提供同样的可见性
//!   （调用方 `use` 该 trait 后点号语法照旧可用）⇒ **公开 API 不破坏**。
//! - **`pub(crate)`**：`ClothBodyExt` / `ClothSelfCollideExt` / `ClothDamageExt`——
//!   它们的方法原本就是 `pub(crate)` 或私有 ⇒ 保持 crate 内可见 **不外扩**
//!   （顺带避免 `BodyHit` 这类 `pub(crate)` 类型出现在 `pub` trait 签名里
//!   ⇒ Rust 的 `private_interfaces` lint 不响）。
//!
//! **挂载路径**：本目录挂在 `cloth` 下 ⇒ `cloth.rs` 里 `pub mod ext;` ⇒
//! `lib.rs` 显式再导出**两个 `pub` 的** trait 到 crate 根
//! （`pub use cloth::ext::{ClothPlasticExt, ClothTearExt};`）⇒ 外部 crate 照旧
//! `use vxl_phys_soft::ClothTearExt;`（**不破坏**）。
mod body;
mod damage;
mod plastic;
mod selfcollide;
mod tear;

pub(crate) use body::ClothBodyExt;
pub(crate) use damage::ClothDamageExt;
pub use plastic::ClothPlasticExt;
pub(crate) use selfcollide::ClothSelfCollideExt;
pub use tear::ClothTearExt;
