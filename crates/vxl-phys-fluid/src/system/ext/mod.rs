//! **system/ext**：`FluidSystem` 的**管线扩展 trait**（god 门处置方案的执行面）。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-fluid::FluidSystem"]` 写明——
//! 数据侧已按域打包（**字段 26 → 4**），剩余待做是「**数据+管线同体的那一半**」：
//! `FluidSystem` 的 40 个方法里有一半是**管线内部步**，它们与「SoA 数据访问器」混在同一个
//! 类型上 ⇒ **新加一个相位就往本类型添方法**。处置是「**管线内部步搬进扩展 trait**」，
//! 判据是「新加一个相位**不必再往 `FluidSystem` 添方法**」。
//!
//! **口径**：门禁 `scripts/god_gate.py::brace_metrics` 跳过 `impl Trait for Type`
//! ⇒ 管线内部步不计入 `FluidSystem` 的方法数 ⇒ 门账上的 40 个方法在本目录落地后
//! 降到 25（**全是公开访问器与公开入口**）。
//!
//! **挂载路径**：本目录挂在 `system` 下 ⇒ `system.rs` 里 `pub(crate) mod ext;` ⇒
//! `crate::system::ext::FluidPipelineExt` ⇒ crate 内各文件 `use` 它。
//!
//! **可见性**：`pub(crate)`（方法原本全是 crate 内可见 ⇒ 不外扩）。
mod pipeline;

pub(crate) use pipeline::FluidPipelineExt;
