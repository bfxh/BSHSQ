//! fluid_force：**力相的核**（`kernel` 子模块）。
//!
//! **力相的内部步**（`pressure_pass` / `force_pass` / `force_pass_parallel` /
//! `force_pass_serial`）已搬进扩展 trait
//! [`FluidPipelineExt`](crate::system::ext::FluidPipelineExt)（`system/ext/pipeline.rs`）——
//! god 门处置方案「数据+管线同体的那一半」的执行面
//! （见 `docs/KNOWLEDGE.md`「2026-10-22 · 三条 god 债务同时出榜」一节）。

pub(crate) mod kernel;
