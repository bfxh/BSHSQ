//! fluid_density：**密度相的核**（`kernel` 子模块）+ 本文件原有的自由函数已下沉。
//!
//! **密度相的内部步**（`density_pass` / `density_pass_parallel`）已搬进扩展 trait
//! [`FluidPipelineExt`](crate::system::ext::FluidPipelineExt)（`system/ext/pipeline.rs`）——
//! god 门处置方案「数据+管线同体的那一半」的执行面
//! （见 `docs/KNOWLEDGE.md`「2026-10-22 · 三条 god 债务同时出榜」一节）。
use super::*;

mod wall_gather;

pub(crate) mod kernel;
