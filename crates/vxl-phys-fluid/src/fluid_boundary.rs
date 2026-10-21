//! fluid_boundary：**边界段表**（`FluidSystem` 的公开只读面）。
//!
//! **本文件现在只留公开段表**（`boundary_spans`）—— 边界相的**内部步**
//! （`boundary_pass` / `aggregate_reactions` / `separate_coincident`）已搬进扩展 trait
//! [`FluidPipelineExt`](crate::system::ext::FluidPipelineExt)（`system/ext/pipeline.rs`）——
//! god 门处置方案「数据+管线同体的那一半」的执行面
//! （见 `god.gate.json` 的 `_god_pairs_empty_doc` 与 `docs/KNOWLEDGE.md`
//! 「2026-10-22 · 三条 god 债务同时出榜」一节）。
use super::*;

impl FluidSystem {
    /// **边界段表**（只读；GPU 档与耦合回路用）：段序 = 生成序（确定性），每项
    /// `(体 id, 体原点, 起始索引, 结束索引)`。`[start, end)` 即该体的边界粒子区间，
    /// 也正是 `boundary_reactions()` 的聚合单位 ⇒ GPU 侧按同一段表聚合即可**逐体对拍**。
    pub fn boundary_spans(&self) -> &[(u32, Vec3, u32, u32)] {
        &self.dom.bnd.spans
    }
}
