//! **DiagExt**：`World` 的**诊断面（流形表/状态哈希）**——本域**不往 `World` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys::World"]` 把 `World` 登记为
//! **已登记债务（非豁免）**，处置写的是「**域侧 API 拆成扩展 trait**（`impl FluidExt for World`
//! 等），门面只留管线组装」，判据是「新加一域**不必再往 `World` 添方法**」。
//!
//! **先例**（同一判据的既有落法）：`world_step/destruction.rs` 的 `DestructionExt`、
//! `world_step/conversion.rs` 的 `VoxelConversionExt`、`vxl-phys-sense` 的 `SenseExt`、
//! `world_aero_wake.rs` 的 `AeroWakeExt`、`world_kinematic.rs` 的 `KinematicExt`。
//!
//! **门的口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! （`god_gate.py:216`）⇒ 搬进 trait 后 `World` 的方法数**不计入本类型**。
//!
//! **数值与顺序一字不改**（纯搬移：方法体逐字搬过来，未改一个算式、一个遍历序）。
use crate::*;

/// **DiagExt**：`World` 的诊断面（流形表/状态哈希）。
pub trait DiagExt {
    fn manifolds(&self) -> &[Manifold];
    fn state_hash(&mut self) -> u128;
}

impl DiagExt for World {
    /// 当前接触流形（渲染/游戏逻辑只读视图）。
    fn manifolds(&self) -> &[Manifold] {
        &self.manifolds
    }
    /// 规范状态哈希（§5 xxh3-128 唯一真值；热组 32B 记录按 id 序打包）。
    ///
    /// 规范化缓冲走帧级相位 arena（`arenas.hash`）：每次调用 alloc→打包→reset，
    /// 全帧零堆分配、缓冲永不增长。因世界自持暂存，故取 `&mut self`。
    fn state_hash(&mut self) -> u128 {
        let scratch = self
            .arenas
            .hash
            .alloc(HASH_SCRATCH_BYTES, 8)
            .expect("哈希暂存预算固定 8KB，流式打包不随体数增长");
        let h = vxl_phys_replay::hash_bodies_streaming(&self.bodies, scratch);
        self.arenas.hash.reset();
        h
    }
}
