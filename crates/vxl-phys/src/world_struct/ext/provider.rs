//! **ProviderExt**：`World` 的**碰撞提供者的面**——本域**不往 `World` 添方法**。
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

/// **ProviderExt**：`World` 的碰撞提供者的面。
pub trait ProviderExt {
    fn providers(&self) -> &Providers;
    fn refresh_provider_bounds(&mut self);
}

impl ProviderExt for World {
    /// 提供者集合只读视图（体素体诊断/可视化用）。
    fn providers(&self) -> &Providers {
        &self.providers
    }
    /// 提供者数据变化（如体素挖洞）后刷新宽相 AABB（确定性：按 id 序全量重算）。
    fn refresh_provider_bounds(&mut self) {
        for id in 0..self.providers.len() as u32 {
            if let Some(b) = self.providers.bounds(id) {
                self.provider_bounds[id as usize] = b;
            }
        }
    }
}
