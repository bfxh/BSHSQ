//! **FluidExt**：`World` 的**液体域（边界耦合查询）**——本域**不往 `World` 添方法**。
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
use crate::World;

/// **FluidExt**：`World` 的液体域（边界耦合查询）。
pub trait FluidExt {
    fn fluid_boundary_coupling(&self, fluid: usize) -> bool;
    fn fluid_boundary_covered(&self, fluid: usize) -> &[bool];
    fn fluids(&self) -> &[crate::world_step::fluid_stepper::FluidSlot];
}

impl FluidExt for World {
    /// 该流体是否开了 2b（对账/测试用）。
    fn fluid_boundary_coupling(&self, fluid: usize) -> bool {
        self.fluid_boundary.is_two_b(fluid)
    }
    /// 2b **覆盖集**快照（第 `fi` 个流体那份，与 `bodies` 同序；越界给空切片）。
    /// §2 C4 起按流体各一份（原先跨流体单数组，只有最后一个 2b 流体的有效）。
    fn fluid_boundary_covered(&self, fluid: usize) -> &[bool] {
        match self.fluid_boundary.covered.get(fluid) {
            Some(c) => c,
            None => &[],
        }
    }
    /// 已注册流体系统及其边界 provider id（渲染读 `.0.positions()` / `.0.velocities()`；第三槽 = 卡上步进后端）。
    fn fluids(&self) -> &[crate::world_step::fluid_stepper::FluidSlot] {
        &self.fluids
    }
}
