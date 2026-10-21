//! **AeroExt**：`World` 的**气动域（面力/面力矩）**——本域**不往 `World` 添方法**。
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
use crate::{Vec3, World};

/// **AeroExt**：`World` 的气动域（面力/面力矩）。
pub trait AeroExt {
    fn set_aero(&mut self, cfg: vxl_phys_aero::AeroConfig);
    fn aero_force(&self, body: u32) -> Vec3;
    fn aero_torque(&self, body: u32) -> Vec3;
}

impl AeroExt for World {
    /// **开启面元气动**（T4；`AeroConfig` = 密度/Cd/风）。未调用 ⇒ `aero_pass` 短路 ⇒
    /// 默认档逐位不变。重复调用 = 覆盖配置（快照在下一子步重写）。
    fn set_aero(&mut self, cfg: vxl_phys_aero::AeroConfig) {
        self.aero = Some(vxl_phys_aero::AeroState::new(cfg));
    }
    /// 本子步施加在 `body` 上的**气动合力**（未开启 / 非三角网体 ⇒ `Vec3::ZERO`）。
    /// 仪器与被测同源：读的就是 `aero_pass` 刚施加的那份（不是另算一份）。
    fn aero_force(&self, body: u32) -> Vec3 {
        self.aero
            .as_ref()
            .and_then(|st| st.forces.get(body as usize))
            .copied()
            .unwrap_or(Vec3::ZERO)
    }
    /// 本子步施加在 `body` 上的**气动力矩**（关于体原点；口径同 `bodies.torque`）。
    fn aero_torque(&self, body: u32) -> Vec3 {
        self.aero
            .as_ref()
            .and_then(|st| st.torques.get(body as usize))
            .copied()
            .unwrap_or(Vec3::ZERO)
    }
}
