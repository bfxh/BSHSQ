//! `world_step::wind`：**风的下发**（把同一份 `AeroConfig` 交给各受体的两条腿）。
//!
//! 为什么单开文件：`world_step/aero.rs` 在 god 门 file_lines 基线上（78 行 / 最长函数 33）
//! ⇒ 加行必须让最长函数**变短**；把两条受体腿搬出来既守住门账，也让 aero 那边只剩
//! "体元施力 + 快照"。
//!
//! - **布 × 风**：`cfg` 幂等下发到每张布（`cloth.aero.enabled` 由 `set_aero` 的语义决定）；
//! - **液 × 风**：自由表面逐粒表面驱动（`vxl_phys_fluid::fluid_access::wind::surface_drag`），
//!   反作用沉积回 [`vxl_phys_aero::AeroState`] 的 `absorbed` 审计量。
//!
//! 两条腿都**只在 `set_aero` 开着时被调用**（`aero_pass` 的 `Option` 短路）⇒ 默认档零成本。
// 显式导入（不用 `use super::*`——glob-gate：新文件零通配）。
use super::fluid_stepper::FluidSlot;

/// 布 × 风：同一份风配置下发到每张布（每子步重写同一份 ⇒ 幂等）。
pub(crate) fn config_cloths(
    cloths: &mut [vxl_phys_soft::ClothSheet],
    cfg: vxl_phys_aero::AeroConfig,
) {
    for cloth in cloths.iter_mut() {
        cloth.aero.enabled = true;
        cloth.aero.cfg = cfg;
    }
}

/// 液 × 风：逐流体槽对**自由表面**粒子施加表面驱动（`dt` = 体子步，与面元气动同段位）。
pub(crate) fn drive_liquids(
    liquids: &mut [FluidSlot],
    medium: &mut dyn vxl_phys_core::interop::MediumField,
    cd: f32,
    dt: f32,
) {
    let cfg = vxl_phys_fluid::fluid_access::wind::SurfaceDrag {
        cd,
        ..Default::default()
    };
    for slot in liquids.iter_mut() {
        vxl_phys_fluid::fluid_access::wind::surface_drag(&mut slot.0, medium, cfg, dt);
    }
}
