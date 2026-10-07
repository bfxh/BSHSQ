//! 判据：**执行族要显式失败，不许静默降级**。
//!
//! `FluidConfig::family` 目前**未接线**（`FluidSystem` 是 CPU WCSPH；GPU 族走门面的
//! `World::set_fluid_stepper`）——本判据把"选 GPU 族 ⇒ 显式拒绝"钉住，防止它退化成
//! "悄悄跑 CPU SPH 但配置说自己在 GPU 上"（与 `DestructionConfig::depth` 同款纪律：
//! 配置要求的行为没实现时，入口报错，不静默降级）。
//!
//! 单开文件：`src/tests.rs` 受 god 门棘轮（file_lines 只准减），新文件只判阈值。

use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidFamily, FluidSystem};

#[test]
#[should_panic(expected = "只支持 CpuSph")]
fn gpu_family_is_rejected_not_silently_downgraded() {
    let cfg = FluidConfig {
        family: FluidFamily::GpuPbf,
        ..FluidConfig::default()
    };
    let _ = FluidSystem::new(cfg, Vec3::ZERO, [2, 2, 2], 0.1);
}
