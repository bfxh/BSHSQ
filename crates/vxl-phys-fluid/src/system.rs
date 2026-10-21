//! system：从 lib.rs 按域拆出（纯搬移，语义未改）。
//!
//! **数据侧按域打包**（见 `system/domains.rs`）：`FluidSystem` 只留**域的清单**
//! （`Domains` 一个字段位）+ 配置 + 粒子数 + 相位计时 ⇒ **新加一个域 = 给
//! `Domains` 加一个字段，不必再往本类型加字段**（god 门「字段轴」债务的判据）。
use super::*;

pub(crate) mod domains;
use domains::Domains;

/// WCSPH 粒子流体系统（SoA；零外部依赖）。
///
/// **字段 = 域的清单 + 三样"跨域的轴"**：
/// - [`Domains`]：四个数据域（核参数 / 粒子状态 / 边界账 / 邻域网格）；
/// - `cfg`：运行配置（**输入**，不是流体状态）；
/// - `n_fluid`：**流体粒子数**（`pos`/`vel`/... 的**前缀**长度；`≥ n_fluid` 的是边界
///   粒子，同数组 ⇒ 既有核/式一字不改地作用于边界粒子，见模块头）；
/// - `phase_us`：**相位计时累加器**（微秒；仅诊断，不改行为）。
pub struct FluidSystem {
    /// 域的清单（数据侧全在这一个字段位里；新加一个域 = 给 `Domains` 加字段）。
    pub(crate) dom: Domains,
    /// 运行配置（输入；`FluidConfig` 的一次性派生见 `KernelParams::calibrate`）。
    pub(crate) cfg: FluidConfig,
    /// **流体粒子数**：`pos`/`vel`/... 的**前缀**长度；`≥ n_fluid` 的是边界粒子。
    pub(crate) n_fluid: usize,
    /// **相位计时累加器**（微秒；仅诊断，不改行为）：网格/密度/压力/力/积分+边界。
    /// 口径：每子步各相位一次 `probe::us`（`vxl_phys_core::probe`，wasm 下退化为 0）。
    pub(crate) phase_us: [u64; 5],
}
