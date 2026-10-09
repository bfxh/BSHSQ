//! **离线对比台场景复刻 + 相位剖析**（不依赖浏览器/前端）。
//!
//! 用 PhysArena 的同一批场景（同体数/同尺寸/同出生位姿/同材质）在原生侧跑
//! 固定步长基准，输出每步 p50/p95 与**相位分解**（宽相/窄相/求解/积分/CCD），
//! 用于引擎侧优化迭代。
//!
//! 运行：`cargo run --release -p vxl-phys --example arena_bench [场景] [--iters N]`
//! 场景：pyramid（默认）/ wall / ballpit / trimesh / `--list` 看**注册表全量**
//!       （PhysArena 复刻批在 `registry.rs`；`--all-arena` 全量跑）。
//!
//! 早期场景在 `run.rs` 的分派里（含打印型探针与 `fidelity`/`joints`）；
//! 2026-10-08 起的新场景一律走注册表：`arena_bench <id>` 单个、`--all-arena` 全部。
//! 本文件只留装配面（mod 声明 + 再导出 + `main`），入口逻辑在 `run.rs`。

use vxl_phys::*;
use vxl_phys_core::{FrictionModel, Material, PhysConfig, Shape, Vec3};

pub(crate) const WARMUP: usize = 30;
pub(crate) const MEASURE: usize = 180;

// ── 按域拆出的子模块（子目录 arena_bench/）
mod kit;
mod probes_a;
mod probes_b;
mod probes_c;
mod probes_domain;
mod probes_dyn;
mod probes_havoc;
mod probes_joint;
mod probes_machine;
mod probes_shapes;
mod probes_stack;
mod probes_stress;
mod registry;
mod report;
mod run;
pub(crate) use self::{
    kit::*, probes_a::*, probes_b::*, probes_c::*, probes_domain::*, probes_dyn::*,
    probes_havoc::*, probes_joint::*, probes_machine::*, probes_shapes::*, probes_stack::*,
    probes_stress::*, report::*,
};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

pub(crate) fn main() {
    run::run();
}
