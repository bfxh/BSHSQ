//! 自适应 shock 扫掠预算：**按岛内质量比**给附加迭代，而不是全局常数。
//!
//! 存在理由（2026-10-09，R5 高质量比静默互穿）：顺序冲量（GS）的收敛率随岛内
//! 质量比恶化。均匀堆（比 ≈1）默认档 2×3 就收敛；"重压轻"（比 ≥15）在默认档下
//! **根本不收敛**——支撑载荷传不到最底层，盒子静默互穿（R5：比 30 时两盒同位、
//! `nan = 0`、位置误差恰一个盒高）。同一份全局预算同时服务这两种岛是不可能的，
//! 而岛的质量比在求解前**就能算出来** ⇒ 预算按岛给。
//!
//! 为什么是新文件：`island.rs` 受 god 门 file_lines 棘轮（只准减），把新函数塞进去
//! 会让"文件变胖但最长函数没变短"判红。新文件只受阈值管（≤800 行 / 函数 ≤120 行）。
use vxl_phys_core::BodySet;

use crate::types::Island;

/// 岛内**质量比**触发自适应 shock 扫掠的下限（比 = 最重体 / 最轻体）。
///
/// 阈值来自 R5 扫描（见 `docs/KNOWLEDGE.md` §O）：比 ≤14 时默认档 2×3 已收敛，
/// 15 起才开始需要附加反序扫掠。取 8 是留余量，同时**高于全部冻结金样场景**
/// （同密度同形状 ⇒ 比 1.0；混合形状最多 ≈3.0）⇒ 均匀岛恒返回 `base`、逐位同旧。
pub(crate) fn mass_ratio_shock_min() -> f32 {
    8.0
}

/// 自适应 shock 扫掠的**上界**（防病态质量比把一帧拖死）。
pub(crate) fn mass_ratio_shock_cap() -> u32 {
    16
}

/// 按岛内质量比给 shock 附加扫掠预算（`base` = `PhysConfig::shock_iterations`）。
///
/// 斜率来自同一次 R5 扫描（比 15/30/100 分别需 1/3/7 次附加扫掠）：
/// `ceil(2·log2(比/8))` 到比 ≈100 都有余量，再往上由 `cap` 兜底（诚实承认病态
/// 质量比仍不在覆盖区）。`base.max(extra)` ⇒ 显式配置的 `shock_iterations`
/// 仍是**下界**，用户既有配置的语义不丢。
pub(crate) fn island_shock_budget(
    bodies: &BodySet,
    isl: &Island,
    base: u32,
    de_penetration_pass: bool,
    resid: f32,
) -> u32 {
    // 无偏置趟（`cleanup`，其标志 = 该趟把 `max_corr` 归零）不做自适应追加：
    // 那一趟的职责是把修正速度从最终速度里移除，再塞去穿透扫掠会把它写回去。
    if !de_penetration_pass {
        return base;
    }
    // **收敛门**：主迭代已经收敛的岛一律不追加。这一条让"质量比大但本来就收敛"的场景
    // （如 mesh_vs_mesh 的双面片：比 20，却因接触少而早早收敛）与旧行为**逐位一致** ——
    // 只有"质量比大且真的没收敛"才付这笔钱。`early_exit_eps` 是主迭代自己的收敛阈值。
    if resid < crate::island::early_exit_eps() {
        return base;
    }
    let (mut lo, mut hi) = (f32::INFINITY, 0.0f32);
    for &bi in &isl.bodies {
        let im = bodies.inv_mass[bi as usize];
        if im > 0.0 {
            lo = lo.min(im);
            hi = hi.max(im);
        }
    }
    if !lo.is_finite() || lo <= 0.0 || hi <= 0.0 {
        return base;
    }
    let ratio = hi / lo; // inv_mass 之比 = 质量比（最重 / 最轻）
    let min_ratio = mass_ratio_shock_min();
    if ratio <= min_ratio {
        return base;
    }
    let extra = (2.0 * (ratio / min_ratio).log2()).ceil();
    base.max(extra as u32).min(mass_ratio_shock_cap())
}
