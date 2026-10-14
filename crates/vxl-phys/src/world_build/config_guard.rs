//! config_guard：`PhysConfig` 的**入口收口**（安全审计 F-11 / F-16，系统簇 S1「入口零校验」）。
//!
//! **为什么在门面而不是 core**：`PhysConfig` 的字段是 `pub`，内核按"输入合法"写；门面是集成
//! 方唯一必须经过的入口（[`crate::World::new`] / `with_broadphase`）——把"病态但可解释"的参数
//! 在进内核**之前**钳到物理有意义的档，是最小侵入的防御层。内核里不加断言（审计 §5 明确
//! 不建议"把可见崩溃换成更隐蔽的逻辑错误"，也不建议往热路径塞 panic 分支）。
//!
//! **三条口径**：
//! ① **默认档逐位不变**：`PhysConfig::default()`、四个 `Preset`、仓库内所有既有配置（含
//!    `bench/realism-probe` 与 `arena_bench`）都落在档内 ⇒ 本层对它们返回 0，零行为变化
//!    （判据 = determinism / m0_gates / m1_islands 三行为门 + 金样 8 项）；
//! ② 只收口**确定有害**的字段：会挂起的迭代上限（F-16：`substeps = u32::MAX` 首步不返回）、
//!    会静默穿地的步长/接触带（F-11：`dt = 10` 实测穿地到 y ≈ −1.2e5）、非有限的
//!    重力与阈值（F-08 的入口面）；
//! ③ **合法的极端值原样保留**：`sleep_time = INFINITY`（关睡眠探针）、`sleep_linear = 0`
//!    （基准对照）、`restitution = 1`（R2 弹性落体）、`ccd_speed_threshold = INFINITY`
//!    （**显式关 CCD**；引擎默认是选择性 10 m/s）都通过——收口不是"合理化到默认值"。
//!
//! **返回被修正的字段数**（0 = 原样）⇒ 调用方可断言"没有静默改写"。**边界（写清，不是漏）**：
//! 字段是 `pub`，运行期直接 `world.config.dt = NaN` 会绕过本层；收口只覆盖构造入口，运行期
//! 的 NaN 体隔离属审计 F-08 第 3 点（另案）。

use vxl_phys_core::PhysConfig;

/// 步长上限：60 Hz 契约（`SPEC.md` §5）的 60× 容忍——再大就越过"一 tick 内不穿"的保守假设。
const DT_MAX: f32 = 1.0;
/// 步长下限：60 Hz 的 1/6000；再小只是白烧 CPU（且 `dt` 作分母的地方会逼近 f32 精度边界）。
const DT_MIN: f32 = 1.0 / 6000.0;
/// 接触带（speculative skin）上限：默认 0.02、四档最大 0.02；0.5 已是"半米内全算接触"。
const SKIN_MAX: f32 = 0.5;
/// 子步上限：`SPEC.md` §4.2 的档位表是 `{1,2,4,8,16}` ⇒ 16。
const SUBSTEPS_MAX: u32 = 16;
/// 迭代类上限：§4.1 的档位表是 `{1,4,8,16,32,64}` ⇒ 64（`joint_iterations` 同表）。
const ITERS_MAX: u32 = 64;
/// 法向内层扫掠上限（默认 1；shock/settled/stabilization 三族都按同一数量级封顶）。
const INNER_MAX: u32 = 16;

/// `f32` 字段：落在闭区间 `[lo, hi]` 才放行（NaN 因比较全假而同样回落——正是要收的病态值）。
fn rangef(v: &mut f32, lo: f32, hi: f32, fallback: f32) -> u32 {
    if *v >= lo && *v <= hi {
        0
    } else {
        *v = fallback;
        1
    }
}

/// `u32` 字段：落在闭区间 `[lo, hi]` 才放行。
fn rangeu(v: &mut u32, lo: u32, hi: u32, fallback: u32) -> u32 {
    if *v >= lo && *v <= hi {
        0
    } else {
        *v = fallback;
        1
    }
}

/// `f32` 字段：**非 NaN 且 ≥ 0** 才放行（`+INFINITY` 是合法档，如"关睡眠"）。
fn nonneg(v: &mut f32, fallback: f32) -> u32 {
    if *v >= 0.0 {
        0
    } else {
        *v = fallback;
        1
    }
}

/// `f32` 字段：**严格正或 `+INFINITY`** 才放行（0 = 闸死一切运动，是笔误不是配置）。
fn positive(v: &mut f32, fallback: f32) -> u32 {
    if *v > 0.0 || *v == f32::INFINITY {
        0
    } else {
        *v = fallback;
        1
    }
}

/// 本机可用并行度（取不到 ⇒ 1）。线程预算用它当上界。
fn parallelism() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
}

/// 收口一张配置（就地改写，返回被修正的字段数）。
pub(crate) fn sanitize(cfg: &mut PhysConfig) -> u32 {
    let d = PhysConfig::default();
    let mut n = 0;
    // F-11：步长（`dt ≤ 0` ⇒ 反向/静止；极大步长 ⇒ 穿地）与接触带（NaN ⇒ 判据全假 ⇒ 无接触）。
    n += rangef(&mut cfg.dt, DT_MIN, DT_MAX, d.dt);
    n += rangef(&mut cfg.contact_skin, 0.0, SKIN_MAX, d.contact_skin);
    n += rangef(&mut cfg.gjk_tolerance, 0.0, 1.0, d.gjk_tolerance);
    n += rangef(&mut cfg.restitution, 0.0, 1.0, d.restitution);
    // F-16：迭代上限。`u32::MAX` 档在旧实现里是"首步不返回"（每个子步都跑满 u32::MAX 遍）。
    n += rangeu(&mut cfg.substeps, 1, SUBSTEPS_MAX, d.substeps);
    n += rangeu(
        &mut cfg.velocity_iterations,
        1,
        ITERS_MAX,
        d.velocity_iterations,
    );
    n += rangeu(&mut cfg.joint_iterations, 1, ITERS_MAX, d.joint_iterations);
    n += rangeu(&mut cfg.normal_inner, 1, INNER_MAX, d.normal_inner);
    n += rangeu(&mut cfg.shock_iterations, 0, INNER_MAX, d.shock_iterations);
    n += rangeu(
        &mut cfg.settled_hold_iterations,
        0,
        INNER_MAX,
        d.settled_hold_iterations,
    );
    n += rangeu(
        &mut cfg.stabilization_iterations,
        0,
        INNER_MAX,
        d.stabilization_iterations,
    );
    // F-08 入口面：限速 0 是笔误（闸死运动）；NaN 会让每次比较都走"不钳"的分支。
    n += positive(&mut cfg.max_linear_velocity, d.max_linear_velocity);
    n += positive(&mut cfg.max_angular_velocity, d.max_angular_velocity);
    // 睡眠三参：0 合法（基准对照）、`+INFINITY` 合法（关睡眠），只有 NaN/负数是笔误。
    n += nonneg(&mut cfg.sleep_linear, d.sleep_linear);
    n += nonneg(&mut cfg.sleep_angular, d.sleep_angular);
    n += nonneg(&mut cfg.sleep_time, d.sleep_time);
    n += nonneg(&mut cfg.ccd_speed_threshold, d.ccd_speed_threshold);
    // 滚动阻力：0 合法（关档）、有限非负；NaN/负数 ⇒ 回默认（0）。
    n += nonneg(&mut cfg.roll_resistance, d.roll_resistance);
    n += nonneg(&mut cfg.film_drag, d.film_drag);
    n += nonneg(&mut cfg.added_mass_coeff, d.added_mass_coeff);
    // F-17：线程数。`ScopedPool` 每个并行区域开 `threads − 1` 个 OS 线程、与工作量无关
    // ⇒ 超过本机并行度只多占线程（实测 2600 体 28 → 48 ms）而不增吞吐。0/1 是合法档（串行）。
    let par = parallelism();
    if cfg.threads > par {
        cfg.threads = par;
        n += 1;
    }
    // 重力：一个 NaN 分量会让**全世界**在若干 tick 内 NaN（审计 F-08 实测 7/7）。
    if !(cfg.gravity.x.is_finite() && cfg.gravity.y.is_finite() && cfg.gravity.z.is_finite()) {
        cfg.gravity = d.gravity;
        n += 1;
    }
    n
}
