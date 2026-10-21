//! **扫掠亚步内计数判据**（2026-10-13，`OPEN-PROBLEMS.md` **P20** 的结论落地）：
//! 动-动一类的差异**从外部看不见** —— tick 末端的埋深正好被皮肤带（离散接触允许的重叠）吃满，
//! 之前试的"目标侧姿态插值"在外部读数上只从 0.0200 动到 0.0194（噪声内）⇒ 判不出好坏。
//!
//! 所以先把**亚步内**的量做出来：`vxl_phys::ccd_sweep_stats_take()` 读后清零
//! `(采样数, 命中数, 二分迭代数, 命中样本最深穿透 m, 命中段号之和)`（`vxl-phys-solver/src/stats.rs`
//! 的 `ccd_*` 一族，与本仓既有的 `warm_match_stats_take`/`solve_accounting_take` 同址同口径；
//! **只计数、不参与决策 ⇒ 逐位中性**：本文件落地时 `determinism` 与 `m0_gates` 两档哈希
//! 与健康读数一字未动）。
//!
//! | 用例 | 钉住的事 |
//! |---|---|
//! | ① 薄壁档 | 开档时计数**在动**（采样/命中/二分都 > 0）——仪器真的接在扫掠上 |
//! | ② 关档档 | 关档时计数**全 0**——仪器不会把别处的窄相算进来 |
//! | ③ 动-动档（旋翼 × 弹体，24 相位） | **P20 的对照基线**（改 CCD ⇒ 必然变 ⇒ 一起更新并按 ADR-0004 记账） |

use vxl_phys::{ccd_sweep_stats_take, KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// ① 薄壁：1 m 盒 60 m/s 撞 5 mm 立板（同 `ccd_thin_shell.rs` 的场景）。
fn thin_wall(ccd_on: bool) -> (u64, u64, u64) {
    let cfg = PhysConfig {
        ccd_speed_threshold: if ccd_on { 10.0 } else { f32::INFINITY },
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.0025, 2.0, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(-5.0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(60.0, 0.0, 0.0);
    let _ = ccd_sweep_stats_take(); // 清掉旧账
    for _ in 0..120 {
        w.step();
    }
    let (samples, hits, refine, _depth, _step) = ccd_sweep_stats_take();
    (samples, hits, refine)
}

/// **① 开档：计数在动**（采样 / 命中 / 二分都 > 0）。
#[test]
fn counters_move_when_ccd_is_on() {
    let (samples, hits, refine) = thin_wall(true);
    assert!(samples > 0, "开档必须扫掠：采样数 {samples}");
    assert!(hits > 0, "薄壁必须被命中：命中数 {hits}");
    assert!(refine > 0, "命中后必须有 TOI 二分：迭代数 {refine}");
}

/// **② 关档：计数全 0**（仪器只数 CCD 自己的扫掠）。
#[test]
fn counters_stay_zero_when_ccd_is_off() {
    let (samples, hits, refine) = thin_wall(false);
    assert_eq!(
        (samples, hits, refine),
        (0, 0, 0),
        "关档不该有任何扫掠计数（别把主窄相算进来）"
    );
}

/// ③ 动-动档：运动学旋翼（ω=20 rad/s）× 60 m/s 弹体，旋翼初始角扫 24 档（每档 15°）。
/// 返回 `(采样数, 命中数, 二分迭代数, 命中样本最深穿透, 命中段号之和)`。
///
/// ⚠️ **固定在 S2 关档**（`ccd_target_substep = false`）：默认档自 2026-10-21 起**含 S2**
/// （见 `default_tier_target_substep_is_pinned`），而本基线是 P20 的**对照口径**（只带平移的
/// 旧行为）⇒ 显式写死开关 ⇒ 读数不随默认档改动而漂。
fn dyn2_sweep(fine: bool) -> (u64, u64, u64, f32, u64) {
    dyn2_sweep_sub(Some(false), fine)
}

/// **③ 动-动对照基线**（有意**逐字钉住**）：这几个数是 P20 的定量底账。
///
/// **2026-10-20 换代（P20 两半）**：目标侧按时间插值**姿态**（`Target{rot1,omega}`），
/// 且 `eval_hit` 的对方速度取**接触点速度** `v + ω×r`（`ω=0` 逐位不变）。
/// 本场景的叶片以 20 rad/s 自旋 ⇒ 采样看见的是 tick 中途的叶片而不是 tick 末那一份：
/// 命中数 **231 → 126**、精化趟数 **1848 → 1008**、平均命中段号 **4.58 → 4.10**（更早），
/// 命中样本最深穿透 **0.16097 → 0.15948**（P21 的关键读数，略好但仍在 0.16 量级）。
/// 代价：`m0_gates` 门槛档唤醒抖动 **3 → 6**（0.30 → 0.60 次/秒，SPEC 阈值 <1 ⇒ 过）、
/// 门槛档最大深度 0.5003 → **0.4536**（更好）；压力档健康读数不变、只有轨迹哈希换代。
/// ⚠️ 改 CCD（尤其动目标侧）会让这些数变 ⇒ 必须**一起更新**并按 ADR-0004 记账。
#[test]
fn dyn2_intra_tick_baseline_is_pinned() {
    let (samples, hits, refine, deepest, step_sum) = dyn2_sweep(false);
    eprintln!("dyn2: samples={samples} hits={hits} refine={refine} step_sum={step_sum} deepest={deepest:.5}");
    assert_eq!(
        (samples, hits, refine, step_sum),
        (268706, 126, 1008, 517),
        "动-动档的亚步内计数变了 ⇒ 先看是不是 CCD 动目标侧改动，再按 ADR-0004 更新本行"
    );
    assert!(
        (deepest - 0.15948).abs() < 1e-4,
        "命中样本最深穿透（P20 的关键读数）实测 {deepest}，基线 0.15948"
    );
}

/// **P21 细采样档的仪表**（`--ignored`，只打印不断言）：同一动-动档在
/// `ccd_fine_sample = false/true` 下的「首命中深度 / 段数 / 命中数」。
/// 用来回答"要采多细才够"，不参与 CI 判定。
/// 静态薄墙 × 快盒：P21 的典型形态（薄目标 + 快体整段跳过皮肤带）。
fn wall_sweep(fine: bool) -> (u64, u64, u64, f32, u64) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        ccd_fine_sample: fine,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.005, 1.0, 1.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(-3.0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(80.0, 0.0, 0.0);
    let _ = ccd_sweep_stats_take();
    for _ in 0..20 {
        w.step();
    }
    ccd_sweep_stats_take()
}

#[test]
#[ignore = "P21 细采样 off/on 读数（只打印）"]
fn diag_fine_sampling() {
    for fine in [false, true] {
        let (samples, hits, refine, deepest, step_sum) = wall_sweep(fine);
        eprintln!(
            "wall fine={fine}: samples={samples} hits={hits} refine={refine} deepest={deepest:.5} step_sum={step_sum}"
        );
    }
    for fine in [false, true] {
        let (samples, hits, refine, deepest, step_sum) = dyn2_sweep(fine);
        eprintln!(
            "fine={fine}: samples={samples} hits={hits} refine={refine} deepest={deepest:.5} step_sum={step_sum}"
        );
    }
}

/// **S2 目标子步化**的档位仪表：自旋薄片档。
/// `sub = None` ⇒ **默认档**（不显式写 `ccd_target_substep`）；`Some(b)` ⇒ 显式写该开关。
fn dyn2_sweep_sub(sub: Option<bool>, fine: bool) -> (u64, u64, u64, f32, u64) {
    let mut tot = (0u64, 0u64, 0u64, 0.0f32, 0u64);
    for k in 0..24 {
        let mut cfg = PhysConfig {
            gravity: Vec3::ZERO,
            max_linear_velocity: 5000.0,
            ccd_fine_sample: fine,
            ..PhysConfig::default()
        };
        if let Some(on) = sub {
            cfg.ccd_target_substep = on;
        }
        let mut w = World::new(cfg);
        let blade = w.add_kinematic(
            Shape::Box {
                half: Vec3::new(0.6, 0.4, 0.02),
            },
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, (k as f32 * 15.0).to_radians()),
        );
        w.bodies
            .set_angvel(blade as usize, Vec3::new(0.0, 20.0, 0.0));
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.1),
            },
            Vec3::new(-6.0, 0.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        w.bodies.linvel[b] = Vec3::new(60.0, 0.0, 0.0);
        let _ = ccd_sweep_stats_take();
        for _ in 0..300 {
            w.step();
        }
        let (s, h, r, d, st) = ccd_sweep_stats_take();
        tot = (tot.0 + s, tot.1 + h, tot.2 + r, tot.3.max(d), tot.4 + st);
    }
    let _ = ccd_sweep_stats_take();
    tot
}

#[test]
#[ignore = "S2 子步化 off/on 读数（只打印）"]
fn diag_target_substep() {
    for (on, fine) in [(false, false), (true, false), (true, true)] {
        let (samples, hits, refine, deepest, step_sum) = dyn2_sweep_sub(Some(on), fine);
        eprintln!(
            "substep={on} fine={fine}: samples={samples} hits={hits} refine={refine} deepest={deepest:.5} step_sum={step_sum}"
        );
    }
}

/// **默认档**含 S2（`ccd_target_substep` 默认**开**，2026-10-21）：自旋薄片档按**默认配置**跑。
///
/// 读数 = S2 落地时 A/B 档的 on 值：samples 291642 / hits 99 / refine 792 / step_sum 686、
/// **deepest 0.01711（< skin 0.02）** ⇒ "默认开"不是纸面声明，默认档实测就给这组数。
///
/// 背景（S2 定向推出，2026-10-21）：原状动体中心正落在叶片中面时 `n` 的符号无意义
/// （`n·rel = 0`），相邻子步推向相反侧 ⇒ 净位移 ≈ 0、盒留在叶片里 ⇒ 自旋档 deepest 卡在
/// **0.08478**（≥ 4×skin）。修后：从哪侧进来就推回哪侧（位移 `sep + depth`）⇒ deepest 0.01711。
/// ⚠️ 改 S2 ⇒ 先看 `push_out` 的「所在侧」判定，再按 ADR-0004 更新本行。
#[test]
fn default_tier_target_substep_is_pinned() {
    assert!(
        PhysConfig::default().ccd_target_substep,
        "默认档必须含 S2（2026-10-21 起默认开）"
    );
    let (samples, hits, refine, deepest, step_sum) = dyn2_sweep_sub(None, false);
    eprintln!("default tier: samples={samples} hits={hits} refine={refine} step_sum={step_sum} deepest={deepest:.5}");
    assert_eq!(
        (samples, hits, refine, step_sum),
        (291642, 99, 792, 686),
        "默认档的 S2 读数变了 ⇒ 先看 push_out 的「所在侧」判定与 S2 的生效条件，再按 ADR-0004 更新本行"
    );
    assert!(
        deepest <= 0.02,
        "自旋档 deepest 必须落在皮肤带内（0.02），实测 {deepest}"
    );
}
