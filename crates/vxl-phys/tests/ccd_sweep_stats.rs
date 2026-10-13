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
fn dyn2_sweep() -> (u64, u64, u64, f32, u64) {
    let mut tot = (0u64, 0u64, 0u64, 0.0f32, 0u64);
    for k in 0..24 {
        let cfg = PhysConfig {
            gravity: Vec3::ZERO,
            max_linear_velocity: 5000.0,
            ..PhysConfig::default()
        };
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

/// **③ 动-动对照基线**（有意**逐字钉住**）：这几个数是 P20 那笔"试过、未落"的定量底账 ——
/// 落地时点：目标侧**只带平移**（`Target{q1,vel}`）。
///
/// ⚠️ 改 CCD（尤其动目标侧）会让它们变 ⇒ 必须**一起更新**并按 ADR-0004 记账；
/// 参考：把"目标侧姿态插值 + 接触点速度"打上时，命中数 **231 → 126**、平均命中段号
/// **4.58 → 4.10**（更早），但**命中样本最深穿透 0.16097 → 0.15948 基本不动**，
/// 而 `m0_gates` 门槛档唤醒抖动 **7 → 10**（1.000/s 贴边、判红）⇒ 不落。
#[test]
fn dyn2_intra_tick_baseline_is_pinned() {
    let (samples, hits, refine, deepest, step_sum) = dyn2_sweep();
    assert_eq!(
        (samples, hits, refine, step_sum),
        (256790, 231, 1848, 1058),
        "动-动档的亚步内计数变了 ⇒ 先看是不是 CCD 动目标侧改动，再按 ADR-0004 更新本行"
    );
    assert!(
        (deepest - 0.16097).abs() < 1e-4,
        "命中样本最深穿透（P20 的关键读数）实测 {deepest}，基线 0.16097"
    );
}
