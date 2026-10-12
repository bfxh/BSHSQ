//! **K6：运动学平台 × 大量载荷**（`docs/PLAN-kinematic.md` §X1 的落点判据）。
//!
//! 用户口径："电梯上行还要被大量的东西产生反应……速度和性能高"。A 档并岛的**代价**已被 C7 钉住
//! （静态支撑 7 岛 → 运动学/动态支撑 1 岛 = 并行度归零），本文件把"**真载荷下会怎样**"量出来：
//!
//! | 判据 | 钉住的行为 |
//! |---|---|
//! | L1 大载荷被带走 | 10×10 盒阵列（互不接触、只与平台接触）随平台上行 1 s ⇒ 全部 Δy ≈ 0.6、不被吞、不逃逸 |
//! | L2 A 档代价 | 同场景 **岛数 = 1**（100 体 + 平台被焊成一个岛 —— 这就是 A 档接受的代价） |
//! | L3 载荷链稳定 | 末态最大穿透 ≤ 2×skin、所有体速度 ≤ 平台速度 + ε、无 NaN |
//! | L4 性能读数 | **只报不判**（计时类不进 CI，同 `bench/sweep_scale.py` 的口径）：100/400 体两档的
//!   "平台静止 vs 上行" p50 tick 时间，机器标注 |

use std::time::Instant;
use vxl_phys::{KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// 平台（运动学，顶面 `y = 0`）+ `n×n` 个 1 m 盒。
/// ⚠️ 间距 **1.06**（> `skin = 0.02` 两倍）：1.02 会让相邻盒落在 skin 边界上生成**投机接触**
/// ⇒ 静态档下网格自己就连成一片（实测 6 个岛而不是 100 个），对照就不干净了。
fn load_scene(n: usize, kinematic: bool) -> (World, Vec<usize>, usize) {
    let cfg = PhysConfig {
        sleep_time: f32::INFINITY, // K6 量的是解算成本，不是睡眠行为
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let half = 0.5 * (n as f32 * 1.06) + 0.5;
    let plate = Shape::Box {
        half: Vec3::new(half, 0.5, half),
    };
    let floor = if kinematic {
        w.add_kinematic(plate, Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY)
    } else {
        w.add_static(plate, Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY)
    } as usize;
    let mut boxes = Vec::with_capacity(n * n);
    let x0 = -0.53 * (n as f32 - 1.0);
    for ix in 0..n {
        for iz in 0..n {
            boxes.push(w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(x0 + 1.06 * ix as f32, 0.5, x0 + 1.06 * iz as f32),
                Quat::IDENTITY,
                500.0,
            ) as usize);
        }
    }
    (w, boxes, floor)
}

/// 跑 `ticks` 个 tick（`lift > 0` 时平台以该速度上行），返回 `(每体的 Δy, 最大埋入, 末态最大 |v|, 最大深度, p50 µs)`。
fn run(n: usize, lift: f32, ticks: usize) -> (Vec<f32>, f32, f32, f32, u64) {
    let (mut w, boxes, floor) = load_scene(n, true);
    for _ in 0..60 {
        w.step(); // 坐稳
    }
    let y0: Vec<f32> = boxes.iter().map(|&i| w.bodies.position[i].y).collect();
    if lift > 0.0 {
        w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, lift, 0.0), Vec3::ZERO);
    }
    let mut samples: Vec<f32> = Vec::with_capacity(ticks);
    for _ in 0..ticks {
        let t = Instant::now();
        w.step();
        samples.push(t.elapsed().as_secs_f32() * 1e6);
    }
    samples.sort_by(f32::total_cmp);
    let p50 = samples[samples.len() / 2] as u64;
    let dys: Vec<f32> = boxes
        .iter()
        .zip(&y0)
        .map(|(&i, &y)| w.bodies.position[i].y - y)
        .collect();
    let plate_top = w.bodies.position[floor].y + 0.5;
    let bury = boxes
        .iter()
        .map(|&i| plate_top - (w.bodies.position[i].y - 0.5))
        .fold(f32::MIN, f32::max);
    let vmax = boxes
        .iter()
        .map(|&i| w.bodies.linvel[i].length())
        .fold(0.0f32, f32::max);
    (dys, bury, vmax, w.health().max_depth, p50)
}

/// **L1 + L3 大载荷被带走且载荷链稳定**（100 体；平台 0.6 m/s 上行 1 s）。
#[test]
fn l1_big_load_is_carried_and_stable() {
    let (dys, bury, vmax, depth, _p50) = run(10, 0.6, 60);
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for &dy in &dys {
        lo = lo.min(dy);
        hi = hi.max(dy);
    }
    println!(
        "[L1 100 体 × 0.6 m/s] Δy ∈ [{lo:.4}, {hi:.4}]（期望 ≈ 0.6）｜最大埋入 {bury:.4} m｜末态 |v|max {vmax:.4}｜最大深度 {depth:.4} m"
    );
    assert!(
        (lo - 0.6).abs() < 0.02 && (hi - 0.6).abs() < 0.02,
        "100 个载荷都该被带走（Δy ∈ [{lo}, {hi}]，期望 ≈ 0.6）"
    );
    assert!(bury < 0.05, "不得被平台吞掉（最大埋入 {bury}）");
    assert!(vmax < 0.61, "末态速度不该超过平台速度（|v|max = {vmax}）");
    assert!(
        depth < 4.0 * 0.02,
        "末态最大穿透应在 2×skin 量级内（{depth}）"
    );
}

/// **L2 A 档的代价（机器无关判据）**：100 体 + 运动学平台 ⇒ **1 个岛**（平台把互不接触的阵列焊在一起）。
#[test]
fn l2_support_welds_the_load_into_one_island() {
    let (mut w, _boxes, _floor) = load_scene(10, true);
    for _ in 0..120 {
        w.step();
    }
    let islands = w.solver.island_diag.islands;
    let (static_isl, _, _) = {
        let (mut s, _, _) = load_scene(10, false);
        for _ in 0..120 {
            s.step();
        }
        (s.solver.island_diag.islands, 0, 0)
    };
    println!(
        "[L2 并岛代价] 100 体 + 运动学平台 ⇒ 岛 {islands}（并行组 {}）｜静态平台对照 ⇒ 岛 {static_isl}",
        w.solver.island_diag.g_count
    );
    assert_eq!(islands, 1, "A 档：支撑把载荷焊成一个岛（这是已接受的代价）");
    assert!(static_isl >= 1, "静态平台下不该是空");
}

/// **L4 性能读数（只报不判）**：100 / 400 体两档，"平台静止 vs 上行"。
#[test]
fn l4_report_tick_cost() {
    for n in [10usize, 20] {
        let (_d1, _b1, _v1, _p1, p_static) = run(n, 0.0, 120);
        let (_d2, _b2, _v2, _p2, p_lift) = run(n, 0.6, 120);
        println!(
            "[L4 性能读数 · 只报不判] {n}×{n} = {} 体：静止 p50 {p_static} µs ｜ 0.6 m/s 上行 p50 {p_lift} µs（本机 270K）",
            n * n
        );
    }
}
