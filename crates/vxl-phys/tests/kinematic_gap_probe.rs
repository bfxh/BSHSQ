//! **运动学缺口的现状快照**（取证文档 `docs/SURVEY-RIGID-HARD-CASES.md` §2）。
//!
//! 问的是同一条假设（"位置不变的东西就是静态的"）的四个侧面：**把"静态体被宿主搬动"当
//! 电梯 / 传送带 / 可瞬移地形用，现在会发生什么**（五条读数）。
//! `SPEC.md` §12.2 把"运动学体"记在 M2 ⇒ 这些**都不是**"已经支持"，
//! 本文件是把"不支持的具体形态"钉成可复现读数，供 M2 落地时**红转绿**。
//!
//! 口径（与 `tests/ccd_tunneling.rs` 的"默认档 = 文档化边界"同款）：
//! - 睡眠一律关（`sleep_time = INFINITY`）—— 测的是接触/宽相语义，不是睡没睡；
//! - 每个场景先坐稳 30~40 tick 再施加驱动；
//! - 断言钉的是**现状**，并在注释里写清"正确行为应该是什么"（运动学体落地后必须改这里）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 关睡眠：本文只测"接触里看不看得见驱动速度"与"宽相更不更新"。
fn cfg() -> PhysConfig {
    PhysConfig {
        sleep_time: f32::INFINITY,
        ..PhysConfig::default()
    }
}

/// 地板（静态薄板，顶面 y=0）+ 其上静置的 1 m 盒。
fn resting_scene() -> (World, usize, usize) {
    let mut w = World::new(cfg());
    let floor = w.add_static(
        Shape::Box {
            half: Vec3::new(8.0, 0.5, 8.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let cube = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    );
    (w, cube as usize, floor as usize)
}

/// **① 传送带**：平台沿 x 以 0.6 m/s 平移 1 s。μ=0.5 ⇒ 正确行为是**被带走 ≈ 0.51 m**
/// （加速段 0.6/μg ≈ 0.12 s），且稳态 `vx ≈ 0.6`。
#[test]
fn probe_belt_does_not_carry() {
    let (mut w, cube, floor) = resting_scene();
    for _ in 0..30 {
        w.step();
    }
    let x0 = w.bodies.position[cube].x;
    for _ in 0..60 {
        w.bodies.position[floor].x += 0.01;
        w.step();
    }
    let dx = w.bodies.position[cube].x - x0;
    let vx = w.bodies.linvel[cube].x;
    println!(
        "[探针①传送带] 带速 0.6 m/s · μ=0.5 · 1 s ⇒ 盒 Δx = {dx:.4} m、vx = {vx:.4} m/s、接触数 {}",
        w.manifolds().len()
    );
    // 正确行为：Δx ≈ 0.51 m、vx ≈ 0.6 m/s（静摩擦把载荷带走）。现状是"纹丝不动"。
    assert!(dx.abs() < 0.05, "现状：传送带不携带（Δx = {dx}）");
    assert!(vx.abs() < 0.05, "现状：载荷速度不跟随带速（vx = {vx}）");
}

/// **② 电梯（慢速上行）**：平台以 0.6 m/s 上行 1 s（升 0.6 m）。
/// 正确行为：载荷被"托着"走，相对位移 ≈ 0、`vy ≈ 0.6`。
#[test]
fn probe_elevator_slow_lift() {
    let (mut w, cube, floor) = resting_scene();
    for _ in 0..30 {
        w.step();
    }
    let (y0, f0) = (w.bodies.position[cube].y, w.bodies.position[floor].y);
    for _ in 0..60 {
        w.bodies.position[floor].y += 0.01;
        w.step();
    }
    let dy = w.bodies.position[cube].y - y0;
    let lift = w.bodies.position[floor].y - f0;
    let vy = w.bodies.linvel[cube].y;
    println!(
        "[探针②电梯·慢] 平台升 {lift:.4} m ⇒ 盒 Δy = {dy:.4} m（相对 {:.4}）、vy = {vy:.4} m/s、接触数 {}",
        dy - lift,
        w.manifolds().len()
    );
    // 正确行为：相对位移 ≈ 0、vy ≈ 0.6。现状：载荷落后平台、最后在**自由下落**（vy < 0）。
    assert!(
        dy - lift < -0.02,
        "现状：载荷跟不上平台（相对 {:.4} m）",
        dy - lift
    );
    assert!(vy < 0.3, "现状：载荷没有继承平台速度（vy = {vy}）");
}

/// **③ 电梯（快速上行，6 m/s）**：平台每 tick 升 0.1 m。位置修正的上限是
/// `max_corrective_velocity = 3 m/s` ⇒ 超过它就别指望"去穿透"能把载荷顶上去。
#[test]
fn probe_elevator_fast_swallows() {
    let (mut w, cube, floor) = resting_scene();
    for _ in 0..30 {
        w.step();
    }
    let f0 = w.bodies.position[floor].y;
    for _ in 0..60 {
        w.bodies.position[floor].y += 0.1;
        w.step();
    }
    let dy = w.bodies.position[cube].y - 0.5;
    let lift = w.bodies.position[floor].y - f0;
    // 平台顶面 = 平台中心 + 0.5；载荷底面 = 载荷中心 − 0.5 ⇒ 负数 = 载荷已经**埋进平台里**
    let bury = (w.bodies.position[floor].y + 0.5) - (w.bodies.position[cube].y - 0.5);
    println!(
        "[探针③电梯·快] 平台升 {lift:.4} m ⇒ 盒 Δy = {dy:.4} m、埋入平台 {bury:.4} m、接触数 {}",
        w.manifolds().len()
    );
    // 正确行为：载荷被顶上去（埋入 ≈ −0.5 ⇒ 载荷底面高于平台顶面）。现状：**整个穿过**。
    assert!(bury > 0.5, "现状：快速平台把载荷穿过去了（埋入 {bury} m）");
}

/// **④ 瞬移地板（宽相陈旧代理）**：地板先在 x=−20，第 40 tick 瞬移到 x=0（正好落到自由
/// 下落的盒下方）。对照组从一开始就在 x=0 ⇒ 盒应落在 y≈0.7。实验组若**漏对** ⇒ 盒穿过去。
///
/// 第三档是**否证**：瞬移之后再"添一个远处的静态体"（体数变化 ⇒ 宽相走全量 AABB 重算分支）
/// —— 实测**救不回来** ⇒ 锅不在"AABB 数组没刷新"，而在 `update_proxies` **从不移动静态体的
/// 树代理盒**（只对 `is_dynamic && awake` 调 `move_proxy_scaled`）。机制对照见下一个测试。
#[test]
fn probe_teleport_needs_full_rebuild() {
    let run = |teleport: bool, rebuild: bool| {
        let start_x = if teleport { -20.0 } else { 0.0 };
        let mut w = World::new(cfg());
        let floor = w.add_static(
            Shape::Box {
                half: Vec3::new(4.0, 0.1, 4.0),
            },
            Vec3::new(start_x, 0.1, 0.0),
            Quat::IDENTITY,
        );
        let cube = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            500.0,
        ) as usize;
        for t in 0..120 {
            if teleport && t == 40 {
                w.bodies.position[floor as usize].x = 0.0; // 瞬移：一次到脚下
            }
            if rebuild && t == 41 {
                // 体数变化 ⇒ `leaves_missing` ⇒ 宽相全量重算 AABB（远处，不参与接触）。
                w.add_static(
                    Shape::Box {
                        half: Vec3::splat(0.1),
                    },
                    Vec3::new(200.0, 200.0, 200.0),
                    Quat::IDENTITY,
                );
            }
            w.step();
        }
        (w.bodies.position[cube].y, w.manifolds().len())
    };
    let (y_ctrl, m_ctrl) = run(false, false);
    let (y_tele, m_tele) = run(true, false);
    let (y_fix, m_fix) = run(true, true);
    println!(
        "[探针④瞬移] 对照 y = {y_ctrl:.4}（接触 {m_ctrl}）｜瞬移 y = {y_tele:.4}（接触 {m_tele}）\
         ｜瞬移+触发重建 y = {y_fix:.4}（接触 {m_fix}）",
    );
    assert!(
        (y_ctrl - 0.7).abs() < 0.05,
        "对照组必须稳稳落在地板上（y≈0.7），实得 {y_ctrl}"
    );
    assert!(
        y_tele < -1.0,
        "现状：瞬移后的地板接不住东西（盒穿到 y = {y_tele}）"
    );
    assert!(
        y_fix < -1.0,
        "否证：全量 AABB 重算（体数变化）也救不回来（y = {y_fix}）⇒ 修点在树代理盒"
    );
}

/// **⑤ 机制对照：树代理盒只跟着"动态清醒体"走**。同一条瞬移（地板先在 x=−20，第 20 tick
/// 瞬移到 x=0），只换地板是**静态**还是**动态**（零重力 ⇒ 只比"代理盒挪没挪"）：
/// - 动态：`update_proxies` 会 `move_proxy_scaled` ⇒ 接得住；
/// - 静态：永不移动代理 ⇒ 穿过去。
///
/// 这一档把"断链②"的原因从【推断】钉成【实测】。
#[test]
fn probe_teleport_moves_dynamic_proxy_only() {
    let run = |dynamic_floor: bool| {
        let cfg = PhysConfig {
            sleep_time: f32::INFINITY,
            gravity: Vec3::ZERO, // 零重力：只比"代理盒挪没挪"，不比"谁托得住"
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        let shape = Shape::Box {
            half: Vec3::new(4.0, 0.1, 4.0),
        };
        let start = Vec3::new(-20.0, 0.1, 0.0);
        let floor = if dynamic_floor {
            w.add_dynamic(shape, start, Quat::IDENTITY, 1.0e6) // 质量巨大 ⇒ 被撞也不动
        } else {
            w.add_static(shape, start, Quat::IDENTITY)
        };
        let cube = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            500.0,
        ) as usize;
        w.bodies.linvel[cube] = Vec3::new(0.0, -3.0, 0.0); // 3 m/s 下落（0.05 m/tick，不隧穿）
        for t in 0..150 {
            if t == 20 {
                w.bodies.position[floor as usize].x = 0.0;
            }
            w.step();
        }
        (w.bodies.position[cube].y, w.manifolds().len())
    };
    let (y_dyn, m_dyn) = run(true);
    let (y_static, m_static) = run(false);
    println!(
        "[探针⑤代理机制] 动态地板 y = {y_dyn:.4}（接触 {m_dyn}）｜静态地板 y = {y_static:.4}（接触 {m_static}）"
    );
    assert!(
        (y_dyn - 0.7).abs() < 0.05,
        "动态地板瞬移后应被接住（y≈0.7），实得 {y_dyn}"
    );
    assert!(y_static < -1.0, "静态地板瞬移后接不住（y = {y_static}）");
}

/// **⑥ 极端工况：睡熟的堆 × 上行支撑**（用户点名的"电梯上行还要被大量的东西产生反应"）。
/// 三层堆先按默认档睡熟（`sleep_time = 0.5 s`），平台再以 1.2 m/s 上行 1 s。
/// 关键：**睡眠体既不建岛也不查询**（`bvh_phase` 的"睡眠体不查询"、`solver_impl` 的
/// "睡眠体不建岛 / 求解期按静态处理"），而平台是静态体、代理盒也不会移动 ⇒ 预期
/// **一次都不唤醒、位置一动不动**，平台直接从堆里穿过去。
#[test]
fn probe_moving_support_under_sleeping_stack() {
    let mut w = World::new(PhysConfig::default()); // 睡眠按默认：开
    let floor = w.add_static(
        Shape::Box {
            half: Vec3::new(4.0, 0.5, 4.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    ) as usize;
    let cubes: Vec<usize> = (0..3)
        .map(|k| {
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(0.0, 0.5 + k as f32, 0.0),
                Quat::IDENTITY,
                500.0,
            ) as usize
        })
        .collect();
    for _ in 0..600 {
        w.step();
    }
    let awake_before = cubes.iter().filter(|&&i| w.bodies.awake[i]).count();
    let y_bottom_before = w.bodies.position[cubes[0]].y;
    let mut awake_ticks = 0usize;
    for _ in 0..60 {
        w.bodies.position[floor].y += 0.02; // 1.2 m/s 上行
        w.step();
        awake_ticks += cubes.iter().filter(|&&i| w.bodies.awake[i]).count();
    }
    let dy = w.bodies.position[cubes[0]].y - y_bottom_before;
    let plate_top = w.bodies.position[floor].y + 0.5;
    let bottom_center = w.bodies.position[cubes[0]].y;
    println!(
        "[探针⑥睡堆×电梯] 驱动前醒着 {awake_before}/3、平台顶面 {plate_top:.4}、底盒中心 {bottom_center:.4}\
         （Δy = {dy:.4}）、驱动期 awake 累计 {awake_ticks} tick·体、接触数 {}",
        w.manifolds().len()
    );
    assert_eq!(awake_before, 0, "三层堆应当先按默认档睡熟");
    assert_eq!(awake_ticks, 0, "现状：移动支撑对睡熟的堆零唤醒");
    assert!(dy.abs() < 1e-4, "现状：睡熟的堆位置一动不动（Δy = {dy}）");
}

/// **⑦ 极端工况：动态支撑会把"互相独立的塔"焊成一个巨型岛**。
/// 同一几何（7 根独立的两层柱，**柱间留 1 m 空隙**——挨着放会因侧面接触自己并成 1 个岛），
/// 只换支撑：静态板 vs 动态板（动态板架在静态地板上）。
/// 岛由**清醒动体**经接触并查集合并（`build_union_find` 只并 `dynamic × dynamic`，
/// 静态侧不并）⇒ 预期 7 vs **1**。
/// 这是"电梯 + 大量物体"的性能风险来源：支撑一旦成为岛内成员，全部塔共用一个岛，
/// 岛的并行度（`g_count`）就塌了（本仓并行是**岛级**）。
#[test]
fn probe_dynamic_support_welds_islands() {
    let run = |dynamic_plate: bool| {
        let cfg = PhysConfig {
            sleep_time: f32::INFINITY, // 不睡：睡眠体不建岛，会污染计数
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        let plate_half = Vec3::new(8.0, 0.1, 2.0);
        let plate = if dynamic_plate {
            w.add_static(
                Shape::Box {
                    half: Vec3::new(9.0, 0.5, 2.0),
                },
                Vec3::new(0.0, -1.5, 0.0), // 地板顶面 y = −1.0
                Quat::IDENTITY,
            );
            w.add_dynamic(
                Shape::Box { half: plate_half },
                Vec3::new(0.0, -0.9, 0.0),
                Quat::IDENTITY,
                5000.0,
            )
        } else {
            w.add_static(
                Shape::Box { half: plate_half },
                Vec3::new(0.0, -0.9, 0.0),
                Quat::IDENTITY,
            )
        };
        for col in [-6.0f32, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0] {
            for k in 0..2 {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.5),
                    },
                    Vec3::new(col, -0.3 + k as f32, 0.0),
                    Quat::IDENTITY,
                    500.0,
                );
            }
        }
        for _ in 0..180 {
            w.step();
        }
        (
            w.solver.island_diag.islands,
            w.solver.island_diag.g_count,
            w.bodies.position[plate as usize].y,
        )
    };
    let (isl_static, g_static, y_static) = run(false);
    let (isl_dyn, g_dyn, y_dyn) = run(true);
    println!(
        "[探针⑦支撑合并岛] 静态支撑：岛 {isl_static}、并行组 {g_static}、板 y = {y_static:.4}｜\
         动态支撑：岛 {isl_dyn}、并行组 {g_dyn}、板 y = {y_dyn:.4}"
    );
    assert_eq!(
        isl_static, 7,
        "静态支撑下应是 7 个独立柱岛，实得 {isl_static}"
    );
    assert_eq!(isl_dyn, 1, "动态支撑把 7 根柱焊成 1 个岛，实得 {isl_dyn}");
}
