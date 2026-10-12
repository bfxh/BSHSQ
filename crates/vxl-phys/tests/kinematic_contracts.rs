//! **运动学体的契约判据**（M2 · `docs/PLAN-kinematic.md` 的 K1 + K2 验收）。
//!
//! 口径：`BodyType::Kinematic` = 位置由宿主驱动、质量无限（`inv_mass = local_inv_inertia = 0`）、
//! 不受力，**但速度是输入且进岛**（A 档并岛）⇒ 摩擦因此携带载荷。
//!
//! | 判据 | 钉住的行为 |
//! |---|---|
//! | C1 传送带 | 速度驱动的平台**带着载荷走**（稳态速度 = 带速） |
//! | C2 电梯（慢） | 载荷被托着走：相对位移 ≈ 0、速度继承 |
//! | C3 电梯（快，6 m/s） | 载荷**不被平台吞掉**（不埋进平台） |
//! | C4 电梯 × 载荷链 | 三层堆整体被带上（载荷链不散） |
//! | C5 瞬移 | 运动学体换位接得住；**静态体不可挪 = 文档化边界** |
//! | C6 睡堆 × 上行支撑 | 睡熟的堆被**唤醒并带走**（阈值唤醒，不是"有接触就唤醒"） |
//! | C7 岛划分 | **A 档的代价**：支撑并岛（静态支撑 7 岛 → 运动学/动态支撑 1 岛） |
//! | C8 下行分离 | 下行**慢于 g** ⇒ 载荷跟随；下行**快于 g** ⇒ 载荷离地（间隙 = `(v−½gt)t`） |
//!
//! ⚠️ **一处判据修正（2026-10-12，写 C8 时发现）**：`PLAN-kinematic.md` §5.1 原写"`a > μg` 时载荷滑移"
//! —— 那条对**竖向**电梯是错的：上行时载荷是被**法向力**推上去的，摩擦不是限制项（接触法向竖直、
//! 切向无需求）。真正会分离的是**下行比 g 更快**（自由落体追不上平台）。判据已按后者实现。
//!
//! 本文件的前身是 `kinematic_gap_probe.rs`（同一批场景的"现状快照"：那时它们用**静态体**当
//! 平台，钉的是三条断链的坏行为）。K1/K2 落地后改写为契约 —— 断链②的机制说明仍留在
//! `SURVEY-RIGID-HARD-CASES.md` §2。静态体那条路**保持不支持**（C5 的边界格把它钉住）。

use vxl_phys::{KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// 不睡（除 C6 外）：本文件测的是接触/宽相语义，不是睡没睡。
fn cfg() -> PhysConfig {
    PhysConfig {
        sleep_time: f32::INFINITY,
        ..PhysConfig::default()
    }
}

/// 顶面在 `y = 0` 的平板 + 其上静置的 1 m 盒。`kinematic` ⇒ 板是运动学体（可驱动）。
fn lift_scene(kinematic: bool) -> (World, usize, usize) {
    let mut w = World::new(cfg());
    let plate = Shape::Box {
        half: Vec3::new(8.0, 0.5, 8.0),
    };
    let at = Vec3::new(0.0, -0.5, 0.0);
    let floor = if kinematic {
        w.add_kinematic(plate, at, Quat::IDENTITY)
    } else {
        w.add_static(plate, at, Quat::IDENTITY)
    };
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

/// **C1 传送带**：速度驱动 0.6 m/s（μ=0.5 ⇒ `μg = 4.9 m/s²`）。
/// 正确行为：加速段 `t = v/μg ≈ 0.12 s`（走 0.037 m），其后匀速 ⇒ 1 s 走 **≈ 0.55 m**、末速 **0.6**。
#[test]
fn c1_belt_carries_the_load() {
    let (mut w, cube, floor) = lift_scene(true);
    for _ in 0..30 {
        w.step(); // 先坐稳
    }
    w.set_kinematic_velocity(floor as u32, Vec3::new(0.6, 0.0, 0.0), Vec3::ZERO);
    let x0 = w.bodies.position[cube].x;
    for _ in 0..60 {
        w.step();
    }
    let dx = w.bodies.position[cube].x - x0;
    let vx = w.bodies.linvel[cube].x;
    println!("[C1 传送带] 带速 0.6 m/s · μ=0.5 · 1 s ⇒ 盒 Δx = {dx:.4} m、vx = {vx:.4} m/s");
    assert!(dx > 0.45, "载荷应当被带走（Δx ≈ 0.55），实得 {dx}");
    assert!((vx - 0.6).abs() < 0.05, "稳态速度应等于带速，实得 {vx}");
}

/// **C2 电梯（慢）**：平台 0.6 m/s 上行 1 s ⇒ 载荷被托着走：相对位移 ≈ 0、速度继承。
#[test]
fn c2_slow_lift_carries_the_load() {
    let (mut w, cube, floor) = lift_scene(true);
    for _ in 0..30 {
        w.step();
    }
    let (y0, f0) = (w.bodies.position[cube].y, w.bodies.position[floor].y);
    w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, 0.6, 0.0), Vec3::ZERO);
    for _ in 0..60 {
        w.step();
    }
    let (dy, lift) = (
        w.bodies.position[cube].y - y0,
        w.bodies.position[floor].y - f0,
    );
    let vy = w.bodies.linvel[cube].y;
    println!(
        "[C2 电梯·慢] 平台升 {lift:.4} m ⇒ 盒 Δy = {dy:.4} m（相对 {:.4}）、vy = {vy:.4} m/s",
        dy - lift
    );
    assert!(
        (lift - 0.6).abs() < 0.01,
        "平台按给定速度走 0.6 m，实得 {lift}"
    );
    assert!((dy - lift).abs() < 0.01, "载荷应随平台走（相对位移 ≈ 0）");
    assert!((vy - 0.6).abs() < 0.02, "载荷速度应继承平台速度，实得 {vy}");
}

/// **C3 电梯（快，6 m/s）**：载荷**不被吞掉** —— 底盒底面不低于平台顶面（容差 5 cm）。
#[test]
fn c3_fast_lift_does_not_swallow_the_load() {
    let (mut w, cube, floor) = lift_scene(true);
    for _ in 0..30 {
        w.step();
    }
    w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, 6.0, 0.0), Vec3::ZERO);
    for _ in 0..60 {
        w.step();
    }
    let plate_top = w.bodies.position[floor].y + 0.5;
    let cube_bottom = w.bodies.position[cube].y - 0.5;
    let bury = plate_top - cube_bottom;
    println!("[C3 电梯·快] 平台顶面 {plate_top:.4}、载荷底面 {cube_bottom:.4} ⇒ 埋入 {bury:.4} m");
    assert!(bury < 0.05, "快速上行不得把载荷吞进平台，实得埋入 {bury}");
}

/// **C4 电梯 × 载荷链**：三层堆站在运动学平台上，1 s 上行 0.6 m ⇒ 三层整体被带走。
#[test]
fn c4_lift_carries_a_stack() {
    let (mut w, _cube, floor) = lift_scene(true);
    let upper: Vec<usize> = (1..3)
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
    for _ in 0..60 {
        w.step();
    }
    let y0: Vec<f32> = upper.iter().map(|&i| w.bodies.position[i].y).collect();
    w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, 0.6, 0.0), Vec3::ZERO);
    for _ in 0..60 {
        w.step();
    }
    let dys: Vec<f32> = upper
        .iter()
        .zip(&y0)
        .map(|(&i, &y)| w.bodies.position[i].y - y)
        .collect();
    println!("[C4 载荷链] 顶层两块的 Δy = {dys:?}（期望 ≈ 0.6）");
    for (k, dy) in dys.iter().enumerate() {
        assert!(
            (dy - 0.6).abs() < 0.02,
            "第 {} 层未随平台走：Δy = {dy}",
            k + 1
        );
    }
}

/// **C5 瞬移**：地板先在 x=−20，第 20 tick 换到 x=0（自由落体正好落到它上方）。
/// - **运动学体**：接得住（下一 tick 宽相就在新位置看见它）；
/// - **静态体**：**不支持**（树代理盒永不移动）—— 这条边界是**故意**钉住的，不是待修项。
#[test]
fn c5_teleport_works_for_kinematic_only() {
    let run = |kinematic: bool| {
        let cfg = PhysConfig {
            sleep_time: f32::INFINITY,
            gravity: Vec3::ZERO, // 零重力：只比"换位之后接不接得住"
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        let shape = Shape::Box {
            half: Vec3::new(4.0, 0.1, 4.0),
        };
        let at = Vec3::new(-20.0, 0.1, 0.0);
        let floor = if kinematic {
            w.add_kinematic(shape, at, Quat::IDENTITY)
        } else {
            w.add_static(shape, at, Quat::IDENTITY)
        } as usize;
        let cube = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            Quat::IDENTITY,
            500.0,
        ) as usize;
        w.bodies.linvel[cube] = Vec3::new(0.0, -3.0, 0.0); // 不隧穿的下落速度
        for t in 0..150 {
            if t == 20 {
                // 换位：运动学体写位置 + 清速度（它的 AABB/代理跟着刷新）
                w.bodies.position[floor] = Vec3::new(0.0, 0.1, 0.0);
                if kinematic {
                    w.set_kinematic_velocity(floor as u32, Vec3::ZERO, Vec3::ZERO);
                }
            }
            w.step();
        }
        w.bodies.position[cube].y
    };
    let y_kin = run(true);
    let y_static = run(false);
    println!(
        "[C5 瞬移] 运动学地板 y = {y_kin:.4}（被拦住）｜静态地板 y = {y_static:.4}（边界：接不住）"
    );
    // ⚠️ 本场景**零重力**（只为隔离"换位后宽相看不看得见"）：落到板面后不再有重力把它压住，
    // 去穿透偏置可能把它抬出十几厘米 ⇒ 判据是"**被拦住**"（没穿过去），不是"静止在 0.7"。
    assert!(
        (0.6..1.5).contains(&y_kin),
        "运动学体换位后应拦住落体（0.6~1.5 m），实得 {y_kin}"
    );
    assert!(
        y_static < -1.0,
        "静态体换位**不支持**（这是文档化边界，不是回归）：实得 {y_static}"
    );
}

/// **C6 睡堆 × 上行支撑**（默认睡眠档）：三层堆先睡熟，运动学平台再以 1.2 m/s 上行 1 s。
/// 正确行为：全岛被**唤醒**（相对速度超阈 ⇒ 阈值唤醒，不是"有接触就唤醒"）并被带走。
#[test]
fn c6_sleeping_stack_is_woken_and_carried() {
    let mut w = World::new(PhysConfig::default()); // 睡眠按默认：开
    let floor = w.add_kinematic(
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
    let y0 = w.bodies.position[cubes[0]].y;
    w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    for _ in 0..60 {
        w.step();
    }
    let dy = w.bodies.position[cubes[0]].y - y0;
    let bury = (w.bodies.position[floor].y + 0.5) - (w.bodies.position[cubes[0]].y - 0.5);
    println!("[C6 睡堆×电梯] 驱动前醒着 {awake_before}/3 ⇒ 底盒 Δy = {dy:.4} m、埋入 {bury:.4} m");
    assert_eq!(awake_before, 0, "三层堆应当先睡熟（前置条件）");
    assert!(dy > 0.5, "睡熟的堆应被唤醒并带走（Δy ≈ 1.2），实得 {dy}");
    assert!(bury < 0.05, "载荷不得被吞进平台，实得埋入 {bury}");
}

/// **C7 岛划分 = A 档的代价**（`PLAN-kinematic.md` §2）：7 根独立两层柱，只换支撑。
/// 静态支撑 **7 个岛**（静态不进岛）；运动学/动态支撑 **1 个岛**（平台把它们焊在一起）。
/// ⚠️ 这是**已知并接受**的代价（本仓并行是岛级）：它被钉成判据，谁改动并岛规则都会在这里显形。
#[test]
fn c7_support_type_decides_island_count() {
    let run = |support: u8| {
        let mut w = World::new(cfg());
        let plate = Shape::Box {
            half: Vec3::new(8.0, 0.1, 2.0),
        };
        match support {
            0 => {
                w.add_static(plate, Vec3::new(0.0, -0.9, 0.0), Quat::IDENTITY);
            }
            1 => {
                w.add_kinematic(plate, Vec3::new(0.0, -0.9, 0.0), Quat::IDENTITY);
            }
            _ => {
                w.add_static(
                    Shape::Box {
                        half: Vec3::new(9.0, 0.5, 2.0),
                    },
                    Vec3::new(0.0, -1.5, 0.0),
                    Quat::IDENTITY,
                );
                w.add_dynamic(plate, Vec3::new(0.0, -0.9, 0.0), Quat::IDENTITY, 5000.0);
            }
        }
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
        w.solver.island_diag.islands
    };
    let (isl_static, isl_kin, isl_dyn) = (run(0), run(1), run(2));
    println!(
        "[C7 岛划分] 静态支撑 {isl_static} 岛｜运动学支撑 {isl_kin} 岛｜动态支撑 {isl_dyn} 岛"
    );
    assert_eq!(isl_static, 7, "静态支撑下 7 根柱各成一岛（静态不进岛）");
    assert_eq!(
        isl_kin, 1,
        "A 档：运动学支撑把 7 根柱焊成 1 个岛（已知代价）"
    );
    assert_eq!(isl_dyn, 1, "对照：动态支撑同样是 1 个岛");
}

/// **C8 下行分离**：慢速下行（0.6 m/s）载荷跟随；快速下行（12 m/s > g·t 的自由落体）载荷**离地**。
#[test]
fn c8_load_follows_slow_descent_but_leaves_fast_one() {
    let run = |v_down: f32| {
        let (mut w, cube, floor) = lift_scene(true);
        for _ in 0..30 {
            w.step();
        }
        w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, -v_down, 0.0), Vec3::ZERO);
        for _ in 0..30 {
            w.step();
        }
        // 间隙 = 平台顶面 − 载荷底面：≈0 = 贴住；**负 = 平台跑到载荷下方（离地）**。
        (w.bodies.position[floor].y + 0.5) - (w.bodies.position[cube].y - 0.5)
    };
    let gap_slow = run(0.6);
    let gap_fast = run(12.0);
    println!(
        "[C8 下行分离] 慢（0.6 m/s）间隙 {gap_slow:.4} m（贴住）｜快（12 m/s）间隙 {gap_fast:.4} m（离地；解析 ≈ −4.77）"
    );
    assert!(
        gap_slow.abs() < 0.05,
        "慢速下行时载荷应贴住平台，实得间隙 {gap_slow}"
    );
    assert!(
        gap_fast < -2.0,
        "下行快于自由落体时载荷应离地（解析 ≈ −4.77 m/0.5 s），实得 {gap_fast}"
    );
}
