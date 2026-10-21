//! **运动学支撑的唤醒判据**（`PLAN-kinematic.md` K3 一直没立的那条，2026-10-19 立）。
//!
//! K3 的唤醒是靠既有阈值规则达成的：`SUBISLAND_WAKE_MULT × sleep_linear = 2 × 0.04 = 0.08 m/s`。
//! 立这条判据时量出的**缺陷**：运动学体的速度是**宿主给的确定输入**，以 0.005–0.08 m/s 上行的
//! 平台在 10 s 内会从睡熟的载荷里**穿过 0.1–1.6 m** 而**一次都不唤醒**（flips = 0、无 NaN、无报错），
//! 慢速下「速度超阈」永远不成立 ⇒ 静默吞载荷。修法是 `sleep.rs` 的**运动学支撑深度门**：
//! 接触已经嵌进半个皮肤带（`depth > 0.5 × contact_skin`）就唤醒，不看速度大小。
//!
//! ⚠️ 这条门只对**运动学体**开：静态/动态支撑仍走原阈值（动态接触的抖动正需要那个滞回）。
//! 修后读数（本机 debug，10 s）：静置 0 翻转；8 cm/s 翻转 3 次、Δy = 0.80（平台 0.8）、埋入 0.004 m。
//! ⚠️ **已知边角**：0.005–0.02 m/s 的极慢档会「醒—睡—醒」（0.01 m/s 实测 0.65 次/秒/体，SPEC §3
//! 的抖动阈值是 < 1；0.02 m/s 实测 0.95，贴近阈值）—— 载荷速度低于 `sleep_linear` 本来就该睡，
//! 但它同时在被推动 ⇒ 这个边角是「慢到该睡、又在动」的固有矛盾，不是门的实现问题。

use vxl_phys::{BodyBuildExt, KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// 睡熟的三层堆 + 运动学平台；返回 `(睡→醒翻转次数, 末态醒着数, 底盒 Δy, 平台顶 − 底盒底)`。
/// `speed` = 平台上行速度（0 ⇒ 静置）；`cfg` 决定睡眠档与唤醒门（`wake_gate_k`）。
fn sleeping_stack_on_platform(speed: f32, ticks: u32, cfg: PhysConfig) -> (u32, usize, f32, f32) {
    let mut w = World::new(cfg);
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
    assert!(
        cubes.iter().all(|&i| !w.bodies.awake[i]),
        "前置条件：三层堆要先睡熟（awake = {:?}、|v| = {:?}）",
        cubes.iter().map(|&i| w.bodies.awake[i]).collect::<Vec<_>>(),
        cubes
            .iter()
            .map(|&i| w.bodies.linvel[i].length())
            .collect::<Vec<_>>()
    );
    let y0 = w.bodies.position[cubes[0]].y;
    if speed != 0.0 {
        w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, speed, 0.0), Vec3::ZERO);
    }
    let mut prev: Vec<bool> = cubes.iter().map(|&i| w.bodies.awake[i]).collect();
    let mut flips = 0u32;
    for _ in 0..ticks {
        w.step();
        for (k, &i) in cubes.iter().enumerate() {
            let a = w.bodies.awake[i];
            if !prev[k] && a {
                flips += 1;
            }
            prev[k] = a;
        }
    }
    let awake_end = cubes.iter().filter(|&&i| w.bodies.awake[i]).count();
    let dy = w.bodies.position[cubes[0]].y - y0;
    let bury = (w.bodies.position[floor].y + 0.5) - (w.bodies.position[cubes[0]].y - 0.5);
    (flips, awake_end, dy, bury)
}

/// **W1 静置支撑 × 睡熟的载荷：唤醒风暴 = 0**。平台不动再跑 10 s：0 翻转、保持全睡、不下沉。
#[test]
fn still_support_does_not_wake_sleeping_load() {
    let (flips, awake_end, _dy, bury) = sleeping_stack_on_platform(0.0, 600, PhysConfig::default());
    println!("[W1 静置支撑×睡堆] 10 s：翻转 {flips} 次、末态醒着 {awake_end}/3、埋入 {bury:.4} m");
    assert_eq!(flips, 0, "静置支撑不得唤醒睡熟的载荷（唤醒风暴 = 0）");
    assert_eq!(awake_end, 0, "静置 10 s 后应保持全睡");
    assert!(
        bury < 0.01,
        "静置时不得下沉（软重叠 ≈ 0.005），实得埋入 {bury}"
    );
}

/// **W2 慢速支撑（8 cm/s）不许穿过睡熟的载荷**（修前读了 flips = 0、穿过 1.6 m）。
#[test]
fn slow_support_does_not_tunnel_through_sleeping_load() {
    let (flips, _awake, dy, bury) = sleeping_stack_on_platform(0.08, 600, PhysConfig::default());
    println!(
        "[W2 慢速支撑×睡堆] 8 cm/s × 10 s：翻转 {flips} 次、Δy {dy:.4} m（平台 0.8）、埋入 {bury:.4} m"
    );
    assert!(flips >= 1, "慢速支撑必须唤醒载荷");
    assert!(dy > 0.7, "载荷应被带走（Δy ≈ 0.8），实得 {dy}");
    assert!(bury < 0.02, "残留埋入不得超皮肤带，实得 {bury}");
}

/// **W5 极慢自转（0.01 rad/s @ r = 2 m ⇒ 切向 0.02 m/s）也必须唤醒**。
///
/// 这条钉住判据的**口径**而不是某个数值：运动学速度是宿主给的确定输入，不是求解噪声。
/// 按 `2×sleep_linear = 0.08 m/s` 判"不算动"是错的 —— 0.02 m/s × 10 s = 0.2 m 真实弧长，
/// 载荷会被静默留在原地。滞回只该给动态接触的闪烁体用。
#[test]
fn very_slow_spinning_support_still_wakes() {
    let (flips, _awake_end, disp, radius, bury) =
        sleeping_cube_on_spinning_platform(0.01, 600, PhysConfig::default());
    println!(
        "[W5 自转 0.01 rad/s × 10 s] 翻转 {flips} 次、位移 {disp:.4} m（弧长 0.2）、半径 {radius:.4} m、埋入 {bury:.4} m"
    );
    assert!(
        flips >= 1,
        "极慢自转支撑也必须唤醒载荷（0.02 m/s 是真实位移，不是抖动）"
    );
    assert!(disp > 0.1, "载荷应被带着走（弧长 ≈ 0.2），实得 {disp}");
    assert!(
        (radius - 2.0).abs() < 0.5,
        "载荷不得滑离原半径（2.0），实得 {radius}"
    );
    assert!(bury < 0.02, "残留埋入不得超皮肤带，实得 {bury}");
    // 抖动预算：被运动学支撑带着走的载荷**不许**反复入睡/唤醒（SPEC §3 抖动 < 1 次/秒）。
    // 修前这条是 12 次 / 10 s（≈1.2 次/秒，已超阈值）；入睡否决后应为 1 次。
    assert!(
        flips <= 2,
        "被搬运的载荷不许反复醒睡（10 s 内翻转 {flips} 次，预算 ≤2）"
    );
}

/// 睡熟的单盒 + 运动学平台（**只有 1 条流形**）；返回 `(翻转次数, 末态醒着数, Δy, 埋入)`。
fn sleeping_cube_on_platform(speed: f32, ticks: u32, cfg: PhysConfig) -> (u32, usize, f32, f32) {
    let mut w = World::new(cfg);
    let floor = w.add_kinematic(
        Shape::Box {
            half: Vec3::new(4.0, 0.5, 4.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    ) as usize;
    let cube = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    ) as usize;
    for _ in 0..600 {
        w.step();
    }
    assert!(!w.bodies.awake[cube], "前置条件：单盒要先睡熟");
    let y0 = w.bodies.position[cube].y;
    if speed != 0.0 {
        w.set_kinematic_velocity(floor as u32, Vec3::new(0.0, speed, 0.0), Vec3::ZERO);
    }
    let mut prev = w.bodies.awake[cube];
    let mut flips = 0u32;
    for _ in 0..ticks {
        w.step();
        let a = w.bodies.awake[cube];
        if !prev && a {
            flips += 1;
        }
        prev = a;
    }
    let dy = w.bodies.position[cube].y - y0;
    let bury = (w.bodies.position[floor].y + 0.5) - (w.bodies.position[cube].y - 0.5);
    let awake_end = usize::from(w.bodies.awake[cube]);
    (flips, awake_end, dy, bury)
}

/// **W3 `wake_gate_k > 0` 时深度门不许被接触数门吃掉**。
///
/// `wake_gate_k` 的语义是「同一次 `solve_phase` 调用内累计 ≥K 条 hot 接触观测才唤醒」，
/// 它防的是「被 1–2 个抖动的清醒邻居碰」；但**运动学支撑的深度门是几何信号，不是抖动**：
/// 单个平台托一个载荷天然只有 1 条流形 ⇒ 无论 K 取多少，累计永远到不了 K。
/// 判据：`K = 8` 与 `K = 0` 下，8 cm/s 的支撑都必须把睡熟的载荷带走。
#[test]
fn deep_gate_survives_wake_gate_k() {
    let cfg = PhysConfig {
        wake_gate_k: 8,
        ..PhysConfig::default()
    };
    let (flips, _awake, dy, bury) = sleeping_cube_on_platform(0.08, 600, cfg);
    println!(
        "[W3 wake_gate_k=8 × 8 cm/s] 翻转 {flips} 次、Δy {dy:.4} m（平台 0.8）、埋入 {bury:.4} m"
    );
    assert!(
        flips >= 1,
        "深度门是几何信号，不得被接触数门（K=8）吃掉：翻转 {flips}"
    );
    assert!(dy > 0.7, "载荷应被带走（Δy ≈ 0.8），实得 {dy}");
    assert!(bury < 0.02, "残留埋入不得超皮肤带，实得 {bury}");
}

/// 绕 Y 轴自转的运动学平台 + 半径 2 m 处睡熟的单盒。
/// 返回 `(翻转次数, 末态醒着数, 位移模长, 末态半径, 埋入)`。
fn sleeping_cube_on_spinning_platform(
    omega: f32,
    ticks: u32,
    cfg: PhysConfig,
) -> (u32, usize, f32, f32, f32) {
    let mut w = World::new(cfg);
    let floor = w.add_kinematic(
        Shape::Box {
            half: Vec3::new(8.0, 0.5, 8.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    ) as usize;
    let cube = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(2.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    ) as usize;
    for _ in 0..600 {
        w.step();
    }
    assert!(!w.bodies.awake[cube], "前置条件：单盒要先睡熟");
    let p0 = w.bodies.position[cube];
    if omega != 0.0 {
        w.set_kinematic_velocity(floor as u32, Vec3::ZERO, Vec3::new(0.0, omega, 0.0));
    }
    let mut prev = w.bodies.awake[cube];
    let mut flips = 0u32;
    for _ in 0..ticks {
        w.step();
        let a = w.bodies.awake[cube];
        if !prev && a {
            flips += 1;
        }
        prev = a;
    }
    let p1 = w.bodies.position[cube];
    let disp = Vec3::new(p1.x - p0.x, 0.0, p1.z - p0.z).length();
    let radius = Vec3::new(p1.x, 0.0, p1.z).length();
    let bury = (w.bodies.position[floor].y + 0.5) - (p1.y - 0.5);
    let awake_end = usize::from(w.bodies.awake[cube]);
    (flips, awake_end, disp, radius, bury)
}

/// **W4 慢速自转支撑（0.05 rad/s，半径 2 m ⇒ 切向 0.1 m/s）也要把载荷带走**。
///
/// 唤醒判据原来只看**质心**速度/角速度：角速度 0.05 < `2×sleep_angular = 0.1` ⇒ 不 hot；
/// 而面-面接触的深度不随切向滑动增长 ⇒ 深度门也不响。结果是平台在载荷底下转、载荷纹丝不动。
#[test]
fn slow_spinning_support_carries_the_load() {
    let (flips, awake_end, disp, radius, bury) =
        sleeping_cube_on_spinning_platform(0.05, 600, PhysConfig::default());
    println!(
        "[W4 自转 0.05 rad/s × 10 s] 翻转 {flips} 次、末态醒着 {awake_end}、位移 {disp:.4} m（弧长 1.0）、半径 {radius:.4} m、埋入 {bury:.4} m"
    );
    assert!(flips >= 1, "慢速自转支撑必须唤醒睡熟的载荷");
    assert!(disp > 0.6, "载荷应被摩擦带着走（弧长 ≈ 1.0），实得 {disp}");
    assert!(
        (radius - 2.0).abs() < 0.5,
        "载荷不得被甩离/滑离原半径（2.0），实得 {radius}"
    );
    assert!(bury < 0.02, "残留埋入不得超皮肤带，实得 {bury}");
}
