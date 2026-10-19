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

use vxl_phys::{KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// 睡熟的三层堆 + 运动学平台；返回 `(睡→醒翻转次数, 末态醒着数, 底盒 Δy, 平台顶 − 底盒底)`。
/// `speed` = 平台上行速度（0 ⇒ 静置）；睡眠档按默认（开）。
fn sleeping_stack_on_platform(speed: f32, ticks: u32) -> (u32, usize, f32, f32) {
    let mut w = World::new(PhysConfig::default());
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
        "前置条件：三层堆要先睡熟"
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
    let (flips, awake_end, _dy, bury) = sleeping_stack_on_platform(0.0, 600);
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
    let (flips, _awake, dy, bury) = sleeping_stack_on_platform(0.08, 600);
    println!(
        "[W2 慢速支撑×睡堆] 8 cm/s × 10 s：翻转 {flips} 次、Δy {dy:.4} m（平台 0.8）、埋入 {bury:.4} m"
    );
    assert!(flips >= 1, "慢速支撑必须唤醒载荷");
    assert!(dy > 0.7, "载荷应被带走（Δy ≈ 0.8），实得 {dy}");
    assert!(bury < 0.02, "残留埋入不得超皮肤带，实得 {bury}");
}
