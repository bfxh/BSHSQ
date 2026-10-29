//! **睡眠合同：运动学支撑上的载荷必须被叫醒并被带走**（2026-10-28 立）。
//!
//! 为什么需要这道门：`sleep_wake_contract.rs` 只覆盖**高速撞击**（相对速度 ≥ 8×睡眠阈
//! ⇒ 强撞直通）。另一支没有门看着——**已睡着的载荷放在缓慢运动的运动学平台上**：
//! - 相对速度 0.1–0.3 m/s **低于**强撞阀 `WAKE_GATE_FAST_MULT × sleep_linear = 0.32 m/s`；
//! - 一块平台托一个箱只有 **1 条流形** ⇒ 接触数门 `wake_gate_k` 的计数**永远到不了 K**。
//!
//! 探针实测（`wake_gate_k=8`）：载荷 120 tick 内 `awake` 恒为 false、Δx = 0.0000 m，
//! 而平台走了 0.6000 m——即"睡眠 = 隐形传送带"，平台从载荷底下滑走。这正是
//! `wake_signal.rs` 里 `carried_by_moving_kinematic` 那段注释描述的形状，但那个否决
//! 只对**已醒**的体生效（`sleep.rs` 的入睡循环跳过睡着的体）。
//!
//! 三条判据（`wake_gate_k` 取 0 与 8 都必须过）：① 平移平台上醒且跟着走；② **自转**平台
//! （W4 口径：质心速度低于阈、接触点切向速度不低）上醒且按圆弧走；③ **叠箱**逐层传播唤醒，
//! 不发生"底层走了、上层还睡"的解体。

use vxl_phys::{BodyBuildExt, KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

fn cfg(wake_gate_k: u32) -> PhysConfig {
    PhysConfig {
        velocity_iterations: 16,
        substeps: 4,
        threads: 1,
        wake_gate_k,
        ..PhysConfig::default()
    }
}

fn kinematic_platform(w: &mut World) -> usize {
    // 半高 0.1、中心 -0.1 ⇒ 顶面 y = 0；边长 8 m，载荷不会滑出边界。
    w.add_kinematic(
        Shape::Box {
            half: Vec3::new(4.0, 0.1, 4.0),
        },
        Vec3::new(0.0, -0.1, 0.0),
        Quat::IDENTITY,
    ) as usize
}

/// 让这些体自己睡着；返回是否在 600 tick 内全睡。
fn settle(w: &mut World, ids: &[usize]) -> bool {
    for _ in 0..600u32 {
        w.step();
        if ids.iter().all(|&i| !w.bodies.awake[i]) {
            return true;
        }
    }
    ids.iter().all(|&i| !w.bodies.awake[i])
}

fn run_linear(wake_gate_k: u32, speed: f32) {
    let mut w = World::new(cfg(wake_gate_k));
    let plat = kinematic_platform(&mut w);
    let cargo = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 0.25, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    if !settle(&mut w, &[cargo]) {
        // 场景不适用要**明说**，不能假绿（同 `sleep_wake_contract.rs` 的口径）。
        println!("wake_gate_k={wake_gate_k}: ⚠️ 载荷 600 tick 内未入睡 ⇒ 本场景不适用（不是通过）");
        return;
    }
    let x0 = w.bodies.position[cargo].x;
    let p0 = w.bodies.position[plat].x;
    w.set_kinematic_velocity(plat as u32, Vec3::new(speed, 0.0, 0.0), Vec3::ZERO);
    let mut woke_at = None;
    for t in 1..=120u32 {
        w.step();
        if woke_at.is_none() && w.bodies.awake[cargo] {
            woke_at = Some(t);
        }
    }
    let dx = w.bodies.position[cargo].x - x0;
    let dplat = w.bodies.position[plat].x - p0;
    println!(
        "wake_gate_k={wake_gate_k} speed={speed}: 唤醒 tick {woke_at:?} | 载荷 Δx {dx:+.4} m | 平台 Δx {dplat:+.4} m"
    );
    assert!(
        woke_at.is_some(),
        "运动学平台上的载荷**没被叫醒**（睡体 = 隐形传送带）：wake_gate_k={wake_gate_k}、speed={speed}"
    );
    let tol = 0.02 + 0.05 * dplat.abs();
    assert!(
        (dx - dplat).abs() <= tol,
        "载荷醒了但没跟着走：Δx {dx:+.4} vs 平台 {dplat:+.4}（容差 {tol:.4}）——wake_gate_k={wake_gate_k}、speed={speed}"
    );
}

/// ① 接触数门开着或关着，慢速平移都得把睡着的载荷带走。
#[test]
fn sleeping_cargo_is_carried_by_slow_kinematic_platform() {
    for k in [0u32, 8] {
        for speed in [0.10f32, 0.30] {
            run_linear(k, speed);
        }
    }
}

/// ② 自转支撑：0.05 rad/s、半径 2 m ⇒ 质心线速 0.1 m/s（低于动态侧 2×阈 0.08 的接触点口径
///    只有 0.1，质心口径相同）——但**接触点**切向速度非零，W4 要求它算 hot。
#[test]
fn sleeping_cargo_is_carried_by_rotating_kinematic_platform() {
    let mut w = World::new(cfg(8));
    let plat = kinematic_platform(&mut w);
    let cargo = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(2.0, 0.2, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    assert!(
        settle(&mut w, &[cargo]),
        "载荷 600 tick 内未入睡 ⇒ 场景不适用"
    );
    let p0 = w.bodies.position[cargo];
    w.set_kinematic_velocity(plat as u32, Vec3::ZERO, Vec3::new(0.0, 0.05, 0.0));
    let mut woke_at = None;
    let omega_dt = 0.05 * w.config.dt;
    for t in 1..=240u32 {
        w.step();
        if woke_at.is_none() && w.bodies.awake[cargo] {
            woke_at = Some(t);
        }
    }
    assert!(woke_at.is_some(), "自转平台上的载荷没被叫醒");
    // 解析：起点 (2,0,0) 绕 +Y 转 ω·T 后 p = (r·cosθ, ·, -r·sinθ) ⇒ 位移 = p - p0
    let theta = omega_dt * 240.0;
    let want = Vec3::new(2.0 * theta.cos() - 2.0, 0.0, -2.0 * theta.sin());
    let d = w.bodies.position[cargo] - p0;
    println!(
        "[rot] 唤醒 {woke_at:?} | Δ=({:+.3},{:+.3}) 解析=({:+.3},{:+.3})",
        d.x, d.z, want.x, want.z
    );
    assert!(
        (d.x - want.x).abs() < 0.05 && (d.z - want.z).abs() < 0.05,
        "自转平台上没按圆弧走：Δ=({:+.3},{:+.3}) vs 解析 ({:+.3},{:+.3})",
        d.x,
        d.z,
        want.x,
        want.z
    );
}

/// ③ 叠箱：只有底层直接接触平台，上层只能靠"底层醒了并在动"传播唤醒。
///    判据 = 三层都醒、高度不变（不解体）、滞后在容差内（实测 1/5/9 tick、Δx 差 ≤ 0.04 m）。
#[test]
fn sleeping_stack_on_kinematic_platform_wakes_level_by_level() {
    let mut w = World::new(cfg(8));
    let plat = kinematic_platform(&mut w);
    let mut ids = Vec::new();
    for i in 0..3 {
        ids.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.25),
            },
            Vec3::new(0.0, 0.25 + 0.5 * i as f32, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize);
    }
    assert!(settle(&mut w, &ids), "叠箱 600 tick 内未入睡 ⇒ 场景不适用");
    let x0: Vec<f32> = ids.iter().map(|&i| w.bodies.position[i].x).collect();
    w.set_kinematic_velocity(plat as u32, Vec3::new(0.3, 0.0, 0.0), Vec3::ZERO);
    let mut woke = vec![None; ids.len()];
    for t in 1..=120u32 {
        w.step();
        for (n, &i) in ids.iter().enumerate() {
            if woke[n].is_none() && w.bodies.awake[i] {
                woke[n] = Some(t);
            }
        }
    }
    for (n, &i) in ids.iter().enumerate() {
        let p = w.bodies.position[i];
        let dx = p.x - x0[n];
        println!(
            "[stack] box{n} 唤醒 {:?} | Δx {dx:+.4} | y {:+.3}",
            woke[n], p.y
        );
        assert!(woke[n].is_some(), "叠箱第 {n} 层没被唤醒");
        assert!(
            (p.y - (0.25 + 0.5 * n as f32)).abs() < 0.05,
            "叠箱第 {n} 层高度变了（解体）：y={:+.3}",
            p.y
        );
        assert!(
            (dx - 0.5899).abs() < 0.08,
            "叠箱第 {n} 层没跟着平台走：Δx={dx:+.4}（平台 0.6000）"
        );
    }
}
