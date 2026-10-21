//! **S2（快转目标子步化）的代价台架**（`ccd_target_substep`，2026-10-21 随"提为默认档"同片落地）。
//!
//! 跑法：`cargo run --release -q -p vxl-phys --example s2_cost_probe`
//!
//! 场景（两个，各 2 万个 0.2 m 动体铺成 100×200 网格、休眠关掉）：
//! ① `plain`：**没有运动学体** ⇒ 量"默认档对普通场景的额外开销"（S2 预通道每个子步都要扫体表找快转体）。
//! ② `spin`：8 片旋翼（0.6×0.4×0.02 薄板、ω=20 rad/s 绕 y）+ 同样的 2 万动体 ⇒ 量 S2 机器本身的开销。
//!
//! **实测（release，200 tick 取 3 跑最小值；本机 2026-10-21）**：
//!
//! | 场景 | S2 关 | S2 开 | 差 |
//! |---|---|---|---|
//! | plain（无运动学体） | 2.577 ms/tick | 2.495 ms/tick | **−3.2%（噪声级）** |
//! | spin（8 旋翼） | 2.702 ms/tick | 3.031 ms/tick | **+12.2%** |
//!
//! ⇒ **普通场景零代价**（预通道"先扫快转体、无则直接返回"⇒ 不再每子步拷全体位置：
//! 10 万体场景原本每子步要拷 1.2 MB）；**旋翼场景 +12%**（修前是 **+135%**：候选收集原本
//! "每子步每份"各扫一遍全体体 ⇒ 8 旋翼 × N=5 份 × 2 万体 = 16 万次距离判定/子步；
//! 改成"每个旋翼每子步只收集一次候选"后 11× 便宜）。

use std::time::Instant;

use vxl_phys::{KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

const BODIES: usize = 20_000;
const WARMUP: u32 = 20;
const TICKS: u32 = 200;
const REPS: usize = 3;
const BLADES: usize = 8;

fn config(s2: bool) -> PhysConfig {
    PhysConfig {
        gravity: Vec3::ZERO,
        ccd_target_substep: s2,
        // 永不休眠（否则测到的是"睡着的场景"，与真实负载不符）。
        sleep_time: 1.0e9,
        ..PhysConfig::default()
    }
}

/// 铺一片动体（0.2 m 立方，网格 100×200）。
fn fill_bodies(w: &mut World) {
    for i in 0..BODIES {
        let x = (i % 100) as f32 * 0.6 - 30.0;
        let z = (i / 100) as f32 * 0.6 - 60.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.1),
            },
            Vec3::new(x, 0.0, z),
            Quat::IDENTITY,
            1.0,
        );
    }
}

fn scene_plain(s2: bool) -> World {
    let mut w = World::new(config(s2));
    fill_bodies(&mut w);
    w
}

fn scene_spin(s2: bool) -> World {
    let mut w = World::new(config(s2));
    for k in 0..BLADES {
        let blade = w.add_kinematic(
            Shape::Box {
                half: Vec3::new(0.6, 0.4, 0.02),
            },
            Vec3::new(0.0, k as f32 * 0.6, 0.0),
            Quat::IDENTITY,
        );
        w.bodies
            .set_angvel(blade as usize, Vec3::new(0.0, 20.0, 0.0));
    }
    fill_bodies(&mut w);
    w
}

/// 跑一遍并返回 **每 tick 毫秒数**（3 跑取最小）。
fn bench(build: fn(bool) -> World, s2: bool) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..REPS {
        let mut w = build(s2);
        for _ in 0..WARMUP {
            w.step();
        }
        let t0 = Instant::now();
        for _ in 0..TICKS {
            w.step();
        }
        let ms = t0.elapsed().as_secs_f64() * 1000.0 / TICKS as f64;
        if ms < best {
            best = ms;
        }
    }
    best
}

fn main() {
    for (name, build) in [
        ("plain（无运动学体）", scene_plain as fn(bool) -> World),
        ("spin（8 旋翼）", scene_spin),
    ] {
        let off = bench(build, false);
        let on = bench(build, true);
        let diff = on - off;
        println!(
            "{name}: S2 关 {off:8.3} ms/tick | S2 开 {on:8.3} ms/tick | 差 {diff:+8.3} ms ({pct:+.1}%)",
            pct = diff / off * 100.0
        );
    }
}
