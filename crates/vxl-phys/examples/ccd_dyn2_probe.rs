//! **动-动覆盖探针**（2026-10-13，`OPEN-PROBLEMS.md` P20 的取证器械）：
//! **运动学旋翼**（薄板绕 y 自旋）+ **高速弹体**（0.1 m 立方、60 m/s 沿 +x），相位扫 24 档，
//! 逐档量"弹体材料有没有嵌进旋翼盒"（tick 末端逐帧取最大）。
//!
//! **实测（本探针就是判据器械）**：
//!
//! | 目标侧口径 | ω=20 rad/s 最坏埋深 | ω=200 rad/s 最坏埋深 |
//! |---|---|---|
//! | 只带平移（现状，65d473a） | 0.0200 | 0.0200 |
//! | 姿态插值 + 接触点速度（试过，未落） | 0.0194 | 0.0192 |
//!
//! ⇒ **差异在噪声内**（~0.02 ≈ 皮肤带 = 离散接触允许的重叠量），即这两条场景里 **离散管线把
//! CCD 的差异完全掩盖**了；而那次实现把 m0 门槛档的唤醒抖动从 7 顶到 **10（= 1.000/s 贴边）**、
//! `verdict` 直接红 ⇒ **没有收益却有硬代价，按纪律不落**（详见 P20）。
//!
//! 跑法：`cargo run --release -q -p vxl-phys --example ccd_dyn2_probe`

use vxl_phys::{KinematicExt, PhysConfig, Quat, Shape, Vec3, World};

/// 旋翼半尺寸（薄板：0.6 × 0.4 × 0.02）。
const BLADE: Vec3 = Vec3::new(0.6, 0.4, 0.02);
/// 旋翼角速度（绕 y）。两档都跑过：20（板角 14 m/s）与 200（板角 120 m/s）。
const OMEGA: f32 = 20.0;
/// 弹体半长（0.1 m 立方）。
const BULLET: f32 = 0.1;
/// 弹体初速 / 起始 x / tick 数。
const SPEED: f32 = 60.0;
const START_X: f32 = -6.0;
const TICKS: u32 = 300;

/// 建场：旋翼（运动学，绕 y 自旋）+ 弹体（沿 +x 飞）。`phase` = 旋翼初始角（度）。
fn scene(phase: f32) -> (World, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    // 旋翼中心 (0,0,0)：板的长轴在 x–z 平面里转，厚 0.04 m ⇒ "面对弹体"的角度窗口很窄。
    let blade = w.add_kinematic(
        Shape::Box { half: BLADE },
        Vec3::ZERO,
        Quat::from_axis_angle(Vec3::Y, phase.to_radians()),
    );
    w.bodies
        .set_angvel(blade as usize, Vec3::new(0.0, OMEGA, 0.0));
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(BULLET),
        },
        Vec3::new(START_X, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(SPEED, 0.0, 0.0);
    (w, b)
}

/// 弹体材料点落进旋翼盒的最深值（tick 末端逐帧取最大）。
fn burial(w: &World, b: usize) -> f32 {
    let (p, q) = (w.bodies.position[b], w.bodies.rot(b));
    let mut deepest = 0.0f32;
    for sx in [-BULLET, BULLET] {
        for sy in [-BULLET, BULLET] {
            for sz in [-BULLET, BULLET] {
                let d = p + q.rotate_vec3(Vec3::new(sx, sy, sz));
                if d.x.abs() <= BLADE.x && d.y.abs() <= BLADE.y && d.z.abs() <= BLADE.z {
                    deepest = deepest.max(
                        (BLADE.x - d.x.abs())
                            .min(BLADE.y - d.y.abs())
                            .min(BLADE.z - d.z.abs()),
                    );
                }
            }
        }
    }
    deepest
}

/// 跑一遍：返回 `(峰值埋深, 弹体末态 x, 末态|v|)`。
fn run(phase: f32) -> (f32, f32, f32) {
    let (mut w, b) = scene(phase);
    let mut deep = 0.0f32;
    for _ in 0..TICKS {
        w.step();
        deep = deep.max(burial(&w, b));
    }
    (deep, w.bodies.position[b].x, w.bodies.linvel[b].length())
}

fn main() {
    // 相位扫描：旋翼初始角扫一圈（每档 15°）—— 逐档量"弹体有没有嵌进旋翼材料"。
    let (mut worst_deep, mut worst_phase) = (0.0f32, 0.0f32);
    let (mut min_end_x, mut max_speed) = (f32::INFINITY, 0.0f32);
    for k in 0..24 {
        let phase = k as f32 * 15.0;
        let (deep, x, sp) = run(phase);
        if deep > worst_deep {
            worst_deep = deep;
            worst_phase = phase;
        }
        min_end_x = min_end_x.min(x);
        max_speed = max_speed.max(sp);
        println!("相位 {phase:6.1}°：峰值埋深={deep:8.4} 末态 x={x:9.3} 末|v|={sp:8.3}");
    }
    println!(
        "== 24 相位汇总：最坏埋深 {worst_deep:.4}（相位 {worst_phase:.1}°）、最小末态 x {min_end_x:.3}、最大末|v| {max_speed:.3}"
    );
}
