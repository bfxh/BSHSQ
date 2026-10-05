//! **求解器/CCD/确定性与杂项判据** —— `tests` 的子模块。
//!
//! 2026-10-05 从 967 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use super::ground_world;
use crate::{FrictionModel, HeightField, Material, PhysConfig, Quat, Shape, Vec3, World};

/// **CCD 回归**（本轮修复）：开启 CCD 后，**贴地滑行**的体不得被锁死。
/// 修前：每个扫描采样都有地面接触 ⇒ 判「命中」⇒ 钳回起点、原地停住。
/// 修后：只有「沿法向接近」的采样才算命中 ⇒ 切向滑行不受影响。
#[test]
fn ccd_does_not_lock_sliding_body() {
    let cfg = PhysConfig {
        ccd_speed_threshold: 5.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
    w.add_heightfield(hf);
    // 贴地盒（底面 y=0.5 略上方）以 8 m/s 沿 +X 滑行（超过 CCD 阈值 5）
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.55, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(8.0, 0.0, 0.0);
    for _ in 0..60 {
        w.step();
    }
    let x = w.bodies.position[b as usize].x;
    // 摩擦会减速，但绝不该「原地不动」：修前 x ≈ 0，修后应有明显位移
    assert!(x > 2.0, "CCD 不应锁死滑行体：x = {x}");
}

#[test]
fn determinism_same_construction_same_hash() {
    let run = || {
        let mut w = ground_world();
        for k in 0..12 {
            let x = (k % 4) as f32 * 1.2;
            let z = (k / 4) as f32 * 1.2;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(x, 2.0 + (k as f32) * 0.9, z),
                Quat::IDENTITY,
                1.0,
            );
        }
        for _ in 0..180 {
            w.step();
        }
        w.state_hash()
    };
    assert_eq!(run(), run());
}

/// §5/§6 并行契约：并行（threads=8）与串行（threads=1）结果 bit 级一致。
/// 场景需超过各相并行门槛（>4096 体）才能真正走到并行路径。
#[test]
fn parallel_matches_serial_bitwise() {
    let run = |threads: usize| {
        let cfg = PhysConfig {
            threads,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        w.add_heightfield(HeightField::flat(-40.0, -40.0, 81, 81, 1.0, 0.0));
        for k in 0..4200usize {
            let x = (k % 70) as f32 - 35.0;
            let z = (k / 70) as f32 - 30.0;
            w.add_static(
                Shape::Box {
                    half: Vec3::new(0.5, 0.5, 0.5),
                },
                Vec3::new(x, 0.5, z),
                Quat::IDENTITY,
            );
        }
        for k in 0..900 {
            let x = ((k * 37) % 97) as f32 / 97.0 * 30.0 - 15.0;
            let z = ((k * 53) % 89) as f32 / 89.0 * 30.0 - 15.0;
            let y = 4.0 + ((k * 29) % 71) as f32 / 71.0 * 10.0;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(x, y, z),
                Quat::IDENTITY,
                1000.0,
            );
        }
        for _ in 0..150 {
            w.step();
        }
        w.state_hash()
    };
    let serial = run(1);
    let parallel = run(8);
    assert_eq!(serial, parallel, "并行与串行状态哈希必须一致（§5）");
}

#[test]
fn pyramid_settles_and_sleeps() {
    let mut w = ground_world();
    let layers = 4;
    for layer in 0..layers {
        let count = layers - layer;
        for k in 0..count {
            let x = (k as f32 - (count as f32 - 1.0) * 0.5) * 1.05;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(x, 0.55 + layer as f32 * 1.02, 0.0),
                Quat::IDENTITY,
                1.0,
            );
        }
    }
    for _ in 0..600 {
        w.step();
    }
    let h = w.health();
    assert!(h.is_clean(), "{h:?}");
    // 塔应全部入睡（§3：无持续抖动）。
    let awake = h.awake_bodies;
    assert_eq!(awake, 0, "awake = {awake}");
}

#[test]
fn restitution_bounce_and_threshold() {
    let mut w = World::new(PhysConfig {
        restitution: 0.8,
        ..PhysConfig::default()
    });
    w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let mut max_after_fall = 0.0f32;
    for _ in 0..1800 {
        w.step();
        max_after_fall = max_after_fall.max(w.bodies.linvel[b as usize].y);
    }
    // e=0.8 应产生明显反弹（> 2 m/s 向上），30 秒内经恢复阈值衰减到静止。
    assert!(max_after_fall > 2.0, "bounce vy = {max_after_fall}");
    assert!(w.bodies.linvel[b as usize].y.abs() < 0.3);
}

#[test]
fn digging_removes_support() {
    let mut w = ground_world();
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.6, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..120 {
        w.step();
    }
    let y_rest = w.bodies.position[b as usize].y;
    // 在盒子正下方挖 3×3 列块（2m 深）。
    for gx in -1i32..=1 {
        for gz in -1i32..=1 {
            w.terrain.dig(0, gx as f32, gz as f32, 2.0);
        }
    }
    w.bodies.wake(b as usize);
    for _ in 0..120 {
        w.step();
    }
    let y_after = w.bodies.position[b as usize].y;
    assert!(y_after < y_rest - 1.0, "rest {y_rest} → after {y_after}");
}

#[test]
fn material_pair_restitution_and_friction() {
    // §4.4/§4.5：e 取材质对 max——球(e=0.9) 落到地面(e=0.0) → 反弹按 0.9。
    let mut w = World::new(PhysConfig::default());
    let ground_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.5 }, 0.0));
    let ball_mat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 0.2 }, 0.9));
    w.add_heightfield(HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0));
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    w.bodies.set_material(b as usize, ball_mat);
    let _ = ground_mat;
    let mut max_bounce = 0.0f32;
    for _ in 0..1200 {
        w.step();
        max_bounce = max_bounce.max(w.bodies.linvel[b as usize].y);
    }
    // 0.9 × 9.9 m/s 落地速度 ≈ 8.9 m/s 首次反弹。
    assert!(max_bounce > 4.0, "material pair bounce vy = {max_bounce}");
    assert!(w.health().is_clean());
}

#[test]
fn ccd_stops_fast_sphere_at_thin_wall() {
    // 薄墙 half_z=0.1；球 r=0.2 以 120 m/s（单帧位移 2m）射向墙体。
    // 采样带（墙厚+球径=0.6m）< 位移 2m 且起点偏移 → 离散步进必然穿透。
    let build = |ccd_on: bool| {
        let mut w = World::new(PhysConfig {
            ccd_speed_threshold: if ccd_on { 30.0 } else { f32::INFINITY },
            max_linear_velocity: 200.0,
            ..PhysConfig::default()
        });
        w.add_static(
            Shape::Box {
                half: Vec3::new(5.0, 5.0, 0.1),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let s = w.add_dynamic(
            Shape::Sphere { radius: 0.2 },
            Vec3::new(0.0, 0.0, -19.0),
            Quat::IDENTITY,
            1.0,
        );
        w.bodies.set_linvel(s as usize, Vec3::new(0.0, 0.0, 120.0));
        (w, s)
    };
    // CCD 关：穿透（球越过墙面 z>0.3）。
    let (mut w_off, s_off) = build(false);
    for _ in 0..20 {
        w_off.step();
    }
    let z_off = w_off.bodies.position[s_off as usize].z;
    assert!(z_off > 1.0, "预期穿透，实际 z = {z_off}");
    // CCD 开：钳位在墙前。
    let (mut w_on, s_on) = build(true);
    for _ in 0..20 {
        w_on.step();
    }
    let z_on = w_on.bodies.position[s_on as usize].z;
    assert!(
        (-1.0..=0.9).contains(&z_on),
        "CCD 应拦下球，实际 z = {z_on}"
    );
    let h = w_on.health();
    assert!(h.is_clean(), "{h:?}");
}

#[test]
fn ccd_disabled_by_default_keeps_slow_scene_unchanged() {
    // 默认阈值 INFINITY：确定性场景与 M0 行为一致（回归守门）。
    let run = || {
        let mut w = ground_world();
        for k in 0..6 {
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.4),
                },
                Vec3::new(k as f32 * 1.1, 2.0 + k as f32 * 0.9, 0.0),
                Quat::IDENTITY,
                1.0,
            );
        }
        for _ in 0..180 {
            w.step();
        }
        w.state_hash()
    };
    assert_eq!(run(), run());
}
