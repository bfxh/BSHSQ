//! **配置入口收口判据**（安全审计 F-11 / F-16）。
//!
//! 审计证据：`PhysConfig { dt: -1.0 }` / `{ dt: 10.0 }` / `{ contact_skin: NaN }` ⇒ 不报错，
//! 物体**静默穿地**（top_y = −0.495 / −119671 / −15.2）；`substeps = u32::MAX` ⇒ **首步不返回**
//! （`velocity_iterations = u32::MAX` 也 23 s）。修法 = 门面构造入口整张收口（见
//! `src/world_build/config_guard.rs`），不在内核加断言。
//!
//! 四条：① **默认档与四个 Preset 逐位不变**（收口对合法配置必须零作用，否则行为门/金样就是
//! 判据）；② **合法的极端值原样保留**（`sleep_time = INFINITY` 关睡眠、`sleep_linear = 0`、
//! `restitution = 1`、限速 1e9——仓内探针真在用）；③ 病态值整张回落到默认档；④ 收口后的
//! "u32::MAX 档"能正常推进（旧实现在此**不返回**）。

use vxl_phys::{PhysConfig, Preset, Quat, Shape, Vec3, World};

#[test]
fn default_and_presets_pass_through_untouched() {
    let w = World::new(PhysConfig::default());
    assert_eq!(w.config, PhysConfig::default(), "默认档必须逐位不变");
    for p in [
        Preset::Arcade,
        Preset::Balanced,
        Preset::Realistic,
        Preset::Extreme,
    ] {
        let cfg = PhysConfig::from_preset(p);
        let w = World::new(cfg.clone());
        assert_eq!(w.config, cfg, "{p:?} 档必须逐位不变");
    }
}

#[test]
fn legit_extremes_are_not_normalized_away() {
    let cfg = PhysConfig {
        // 仓内真在用的合法极端值（bench/realism-probe 的 R2/R8 探针档）。
        sleep_time: f32::INFINITY,
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        restitution: 1.0,
        ccd_speed_threshold: f32::INFINITY,
        max_linear_velocity: 1e9,
        ..PhysConfig::default()
    };
    let want = cfg.clone();
    let w = World::new(cfg);
    assert_eq!(w.config, want, "合法极端值不得被'合理化'成默认值");
}

#[test]
fn poisoned_config_falls_back_to_defaults_at_entry() {
    let cfg = PhysConfig {
        // F-11：步长/接触带。
        dt: -1.0,
        contact_skin: f32::NAN,
        gjk_tolerance: -1e-3,
        restitution: 5.0,
        // F-16：迭代上限（旧实现 = 首步不返回）。
        substeps: u32::MAX,
        velocity_iterations: u32::MAX,
        joint_iterations: 7_000,
        normal_inner: 0,
        shock_iterations: 99,
        settled_hold_iterations: 99,
        stabilization_iterations: 99,
        // F-08 入口面：非有限阈值与重力。
        max_linear_velocity: f32::NAN,
        max_angular_velocity: 0.0,
        sleep_linear: f32::NAN,
        sleep_angular: -1.0,
        sleep_time: f32::NAN,
        ccd_speed_threshold: f32::NAN,
        gravity: Vec3::new(f32::NAN, -9.8, 0.0),
        ..PhysConfig::default()
    };
    let w = World::new(cfg);
    assert_eq!(
        w.config,
        PhysConfig::default(),
        "病态值必须整张回落到默认档"
    );
}

#[test]
fn huge_iteration_config_still_steps_forward() {
    let cfg = PhysConfig {
        substeps: u32::MAX,
        velocity_iterations: u32::MAX,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    for _ in 0..30 {
        w.step();
    }
    assert_eq!(w.tick, 30);
    let p = w.bodies.position[b];
    assert!(
        p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
        "收口后不得出 NaN：{p:?}"
    );
    assert!(p.y < 2.0, "重力下必须下落（未被闸死）：y = {}", p.y);
}

/// **F-17**：`threads` 超过本机并行度 ⇒ 收口到并行度（fork-join 池多开的 OS 线程与工作量无关）。
/// 0 / 1 是合法档（串行），不被改动；本机并行度本身是上限（不放大）。
#[test]
fn threads_are_capped_at_machine_parallelism() {
    let par = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);
    let cfg = PhysConfig {
        threads: 4096,
        ..PhysConfig::default()
    };
    let w = World::new(cfg);
    assert_eq!(w.config.threads, par, "4096 线程应收口到本机并行度");
    // 串行档原样保留（0 与 1 都是"串行"口径，仓内测试真在用）
    for t in [0usize, 1] {
        let cfg = PhysConfig {
            threads: t,
            ..PhysConfig::default()
        };
        let w = World::new(cfg);
        assert_eq!(w.config.threads, t, "串行档不得被改写");
    }
}
