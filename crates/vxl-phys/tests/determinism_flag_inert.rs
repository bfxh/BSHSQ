//! **`strict_determinism` 的契约判据**（2026-10-09）：该字段置 `false` 不改变任何行为。
//!
//! SPEC §4.14 写了两档（严格模式 / 性能模式允许重排），但**性能模式在这条代码线上没有实现**：
//! sim 路径按索引有序归约、并行各相按「离散槽位 + 有序归并」与串行 bit 级一致 ⇒ 引擎恒为严格模式。
//! 于是 `strict_determinism` 是**预留位**：它没有读取点，置 false 与 true 必须同结果。
//!
//! 这条判据把「无行为」钉成可执行契约：将来谁真的实现性能模式（让 false 关掉某些东西），
//! 本测试会红，改动者必须同时更新 SPEC §4.14、`PhysConfig` 的字段注释与这里
//! —— 而不是悄悄改变一个公开开关的语义。
//!
//! 场景选「有接触、有堆叠、有姿态扰动」：确定性差异最容易在接触求解路径上显形。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 同一构造跑 `ticks` tick，返回末态 xxh3-128 哈希。
fn run(strict: bool, ticks: usize) -> u128 {
    let cfg = PhysConfig {
        strict_determinism: strict,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.5, 6.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 4 层 × 4 盒的小塔：接触 + 堆叠 + 逐层姿态扰动（避免完全对称的平凡解）。
    for layer in 0..4u32 {
        for k in 0..4u32 {
            let x = (k % 2) as f32 - 0.5;
            let z = (k / 2) as f32 - 0.5;
            let y = 0.55 + layer as f32 * 1.05;
            let tilt = 0.01 * (layer + k) as f32;
            w.add_dynamic(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(x * 1.02, y, z * 1.02),
                Quat::from_axis_angle(Vec3::Y, tilt),
                1000.0,
            );
        }
    }
    for _ in 0..ticks {
        w.step();
    }
    w.state_hash()
}

#[test]
fn determinism_flag_is_inert() {
    let initial = run(true, 0);
    let strict_on = run(true, 240);
    let strict_off = run(false, 240);
    assert_ne!(
        initial, strict_on,
        "场景必须真的在演化，否则这条判据会平凡成立"
    );
    assert_eq!(
        strict_on, strict_off,
        "strict_determinism = false 不得改变任何行为（SPEC §4.14 的性能模式尚未实现）"
    );
}
