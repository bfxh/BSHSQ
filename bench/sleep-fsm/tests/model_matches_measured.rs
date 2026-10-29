//! **模型 ↔ 实测**：把 `crates/vxl-phys/tests/sleep_kinematic_carry.rs` 与
//! `sleep_wake_contract.rs` 跑出来的读数，逐点对到三值模型上。
//!
//! 没有这一条，模型就只是"又一个镜像"；有它，模型至少在这 5 个测点上与引擎同判。

use bshsq_sleep_fsm::{audit, Consts, Model, Scenario, Ternary, Verdict};

fn wake(rel: f32, k: u32, model: Model) -> Ternary {
    let c = Consts::default();
    let s = Scenario {
        kin: true,
        rel,
        ang: 0.0,
        depth: 0.0,
        wake_gate_k: k,
    };
    s.preds(&c, 0.0).wake(k, model)
}

/// 实测（2026-10-28，`--release`，120 tick 窗口）：
/// - `wake_gate_k=0`、平台 0.10/0.30 m/s ⇒ 唤醒 tick **1**；
/// - `wake_gate_k=8`、平台 0.10/0.30 m/s ⇒ **从未唤醒**（载荷 Δx = 0.0000 m）；
/// - `wake_gate_k=8`、撞击 3.00 m/s ⇒ 醒（`sleep_wake_contract.rs` 的强撞直通）。
#[test]
fn model_matches_measured_engine_points() {
    let before = [
        (0.10f32, 0u32, Ternary::True),
        (0.30, 0, Ternary::True),
        (0.10, 8, Ternary::False),
        (0.30, 8, Ternary::False),
        (3.00, 8, Ternary::True),
    ];
    for (rel, k, want) in before {
        assert_eq!(
            wake(rel, k, Model::Before),
            want,
            "Before 模型与实测不符：rel={rel} k={k}"
        );
    }
    for (rel, k, _) in before {
        assert_eq!(
            wake(rel, k, Model::After),
            Ternary::True,
            "After 应全醒：rel={rel} k={k}"
        );
    }
}

/// 修复前的"能醒"悬崖就在 `8 × sleep_linear = 0.32 m/s`。
#[test]
fn before_fix_cliff_is_at_fast_threshold() {
    let c = Consts::default();
    assert_eq!(wake(0.319, 8, Model::Before), Ternary::False);
    assert_eq!(wake(0.321, 8, Model::Before), Ternary::True);
    assert!((8.0 * c.sleep_linear - 0.32).abs() < 1e-6);
}

/// 分歧带（eps=1e-3）只把阈值附近的格子判成 `X`：判决从"依赖舍入"退化为 `Undetermined`，
/// 但**修复后**不存在任何反例 ⇒ 仍是 Proved。
#[test]
fn epsilon_degrades_to_undetermined_only_without_the_fix() {
    let c = Consts::default();
    let a = audit(&c, 1e-3);
    assert_eq!(a.p_wake_before, Verdict::Refuted);
    assert_eq!(a.p_wake_after, Verdict::Proved);
    assert!(a.x_rows > 0);
}
