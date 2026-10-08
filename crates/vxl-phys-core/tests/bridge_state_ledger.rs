//! **`BridgeState` 的账**（总质量 / 动量 / 角动量 / 动能）与 `handoff` 的两条语义
//! （默认档拒绝带状态的快照 + 成功路径一条）。
//!
//! 纯数据判据：不依赖任何域；各域自己那份实现见
//! `crates/vxl-phys-fluid/tests/state_inheritance.rs`、`crates/vxl-phys-soft/tests/cloth_state_inheritance.rs`、
//! `crates/vxl-phys-splat/tests/splat_state_bridge.rs`。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{handoff, BridgeKind, BridgeState, StateBridge};
use vxl_phys_core::Vec3;

/// 只实现"位置桥"的最小载体（接口默认档 ⇒ `import_state` 必须拒绝带速度/质量的快照）。
struct PositionsOnly(Vec<Vec3>);

impl StateBridge for PositionsOnly {
    fn kind(&self) -> BridgeKind {
        BridgeKind::Particle
    }
    fn export_positions(&self, out: &mut Vec<Vec3>) {
        out.extend_from_slice(&self.0);
    }
    fn import_positions(&mut self, src: &[Vec3]) -> bool {
        if src.len() != self.0.len() {
            return false;
        }
        self.0.copy_from_slice(src);
        true
    }
}

/// 手算两点的刚体运动账：m = (2, 3)、v = (1,0,0)、r₂ = (0,0,2)。
#[test]
fn ledger_matches_hand_computed_two_point_motion() {
    let st = BridgeState {
        pos: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 2.0)],
        vel: vec![Vec3::new(1.0, 0.0, 0.0); 2],
        mass: vec![2.0, 3.0],
    };
    assert!(st.is_consistent(), "三段等长 ⇒ 自洽");
    assert!(st.is_full(), "速度与质量都已登记");
    assert_eq!(st.total_mass(), Some(5.0));
    // Σ m·v = (2+3)·(1,0,0)
    assert_eq!(st.momentum(), Some(Vec3::new(5.0, 0.0, 0.0)));
    // 绕原点：r₂×v = (0,0,2)×(1,0,0) = (0,2,0) ⇒ L = 3·(0,2,0)
    assert_eq!(
        st.angular_momentum_about(Vec3::ZERO),
        Some(Vec3::new(0.0, 6.0, 0.0))
    );
    // 动能 = ½·(2+3)·1²
    assert_eq!(st.kinetic_energy(), Some(2.5));
}

/// **空段 = 不知道，不是 0**：这是本设计存在的前提，判据必须钉住。
#[test]
fn empty_segments_mean_unknown_not_zero() {
    let pos_only = BridgeState::from_positions(vec![Vec3::ZERO; 3]);
    assert!(pos_only.is_consistent(), "空段不参与自洽性判定");
    assert!(!pos_only.is_full());
    assert_eq!(pos_only.total_mass(), None);
    assert_eq!(pos_only.momentum(), None);
    assert_eq!(pos_only.kinetic_energy(), None);

    // 只登记质量（速度未登记）⇒ 总质量可算、动量仍"不知道"
    let mass_only = BridgeState {
        pos: vec![Vec3::ZERO; 3],
        vel: Vec::new(),
        mass: vec![1.0, 2.0, 3.0],
    };
    assert_eq!(mass_only.total_mass(), Some(6.0));
    assert_eq!(mass_only.momentum(), None);
    assert!(!mass_only.is_full());

    // 长度不自洽 ⇒ 一律 None（不许拿前缀凑数）
    let ragged = BridgeState {
        pos: vec![Vec3::ZERO; 3],
        vel: Vec::new(),
        mass: vec![1.0],
    };
    assert!(!ragged.is_consistent());
    assert_eq!(ragged.total_mass(), None);
    assert_eq!(ragged.angular_momentum_about(Vec3::ZERO), None);
}

/// `handoff`：成功路径搬位置；默认档**拒绝**带速度的快照（金丝雀：判据不是恒真）。
#[test]
fn handoff_copies_positions_and_default_refuses_velocity() {
    let src = PositionsOnly(vec![Vec3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, 0.0, 0.5)]);
    let mut dst = PositionsOnly(vec![Vec3::ZERO; 2]);
    assert!(handoff(&src, &mut dst), "位置桥之间的交接必须成功");
    assert_eq!(dst.0, src.0);

    let before = dst.0.to_vec();
    let with_vel = BridgeState {
        pos: src.0.to_vec(),
        vel: vec![Vec3::X; 2],
        mass: Vec::new(),
    };
    assert!(
        !dst.import_state(&with_vel),
        "默认档必须拒绝带速度的快照（不许静默丢动量）"
    );
    assert_eq!(dst.0, before, "拒绝的导入不许改动任何一位");

    // 点数不符 ⇒ 目标拒绝（`handoff` 如实返回 false）
    let mut wrong = PositionsOnly(vec![Vec3::ZERO; 3]);
    assert!(!handoff(&src, &mut wrong), "点数不符必须拒绝");
}
