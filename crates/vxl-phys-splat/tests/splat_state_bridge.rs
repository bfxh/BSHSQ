//! **高斯喷溅状态桥判据**（2026-10-08，`ROUTE.md` §3.1 ①的**反向**半）：
//! 核中心的导出/导入 + 状态（位置/速度/质量）登记口径。
//!
//! 判据链（逐条钉）：
//! ① `kind()` = `Splat`，`export_positions` 按注册序写核中心；
//! ② 长度不符 ⇒ 拒绝且一字不动；长度相符 ⇒ 只换中心，**尺度/姿态/不透明度/颜色不动**；
//! ③ `export_state` 的质量口径 = `opacity`（模块头的物理读法）；速度槽只在**双向耦合开着**
//!    时才登记 —— 未开时 `vel` 空 ⇒ `momentum()` 返回 `None`（"不知道"，不是 0）；
//! ④ 未开双向耦合的场**拒绝**带速度的快照（没有速度槽，静默丢弃就是丢动量）；
//! ⑤ 开双向 + 沉积 ⇒ 速度槽登记、动量账自洽、导入到同形场后逐位相同。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{BridgeKind, BridgeState, MediumField as _, StateBridge};
use vxl_phys_core::Vec3;
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 三核场（不透明度 0.5 / 1.0 / 1.5 ⇒ 质量口径可直接读出）。
fn field() -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.push(Splat::isotropic(Vec3::new(0.0, 0.0, 0.0), 0.2, 0.5));
    f.push(Splat::isotropic(Vec3::new(0.5, 0.0, 0.0), 0.2, 1.0));
    f.push(Splat::isotropic(Vec3::new(0.0, 0.5, 0.0), 0.2, 1.5));
    f
}

#[test]
fn splat_bridge_moves_centers_only() {
    let mut f = field();
    assert_eq!(f.kind(), BridgeKind::Splat);
    let mut pos = Vec::new();
    f.export_positions(&mut pos);
    assert_eq!(pos.len(), 3);
    assert_eq!(pos[1], Vec3::new(0.5, 0.0, 0.0));

    // ② 长度不符 ⇒ 拒绝且一字不动
    let mut wrong = pos.to_vec();
    wrong.pop();
    assert!(!f.import_positions(&wrong), "长度不符必须拒绝");
    let mut after = Vec::new();
    f.export_positions(&mut after);
    assert_eq!(after, pos, "拒绝的导入不许改动任何一位");

    // 长度相符 ⇒ 换中心；尺度/姿态/不透明度不受影响
    let saved = f.splats()[1];
    let moved: Vec<Vec3> = pos.iter().map(|p| *p + Vec3::new(1.0, 0.0, 0.0)).collect();
    assert!(f.import_positions(&moved));
    let mut got = Vec::new();
    f.export_positions(&mut got);
    assert_eq!(got, moved);
    assert_eq!(f.splats()[1].scale, saved.scale, "位置导入不许改尺度");
    assert_eq!(f.splats()[1].opacity, saved.opacity, "位置导入不许改质量");
    assert_eq!(f.splats()[1].rot, saved.rot, "位置导入不许改姿态");
}

#[test]
fn splat_state_registers_mass_and_velocity_slot() {
    let mut f = field();
    let mut st = BridgeState::default();
    f.export_state(&mut st);
    assert_eq!(st.mass, vec![0.5, 1.0, 1.5], "质量口径 = opacity");
    assert!(st.vel.is_empty(), "未开双向耦合 ⇒ 速度槽未登记");
    assert!(!st.is_full());
    assert_eq!(st.momentum(), None, "速度未登记 ⇒ 动量'不知道'（不是 0）");

    // ④ 未开双向 ⇒ 带速度的快照必须拒绝，且一字不动
    let with_vel = BridgeState {
        pos: st.pos.to_vec(),
        vel: vec![Vec3::X; 3],
        mass: st.mass.to_vec(),
    };
    let mut before = BridgeState::default();
    f.export_state(&mut before);
    assert!(!f.import_state(&with_vel), "没有速度槽的场不许静默丢动量");
    let mut after = BridgeState::default();
    f.export_state(&mut after);
    assert_eq!(before, after, "拒绝的导入不许改动任何一位");

    // ⑤ 开双向 + 沉积 ⇒ 速度槽登记、动量账自洽（质量口径 = opacity）
    f.medium_density = 1.0;
    f.set_two_way(true);
    f.deposit(Vec3::ZERO, Vec3::new(0.1, 0.0, 0.0), 0.0, 0.0);
    let mut src = BridgeState::default();
    f.export_state(&mut src);
    assert_eq!(src.vel.len(), 3, "双向开 ⇒ 逐核速度登记");
    assert!(src.is_full(), "速度与质量都已登记");
    let mut want = Vec3::ZERO;
    for i in 0..3 {
        want += src.vel[i] * src.mass[i];
    }
    assert!(want.length() > 0.0, "用例非平凡：沉积确实给了速度");
    let p = src.momentum().unwrap_or(Vec3::ZERO);
    let rel = (p - want).length() / want.length();
    assert!(rel < 1e-6, "动量 = Σ mᵢvᵢ：rel={rel:e}（p={p:?}）");

    // 导入到同形场 ⇒ 逐位相同（域间交接的喷溅侧）
    let mut dst = field();
    dst.medium_density = 1.0;
    dst.set_two_way(true);
    assert!(dst.import_state(&src), "同形 + 已开双向 ⇒ 必须接受");
    let mut got = BridgeState::default();
    dst.export_state(&mut got);
    assert_eq!(got, src, "状态交接后逐位相同");
}
