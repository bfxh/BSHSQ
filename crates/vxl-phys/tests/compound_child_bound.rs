//! **复合体子形状数的入口上限**（安全审计 F-13）。
//!
//! 审计探针：复合体子形状 ≥ 65535 个时，子序号通道回绕——`tag_child_features` 给每条子流形
//! 写的 `space = (ci + 1) as u16` 与 `feature |= (ci + 1) << 16` 双双归零 ⇒ 该子形状的暖启动键
//! 与**非复合体**完全一样（`spaces=[1,0,1,0] features=[65536,0,65536,0]`）。机制确定、可达性低
//! （要 65536 个子形状），故按 S1 口径**在注册入口拒绝**：`u16::MAX` 放行、`u16::MAX + 1` 断言。

use vxl_phys::{CompoundChild, HullExt, PhysConfig, Quat, Shape, Vec3, World};

/// 一个最小的合法子形状（球，局部原点）。
fn child() -> CompoundChild {
    CompoundChild {
        shape: Shape::Sphere { radius: 0.1 },
        offset: Vec3::ZERO,
        rot: Quat::IDENTITY,
    }
}

#[test]
fn compound_at_u16_max_children_is_accepted() {
    let mut w = World::new(PhysConfig::default());
    let kids = vec![child(); u16::MAX as usize];
    let id = w.add_compound(kids);
    assert_eq!(w.compound_children(id).len(), u16::MAX as usize);
}

#[test]
#[should_panic(expected = "u16::MAX")]
fn compound_beyond_u16_max_children_is_rejected_at_entry() {
    let mut w = World::new(PhysConfig::default());
    let kids = vec![child(); u16::MAX as usize + 1];
    let _ = w.add_compound(kids);
}
