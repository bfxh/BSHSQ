//! **参数骨架的档位对拍**（从 `src/params.rs` 的同文件 `mod tests` 搬来 —— 那里受 god 门
//! **文件行数棘轮**，而本片要给 `SelfCollision` 加一个字段；搬到集成测试后 `params.rs` 反而净缩）。
//!
//! 判据 = **逐档对值**（数值来自 `SPEC.md`，改动它们就是改规格 ⇒ 这里红了要先看规格）。
use vxl_phys_soft::{Stiffness, TearStrain};

#[test]
fn alpha_table_matches_spec() {
    assert_eq!(Stiffness::NearRigid.alpha(), 1e-7);
    assert_eq!(Stiffness::Hard.alpha(), 1e-6);
    assert_eq!(Stiffness::Standard.alpha(), 1e-5);
    assert_eq!(Stiffness::Soft.alpha(), 1e-4);
    assert_eq!(Stiffness::Jelly.alpha(), 3e-4);
}

#[test]
fn tear_table_matches_spec() {
    assert_eq!(TearStrain::E03.eps(), 0.3);
    assert_eq!(TearStrain::E05.eps(), 0.5);
    assert_eq!(TearStrain::E10.eps(), 1.0);
    assert_eq!(TearStrain::None.eps(), f32::INFINITY);
}

/// **自摩擦默认必须是关**（`0`）：规格书**没有**规定自摩擦 ⇒ 本片选的默认是"0 = 关"
/// （与全仓"0=关字段"先例一致）⇒ 切片 T3 的读数因此逐位不变。
#[test]
fn self_collision_defaults_are_off() {
    let d = vxl_phys_soft::SelfCollision::default();
    assert!(!d.enabled, "自碰撞默认关");
    assert_eq!(d.friction, 0.0, "自摩擦默认关（0）");
}
