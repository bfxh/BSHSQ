//! **`crossed_face` 的单元对拍**（§8.4.5.2 的第一步：先证明零件对，再装机器）。
//!
//! 手算四种情形直接调它（不经绳索/刚体/子步）。盒：`half = (0.3, 0.05, 0.3)`、心 `(0, 1.0, 0)`、
//! 无旋转、`radius = 0.02` ⇒ **底面在 y = 0.95**、顶面 1.05、+x 面在 x = 0.3。

use vxl_phys_core::{Quat, Vec3};
use vxl_phys_soft::rigid::crossed_face;

const HALF: Vec3 = Vec3::new(0.3, 0.05, 0.3);
const CENTER: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const R: f32 = 0.02;

fn call(prev: Vec3, now: Vec3) -> Option<(Vec3, f32, Vec3, u8)> {
    // 单元情形里盒不动 ⇒ 两个位姿相同
    // 单元情形里盒不动**也不转** ⇒ 两个位姿的朝向相同（2c-1 后签名要求两个朝向）
    crossed_face(
        Quat::IDENTITY,
        Quat::IDENTITY,
        HALF,
        CENTER,
        CENTER,
        prev,
        now,
        R,
    )
}

/// ⑥ **动的是盒子**（粒子不动）—— 真实场景的机制：盒从 1.02 降到 1.00，粒子静止在 0.945
/// ⇒ 必须判成"穿过底面"。（这**正是**整场"缓存命中 = 0、盒子穿过绳线"的根因：只给一个位姿时，
/// 穿越只能来自粒子的位移，而这里动的是盒。）
#[test]
fn moving_box_onto_still_particle() {
    let p = Vec3::new(0.0, 0.945, 0.0);
    let r = crossed_face(
        Quat::IDENTITY,
        Quat::IDENTITY,
        HALF,
        Vec3::new(0.0, 1.02, 0.0), // 上一子步：盒底 0.97 ⇒ 粒子在带外 0.025 ≥ 0.02
        Vec3::new(0.0, 1.00, 0.0), // 当前：盒底 0.95 ⇒ 粒子进带内 0.005 < 0.02
        p,
        p,
        R,
    );
    let (n, depth, _, face) = r.expect("盒在动也该判成穿过底面（这一条是整场'零接触'的根因）");
    assert_eq!(face, 2, "面号应为 y−（2），实得 {face}");
    assert!(n.y < -0.99, "法线应指 −y，实得 {n:?}");
    assert!((depth - 0.015).abs() < 1e-4, "深度应为 0.015，实得 {depth}");
}

/// ① 从**下方**进底面（`y−` 面）：`face` 应为 2、法线 `(0,−1,0)`、`depth = radius − d_b`。
#[test]
fn enters_bottom_face_from_below() {
    // 上一步在带外（d_a = 0.075−0.05 = 0.025 ≥ 0.02），这一步进带内（d_b = 0.055−0.05 = 0.005 < 0.02）
    let r = call(Vec3::new(0.0, 0.925, 0.0), Vec3::new(0.0, 0.945, 0.0));
    let (n, depth, q, face) = r.expect("该判为穿过底面");
    assert_eq!(face, 2, "面号应为 y−（2），实得 {face}");
    assert!(n.y < -0.99, "法线应指 −y，实得 {n:?}");
    assert!((depth - 0.015).abs() < 1e-4, "深度应为 0.015，实得 {depth}");
    assert!(
        (q.y - 0.95).abs() < 1e-4,
        "交点应在底面 y=0.95，实得 {}",
        q.y
    );
}

/// ② 压得**更深**：`depth` 随进入量增长（0.02 + 进入量）。
#[test]
fn deeper_entry_gives_bigger_depth() {
    let (n, depth, _, face) =
        call(Vec3::new(0.0, 0.90, 0.0), Vec3::new(0.0, 0.96, 0.0)).expect("该判为穿过底面");
    assert_eq!(face, 2);
    assert!(n.y < -0.99);
    assert!((depth - 0.03).abs() < 1e-4, "深度应为 0.03，实得 {depth}");
}

/// ③ 从**侧面**进 `x+` 面：`face` 应为 1、法线 `(1,0,0)`。
#[test]
fn enters_side_face() {
    let (n, depth, q, face) =
        call(Vec3::new(0.325, 1.0, 0.0), Vec3::new(0.305, 1.0, 0.0)).expect("该判为穿过 +x 面");
    assert_eq!(face, 1, "面号应为 x+（1），实得 {face}");
    assert!(n.x > 0.99, "法线应指 +x，实得 {n:?}");
    assert!((depth - 0.015).abs() < 1e-4, "深度应为 0.015，实得 {depth}");
    assert!((q.x - 0.3).abs() < 1e-4, "交点应在 x=0.3，实得 {}", q.x);
}

/// ④ 粒子**已在体内**（上一步就不在带外）⇒ `None`（调用方退回点式）。
#[test]
fn inside_particle_returns_none() {
    assert!(
        call(Vec3::new(0.0, 0.99, 0.0), Vec3::new(0.0, 0.995, 0.0)).is_none(),
        "已在体内不该判成'穿越'（改由调用方的兜底路径处理）"
    );
}

/// ⑤ 穿过的是某面的**延长平面**（在面外范围内）⇒ 不算该面。
#[test]
fn crossing_outside_face_extent_is_ignored() {
    // 在 y 方向穿越底面所在平面，但 x 远在盒外（|x| > half.x）
    assert!(
        call(Vec3::new(1.0, 0.925, 0.0), Vec3::new(1.0, 0.945, 0.0)).is_none(),
        "面外范围不该算作穿过这张面"
    );
}
