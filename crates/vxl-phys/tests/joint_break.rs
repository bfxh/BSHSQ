//! 关节**可断裂**的判据（ROUTE M6「约束族（可断裂/限位）」里"可断裂"那一半）。
//!
//! 语义（照抄 [`vxl_phys::Joint::with_motor`] 的 "`max_force <= 0` = 关" 约定）：
//! `break_force <= 0` ⇒ **不断**、逐位等价于没有这个功能；`> 0` ⇒ 该关节的约束力超过它
//! 后**永久**断开、之后不再施加任何约束。
//!
//! 判据用**力**（本子步最大冲量 / `dt`）而不是几何拉伸：这个求解器把位置约束做得极硬
//! （实测给 5 m/s 初速，锚点间隙也只涨不到 0.01 m）⇒ 拉伸量没有可用的量级。
//!
//! ⚠️ 另一个踩过的坑：**两端都是自由动态体**的距离关节在无重力下会发散
//! （实测间距 0.8 → 25.6）⇒ 本档用静态锚点 + 动态球。
//!
//! `rest` 仍取**初始距离**，但**不再**是必需的：`rest ≠ 初始距离` 的那个缺陷已于 2026-10-17
//! 修好（根因是约束行的冲量符号写反，迭代在发散；回归判据见本文件末尾的
//! `distance_rest_differs_from_initial_distance_converges`）。
// 新文件：glob 门对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};

/// 静态锚点 + 动态球（`rest` = 初始距离 0.5），给球朝外的初速度 `speed` 制造约束力。
fn scene(speed: f32, force: f32) -> (World, u32, u32) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let anchor = w.add_static(Shape::Sphere { radius: 0.05 }, Vec3::ZERO, Quat::IDENTITY);
    let body = w.add_dynamic(
        Shape::Sphere { radius: 0.05 },
        Vec3::new(0.5, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.bodies.linvel[body as usize] = Vec3::new(speed, 0.0, 0.0);
    let joint =
        Joint::new(JointKind::Distance, anchor, body, Vec3::ZERO, Vec3::ZERO).with_rest(0.5);
    let j = w.joints.add_breakable(joint, force);
    (w, body, j)
}

/// 球到锚点的距离（锚点在原点、锚点偏移为零 ⇒ 直接量位移）。
fn gap_of(w: &World, body: u32) -> f32 {
    w.bodies.position[body as usize].length()
}

/// `break_force = 0`（默认）⇒ 无论约束力多大都不断：这是"没有这个功能"的等价性保证。
#[test]
fn zero_threshold_never_breaks() {
    let (mut w, _b, j) = scene(5.0, 0.0);
    for _ in 0..60 {
        w.step();
    }
    assert!(
        !w.joints.is_broken(j),
        "break_force=0 必须逐位等价于「无此功能」，实测却断了"
    );
    assert_eq!(w.joints.broken_count(), 0);
}

/// 约束力超阈 ⇒ 断；且**永久**（再跑很多步也不恢复）。
#[test]
fn force_beyond_threshold_breaks_and_stays_broken() {
    // 球质量 ≈ 0.52 kg、初速 5 m/s ⇒ 冲量 ≈ 2.6 N·s ⇒ 力 ≈ 156 N（dt = 1/60）。
    // 阈值 10 N 远小于它 ⇒ 必然断开。
    let (mut w, _b, j) = scene(5.0, 10.0);
    for _ in 0..60 {
        w.step();
    }
    assert!(
        w.joints.is_broken(j),
        "约束力 ≈156 N 远超阈值 10 N，应当断开"
    );
    for _ in 0..60 {
        w.step();
    }
    assert!(w.joints.is_broken(j), "断开是永久的（不自动恢复）");
    assert_eq!(w.joints.broken_count(), 1);
}

/// 阈内 ⇒ 不断，且关节仍在起约束作用（球被拉住，而不是飞走）。
#[test]
fn within_threshold_holds() {
    let (mut w, b, j) = scene(5.0, 10_000.0);
    for _ in 0..60 {
        w.step();
    }
    assert!(
        !w.joints.is_broken(j),
        "阈值 1e4 N 远大于任何约束力，不该断"
    );
    let d = gap_of(&w, b);
    assert!(
        d < 1.5,
        "关节应把球拉住（无阻尼 ⇒ 振荡，但远不该飞到 {d} m）"
    );
}

/// 断后不再施加约束：再推一把朝外，间距应当**继续增大**（若约束还在，会被拉回去）。
#[test]
fn broken_joint_exerts_no_more_constraint() {
    let (mut w, b, j) = scene(5.0, 10.0);
    for _ in 0..60 {
        w.step();
    }
    assert!(w.joints.is_broken(j));
    let bi = b as usize;
    w.bodies.linvel[bi] = Vec3::new(1.0, 0.0, 0.0);
    let before = gap_of(&w, b);
    for _ in 0..30 {
        w.step();
    }
    let after = gap_of(&w, b);
    assert!(
        after > before,
        "断开后仍被约束回缩：间距 {before} → {after}（应当增大）"
    );
}

/// **`rest ≠ 初始距离` 必须收敛到 `rest`**（回归：修好之前的约束行冲量符号写反，
/// 第一次迭代后 `v·n` 与偏置同号 ⇒ 迭代发散 —— 实测 `v·n` 序列 `0 → −7.2 → −21.6 →
/// −50.4 → … → −914`，间距一步从 `0.8` 砸到 `0.0129`、第二 tick 到 `0` 后再也不动）。
///
/// 现在：每 tick 消除约 `β = 0.2` 的误差，单调逼近 `rest`。
#[test]
fn distance_rest_differs_from_initial_distance_converges() {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let anchor = w.add_static(Shape::Sphere { radius: 0.05 }, Vec3::ZERO, Quat::IDENTITY);
    let body = w.add_dynamic(
        Shape::Sphere { radius: 0.05 },
        Vec3::new(0.8, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    // 初始间距 0.8、目标 0.5 ⇒ 必须**缩短**，而不是被压到 0。
    w.joints
        .add(Joint::new(JointKind::Distance, anchor, body, Vec3::ZERO, Vec3::ZERO).with_rest(0.5));
    for _ in 0..60 {
        w.step();
    }
    let d = gap_of(&w, body);
    assert!(
        (d - 0.5).abs() < 0.02,
        "间距应收敛到 rest=0.5，实得 {d}（不收敛或掉到 0 都算失败）"
    );
    assert!(d > 0.4, "间距被压塌了：{d}");
}
