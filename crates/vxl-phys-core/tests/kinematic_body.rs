//! **运动学体的核心不变式**（M2 · `docs/PLAN-kinematic.md` K1）。
//!
//! 契约（三条，缺一条都会让上层静默错）：
//! 1. **质量/惯量恒零** ⇒ 不受力、不被接触冲量改写（A 档里它进岛、速度被 gather 进求解缓冲，
//!    但冲量施加是空操作 —— 这正是"平台速度看得见、平台本身推不动"）；
//! 2. **谓词三分**：`Static` / `Dynamic` / `Kinematic` 在 `is_dynamic`、`is_kinematic`、
//!    `is_dynamic_or_kinematic` 三个问法上的取值表（上层按这张表分流）；
//! 3. **唤醒语义**：它是"会动的"，构造即醒，且永远不由引擎入睡
//!    （入睡规则在 `vxl-phys-solver` 的 `sleep_pass`，这里只钉"醒着 + 速度可写"）。
use vxl_phys_core::{BodySet, BodyType, Quat, Shape, Vec3};

#[test]
fn kinematic_is_a_massless_mover() {
    let mut b = BodySet::new();
    let half = Shape::Box {
        half: Vec3::splat(0.5),
    };
    let k = b.push_kinematic(half, Vec3::Y, Quat::IDENTITY) as usize;
    let s = b.push_static(half, Vec3::ZERO, Quat::IDENTITY) as usize;
    let d = b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::ZERO,
        Quat::IDENTITY,
        1.0,
    ) as usize;

    // 谓词三分表（上层按它分流：受力的地方用 is_dynamic、会动/进岛的用 is_dynamic_or_kinematic）。
    assert_eq!(b.body_type[k], BodyType::Kinematic);
    assert!(b.is_kinematic(k) && !b.is_dynamic(k) && b.is_dynamic_or_kinematic(k));
    assert!(!b.is_kinematic(s) && !b.is_dynamic(s) && !b.is_dynamic_or_kinematic(s));
    assert!(!b.is_kinematic(d) && b.is_dynamic(d) && b.is_dynamic_or_kinematic(d));

    // 质量/惯量恒零 —— 这是"推不动"的全部来源。
    assert_eq!(b.inv_mass[k], 0.0);
    assert_eq!(b.local_inv_inertia[k], Vec3::ZERO);

    // 速度可写（宿主输入），但**冲量不能改写它**。
    b.set_linvel(k, Vec3::new(1.0, 0.0, 0.0));
    b.apply_impulse(k, Vec3::new(1.0e3, 0.0, 0.0), Vec3::Y);
    assert_eq!(
        b.linvel[k],
        Vec3::new(1.0, 0.0, 0.0),
        "运动学体不被冲量改写"
    );
    assert_eq!(b.angvel(k), Vec3::ZERO, "运动学体不因冲量获得角速度");

    // 构造即醒：它是"会动的"（睡眠侧只由求解器的 sleep_pass 决定，且那里对运动学体直接跳过）。
    assert!(b.awake[k], "运动学体构造即醒");
    b.wake(k);
    assert!(b.awake[k]);
}

/// **静态/动态的字节流不受影响**（零换代的半个判据）：新标签只在运动学体上出现。
#[test]
fn kinematic_label_does_not_touch_static_or_dynamic() {
    let mut b = BodySet::new();
    let half = Shape::Box {
        half: Vec3::splat(0.5),
    };
    let s = b.push_static(half, Vec3::ZERO, Quat::IDENTITY) as usize;
    let d = b.push_dynamic(half, Vec3::ZERO, Quat::IDENTITY, 1.0) as usize;
    assert_eq!(b.body_type[s], BodyType::Static);
    assert_eq!(b.body_type[d], BodyType::Dynamic);
    assert!(b.is_dynamic(d) && b.inv_mass[d] > 0.0);
}
