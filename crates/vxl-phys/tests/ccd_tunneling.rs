//! **选择性 CCD 的穿透判据**（安全审计 F-15 的三条）。
//!
//! 审计的三条实测（`main@51278a5`，探针原文）：
//!
//! | 场景 | 旧行为 |
//! |---|---|
//! | ① 默认档（CCD 关）、2000 m/s 球 × 0.1 m 墙 | **穿**（速度被 `max_linear_velocity` 钳到 100 m/s，仍 1.67 m/tick ≫ 0.1 m） |
//! | ② 开 CCD、3000 m/s | 仍**穿**（段数被静默截到 `ccd_max_steps` = 64 ⇒ 段长 0.78 m > 墙厚） |
//! | ③ 开 CCD、目标**在运动**（移动墙） | 必**穿**（目标集只留"静态/沉睡"） |
//! | ④ 对照：开 CCD、静态墙 | 停在墙前 ✅ |
//!
//! 本文件把五条都钉住：① 是**文档化的边界**（默认关；改默认值必须同时改这里与模块头注），
//! ①′ 是**关档的语义**（无穿透保证，且**取决于采样相位** —— 给出必然穿的对齐相位，
//! 反面的"被相位接住"写在头注里），②③ 是 F-15 修好的（限速到预算覆盖得了的速度；
//! 目标集含运动体 + 相对运动判据），④ 静态墙对照。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 薄墙（0.1 m 厚）+ 远端的球（z = −40）。`wall_moves` ⇒ 墙是**动态体**并以 50 m/s 迎面开来。
fn scene(speed: f32, ccd: bool, max_vel: f32, wall_moves: bool) -> (World, usize, usize) {
    let cfg = PhysConfig {
        ccd_speed_threshold: if ccd { 5.0 } else { f32::INFINITY },
        max_linear_velocity: max_vel,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let wall = if wall_moves {
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::new(2.0, 2.0, 0.05),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
            1000.0,
        );
        w.bodies.linvel[id as usize] = Vec3::new(0.0, 0.0, -50.0);
        id
    } else {
        w.add_static(
            Shape::Box {
                half: Vec3::new(2.0, 2.0, 0.05),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        )
    };
    let ball = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.05),
        },
        Vec3::new(0.0, 0.0, -40.0),
        Quat::IDENTITY,
        2000.0,
    );
    w.bodies.linvel[ball as usize] = Vec3::new(0.0, 0.0, speed);
    (w, ball as usize, wall as usize)
}

/// **① 默认档 = 文档化的边界**：`ccd_speed_threshold` 默认 `INFINITY`（关）。审计实测：默认档下
/// 2000 m/s 的球（被 `max_linear_velocity` 钳到 100 m/s ⇒ 1.67 m/tick，是 0.1 m 墙厚的 16 倍）
/// **会**穿过薄墙；本仓同一档位下是否穿还取决于**相位**（子步落点 vs 墙带宽度）——所以这条
/// 判据只钉"默认档没有穿透保证"这个**决策点**：谁要把 CCD 改成默认开，必须同时改这里与
/// `world_ccd/sweep.rs`、`vxl-phys-solver/src/ccd.rs` 的头注。
/// ⚠️ **2026-10-13 实测：默认开已被自己的门挡下**（改成 10 m/s 后 R6 关节链的分离 13× 恶化，
/// `sep_p99` 0.00966 → 0.12382、>5cm 违规率 0 → 1.97%）⇒ 默认保持关，互锁记在
/// `OPEN-PROBLEMS.md` P17，快照判据见 `tests/ccd_joint_interlock.rs`。
#[test]
fn ccd_is_off_by_default_and_that_is_documented() {
    assert!(
        PhysConfig::default().ccd_speed_threshold.is_infinite(),
        "默认档 CCD 是关的（互锁未解，见 OPEN-PROBLEMS P17）；改默认值要一并改文档与判据"
    );
}

/// **①′ 关档 = 无穿透保证（取决于相位）**：这里把起点**对齐到采样格点** —— 速度被钳到 100 m/s、
/// 2 子步 ⇒ 每子步走 `100/120 = 0.8333 m`；起点 `z = −0.3` ⇒ 采样序列 `−0.3, +0.533, …`
/// **跨过**墙带（墙 `±0.05` + 球 `±0.05` + skin `0.02` ⇒ `±0.12`）而没有一点落进去 ⇒ **必然穿**。
/// ⚠️ 反面对照（起点 z = −40 的同款场景）实测**被相位接住**（`z = −5.0` 那一步恰好落进带内
/// ⇒ 弹回 `vz = −4.58`）—— 两端都是实测，"有无保证全看相位"从此是钉住的事实。
#[test]
fn ccd_off_has_no_guarantee_depends_on_phase() {
    let cfg = PhysConfig {
        ccd_speed_threshold: f32::INFINITY, // 关档
        max_linear_velocity: 100.0,
        gravity: Vec3::ZERO, // 只留 z 向运动（排除重力把球带离墙的 y 范围）
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(2.0, 2.0, 0.05),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let ball = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.05),
        },
        Vec3::new(0.0, 0.0, -0.3),
        Quat::IDENTITY,
        2000.0,
    ) as usize;
    w.bodies.linvel[ball] = Vec3::new(0.0, 0.0, 100.0);
    for _ in 0..30 {
        w.step();
    }
    let z = w.bodies.position[ball].z;
    assert!(
        z > 1.0,
        "关档 + 采样点全部越过墙带 ⇒ 必然穿（z = {z}）；若红了先查采样对齐是否被改动"
    );
}

/// **② 采样预算**：3000 m/s 的位移远超 `ccd_max_steps`（64）× 半段长 ⇒ 旧实现静默截段后仍穿。
/// 现在限速到预算覆盖得了的速度 ⇒ 必须停在墙前。
#[test]
fn ccd_covers_the_sampling_budget() {
    let (mut w, ball, _) = scene(3000.0, true, 5000.0, false);
    for _ in 0..240 {
        w.step();
    }
    let z = w.bodies.position[ball].z;
    assert!(
        z < 0.0,
        "开 CCD 后必须停在墙前（预算不够时应限速而不是静默穿透），z = {z}"
    );
    let sp = w.bodies.linvel[ball].length();
    assert!(sp <= 250.0, "限速后速度应落在预算可行域内（实际 {sp}）");
}

/// **③ 运动目标**：移动墙（动态体，50 m/s 迎面）——旧实现把动态目标整条排除 ⇒ 必穿。
/// 现在按相对运动采样与判定 ⇒ 两体都不许穿过对方。
#[test]
fn ccd_covers_moving_targets() {
    let (mut w, ball, wall) = scene(3000.0, true, 5000.0, true);
    for _ in 0..240 {
        w.step();
    }
    let (bz, wz) = (w.bodies.position[ball].z, w.bodies.position[wall].z);
    assert!(
        bz < wz,
        "球不得穿过迎面开来的墙（球 z = {bz}、墙 z = {wz}）"
    );
}

/// **④ 对照**：默认速度钳位（100 m/s）+ 开 CCD ⇒ 静态薄墙前停下（审计的 ✅ 用例）。
#[test]
fn ccd_on_static_wall_stops() {
    let (mut w, ball, _) = scene(3000.0, true, 100.0, false);
    for _ in 0..240 {
        w.step();
    }
    let z = w.bodies.position[ball].z;
    assert!(z < 0.0, "开 CCD 时静态薄墙必须挡住（z = {z}）");
}
