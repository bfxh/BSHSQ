//! 运行期 **NaN 隔离**（安全审计 F-08 第 3 点）。
//!
//! 审计读数：一具 NaN 速度的体放在 5 箱堆里 ⇒ **5 具全污染**；`dt`/`gravity` 为 NaN ⇒ 全世界
//! 7/7 NaN。F-08 的前两点（入口收口）在 2026-10 的两轮里修掉了：`PhysConfig` 的 `dt`/重力在
//! 门面构造时收口（`world_build/config_guard.rs`）。剩下的是**运行期**那一半：体状态在跑动
//! 中被写脏（`set_linvel(NaN)`、外力数值爆掉、用户直接改 `bodies` 数组）。
//!
//! **隔离语义（本件定的口径，写清不是漏）**：
//!
//! - **就地冻结这一子步**：发现 `position` / 线速度 / 角速度 / 姿态有非有限分量 ⇒ 速度清零、
//!   姿态与位置里的非有限分量钳成**有限**（位置分量→0、四元数→单位），并置 `awake = false`。
//!   这既挡住了对邻居的污染（窄相/解算器不再看到 NaN 法线或 NaN 深度），也不改动体的
//!   `inv_mass`。
//! - **不做永久隔离**：故意**不**把 `inv_mass` 清零——那会把该体变成一个"隐形墙"（本仓在睡眠
//!   门那条线上专门讨论过这个病理）。调用方用 `set_linvel`（会唤醒）给回一个有限速度即可恢复。
//! - **观测口径**（判据只钉稳定可观测的那几条）：隔离发生在**子步开头**，所以同一 tick 的
//!   物理会在之后照常推进该体 ⇒ 它的速度/`awake` **不是**稳定读数。稳定读数是：
//!   `World::health().nan_bodies == 0`（跑过 step 之后）、全世界有限、`inv_mass` 未变。
//!   判据 `tests/nan_guard.rs` 把这三条钉住（第一版判据错在把"被冻结当 tick 的速度为零"
//!   当成稳定读数，实测该体在 tick 末已按物理正常下落）。
//! - **确定性**：逐体按体号升序、只做"有限化"与清零，不引入任何依赖平台的分支。

use vxl_phys_core::{Quat, Vec3};

use crate::World;

/// 位置 / 姿态 / 两侧速度里是否有非有限分量。
fn dirty(world: &World, i: usize) -> bool {
    let p = world.bodies.position[i];
    let v = world.bodies.linvel[i];
    let w = world.bodies.angvel(i);
    let q = world.bodies.rot(i);
    !(p.is_finite()
        && v.is_finite()
        && w.is_finite()
        && q.x.is_finite()
        && q.y.is_finite()
        && q.z.is_finite()
        && q.w.is_finite())
}

/// 有限化：非有限分量→0（四元数→单位四元数、位置→0），保证下游看到的全是有限值。
fn finitize(v: Vec3) -> Vec3 {
    let f = |x: f32| if x.is_finite() { x } else { 0.0 };
    Vec3::new(f(v.x), f(v.y), f(v.z))
}

fn finitize_quat(q: Quat) -> Quat {
    if q.x.is_finite() && q.y.is_finite() && q.z.is_finite() && q.w.is_finite() {
        q
    } else {
        Quat::IDENTITY
    }
}

/// 本子步开头跑一次：把脏体冻结/有限化（顺序 = 体号升序 ⇒ 确定性）。
pub(crate) fn guard(world: &mut World) {
    for i in 0..world.bodies.len() {
        if !dirty(world, i) {
            continue;
        }
        let p = world.bodies.position[i];
        let q = world.bodies.rot(i);
        world.bodies.position[i] = finitize(p);
        world.bodies.set_rot(i, finitize_quat(q));
        world.bodies.linvel[i] = Vec3::ZERO;
        world.bodies.set_angvel_raw(i, Vec3::ZERO);
        world.bodies.awake[i] = false;
    }
}
