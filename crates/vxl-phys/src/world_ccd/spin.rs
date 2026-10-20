//! CCD 的 **S2 目标子步化**（默认关）：快转运动学体的姿态按 `skin` 分辨率分 N 份推进，
//! 每份后做一趟窄相 + 位置级推出 ⇒ 重叠不在两次解算之间形成（P18/P21）。
//!
//! 为什么单独一个文件：`world_ccd.rs` 在 god 门的行数棘轮上（91 行），整块塞进去是 +56%。
use crate::{Quat, Vec3, World};
use vxl_phys_narrow::NarrowPhase;

/// 一个"快转运动学体"在本子步的子步化计划：`q0` = 积分前姿态，`omega` = 本子步要走的角速度。
pub(crate) struct SpinSave {
    id: usize,
    q0: Quat,
    omega: Vec3,
    steps: u32,
}

/// **积分前**挑出"每子步角位移 > skin"的运动学体并暂时清零其角速度（位置积分不转它们）。
/// 返回的计划交给 [`spin_advance`] 在积分后按 N 份补上。
pub(crate) fn spin_prepass(world: &mut World, dt: f32) -> (Vec<SpinSave>, Vec<Vec3>) {
    if !world.config.ccd_target_substep || !dt.is_finite() || dt <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    let skin = world.config.contact_skin;
    if !skin.is_finite() || skin <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    // 积分前的位置快照：`push_out` 用它决定「从哪一侧进 blades 的」，避免法线退化时来回推。
    let pre: Vec<Vec3> = (0..world.bodies.len())
        .map(|k| world.bodies.position[k])
        .collect();
    let mut out = Vec::new();
    for i in 0..world.bodies.len() {
        if !world.bodies.is_kinematic(i) {
            continue;
        }
        let omega = world.bodies.angvel(i);
        let r = vxl_phys_solver::ccd::bounding_radius(&world.bodies.shape[i]);
        if omega == Vec3::ZERO || !r.is_finite() || r <= 0.0 {
            continue;
        }
        let tip = omega.length() * r * dt;
        if tip <= skin {
            continue;
        }
        out.push(SpinSave {
            id: i,
            q0: world.bodies.rot(i),
            omega,
            steps: (tip / skin).ceil().clamp(1.0, 32.0) as u32,
        });
        world.bodies.set_angvel_raw(i, Vec3::ZERO);
    }
    (out, pre)
}

/// **积分后**把停下不转的运动学体按 `N` 份补转，每份后做一趟窄相 + 位置级推出。
pub(crate) fn spin_advance(world: &mut World, dt: f32, spins: &[SpinSave], pre: &[Vec3]) {
    for s in spins {
        world.bodies.set_angvel_raw(s.id, s.omega);
        for k in 1..=s.steps {
            let t = dt * k as f32 / s.steps as f32;
            world
                .bodies
                .set_rot(s.id, s.q0.integrate_angular(s.omega, t));
            resolve_spin(world, s.id, pre);
        }
    }
}

/// 把与运动学体 `i` 重叠的**动体**沿法线推出到表面（位置级）。
///
/// 目的不是"求一个正确的冲量"，而是**不让重叠留到下一个子步**：本函数只在子步化档里被调用，
/// 次数 = ΣN，且只对邻近动体做（中心距预筛）。关节/传导约束不在这里处理（P17 的投影另有入口）。
fn resolve_spin(world: &mut World, i: usize, pre: &[Vec3]) {
    let skin = world.config.contact_skin;
    let pi = world.bodies.position[i];
    let ri = vxl_phys_solver::ccd::bounding_radius(&world.bodies.shape[i]);
    let mut pairs: Vec<(u32, u32)> = Vec::new();
    for j in 0..world.bodies.len() {
        if j == i || !world.bodies.is_dynamic(j) {
            continue;
        }
        let rj = vxl_phys_solver::ccd::bounding_radius(&world.bodies.shape[j]);
        if (world.bodies.position[j] - pi).length() > ri + rj + 2.0 * skin {
            continue;
        }
        let (a, b) = if j < i {
            (j as u32, i as u32)
        } else {
            (i as u32, j as u32)
        };
        pairs.push((a, b));
    }
    if pairs.is_empty() {
        return;
    }
    for _ in 0..2 {
        world.narrow.set_predict_dt(0.0);
        world.narrow.collide(
            &world.bodies,
            &pairs,
            world.terrain.slice(),
            &world.providers,
            &mut world.ccd_manifolds,
            world.jobs.as_ref(),
        );
        push_out(world, i, pre);
    }
}

/// 把 `ccd_manifolds` 里与 `i` 重叠的动体推出去（[`resolve_spin`] 的单趟）。
fn push_out(world: &mut World, i: usize, pre: &[Vec3]) {
    for m in world.ccd_manifolds.iter() {
        let other = if m.a == i as u32 { m.b } else { m.a } as usize;
        let depth = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        if depth <= 0.0 {
            continue;
        }
        let n = if m.a == i as u32 { m.normal } else { -m.normal };
        let before = world.bodies.position[other];
        // 法线退化（动体中心正落在目标中面上）时，`n` 的符号没有意义 ⇒ 按**积分前所在侧**定方向：
        // 从 -n 侧进来的，就推回 -n 侧（`sep + depth` 才能穿回原位，而不是只推 `depth` 留在里面）。
        let sep = (before - world.bodies.position[i]).dot(n);
        let came = pre
            .get(other)
            .map_or(sep, |p| (*p - world.bodies.position[i]).dot(n));
        let push = if came * sep < 0.0 {
            -n * (sep.abs() + depth)
        } else {
            n * depth
        };
        world.bodies.position[other] += push;
        world.bodies.wake(other);
    }
}
