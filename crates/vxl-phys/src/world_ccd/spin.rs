//! CCD 的 **S2 目标子步化**（**默认开**，2026-10-21）：快转运动学体的姿态按 `skin` 分辨率分 N 份推进，
//! 每份后做一趟窄相 + 位置级推出 ⇒ 重叠不在两次解算之间形成（P18/P21）。
//!
//! **读数**（自旋薄片档：运动学旋翼 ω=20 rad/s × 60 m/s 弹体、24 相位）：命中样本最深穿透
//! **0.15948 → 0.01711 m（< skin 0.02）**、命中 126 → 99、二分 1008 → 792、采样 268706 → 291642。
//!
//! ⚠️ **服从 CCD 总开关**：`ccd_speed_threshold` 非有限（CCD 关档）⇒ 本件整体不生效 ⇒
//! "CCD 关 = 无穿透保证"（`ccd_speed_threshold` 的注释口径）不被本件悄悄破掉，且关档逐位不变。
//!
//! 为什么单独一个文件：`world_ccd.rs` 在 god 门的行数棘轮上（91 行），整块塞进去是 +56%。
use crate::{PhysConfig, Quat, Vec3, World};
use vxl_phys_narrow::NarrowPhase;

/// **S2 是否生效**：开关开 **且** CCD 总开关开。
///
/// CCD 总开关 = `ccd_speed_threshold` 有限（与 `sweep.rs` / `ccd.rs` 同一判据）：CCD 关档时
/// 本件整体不生效 ⇒ "CCD 关 = 无穿透保证" 的口径不被破掉，且关档逐位不变。
pub(crate) fn s2_enabled(cfg: &PhysConfig) -> bool {
    cfg.ccd_target_substep && cfg.ccd_speed_threshold.is_finite()
}

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
    if !s2_enabled(&world.config) || !dt.is_finite() || dt <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    let skin = world.config.contact_skin;
    if !skin.is_finite() || skin <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    // ① **先扫"快转运动学体"**（不分配、不快照）。默认档下本件对**每个场景每个子步**都跑 ⇒
    //    一上来就快照全体位置 = 10 万体场景每子步拷 10 万个位置（1.2 MB）—— 那才是真代价。
    //    没有快转体 ⇒ 直接返回（**零分配**）；快转体存在时才付 ② 的快照钱。
    let mut out = Vec::new();
    for (i, ty) in world.bodies.body_type.iter().enumerate() {
        if *ty != vxl_phys_core::BodyType::Kinematic {
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
    }
    if out.is_empty() {
        return (Vec::new(), Vec::new());
    }
    // ② 积分前的位置快照：`push_out` 用它决定「从哪一侧进 blades 的」，避免法线退化时来回推。
    //    **只在真有快转体时才付这份钱**（快照 = 全体位置 ⇒ 与体数同阶）。
    let pre: Vec<Vec3> = (0..world.bodies.len())
        .map(|k| world.bodies.position[k])
        .collect();
    for s in &out {
        world.bodies.set_angvel_raw(s.id, Vec3::ZERO);
    }
    (out, pre)
}

/// **积分后**把停下不转的运动学体按 `N` 份补转，每份后做一趟窄相 + 位置级推出。
pub(crate) fn spin_advance(world: &mut World, dt: f32, spins: &[SpinSave], pre: &[Vec3]) {
    if spins.is_empty() {
        return;
    }
    // **候选只收集一次**（每个旋翼一份）：子步内位置/姿态都不变（只有动体被推出会挪位，
    // 而推出只会把动体挪开 ⇒ 不会有"新进"的候选）⇒ 按子步重复扫全体体是纯浪费：
    // 8 旋翼 × N=5 份 × 2 万体 = 16 万次距离判定/子步（实测墙钟 +135%）。
    let pairs = spin_candidates(world, spins);
    for (k, s) in spins.iter().enumerate() {
        world.bodies.set_angvel_raw(s.id, s.omega);
        for step in 1..=s.steps {
            let t = dt * step as f32 / s.steps as f32;
            world
                .bodies
                .set_rot(s.id, s.q0.integrate_angular(s.omega, t));
            resolve_spin(world, s.id, &pairs[k], pre);
        }
    }
}

/// 收集每个快转运动学体附近的**动体对**（**一趟扫全体体**：每个体的包围球半径只算一次）。
fn spin_candidates(world: &World, spins: &[SpinSave]) -> Vec<Vec<(u32, u32)>> {
    let skin = world.config.contact_skin;
    let mut out: Vec<Vec<(u32, u32)>> = spins.iter().map(|_| Vec::new()).collect();
    let centers: Vec<Vec3> = spins.iter().map(|s| world.bodies.position[s.id]).collect();
    let radii: Vec<f32> = spins
        .iter()
        .map(|s| vxl_phys_solver::ccd::bounding_radius(&world.bodies.shape[s.id]))
        .collect();
    for j in 0..world.bodies.len() {
        if !world.bodies.is_dynamic(j) {
            continue;
        }
        let rj = vxl_phys_solver::ccd::bounding_radius(&world.bodies.shape[j]);
        let pj = world.bodies.position[j];
        for (k, s) in spins.iter().enumerate() {
            if s.id == j {
                continue;
            }
            if (pj - centers[k]).length() > radii[k] + rj + 2.0 * skin {
                continue;
            }
            let (a, b) = if j < s.id {
                (j as u32, s.id as u32)
            } else {
                (s.id as u32, j as u32)
            };
            out[k].push((a, b));
        }
    }
    out
}

/// 把与运动学体 `i` 重叠的**动体**沿法线推出到表面（位置级）。
///
/// 目的不是"求一个正确的冲量"，而是**不让重叠留到下一个子步**：本函数只在子步化档里被调用，
/// 次数 = ΣN，且只对邻近动体做（中心距预筛）。关节/传导约束不在这里处理（P17 的投影另有入口）。
fn resolve_spin(world: &mut World, i: usize, pairs: &[(u32, u32)], pre: &[Vec3]) {
    if pairs.is_empty() {
        return;
    }
    for _ in 0..2 {
        world.narrow.set_predict_dt(0.0);
        world.narrow.collide(
            &world.bodies,
            pairs,
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

#[cfg(test)]
mod tests {
    //! **S2（快转目标子步化）的生效判据**（2026-10-21，S2 提为**默认档**）。
    //!
    //! S2 只在「开关开 **且** CCD 总开关开」时生效：CCD 关档（`ccd_speed_threshold` 非有限）
    //! ⇒ S2 整体不生效 ⇒ "CCD 关 = 无穿透保证"（`ccd_speed_threshold` 的注释口径）不被破掉，
    //! 且关档**逐位不变**。
    //!
    //! | 档 | `ccd_target_substep` | `ccd_speed_threshold` | S2 计划 |
    //! |---|---|---|---|
    //! | 默认档 | **true** | 10.0 | 有（快转运动学体） |
    //! | 显式关 S2（回退档） | false | 10.0 | 无 |
    //! | CCD 关档 | true | INFINITY | **无**（总开关压住） |
    //!
    //! 判据直接打在**机制层**（`spin_prepass` 排不排计划、`ω` 有没有被清零）⇒ 不依赖场景是否
    //! 恰好重叠，不会变成"空断言"。

    use super::{spin_prepass, PhysConfig, Quat, Vec3, World};
    use crate::{KinematicExt, Shape};

    /// 造一个「快转旋翼 + 子弹」的世界（`tip = ω·r·dt = 20·0.6/60 = 0.2 m > skin 0.02` ⇒ 会触发）。
    fn spinner_world(cfg: PhysConfig) -> (World, usize) {
        let mut w = World::new(cfg);
        let blade = w.add_kinematic(
            Shape::Box {
                half: Vec3::new(0.6, 0.4, 0.02),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let id = blade as usize;
        w.bodies.set_angvel(id, Vec3::new(0.0, 20.0, 0.0));
        (w, id)
    }

    /// **默认档：S2 生效** —— 快转运动学体拿到子步化计划，且 `ω` 在预通道里被暂时清零
    /// （位置积分不转它 ⇒ 旋转被推迟到积分后按 N 份补上）。
    #[test]
    fn default_tier_engages_s2() {
        assert!(
            PhysConfig::default().ccd_target_substep,
            "默认档必须含 S2（2026-10-21 起默认开）"
        );
        let (mut w, id) = spinner_world(PhysConfig::default());
        let (spins, _pre) = spin_prepass(&mut w, 1.0 / 60.0);
        assert_eq!(spins.len(), 1, "默认档必须给快转运动学体排一条子步化计划");
        assert_eq!(
            w.bodies.angvel(id),
            Vec3::ZERO,
            "预通道必须先清零 ω（旋转推迟到积分后）"
        );
    }

    /// **回退档（`ccd_target_substep = false`）：不生效** —— 无计划、`ω` 不动。
    #[test]
    fn s2_off_engages_nothing() {
        let cfg = PhysConfig {
            ccd_target_substep: false,
            ..PhysConfig::default()
        };
        let (mut w, id) = spinner_world(cfg);
        let (spins, _pre) = spin_prepass(&mut w, 1.0 / 60.0);
        assert!(spins.is_empty(), "显式关 S2 ⇒ 不得排计划");
        assert_ne!(w.bodies.angvel(id), Vec3::ZERO, "S2 关 ⇒ 预通道不得动 ω");
    }

    /// **CCD 关档：S2 不生效**（总开关压住）—— 无计划、`ω` 不动 ⇒ "CCD 关 = 无穿透保证" 的
    /// 口径不被本件破掉（CCD 关档用户不会白付子步化的钱，也不会看到位置级推出）。
    #[test]
    fn ccd_off_disengages_s2() {
        let cfg = PhysConfig {
            ccd_speed_threshold: f32::INFINITY,
            ..PhysConfig::default()
        };
        let (mut w, id) = spinner_world(cfg);
        let (spins, _pre) = spin_prepass(&mut w, 1.0 / 60.0);
        assert!(spins.is_empty(), "CCD 关档 ⇒ S2 不得排计划");
        assert_ne!(w.bodies.angvel(id), Vec3::ZERO, "CCD 关档 ⇒ 预通道不得动 ω");
    }
}
