//! probes_machine：PhysArena「约束与关节」（`joints.ts`）复刻批 —— **机构**这一半
//! （塔吊/活塞/悬索桥/布娃娃堆/齿轮/剪叉/弹簧床）。
//!
//! 拆自 `probes_joint.rs`（2026-10-08：避免单文件超 god 门 800 行）；另一半
//! （绳桥/布娃娃/曲柄滑块/车轮/弹簧网）在 `probes_joint.rs`。
use super::*;

/// arena `crane`（塔身→转台→吊臂→吊索→重物，四级串联）。
pub(crate) fn scene_crane(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 180.0, 0.9);
    let tower_h = 13.0f32;
    let tower = add_static_box(
        &mut w,
        Vec3::new(0.0, tower_h / 2.0, 0.0),
        Vec3::new(0.7, tower_h / 2.0, 0.7),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let slew = add_box_r(
        &mut w,
        Vec3::new(0.0, tower_h + 0.35, 0.0),
        Vec3::new(1.1, 0.35, 1.1),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.4, 0.05),
        4000.0,
    );
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            tower as u32,
            slew as u32,
            Vec3::new(0.0, tower_h + 0.35, 0.0),
            Vec3::ZERO,
        )
        .with_axis(Vec3::Y),
    );
    let mast_y = tower_h + 0.7;
    let jib = add_box_r(
        &mut w,
        Vec3::new(4.5, mast_y + 0.35, 0.0),
        Vec3::new(4.5, 0.35, 0.5),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.5, 0.05),
        2200.0,
    );
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            slew as u32,
            jib as u32,
            Vec3::new(0.0, 0.35, 0.0),
            Vec3::new(-4.5, 0.0, 0.0),
        )
        .with_axis(Vec3::Y),
    );
    add_box_r(
        &mut w,
        Vec3::new(-0.9, mast_y + 0.35, 0.0),
        Vec3::new(0.9, 0.5, 0.5),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.5, 0.05),
        9000.0,
    );
    let load = add_box_r(
        &mut w,
        Vec3::new(8.4, mast_y - 3.6, 0.0),
        Vec3::splat(0.6),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.5, 0.05),
        6000.0,
    );
    w.add_joint(
        Joint::new(
            JointKind::Distance,
            jib as u32,
            load as u32,
            Vec3::new(4.2, 0.0, 0.0),
            Vec3::new(0.0, 0.6, 0.0),
        )
        .with_rest(3.4),
    );
    w
}

/// arena `piston-bank`（4 缸曲柄滑块并联，零重力 + 马达，含棱柱限位）。
pub(crate) fn scene_piston_bank(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.gravity = Vec3::ZERO;
    let mut w = World::new(cfg);
    // arena: ground(120, 1, -6, {friction: 0.8})——顶面在 y = −6。
    add_static_box(
        &mut w,
        Vec3::new(0.0, -7.0, 0.0),
        Vec3::new(60.0, 1.0, 60.0),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.8, 0.05),
    );
    let spacing = 1.6f32;
    let n = (16usize / 4).clamp(1, 6);
    // arena 写的是 `[sin(π/4), 0, 0, cos(π/4)]` = 绕 X **转角 π/2**（分量是半角 π/4）。
    let d90 = std::f32::consts::FRAC_PI_2;
    for i in 0..n {
        let z = (i as f32 - (n as f32 - 1.0) / 2.0) * spacing;
        let crank = add_cyl(
            &mut w,
            Vec3::new(0.0, 3.0, z),
            0.7,
            0.18,
            rot_axis(Vec3::X, d90),
            s2(0.3, 0.05),
            4000.0,
        );
        let anchor = add_static_box(
            &mut w,
            Vec3::new(0.0, 3.0, z),
            Vec3::splat(0.16),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                anchor as u32,
                crank as u32,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .with_axis(Vec3::Z)
            .with_motor(6.0, 4000.0),
        );
        let rod = add_box_r(
            &mut w,
            Vec3::new(1.5, 3.0, z),
            Vec3::new(1.5, 0.14, 0.14),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.05),
            1800.0,
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                crank as u32,
                rod as u32,
                Vec3::new(0.62, 0.0, 0.0),
                Vec3::new(-1.5, 0.0, 0.0),
            )
            .with_axis(Vec3::Z),
        );
        add_static_box(
            &mut w,
            Vec3::new(4.4, 3.0, z),
            Vec3::new(1.4, 0.5, 0.5),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.1, 0.05),
        );
        let piston = add_box_r(
            &mut w,
            Vec3::new(3.6, 3.0, z),
            Vec3::new(0.45, 0.42, 0.42),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.08, 0.05),
            2600.0,
        );
        w.add_joint(
            Joint::new(
                JointKind::Prismatic,
                piston as u32,
                rod as u32,
                Vec3::ZERO,
                Vec3::new(1.5, 0.0, 0.0),
            )
            .with_axis(Vec3::X)
            .with_limits(-1.2, 1.2),
        );
    }
    w
}

/// arena `suspension-span`（悬索桥：主缆 + 吊索 + 桥面 + 载荷）。
pub(crate) fn scene_suspension_span(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 200.0, 0.85);
    let span = 24.0f32;
    let top_y = 11.0f32;
    for x in [-span / 2.0, span / 2.0] {
        add_static_box(
            &mut w,
            Vec3::new(x, top_y / 2.0, 0.0),
            Vec3::new(0.5, top_y / 2.0, 3.0),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let links = (46usize / 2).clamp(8, 28);
    let cable_y = |t: f32| top_y + 0.4 - (t * std::f32::consts::PI).sin() * 3.2;
    let seg_half = span / links as f32 / 2.0;
    let mut cable_ids: Vec<usize> = Vec::with_capacity(links + 1);
    let mut prev: Option<usize> = None;
    for i in 0..=links {
        let t = i as f32 / links as f32;
        let x = -span / 2.0 + t * span;
        let seg = add_box_r(
            &mut w,
            Vec3::new(x, cable_y(t), 0.0),
            Vec3::new(seg_half + 0.02, 0.14, 0.14),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            2400.0,
        );
        cable_ids.push(seg);
        if let Some(p) = prev {
            w.add_joint(
                Joint::new(
                    JointKind::Distance,
                    p as u32,
                    seg as u32,
                    Vec3::new(seg_half, 0.0, 0.0),
                    Vec3::new(-seg_half, 0.0, 0.0),
                )
                .with_rest(0.05),
            );
        }
        prev = Some(seg);
    }
    let deck_segs = (46usize / 3).clamp(6, 20);
    let deck_y = 4.2f32;
    let deck_half = span / deck_segs as f32 / 2.0 - 0.05;
    let mut prev_deck: Option<usize> = None;
    for i in 0..deck_segs {
        let x = -span / 2.0 + 2.0 + (i as f32 / (deck_segs as f32 - 1.0)) * (span - 4.0);
        let slab = add_box_r(
            &mut w,
            Vec3::new(x, deck_y, 0.0),
            Vec3::new(deck_half, 0.16, 2.2),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.75, 0.05),
            3200.0,
        );
        let ci = ((i as f32 / (deck_segs as f32 - 1.0)) * links as f32).round() as usize;
        let rest = (cable_y(ci as f32 / links as f32) - deck_y).max(0.4);
        w.add_joint(
            Joint::new(
                JointKind::Distance,
                cable_ids[ci.min(links)] as u32,
                slab as u32,
                Vec3::new(0.0, -0.14, 0.0),
                Vec3::new(0.0, 0.16, 0.0),
            )
            .with_rest(rest),
        );
        if let Some(p) = prev_deck {
            w.add_joint(
                Joint::new(
                    JointKind::Distance,
                    p as u32,
                    slab as u32,
                    Vec3::new(deck_half, 0.0, 0.0),
                    Vec3::new(-deck_half, 0.0, 0.0),
                )
                .with_rest(0.05),
            );
        }
        prev_deck = Some(slab);
    }
    for i in 0..(46usize / 20) {
        add_box_r(
            &mut w,
            Vec3::new((i as f32 - 0.5) * 2.2, deck_y + 1.6, 0.0),
            Vec3::splat(0.6),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            7000.0,
        );
    }
    w
}

/// arena `ragdoll-pile`（6 具简化布娃娃同点落下堆叠）。
pub(crate) fn scene_ragdoll_pile(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let count = (60usize / 10).clamp(1, 10);
    for d in 0..count {
        let a = (d as f32 / count as f32) * std::f32::consts::TAU;
        let (cx, cz) = (a.cos() * 1.6, a.sin() * 1.6);
        let y0 = 12.0 + d as f32 * 0.05;
        let torso = add_capsule(
            &mut w,
            Vec3::new(cx, y0 + 1.2, cz),
            0.28,
            0.55,
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            900.0,
        );
        let head = add_ball(
            &mut w,
            Vec3::new(cx, y0 + 2.35, cz),
            0.26,
            s2(0.6, 0.05),
            800.0,
        );
        w.add_joint(Joint::new(
            JointKind::Spherical,
            torso as u32,
            head as u32,
            Vec3::new(0.0, 0.55, 0.0),
            Vec3::new(0.0, -0.26, 0.0),
        ));
        for side in [-1.0f32, 1.0] {
            let leg = add_capsule(
                &mut w,
                Vec3::new(cx + side * 0.34, y0, cz),
                0.2,
                0.5,
                vxl_phys_core::Quat::IDENTITY,
                s2(0.6, 0.05),
                800.0,
            );
            w.add_joint(Joint::new(
                JointKind::Spherical,
                torso as u32,
                leg as u32,
                Vec3::new(side * 0.3, -0.55, 0.0),
                Vec3::new(0.0, 0.5, 0.0),
            ));
        }
    }
    w
}

/// arena `gear-train`（6 齿轮，只有第一个带马达，其余靠侧面摩擦传动）。
pub(crate) fn scene_gear_train(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.gravity = Vec3::ZERO;
    let mut w = World::new(cfg);
    add_static_box(
        &mut w,
        Vec3::new(0.0, -9.0, 0.0),
        Vec3::new(60.0, 1.0, 60.0),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.8, 0.05),
    );
    let n = (12usize / 2).clamp(2, 6);
    let radius = 1.2f32;
    // arena 写的是 `[sin(π/4), 0, 0, cos(π/4)]` = 绕 X **转角 π/2**（分量是半角 π/4）。
    let d90 = std::f32::consts::FRAC_PI_2;
    for i in 0..n {
        let x = -((n as f32 - 1.0) / 2.0) * radius * 2.05 + i as f32 * radius * 2.05;
        let wheel = add_cyl(
            &mut w,
            Vec3::new(x, 3.0, 0.0),
            radius,
            0.3,
            rot_axis(Vec3::X, d90),
            s2(0.9, 0.0),
            2600.0,
        );
        let pin = add_static_box(
            &mut w,
            Vec3::new(x, 3.0, 0.0),
            Vec3::splat(0.14),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        let mut j = Joint::new(
            JointKind::Revolute,
            pin as u32,
            wheel as u32,
            Vec3::ZERO,
            Vec3::ZERO,
        )
        .with_axis(Vec3::Z);
        if i == 0 {
            j = j.with_motor(8.0, 6000.0);
        }
        w.add_joint(j);
    }
    w
}

/// arena `scissor-lift`（3 级剪叉：转动关节平行四边形 + 底部棱柱滑槽）。
pub(crate) fn scene_scissor_lift(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.9);
    let levels = (20usize / 6).clamp(1, 4);
    let arm_half = 2.4f32 / 2.0;
    let thick = 0.13f32;
    let mut base_y = 0.3f32;
    let mut prev_top: Option<(usize, usize)> = None;
    for _l in 0..levels {
        let mid_y = base_y + arm_half * 0.72;
        let a = add_box_r(
            &mut w,
            Vec3::new(-arm_half * 0.72, mid_y, -0.6),
            Vec3::new(arm_half, thick, thick),
            rot_axis(Vec3::Z, 0.62),
            s2(0.7, 0.05),
            1600.0,
        );
        let c = add_box_r(
            &mut w,
            Vec3::new(arm_half * 0.72, mid_y, -0.6),
            Vec3::new(arm_half, thick, thick),
            rot_axis(Vec3::Z, -0.62),
            s2(0.7, 0.05),
            1600.0,
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                a as u32,
                c as u32,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .with_axis(Vec3::Y),
        );
        if let Some((pl, pr)) = prev_top {
            w.add_joint(
                Joint::new(
                    JointKind::Revolute,
                    pl as u32,
                    a as u32,
                    Vec3::new(arm_half, 0.0, 0.0),
                    Vec3::new(-arm_half, 0.0, 0.0),
                )
                .with_axis(Vec3::Y),
            );
            w.add_joint(
                Joint::new(
                    JointKind::Revolute,
                    pr as u32,
                    c as u32,
                    Vec3::new(arm_half, 0.0, 0.0),
                    Vec3::new(-arm_half, 0.0, 0.0),
                )
                .with_axis(Vec3::Y),
            );
        } else {
            let foot_a = add_static_box(
                &mut w,
                Vec3::new(-arm_half * 1.5, 0.2, -0.6),
                Vec3::new(0.5, 0.2, 0.5),
                vxl_phys_core::Quat::IDENTITY,
                ARENA_DEFAULT,
            );
            w.add_joint(
                Joint::new(
                    JointKind::Revolute,
                    foot_a as u32,
                    a as u32,
                    Vec3::ZERO,
                    Vec3::new(-arm_half, 0.0, 0.0),
                )
                .with_axis(Vec3::Y),
            );
            let foot_c = add_static_box(
                &mut w,
                Vec3::new(arm_half * 1.5, 0.2, -0.6),
                Vec3::new(0.5, 0.2, 0.5),
                vxl_phys_core::Quat::IDENTITY,
                ARENA_DEFAULT,
            );
            w.add_joint(
                Joint::new(
                    JointKind::Prismatic,
                    foot_c as u32,
                    c as u32,
                    Vec3::ZERO,
                    Vec3::new(-arm_half, 0.0, 0.0),
                )
                .with_axis(Vec3::X)
                .with_limits(-1.2, 1.2),
            );
        }
        prev_top = Some((a, c));
        base_y += arm_half * 1.44;
    }
    add_box_r(
        &mut w,
        Vec3::new(0.0, base_y + 0.3, -0.6),
        Vec3::new(2.0, 0.25, 1.0),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.8, 0.05),
        2200.0,
    );
    w
}

/// arena `spring-bed`（4×4 立柱 + 距离网 + 3 重箱）。
///
/// ⚠️ arena 的 `spring` 关节（stiffness 90 / damping 0.35）在本仓按**刚性距离**落地。
pub(crate) fn scene_spring_bed(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.9);
    let cols = (((40f32 / 2.0).sqrt().round()) as usize).clamp(3, 7);
    let spacing = 1.5f32;
    let y0 = 4.0f32;
    let mut grid: Vec<Vec<usize>> = Vec::with_capacity(cols);
    for ix in 0..cols {
        let mut col = Vec::with_capacity(cols);
        for iz in 0..cols {
            col.push(add_box_r(
                &mut w,
                Vec3::new(
                    (ix as f32 - (cols as f32 - 1.0) / 2.0) * spacing,
                    y0,
                    (iz as f32 - (cols as f32 - 1.0) / 2.0) * spacing,
                ),
                Vec3::splat(0.22),
                vxl_phys_core::Quat::IDENTITY,
                s2(0.7, 0.05),
                700.0,
            ));
        }
        grid.push(col);
    }
    for ix in 0..cols {
        for iz in 0..cols {
            if ix + 1 < cols {
                w.add_joint(
                    Joint::new(
                        JointKind::Distance,
                        grid[ix][iz] as u32,
                        grid[ix + 1][iz] as u32,
                        Vec3::ZERO,
                        Vec3::ZERO,
                    )
                    .with_rest(spacing),
                );
            }
            if iz + 1 < cols {
                w.add_joint(
                    Joint::new(
                        JointKind::Distance,
                        grid[ix][iz] as u32,
                        grid[ix][iz + 1] as u32,
                        Vec3::ZERO,
                        Vec3::ZERO,
                    )
                    .with_rest(spacing),
                );
            }
        }
    }
    let mut rnd = rng32(20260915);
    let drops = (40usize / 12).clamp(1, 5);
    for i in 0..drops {
        let x = (rnd() - 0.5) * cols as f32 * spacing * 0.5;
        let z = (rnd() - 0.5) * cols as f32 * spacing * 0.5;
        add_box_r(
            &mut w,
            Vec3::new(x, y0 + 3.0 + i as f32 * 1.4, z),
            Vec3::splat(0.5),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            3000.0,
        );
    }
    w
}
