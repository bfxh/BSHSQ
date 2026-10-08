//! probes_dyn：PhysArena「经典动力学」（`dynamics.ts`）复刻批。
//!
//! 体数/尺寸/出生位姿/材质逐字取自 arena 的 `defaultBodies` 档；`ball-pit`
//! 已在 `probes_a`（`scene_ballpit`），不在此重复。
use super::*;

/// arena `domino`（150 体 + 推子）：单排连锁倾倒。
pub(crate) fn scene_domino(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 200.0, 0.85);
    let n = 150usize;
    let (h, t) = (0.6f32, 0.08f32);
    let x0 = -(n as f32) * 0.25;
    for i in 0..n {
        add_box_r(
            &mut w,
            Vec3::new(x0 + i as f32 * 0.5, h, 0.0),
            Vec3::new(t, h, 0.32),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.55, 0.02),
            1000.0,
        );
    }
    let pusher = add_box_r(
        &mut w,
        Vec3::new(x0 - 0.55, h, 0.0),
        Vec3::splat(0.25),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.6, 0.05),
        4000.0,
    );
    w.bodies.set_linvel(pusher, Vec3::new(3.2, 0.0, 0.0));
    w
}

/// arena `domino-spiral`（180 体 + 推子）：沿螺旋线摆放的多米诺。
pub(crate) fn scene_domino_spiral(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let n = 180usize;
    let h = 0.55f32;
    let spacing = 0.75f32;
    let mut pts: Vec<(f32, f32)> = Vec::with_capacity(n);
    let mut t = 0.0f32;
    for i in 0..n {
        let r = 2.5 + (i as f32 / n as f32) * 14.0;
        if i > 0 {
            t += spacing / r.max(0.5);
        }
        pts.push((t.cos() * r, t.sin() * r));
        let yaw = -t + std::f32::consts::FRAC_PI_2;
        add_box_r(
            &mut w,
            Vec3::new(pts[i].0, h, pts[i].1),
            Vec3::new(0.07, h, 0.3),
            rot_y(yaw),
            s2(0.55, 0.02),
            1000.0,
        );
    }
    let (dx, dz) = (pts[1].0 - pts[0].0, pts[1].1 - pts[0].1);
    let len = (dx * dx + dz * dz).sqrt().max(1e-6);
    let pusher = add_box_r(
        &mut w,
        Vec3::new(pts[0].0 - (dx / len) * 0.7, h, pts[0].1 - (dz / len) * 0.7),
        Vec3::splat(0.22),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.6, 0.05),
        4000.0,
    );
    w.bodies
        .set_linvel(pusher, Vec3::new((dx / len) * 3.2, 0.0, (dz / len) * 3.2));
    w
}

/// arena `ramp-roll`（60 体）：球/圆柱/盒同从斜坡滚下。
pub(crate) fn scene_ramp_roll(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 140.0);
    let ang = -std::f32::consts::PI / 10.0;
    add_static_box(
        &mut w,
        Vec3::new(0.0, 4.2, -6.0),
        Vec3::new(12.0, 0.3, 6.0),
        rot_axis(Vec3::Z, ang),
        s2(0.6, 0.05),
    );
    let mut r = rng32(20260915);
    let n = 60usize;
    let d90 = std::f32::consts::FRAC_PI_2;
    for i in 0..n {
        let lane = (i % 6) as f32 - 2.5;
        let row = i / 6;
        let kind = (i + row) % 3;
        let p = Vec3::new(lane, 6.2 + row as f32 * 1.1, -8.0 + r() * 1.5);
        match kind {
            0 => {
                add_ball(&mut w, p, 0.45, s2(0.4, 0.15), 1000.0);
            }
            1 => {
                add_cyl(
                    &mut w,
                    p,
                    0.45,
                    0.45,
                    rot_axis(Vec3::X, d90),
                    s2(0.4, 0.1),
                    1000.0,
                );
            }
            _ => {
                add_box_r(
                    &mut w,
                    p,
                    Vec3::splat(0.42),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.22, 0.05),
                    1000.0,
                );
            }
        }
    }
    w
}

/// arena `newton-cradle`（20 球摆）：距离约束的球形摆链（首球侧向拉偏 0.5 m）。
pub(crate) fn scene_newton_cradle(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let n = 20usize;
    let r = 0.4f32;
    let top = 4.0f32;
    let l = 0.6f32;
    for i in 0..n {
        let x = (i as f32 - (n as f32 - 1.0) / 2.0) * (r * 2.0 + 0.004);
        let anchor = add_static_box(
            &mut w,
            Vec3::new(x, top, 0.0),
            Vec3::splat(0.06),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        let dx = if i == 0 { -0.5f32 } else { 0.0 };
        let dy = -(l * l - dx * dx).max(1e-6).sqrt();
        let ball = add_ball(
            &mut w,
            Vec3::new(x + dx, top + dy, 0.0),
            r,
            s2(0.1, 0.98),
            8000.0,
        );
        w.add_joint(
            Joint::new(
                JointKind::Distance,
                anchor as u32,
                ball as u32,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .with_rest(l),
        );
    }
    w
}

/// arena `spinning-tops`（40 个圆锥陀螺）：高速自转（28 rad/s）+ 顶点朝下。
pub(crate) fn scene_spinning_tops(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.5);
    let n = 40usize;
    let cols = (n as f32).sqrt().ceil() as usize;
    for i in 0..n {
        let x = ((i % cols) as f32 - (cols as f32 - 1.0) / 2.0) * 2.4;
        let z = ((i / cols) as f32 - (cols as f32 - 1.0) / 2.0) * 2.4;
        let idx = add_cone(
            &mut w,
            Vec3::new(x, 0.62, z),
            0.45,
            0.6,
            rot_axis(Vec3::X, std::f32::consts::PI),
            s2(0.35, 0.05),
            6000.0,
        );
        w.bodies.set_angvel(idx, Vec3::new(0.0, 28.0, 0.0));
    }
    w
}

/// arena `bouncy-balls`（200 球）：高恢复系数球从高空落下持续弹跳。
pub(crate) fn scene_bouncy_balls(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_ex(&mut w, 160.0, s2(0.3, 0.92));
    let mut r = rng32(20260915);
    let n = 200usize;
    let span = (n as f32).sqrt() * 0.85;
    for _ in 0..n {
        let rad = 0.18 + r() * 0.22;
        let x = (r() - 0.5) * span;
        let y = 1.0 + r() * 26.0;
        let z = (r() - 0.5) * span;
        add_ball(&mut w, Vec3::new(x, y, z), rad, s2(0.25, 0.9), 900.0);
    }
    w
}

/// arena `free-fall-ladder`（80 体）：阶梯状高度同时释放（只由重力决定落地时间）。
pub(crate) fn scene_free_fall_ladder(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 160.0);
    let n = 80usize;
    for i in 0..n {
        add_box_r(
            &mut w,
            Vec3::new(0.0, 3.0 + i as f32 * 1.6, (i % 8) as f32 * 1.1 - 4.0),
            Vec3::splat(0.35),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.1),
            1000.0,
        );
    }
    w
}

/// arena `teeter-totter`（30 体）：转动关节 + 限位 [-0.7, 0.7] 的跷跷板。
pub(crate) fn scene_teeter_totter(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 140.0);
    let pivot = add_static_box(
        &mut w,
        Vec3::new(0.0, 0.85, 0.0),
        Vec3::splat(0.25),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let plank = add_box_r(
        &mut w,
        Vec3::new(0.0, 1.38, 0.0),
        Vec3::new(5.0, 0.08, 1.2),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.6, 0.05),
        1400.0,
    );
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            pivot as u32,
            plank as u32,
            Vec3::ZERO,
            Vec3::new(0.0, -0.53, 0.0),
        )
        .with_axis(Vec3::Z)
        .with_limits(-0.7, 0.7),
    );
    let n = (30usize / 4).clamp(1, 40);
    for i in 0..n {
        add_box_r(
            &mut w,
            Vec3::new(
                -4.4 + (i % 4) as f32 * 0.7,
                2.6 + (i / 4) as f32 * 0.65,
                (i % 3) as f32 * 0.7 - 0.7,
            ),
            Vec3::splat(0.3),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            400.0,
        );
    }
    for i in 0..n.min(3) {
        add_box_r(
            &mut w,
            Vec3::new(4.4, 2.2 + i as f32 * 0.65, 0.0),
            Vec3::splat(0.3),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            12000.0,
        );
    }
    w
}

/// arena `rotating-platform`（120 体）：自转平台上的散落盒子。
///
/// ⚠️ **降级**：本仓无 kinematic 体 ⇒ 平台改为「静态小锚 + 转动关节马达 1.2 rad/s」
/// 的等效物（体数 +1：多一个锚块）；arena 侧是 kinematic 速度推导，语义不完全同源。
pub(crate) fn scene_rotating_platform(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 140.0);
    let platform = add_cyl(
        &mut w,
        Vec3::new(0.0, 0.5, 0.0),
        5.0,
        0.5,
        vxl_phys_core::Quat::IDENTITY,
        s2(0.9, 0.05),
        1000.0,
    );
    let anchor = add_static_box(
        &mut w,
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::splat(0.05),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            anchor as u32,
            platform as u32,
            Vec3::ZERO,
            Vec3::ZERO,
        )
        .with_axis(Vec3::Y)
        .with_motor(1.2, 5.0e5),
    );
    let mut r = rng32(20260915);
    let n = 120usize;
    for i in 0..n {
        let a = r() * std::f32::consts::TAU;
        let rad = r() * 4.2;
        add_box_r(
            &mut w,
            Vec3::new(
                a.cos() * rad,
                1.6 + (i as f32 / n as f32) * 8.0,
                a.sin() * rad,
            ),
            Vec3::splat(0.26),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            1000.0,
        );
    }
    w
}

/// arena `domino-circle`（60 体 + 弹丸）：环形多米诺（侧向力矩敏感）。
pub(crate) fn scene_domino_circle(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.9);
    let n = 60usize;
    let radius = 6.0f32;
    for i in 0..n {
        let a = (i as f32 / n as f32) * std::f32::consts::TAU;
        // ⚠️ arena 的 `[0, sin(half), 0, cos(half)]`（half = a/2 + π/4）里 half 是**半角**
        // ⇒ 实际偏航 = 2·half = a + π/2。
        let half = a / 2.0 + std::f32::consts::FRAC_PI_4;
        add_box_r(
            &mut w,
            Vec3::new(a.cos() * radius, 1.1, a.sin() * radius),
            Vec3::new(0.5, 1.0, 0.09),
            rot_y(2.0 * half),
            s2(0.6, 0.01),
            900.0,
        );
    }
    let p = add_ball(
        &mut w,
        Vec3::new(radius + 2.2, 1.1, 0.0),
        0.3,
        s2(0.5, 0.05),
        4000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(-10.0, 0.0, 0.0));
    w
}

/// arena `bowling-pins`（15 瓶 + 球）：细长圆柱的连锁倒瓶。
pub(crate) fn scene_bowling_pins(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.7);
    let rows = (40usize / 8).clamp(2, 6);
    let (dx, dz) = (0.75f32, 0.9f32);
    for row in 0..rows {
        for k in 0..=row {
            add_cyl(
                &mut w,
                Vec3::new((k as f32 - row as f32 / 2.0) * dx, 0.75, row as f32 * dz),
                0.22,
                0.75,
                vxl_phys_core::Quat::IDENTITY,
                s2(0.5, 0.05),
                700.0,
            );
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(0.0, 0.45, -3.2),
        0.45,
        s2(0.4, 0.05),
        3000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(0.0, 0.0, 11.0));
    w
}

/// arena `avalanche`（260 球）：24° 斜坡上的亚稳密铺（侧推即整体滑坡）。
pub(crate) fn scene_avalanche(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 200.0, 0.9);
    let slope = 24.0f32 * std::f32::consts::PI / 180.0;
    add_static_box(
        &mut w,
        Vec3::new(0.0, 4.0, 0.0),
        Vec3::new(9.0, 0.4, 11.0),
        rot_axis(Vec3::X, -slope),
        s2(0.55, 0.05),
    );
    let r = 0.32f32;
    let gap = r * 2.06;
    let n = 260usize;
    let cols = 9usize;
    let mut placed = 0usize;
    'outer: for layer in 0..12 {
        for ix in 0..cols {
            for iz in 0..cols {
                if placed >= n {
                    break 'outer;
                }
                add_ball(
                    &mut w,
                    Vec3::new(
                        (ix as f32 - (cols as f32 - 1.0) / 2.0) * gap,
                        5.4 + layer as f32 * gap,
                        (iz as f32 - (cols as f32 - 1.0) / 2.0) * gap,
                    ),
                    r,
                    s2(0.35, 0.02),
                    500.0,
                );
                placed += 1;
            }
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(0.0, 8.0, -7.0),
        0.5,
        s2(0.5, 0.05),
        5000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(0.0, 0.0, 12.0));
    w
}

/// arena `carom`（18 球）：带边库的撞球台（恢复系数 0.92 的球-球分配）。
pub(crate) fn scene_carom(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.6);
    let (half_x, half_z) = (5.0f32, 2.6f32);
    let r = 0.3f32;
    add_static_box(
        &mut w,
        Vec3::new(0.0, 0.4, 0.0),
        Vec3::new(half_x, 0.4, half_z),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.25, 0.05),
    );
    for (pos, half) in [
        (
            Vec3::new(-half_x - 0.2, 0.8, 0.0),
            Vec3::new(0.2, 0.4, half_z + 0.2),
        ),
        (
            Vec3::new(half_x + 0.2, 0.8, 0.0),
            Vec3::new(0.2, 0.4, half_z + 0.2),
        ),
        (
            Vec3::new(0.0, 0.8, -half_z - 0.2),
            Vec3::new(half_x + 0.2, 0.4, 0.2),
        ),
        (
            Vec3::new(0.0, 0.8, half_z + 0.2),
            Vec3::new(half_x + 0.2, 0.4, 0.2),
        ),
    ] {
        add_static_box(
            &mut w,
            pos,
            half,
            vxl_phys_core::Quat::IDENTITY,
            s2(0.5, 0.9),
        );
    }
    let mut rnd = rng32(20260915);
    let n = 18usize;
    for _ in 0..n {
        let x = (rnd() - 0.5) * half_x * 1.4;
        let z = (rnd() - 0.5) * half_z * 1.4;
        let vx = (rnd() - 0.5) * 8.0;
        let vz = (rnd() - 0.5) * 8.0;
        let b = add_ball(&mut w, Vec3::new(x, 0.8, z), r, s2(0.15, 0.92), 1600.0);
        w.bodies.set_linvel(b, Vec3::new(vx, 0.0, vz));
    }
    w
}
