//! probes_stress：PhysArena「极端工况」（`stress.ts`）复刻批。
use super::{
    add_ball, add_box_r, add_dyn_shape, add_static_box, ground, ground_mu, rng32, s2, Joint,
    JointKind, PhysConfig, Shape, Vec3, World, ARENA_DEFAULT,
};

/// arena `ccd-onslaught`（20 发 0.3 m 弹丸 × 260 m/s 打 0.1 m 薄墙）。
///
/// ⚠️ 本仓 CCD 是**全局速度阈值**（无逐体开关）⇒ 本场景打开阈值 20 m/s；
/// arena 侧是逐体 `ccd: true`。对照场景 `ccd-control` 保持默认关。
pub(crate) fn scene_ccd_onslaught(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground(&mut w, 400.0);
    let count = (60usize / 3).clamp(2, 24);
    for i in 0..count {
        let z = (i as f32 - (count as f32 - 1.0) / 2.0) * 2.4;
        add_static_box(
            &mut w,
            Vec3::new(0.0, 2.0, z),
            Vec3::new(0.05, 2.0, 1.0),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        let p = add_ball(&mut w, Vec3::new(-140.0, 2.0, z), 0.3, s2(0.3, 0.2), 8000.0);
        w.bodies.set_linvel(p, Vec3::new(260.0, 0.0, 0.0));
    }
    w
}

/// arena `ccd-control`：与 `ccd-onslaught` 几何相同，CCD 关（本仓默认 `INFINITY`）。
pub(crate) fn scene_ccd_control(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 400.0);
    let count = (60usize / 3).clamp(2, 24);
    for i in 0..count {
        let z = (i as f32 - (count as f32 - 1.0) / 2.0) * 2.4;
        add_static_box(
            &mut w,
            Vec3::new(0.0, 2.0, z),
            Vec3::new(0.05, 2.0, 1.0),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        let p = add_ball(&mut w, Vec3::new(-140.0, 2.0, z), 0.3, s2(0.3, 0.2), 8000.0);
        w.bodies.set_linvel(p, Vec3::new(260.0, 0.0, 0.0));
    }
    w
}

/// arena `cannonball`（80 砖 + 炮弹）：中等速度重弹撞砖墙。
pub(crate) fn scene_cannonball(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 160.0);
    let rows = ((180f32 / 3.0).sqrt().round() as usize).clamp(3, 14);
    let cols = (180f32 / 2.0).sqrt().ceil() as usize;
    let cols = cols.max(3);
    for r in 0..rows {
        for c in 0..cols {
            add_box_r(
                &mut w,
                Vec3::new(
                    (c as f32 - (cols as f32 - 1.0) / 2.0) * 0.62,
                    0.3 + r as f32 * 0.62,
                    0.0,
                ),
                Vec3::splat(0.3),
                vxl_phys_core::Quat::IDENTITY,
                s2(0.55, 0.05),
                1000.0,
            );
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(-20.0, 3.2, 0.0),
        0.7,
        s2(0.4, 0.1),
        12000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(55.0, 1.0, 0.0));
    w
}

/// arena `small-objects`（500 个 2 cm 级碎块）：float32 精度与边距处理的放大镜。
pub(crate) fn scene_small_objects(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.8);
    let mut r = rng32(20260915);
    let n = 500usize;
    let span = (n as f32).sqrt() * 0.34;
    for i in 0..n {
        let s = 0.008 + r() * 0.014;
        let x = (r() - 0.5) * span;
        let z = (r() - 0.5) * span;
        add_box_r(
            &mut w,
            Vec3::new(x, 0.3 + i as f32 * 0.05, z),
            Vec3::splat(s),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.5, 0.05),
            1000.0,
        );
    }
    w
}

/// arena `narrow-corridor`（280 体窄缝拥挤）：接触数量爆炸的吞吐测试。
pub(crate) fn scene_narrow_corridor(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let wd = 1.2f32;
    for side in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(side * (wd / 2.0 + 0.3), 4.0, 0.0),
            Vec3::new(0.3, 4.0, 3.0),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    add_static_box(
        &mut w,
        Vec3::new(0.0, 8.4, 0.0),
        Vec3::new(wd / 2.0 + 0.6, 0.3, 3.0),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let n = 300usize.clamp(20, 280);
    for i in 0..n {
        let ix = i % 2;
        let iz = (i / 2) % 10;
        let iy = i / 20;
        add_box_r(
            &mut w,
            Vec3::new(
                (ix as f32 - 0.5) * 0.6,
                0.8 + iy as f32 * 0.53,
                -2.7 + iz as f32 * 0.6,
            ),
            Vec3::splat(0.26),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.03),
            1000.0,
        );
    }
    w
}

/// arena `mass-ratio`（120 体，质量比 1125:1）：条件数最差的配置。
pub(crate) fn scene_mass_ratio(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.9);
    let mut r = rng32(20260915);
    let n = 120usize;
    let mut top = 0.5f32;
    for i in 0..n {
        let light = i % 8 != 0;
        let s = if light { 0.12f32 } else { 1.1f32 };
        let y = top + s;
        top = y + s + 0.05;
        let x = (r() - 0.5) * 6.0;
        let z = (r() - 0.5) * 6.0;
        add_box_r(
            &mut w,
            Vec3::new(x, y, z),
            Vec3::splat(s),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.7, 0.02),
            if light { 8.0 } else { 9000.0 },
        );
    }
    w
}

/// arena `big-world`（150 体放在离原点 5000 m 处）：f32 精度在远处的表现。
pub(crate) fn scene_big_world(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let o = 5000.0f32;
    add_static_box(
        &mut w,
        Vec3::new(o, -1.0, 0.0),
        Vec3::new(60.0, 1.0, 60.0),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let mut r = rng32(20260915);
    let n = 150usize;
    for i in 0..n {
        let x = o + (r() - 0.5) * 8.0;
        let z = (r() - 0.5) * 8.0;
        add_box_r(
            &mut w,
            Vec3::new(x, 0.5 + i as f32 * 1.05, z),
            Vec3::splat(0.45),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.03),
            1000.0,
        );
    }
    w
}

/// arena `fragmentation`（400 个预制碎块）：近零间距接触 + 长期抖动。
pub(crate) fn scene_fragmentation(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 160.0);
    let mut r = rng32(20260915);
    let n = 400usize;
    let cols = (n as f32).cbrt().ceil() as usize;
    for i in 0..n {
        let ix = i % cols;
        let iy = (i / cols) % cols;
        let iz = i / (cols * cols);
        let x = (ix as f32 - (cols as f32 - 1.0) / 2.0) * 0.34 + (r() - 0.5) * 0.025;
        let z = (iz as f32 - (cols as f32 - 1.0) / 2.0) * 0.34 + (r() - 0.5) * 0.025;
        add_box_r(
            &mut w,
            Vec3::new(x, 0.25 + iy as f32 * 0.34, z),
            Vec3::splat(0.15),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.65, 0.02),
            1000.0,
        );
    }
    w
}

/// arena `multi-contact-grid`（600 体一层压一层）：纯吞吐量。
pub(crate) fn scene_multi_contact_grid(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 180.0);
    let n = 600usize;
    let side = (n as f32).cbrt().ceil().max(3.0) as usize;
    let layers = (n as f32 / (side * side) as f32).ceil().max(2.0) as usize;
    let mut made = 0usize;
    'outer: for l in 0..layers {
        for j in 0..side {
            for i in 0..side {
                if made >= n {
                    break 'outer;
                }
                add_box_r(
                    &mut w,
                    Vec3::new(
                        (i as f32 - (side as f32 - 1.0) / 2.0) * 1.01,
                        0.5 + l as f32 * 1.01,
                        (j as f32 - (side as f32 - 1.0) / 2.0) * 1.01,
                    ),
                    Vec3::splat(0.5),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.6, 0.01),
                    1000.0,
                );
                made += 1;
            }
        }
    }
    w
}

/// arena `stress-long-chain`（100 节球关节链从 26 m 垂落）：约束链误差累积。
pub(crate) fn scene_stress_long_chain(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 180.0, 0.9);
    let n = 100usize.clamp(8, 240);
    let link = 0.36f32;
    let top = 26.0f32;
    let anchor = add_static_box(
        &mut w,
        Vec3::new(0.0, top + 0.4, 0.0),
        Vec3::new(0.5, 0.4, 0.5),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let half = link * 0.45;
    let mut prev = anchor as u32;
    for i in 0..n {
        let seg = add_box_r(
            &mut w,
            Vec3::new(0.0, top - i as f32 * link, 0.0),
            Vec3::splat(half),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.02),
            600.0,
        );
        let a_local = Vec3::new(0.0, if i == 0 { -0.4 } else { -half }, 0.0);
        w.add_joint(Joint::new(
            JointKind::Spherical,
            prev,
            seg as u32,
            a_local,
            Vec3::new(0.0, half, 0.0),
        ));
        prev = seg as u32;
    }
    w
}

/// arena `stress-many-tiny`（900 个 r=8 cm 球挤在浅盘里）：接触对数量极限。
pub(crate) fn scene_stress_many_tiny(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 200.0, 0.7);
    let r = 0.08f32;
    let gap = r * 2.1;
    let side = 6.0f32;
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * side, 0.6, 0.0),
            Vec3::new(0.3, 0.6, side),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        add_static_box(
            &mut w,
            Vec3::new(0.0, 0.6, sx * side),
            Vec3::new(side, 0.6, 0.3),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let n = 900usize;
    let cols = ((side * 2.0) / gap).floor() as usize;
    let mut placed = 0usize;
    'outer: for layer in 0..60 {
        for ix in 0..cols {
            for iz in 0..cols {
                if placed >= n {
                    break 'outer;
                }
                add_ball(
                    &mut w,
                    Vec3::new(
                        -side + r + ix as f32 * gap,
                        r + 0.1 + layer as f32 * gap,
                        -side + r + iz as f32 * gap,
                    ),
                    r,
                    s2(0.5, 0.02),
                    800.0,
                );
                placed += 1;
            }
        }
    }
    w
}

/// arena `stress-slender-rod`（6 根 40:1 细杆 + 顶压块）：惯量/稳定性最差组合。
pub(crate) fn scene_stress_slender_rod(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.95);
    let count = (12usize / 2).clamp(1, 8);
    let h = 6.0f32;
    let radius = h / 20.0 / 2.0;
    for i in 0..count {
        let a = (i as f32 / count as f32) * std::f32::consts::TAU;
        let x = a.cos() * 2.4;
        let z = a.sin() * 2.4;
        add_dyn_shape(
            &mut w,
            Shape::Cylinder {
                half_height: h / 2.0,
                radius,
            },
            Vec3::new(x, h / 2.0, z),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.9, 0.0),
            2000.0,
        );
        add_box_r(
            &mut w,
            Vec3::new(x, h + 0.35, z),
            Vec3::splat(0.28),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.8, 0.05),
            6000.0,
        );
    }
    w
}
