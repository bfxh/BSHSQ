//! probes_havoc：PhysArena「破坏与流体」（`havoc.ts`）复刻批。
//!
//! ⚠️ arena 的这些场景用**低摩擦球堆**假水（无真实 SPH）；本批复刻同一几何与材质。
//! 炮击类场景 arena 侧逐体 `ccd: true`，本仓按**全局速度阈值 20 m/s** 打开 CCD。
use super::*;

/// arena `havoc.ts::FLUID`（假水材质：friction 0.04 / restitution 0.01 / density 1000）。
const FLUID: Surf = Surf {
    friction: 0.04,
    restitution: 0.01,
};
/// arena `havoc.ts::GRAIN`（颗粒：同水但 friction 0.45）。
const GRAIN: Surf = Surf {
    friction: 0.45,
    restitution: 0.01,
};

/// arena `bool-slice`（2×2×2 = 8 块 4 m³ 立方体 + 40 m/s 弹丸）。
pub(crate) fn scene_bool_slice(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.8);
    let cuts = (((8f32).cbrt().round()) as usize).clamp(1, 3);
    let h = 2.0f32 / cuts as f32;
    let y0 = 4.0f32;
    for i in 0..cuts {
        for j in 0..cuts {
            for k in 0..cuts {
                let c = (cuts as f32 - 1.0) / 2.0;
                add_box_r(
                    &mut w,
                    Vec3::new(
                        (i as f32 - c) * h,
                        y0 + (j as f32 - c) * h,
                        (k as f32 - c) * h,
                    ),
                    // 每面缩 0.5%：堆自身"刚好"不互穿（arena 原始注释同义）。
                    Vec3::splat(h / 2.0 - h * 0.005),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.55, 0.02),
                    1000.0,
                );
            }
        }
    }
    let p = add_ball(&mut w, Vec3::new(-10.0, y0, 0.0), 0.4, s2(0.5, 0.1), 4000.0);
    w.bodies.set_linvel(p, Vec3::new(40.0, 0.0, 0.0));
    w
}

/// arena `bool-carve`（挖空薄壁箱 + 140 颗粒 + 26 m/s 弹丸）。
pub(crate) fn scene_bool_carve(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.8);
    let (half, wall, height) = (3.0f32, 0.25f32, 3.0f32);
    add_static_box(
        &mut w,
        Vec3::new(0.0, wall, 0.0),
        Vec3::new(half, wall, half),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * half, wall + height / 2.0, 0.0),
            Vec3::new(wall, height / 2.0, half),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    add_static_box(
        &mut w,
        Vec3::new(0.0, wall + height / 2.0, -half),
        Vec3::new(half, height / 2.0, wall),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let r = 0.3f32;
    let n = 140usize;
    let cols = (((half * 2.0 - wall) / (r * 2.05)).floor() as usize).max(2);
    let per_layer = cols * cols;
    let layers = n.div_ceil(per_layer);
    for i in 0..n {
        let ix = i % cols;
        let iz = (i / cols) % cols;
        let iy = i / per_layer;
        if iy >= layers {
            break;
        }
        add_ball(
            &mut w,
            Vec3::new(
                -half + wall + r + ix as f32 * r * 2.05,
                wall + r + 0.05 + iy as f32 * r * 2.05,
                -half + wall + r + iz as f32 * r * 2.05,
            ),
            r,
            GRAIN,
            1000.0,
        );
    }
    let shot = add_ball(
        &mut w,
        Vec3::new(0.0, wall + height * 0.6, -12.0),
        0.5,
        ARENA_DEFAULT,
        5000.0,
    );
    w.bodies.set_linvel(shot, Vec3::new(0.0, 0.0, 26.0));
    w
}

/// arena `destruct-tower`（15 层 × 4 块的细塔，腰部弱连接 + 46 m/s 弹丸）。
pub(crate) fn scene_destruct_tower(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.85);
    let per_level = 4usize;
    let levels = (60usize / per_level).max(4);
    let h = 0.5f32;
    let span = 0.62f32;
    let waist = (levels as f32 * 0.28) as usize;
    for l in 0..levels {
        let weak = l >= waist && l < waist + 2;
        for i in 0..per_level {
            let a = (i as f32 / per_level as f32) * std::f32::consts::TAU
                + if l % 2 == 1 {
                    std::f32::consts::FRAC_PI_4
                } else {
                    0.0
                };
            add_box_r(
                &mut w,
                Vec3::new(
                    a.cos() * span,
                    h + l as f32 * (h * 2.0 + 0.015),
                    a.sin() * span,
                ),
                Vec3::new(h * 0.96, h, h * 0.96),
                vxl_phys_core::Quat::IDENTITY,
                if weak { s2(0.08, 0.01) } else { s2(0.7, 0.01) },
                if weak { 600.0 } else { 900.0 },
            );
        }
    }
    let hit_y = h + waist as f32 * (h * 2.0 + 0.015);
    let p = add_ball(
        &mut w,
        Vec3::new(-14.0, hit_y, 0.0),
        0.45,
        ARENA_DEFAULT,
        5000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(46.0, 0.0, 0.0));
    w
}

/// arena `destruct-bridge`（24 跨桥面距离链，两端桥台固定 + 中央落重球）。
pub(crate) fn scene_destruct_bridge(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.8);
    let spans = 24usize.clamp(6, 24);
    let seg = 2.2f32;
    let y = 6.0f32;
    for sgn in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sgn * (spans as f32 / 2.0 + 0.6) * seg, y - 0.6, 0.0),
            Vec3::new(0.8, 0.6, 2.0),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let mut ids: Vec<usize> = Vec::with_capacity(spans);
    for i in 0..spans {
        ids.push(add_box_r(
            &mut w,
            Vec3::new((i as f32 - (spans as f32 - 1.0) / 2.0) * seg, y, 0.0),
            Vec3::new(seg / 2.0 - 0.02, 0.12, 1.6),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.7, 0.01),
            800.0,
        ));
    }
    for i in 1..ids.len() {
        w.add_joint(
            Joint::new(
                JointKind::Distance,
                ids[i - 1] as u32,
                ids[i] as u32,
                Vec3::new(seg / 2.0, 0.0, 0.0),
                Vec3::new(-seg / 2.0, 0.0, 0.0),
            )
            .with_rest(0.04),
        );
    }
    add_ball(
        &mut w,
        Vec3::new(0.0, y + 9.0, 0.0),
        0.9,
        ARENA_DEFAULT,
        6000.0,
    );
    w
}

/// arena `fluid-dam-break`（320 粒"水"被薄坝拦在高台，弹丸破坝）。
pub(crate) fn scene_fluid_dam_break(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 200.0, 0.85);
    let r = 0.22f32;
    let gap = r * 2.06;
    let (cols, rows, depth) = (9usize, 9usize, 6usize);
    let base_y = 5.0f32;
    let x0 = -9.0f32;
    let pool_w = cols as f32 * gap;
    add_static_box(
        &mut w,
        Vec3::new(x0 + pool_w / 2.0, base_y / 2.0, 0.0),
        Vec3::new(
            pool_w / 2.0 + 0.6,
            base_y / 2.0,
            (depth as f32 * gap) / 2.0 + 0.6,
        ),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    add_static_box(
        &mut w,
        Vec3::new(x0 + pool_w + 0.25, base_y + 0.9, 0.0),
        Vec3::new(0.25, 0.9, (depth as f32 * gap) / 2.0 + 0.6),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let n = 320usize;
    let mut placed = 0usize;
    'outer: for iy in 0..rows {
        for iz in 0..depth {
            for ix in 0..cols {
                if placed >= n {
                    break 'outer;
                }
                add_ball(
                    &mut w,
                    Vec3::new(
                        x0 + r + ix as f32 * gap,
                        base_y + r + 0.05 + iy as f32 * gap,
                        -((depth as f32 * gap) / 2.0) + r + iz as f32 * gap,
                    ),
                    r,
                    FLUID,
                    1000.0,
                );
                placed += 1;
            }
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(x0 + pool_w + 6.0, base_y + 1.4, 0.0),
        0.5,
        ARENA_DEFAULT,
        6000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(-48.0, 0.0, 0.0));
    w
}

/// arena `fluid-pool`（400 粒"水"垂直落进 U 形池，看点 = 静止堆高）。
pub(crate) fn scene_fluid_pool(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let (half, wall, height) = (4.0f32, 0.3f32, 5.0f32);
    add_static_box(
        &mut w,
        Vec3::new(0.0, wall, 0.0),
        Vec3::new(half, wall, half),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * half, wall + height / 2.0, 0.0),
            Vec3::new(wall, height / 2.0, half),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        add_static_box(
            &mut w,
            Vec3::new(0.0, wall + height / 2.0, sx * half),
            Vec3::new(half, height / 2.0, wall),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let r = 0.22f32;
    let gap = r * 2.06;
    let cols = (((half * 2.0 - wall * 2.0) / gap).floor() as usize).max(2);
    let per_layer = cols * cols;
    let n = 400usize;
    let layers = n.div_ceil(per_layer);
    let mut rnd = rng32(20260915);
    let mut placed = 0usize;
    for layer in (0..layers).rev() {
        for iy in 0..cols {
            for iz in 0..cols {
                if placed >= n {
                    break;
                }
                let jitter = (rnd() - 0.5) * 0.02;
                add_ball(
                    &mut w,
                    Vec3::new(
                        -half + wall + r + iy as f32 * gap,
                        wall + r + 0.1 + (layer + 1) as f32 * gap,
                        -half + wall + r + iz as f32 * gap + jitter,
                    ),
                    r,
                    FLUID,
                    1000.0,
                );
                placed += 1;
            }
        }
        if placed >= n {
            break;
        }
    }
    w
}

/// arena `pressure-vise`（180 轻球床 + 8× 密度压块）。
pub(crate) fn scene_pressure_vise(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.9);
    let r = 0.4f32;
    let gap = r * 2.05;
    let cols = 9usize;
    let n = 180usize;
    let rows = n.div_ceil(cols * cols);
    let mut placed = 0usize;
    'outer: for iy in 0..rows {
        for iz in 0..cols {
            for ix in 0..cols {
                if placed >= n {
                    break 'outer;
                }
                add_ball(
                    &mut w,
                    Vec3::new(
                        (ix as f32 - (cols as f32 - 1.0) / 2.0) * gap,
                        r + 0.05 + iy as f32 * gap,
                        (iz as f32 - (cols as f32 - 1.0) / 2.0) * gap,
                    ),
                    r,
                    s2(0.5, 0.01),
                    300.0,
                );
                placed += 1;
            }
        }
    }
    let span = (cols as f32 * gap) / 2.0 + 0.5;
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * span, 3.0, 0.0),
            Vec3::new(0.4, 3.0, span),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    add_box_r(
        &mut w,
        Vec3::new(0.0, 5.2, 0.0),
        Vec3::new(span - 0.5, 0.9, span - 0.5),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.6, 0.01),
        2400.0,
    );
    w
}

/// arena `pressure-column`（40 层单点堆叠：柱顶漂移最灵敏）。
pub(crate) fn scene_pressure_column(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.95);
    let h = 0.5f32;
    let levels = 40usize.clamp(3, 160);
    for l in 0..levels {
        add_box_r(
            &mut w,
            Vec3::new(0.0, h + l as f32 * (h * 2.0 + 0.004), 0.0),
            Vec3::new(h * 0.98, h, h * 0.98),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.9, 0.0),
            1200.0,
        );
    }
    w
}

/// arena `fluid-cascade`（260 粒"水"逐级跌落到三层台面）。
pub(crate) fn scene_fluid_cascade(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let r = 0.2f32;
    let gap = r * 2.06;
    for (x, y, half_z) in [
        (-6.0f32, 14.0f32, 3.0f32),
        (1.0, 10.0, 3.0),
        (7.0, 6.5, 3.0),
    ] {
        add_static_box(
            &mut w,
            Vec3::new(x, y, 0.0),
            Vec3::new(3.2, 0.25, half_z),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        add_static_box(
            &mut w,
            Vec3::new(x + 3.1, y + 0.35, 0.0),
            Vec3::new(0.2, 0.35, half_z),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let n = 260usize;
    let cols = 7usize;
    let per_layer = cols * cols;
    for i in 0..n {
        let ix = i % cols;
        let iz = (i / cols) % cols;
        let iy = i / per_layer;
        add_ball(
            &mut w,
            Vec3::new(
                -6.0 + (ix as f32 - (cols as f32 - 1.0) / 2.0) * gap,
                16.0 + r + iy as f32 * gap,
                (iz as f32 - (cols as f32 - 1.0) / 2.0) * gap,
            ),
            r,
            FLUID,
            1000.0,
        );
    }
    w
}

/// arena `fluid-drain`（280 粒"水"从底部三倍粒径小孔排队漏出）。
pub(crate) fn scene_fluid_drain(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.8);
    let (half, wall, height) = (3.4f32, 0.3f32, 4.0f32);
    let r = 0.2f32;
    let gap = r * 2.06;
    let hole = gap * 3.0;
    let slab = (half - hole) / 2.0;
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * (hole + slab), wall, 0.0),
            Vec3::new(slab, wall, half),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    for sz in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(0.0, wall, sz * (hole + slab)),
            Vec3::new(hole, wall, slab),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * half, wall + height / 2.0, 0.0),
            Vec3::new(wall, height / 2.0, half),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        add_static_box(
            &mut w,
            Vec3::new(0.0, wall + height / 2.0, sx * half),
            Vec3::new(half, height / 2.0, wall),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    let n = 280usize;
    let cols = (((half * 2.0 - wall * 2.0) / gap).floor() as usize).max(2);
    let mut placed = 0usize;
    'outer: for layer in 0..40 {
        for iy in 0..cols {
            for iz in 0..cols {
                if placed >= n {
                    break 'outer;
                }
                add_ball(
                    &mut w,
                    Vec3::new(
                        -half + wall + r + iy as f32 * gap,
                        wall + r + 0.1 + layer as f32 * gap,
                        -half + wall + r + iz as f32 * gap,
                    ),
                    r,
                    FLUID,
                    1000.0,
                );
                placed += 1;
            }
        }
    }
    w
}

/// arena `destruct-wall`（4 道独立砖墙 + 60 m/s 弹丸依次打穿）。
pub(crate) fn scene_destruct_wall(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.8);
    let mut rnd = rng32(20260915);
    let (rows, cols) = (8usize, 5usize);
    let (bw, bh) = (0.5f32, 0.45f32);
    let mut n = 0usize;
    let max = 180usize;
    'outer: for wall_idx in 0..4 {
        let z = -6.0 + wall_idx as f32 * 4.0;
        for iy in 0..rows {
            let off = if iy % 2 == 1 { bw * 0.5 } else { 0.0 };
            for ix in 0..cols {
                if n >= max {
                    break 'outer;
                }
                let zj = (rnd() - 0.5) * 0.01;
                add_box_r(
                    &mut w,
                    Vec3::new(
                        (ix as f32 - (cols as f32 - 1.0) / 2.0) * bw * 1.02 + off,
                        0.3 + iy as f32 * bh * 1.02,
                        z + zj,
                    ),
                    Vec3::new(bw * 0.49, bh * 0.49, 0.22),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.7, 0.02),
                    1500.0,
                );
                n += 1;
            }
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(-14.0, 3.2, 0.0),
        0.4,
        s2(0.5, 0.05),
        8000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(60.0, 0.0, 0.0));
    w
}

/// arena `destruct-columns`（4 柱撑平台，弹丸逐根打断）。
pub(crate) fn scene_destruct_columns(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.9);
    let span = 3.2f32;
    let seg_h = 0.5f32;
    let levels = (60usize / 12).clamp(3, 12);
    for (px, pz) in [(-span, -span), (span, -span), (span, span), (-span, span)] {
        for l in 0..levels {
            add_box_r(
                &mut w,
                Vec3::new(px, seg_h + l as f32 * seg_h * 2.02, pz),
                Vec3::new(0.42, seg_h, 0.42),
                vxl_phys_core::Quat::IDENTITY,
                s2(0.75, 0.01),
                1400.0,
            );
        }
    }
    let top = seg_h + levels as f32 * seg_h * 2.02;
    add_box_r(
        &mut w,
        Vec3::new(0.0, top + 0.4, 0.0),
        Vec3::new(span + 0.9, 0.4, span + 0.9),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.8, 0.0),
        1900.0,
    );
    let p = add_ball(
        &mut w,
        Vec3::new(-12.0, top * 0.35, -span),
        0.45,
        ARENA_DEFAULT,
        9000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(52.0, 0.0, 0.0));
    w
}

/// arena `destruct-fracture`（126 块随机朝向预制碎块 + 42 m/s 弹丸整体崩碎）。
pub(crate) fn scene_destruct_fracture(cfg: PhysConfig) -> World {
    let mut cfg = cfg;
    cfg.ccd_speed_threshold = 20.0;
    let mut w = World::new(cfg);
    ground_mu(&mut w, 180.0, 0.8);
    let mut rnd = rng32(20260915);
    let max = 150usize;
    let cols = 9usize;
    let rows = (max as f32 / cols as f32).ceil().min(14.0) as usize;
    let (bw, bh) = (0.52f32, 0.44f32);
    let mut n = 0usize;
    'outer: for iy in 0..rows {
        for ix in 0..cols {
            if n >= max {
                break 'outer;
            }
            // ⚠️ arena 的 `[0, sin(half), 0, cos(half)]`（half = rnd()·π）里 half 是**半角**
            // ⇒ 实际偏航 = 2·half。
            let yaw = 2.0 * rnd() * std::f32::consts::PI;
            let zj = (rnd() - 0.5) * 0.04;
            add_box_r(
                &mut w,
                Vec3::new(
                    (ix as f32 - (cols as f32 - 1.0) / 2.0) * bw * 1.01,
                    0.25 + iy as f32 * bh * 1.02,
                    zj,
                ),
                Vec3::new(bw * 0.47, bh * 0.47, 0.2),
                rot_y(yaw),
                s2(0.65, 0.02),
                1600.0,
            );
            n += 1;
        }
    }
    let p = add_ball(
        &mut w,
        Vec3::new(-12.0, 2.2, 0.0),
        0.5,
        ARENA_DEFAULT,
        9000.0,
    );
    w.bodies.set_linvel(p, Vec3::new(42.0, 0.0, 0.0));
    w
}
