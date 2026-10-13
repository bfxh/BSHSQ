//! probes_stack：PhysArena「堆叠与结构」（`stacking.ts`）复刻批。
//!
//! 体数/尺寸/出生位姿/材质逐字取自 arena 的 `defaultBodies` 档（seed 20260915、
//! 60 Hz、30 预热 + 180 测量与 arena 基准同口径）。已复刻的 `pyramid`/`brick-wall`/
//! `ball-pit` 在 `probes_a`，不在此重复。
use super::{
    add_ball, add_box_r, add_dyn_shape, add_static_box, ground, ground_mu, levels_of, rng32,
    rot_axis, rot_y, s2, triangular_levels, PhysConfig, Shape, Vec3, World, ARENA_DEFAULT,
};

/// arena `stacking.ts::BOX`.
const BOX: f32 = 0.5;
/// arena `stacking.ts::PITCH`（= BOX × 2.02，留缝防止开局互穿）。
const PITCH: f32 = BOX * 2.02;

/// arena `pyramid(builder, levels, bodies, jitter)` 的复刻（jitter=0 时**不**消耗 RNG）。
fn arena_pyramid(w: &mut World, levels: usize, bodies: usize, jitter: f32) {
    let mut r = rng32(1337);
    let counts = levels_of(bodies, levels);
    for (k, &n) in counts.iter().enumerate() {
        let y = BOX + k as f32 * PITCH;
        for i in 0..n {
            let x = (i as f32 - (n as f32 - 1.0) / 2.0) * PITCH;
            let (jx, jz) = if jitter != 0.0 {
                ((r() - 0.5) * jitter, (r() - 0.5) * jitter)
            } else {
                (0.0, 0.0)
            };
            add_box_r(
                w,
                Vec3::new(x + jx, y, jz),
                Vec3::splat(BOX),
                vxl_phys_core::Quat::IDENTITY,
                s2(0.6, 0.02),
                1000.0,
            );
        }
    }
}

/// arena `pyramid-jitter`（210 体、抖动 0.06）：随机初始位错的堆叠。
pub(crate) fn scene_pyramid_jitter(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let n = 210usize;
    arena_pyramid(&mut w, triangular_levels(n), n, 0.06);
    w
}

/// arena `tower`（60 体）：宽高比极大的细高塔，层间 2 cm 缝。
pub(crate) fn scene_tower(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let n = 60usize;
    let h = 0.45f32;
    for i in 0..n {
        add_box_r(
            &mut w,
            Vec3::new(0.0, h + i as f32 * (h * 2.0 + 0.02), 0.0),
            Vec3::new(0.55, h, 0.55),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.8, 0.05),
            1000.0,
        );
    }
    w
}

/// arena `random-pile`（300 体）：分层随机大小盒堆（宽相效率场景）。
pub(crate) fn scene_random_pile(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 140.0);
    let mut r = rng32(20260915);
    let n = 300usize;
    let side = (n as f32).sqrt().floor().max(2.0) as usize;
    let span = ((n as f32).sqrt() * 1.15).max(3.0);
    let step = span / side as f32;
    for i in 0..n {
        let layer = i / (side * side);
        let idx = i % (side * side);
        let ix = idx % side;
        let iz = idx / side;
        // RNG 消耗序与 arena 逐字一致：s → jx → jz。
        let s = 0.22 + r() * 0.3;
        let jx = (r() - 0.5) * (step - 2.0 * 0.52).max(0.05);
        let jz = (r() - 0.5) * (step - 2.0 * 0.52).max(0.05);
        add_box_r(
            &mut w,
            Vec3::new(
                (ix as f32 - (side as f32 - 1.0) / 2.0) * step + jx,
                0.6 + layer as f32 * 1.15,
                (iz as f32 - (side as f32 - 1.0) / 2.0) * step + jz,
            ),
            Vec3::splat(s),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.5, 0.05),
            1000.0,
        );
    }
    w
}

/// arena `irregular-block-jenga`（90 体）：交错堆叠的长条木块。
pub(crate) fn scene_jenga_stack(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let layers = (90usize).div_ceil(3).max(3);
    for l in 0..layers {
        let y = 0.15 + l as f32 * 0.32;
        for i in 0..3 {
            if l % 2 == 0 {
                add_box_r(
                    &mut w,
                    Vec3::new(0.0, y, (i as f32 - 1.0) * 1.0),
                    Vec3::new(1.5, 0.15, 0.45),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.6, 0.05),
                    1000.0,
                );
            } else {
                add_box_r(
                    &mut w,
                    Vec3::new((i as f32 - 1.0) * 1.0, y, 0.0),
                    Vec3::new(0.45, 0.15, 1.5),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.6, 0.05),
                    1000.0,
                );
            }
        }
    }
    w
}

/// arena `sphere-pyramid`（120 球）：单点接触金字塔（地面 friction 0.9）。
pub(crate) fn scene_sphere_pyramid(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 120.0, 0.9);
    let n = 120usize;
    let levels = triangular_levels(n);
    let counts = levels_of(n, levels);
    let d = 0.5f32;
    for (k, &cnt) in counts.iter().enumerate() {
        let y = d + k as f32 * d * 1.92;
        for i in 0..cnt {
            add_ball(
                &mut w,
                Vec3::new((i as f32 - (cnt as f32 - 1.0) / 2.0) * d * 2.02, y, 0.0),
                d,
                s2(0.75, 0.02),
                1000.0,
            );
        }
    }
    w
}

/// arena `cylinder-jenga`（90 体）：曲面-曲面接触的圆柱叠叠乐。
pub(crate) fn scene_cylinder_jenga(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let layers = (90usize).div_ceil(3).max(4);
    let d90 = std::f32::consts::FRAC_PI_2;
    for l in 0..layers {
        let y = 0.4 + l as f32 * 0.82;
        for i in 0..3 {
            let (pos, rot) = if l % 2 == 0 {
                (
                    Vec3::new(0.0, y, (i as f32 - 1.0) * 0.72),
                    rot_axis(Vec3::X, d90),
                )
            } else {
                (
                    Vec3::new((i as f32 - 1.0) * 0.72, y, 0.0),
                    rot_axis(Vec3::Z, d90),
                )
            };
            add_dyn_shape(
                &mut w,
                Shape::Cylinder {
                    half_height: 0.4,
                    radius: 0.35,
                },
                pos,
                rot,
                s2(0.7, 0.05),
                1000.0,
            );
        }
    }
    w
}

/// arena `stack-arch`（30 体）：楔块半圆拱 + 压顶石（摩擦 0.95、密度 2600）。
pub(crate) fn scene_stack_arch(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.9);
    let n = 30usize;
    let radius = 5.0f32;
    let thick = 0.6f32;
    for sx in [-1.0f32, 1.0] {
        add_static_box(
            &mut w,
            Vec3::new(sx * (radius + 0.3), 0.6, 0.0),
            Vec3::new(0.6, 0.6, 1.6),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
    }
    for i in 0..n {
        let a = std::f32::consts::PI * (i as f32 + 0.5) / n as f32;
        // ⚠️ arena 的写法是 `[0,0,sin(-half),cos(-half)]`（half = a/2 + π/4）——
        // 四元数分量里的 half 是**半角** ⇒ 实际转角 = 2·half = a + π/2（环厚朝半径向）。
        let half = a / 2.0 + std::f32::consts::FRAC_PI_4;
        add_box_r(
            &mut w,
            Vec3::new(a.cos() * radius, a.sin() * radius + 1.2, 0.0),
            Vec3::new(0.42, thick, 1.4),
            rot_axis(Vec3::Z, -2.0 * half),
            s2(0.95, 0.0),
            2600.0,
        );
    }
    add_box_r(
        &mut w,
        Vec3::new(0.0, radius + 1.2 + thick + 0.3, 0.0),
        Vec3::new(0.55, 0.3, 1.4),
        vxl_phys_core::Quat::IDENTITY,
        s2(0.9, 0.05),
        3000.0,
    );
    w
}

/// arena `stack-honeycomb`（60 体）：圆柱六方密排（单位面积接触最多）。
pub(crate) fn scene_stack_honeycomb(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let r = 0.5f32;
    let h = 0.6f32;
    let dx = r * 2.02;
    let dz = r * 3.0f32.sqrt() * 1.01;
    let n = 60usize;
    let mut placed = 0usize;
    for row in 0..14 {
        for col in 0..14 {
            if placed >= n {
                break;
            }
            let x = (col as f32 - 6.5) * dx + if row % 2 == 1 { dx / 2.0 } else { 0.0 };
            let z = (row as f32 - 6.5) * dz;
            add_dyn_shape(
                &mut w,
                Shape::Cylinder {
                    half_height: h,
                    radius: r,
                },
                Vec3::new(x, h, z),
                vxl_phys_core::Quat::IDENTITY,
                s2(0.7, 0.01),
                1400.0,
            );
            placed += 1;
        }
    }
    w
}

/// arena `stack-mixed-pile`（70 体）：尺寸比 4×、带随机偏转的混合堆。
pub(crate) fn scene_mixed_pile(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 170.0, 0.9);
    let mut rnd = rng32(20260915);
    let n = 70usize;
    let mut y = 0.6f32;
    let mut row_height = 0.0f32;
    for i in 0..n {
        // RNG 消耗序与 arena 一致：s → half(偏航) → x → z。
        let s = 0.22 + rnd() * 0.66;
        // ⚠️ arena 的 `[0, sin(half), 0, cos(half)]`（half = rnd()·π）里 half 是**半角**
        // ⇒ 实际偏航 = 2·half。
        let yaw = 2.0 * rnd() * std::f32::consts::PI;
        row_height = row_height.max(s * 2.0);
        let x = (rnd() - 0.5) * 7.0;
        let z = (rnd() - 0.5) * 7.0;
        add_box_r(
            &mut w,
            Vec3::new(x, y, z),
            Vec3::splat(s),
            rot_y(yaw),
            s2(0.8, 0.01),
            900.0,
        );
        if i % 8 == 7 {
            y += row_height + 0.05;
            row_height = 0.0;
        }
    }
    w
}
