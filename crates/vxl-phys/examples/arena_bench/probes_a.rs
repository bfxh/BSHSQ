//! probes_a：早期 std 场景（金字塔/砖墙/球坑）与它们的复刻工具（**纯搬移**自 main.rs）。
//! 基准 harness 与读数报表已搬到 `report.rs`（2026-10-08，纯搬移）。
use super::*;

/// PhysArena `triangularLevels(n)` 的复刻。
pub(crate) fn triangular_levels(n: usize) -> usize {
    let mut l = 1usize;
    while (l * (l + 1)) / 2 < n {
        l += 1;
    }
    l.clamp(2, 40)
}

/// PhysArena `levelsOf(n, levels)` 的复刻。
pub(crate) fn levels_of(n: usize, levels: usize) -> Vec<usize> {
    let total = (levels * (levels + 1)) / 2;
    (0..levels)
        .map(|i| ((n * (levels - i)) as f32 / total as f32).round().max(1.0) as usize)
        .collect()
}

pub(crate) fn mat(w: &mut World, friction: f32, restitution: f32) -> vxl_phys_core::MaterialId {
    w.add_material(Material {
        friction: FrictionModel::Coulomb { mu: friction },
        restitution,
    })
}

pub(crate) fn add_box(
    w: &mut World,
    pos: Vec3,
    half: Vec3,
    m: vxl_phys_core::MaterialId,
    density: f32,
) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Box { half },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

pub(crate) fn add_sphere(
    w: &mut World,
    pos: Vec3,
    r: f32,
    m: vxl_phys_core::MaterialId,
    density: f32,
) {
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Sphere { radius: r },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
}

pub(crate) fn ground(w: &mut World, size: f32) {
    let m = mat(w, 0.7, 0.05);
    let i = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::new(size / 2.0, 1.0, size / 2.0),
        },
        Vec3::new(0.0, -1.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(i, m);
}

/// PhysArena 金字塔（210 体；层距 PITCH=1.01、箱半长 0.5）。
pub(crate) fn scene_pyramid(cfg: PhysConfig) -> World {
    scene_pyramid_mu(cfg, 0.6)
}

/// 金字塔 + **可变摩擦**（`--mu` 旋钮）：判定"堆不入睡"是否由摩擦（锚点漂移 ⇒
/// 摩擦注能的泵模式）驱动——μ=0 时若堆能停/睡，说明摩擦侧是能量源。
pub(crate) fn scene_pyramid_mu(cfg: PhysConfig, mu: f32) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, mu, 0.02);
    let n = 210usize;
    let box_half = 0.5f32;
    let pitch = box_half * 2.0 * 1.01;
    let levels = triangular_levels(n);
    let counts = levels_of(n, levels);
    for (k, &count) in counts.iter().enumerate() {
        let y = box_half + k as f32 * pitch;
        for i in 0..count {
            let x = (i as f32 - (count as f32 - 1.0) / 2.0) * pitch;
            add_box(
                &mut w,
                Vec3::new(x, y, 0.0),
                Vec3::splat(box_half),
                m,
                1000.0,
            );
        }
    }
    w
}

/// PhysArena 砖墙（200 体，错缝；行距 0.57、列距 1.22）。
pub(crate) fn scene_wall(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let m = mat(&mut w, 0.7, 0.01);
    let (hw, hh, hd) = (0.6f32, 0.28f32, 0.3f32);
    let per_row = 16usize;
    let rows = (200usize).div_ceil(per_row);
    let mut made = 0usize;
    'outer: for row in 0..rows {
        let offset = if row % 2 == 0 { 0.0 } else { hw };
        for i in 0..per_row {
            if made >= 200 {
                break 'outer;
            }
            let x = (i as f32 - (per_row as f32 - 1.0) / 2.0) * (hw * 2.0 + 0.02) + offset;
            let y = hh + row as f32 * (hh * 2.0 + 0.01);
            add_box(
                &mut w,
                Vec3::new(x, y, 0.0),
                Vec3::new(hw, hh, hd),
                m,
                1000.0,
            );
            made += 1;
        }
    }
    w
}

/// PhysArena 球坑（400 球点阵 + 4 面环墙）。
pub(crate) fn scene_ballpit(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let n = 400usize;
    let r = (4.0f32).max((n as f32).cbrt() * 1.4);
    ground(&mut w, 160.0);
    let mw = mat(&mut w, 0.5, 0.05);
    for i in 0..4 {
        let a = (i as f32 / 4.0) * std::f32::consts::PI * 2.0;
        let qi = w.bodies.len();
        w.add_static(
            Shape::Box {
                half: Vec3::new(r * 0.75, 1.5, 0.4),
            },
            Vec3::new(a.cos() * r, 1.5, a.sin() * r),
            vxl_phys_core::Quat {
                x: 0.0,
                y: ((a + std::f32::consts::FRAC_PI_2) / 2.0).sin(),
                z: 0.0,
                w: ((a + std::f32::consts::FRAC_PI_2) / 2.0).cos(),
            },
        );
        w.bodies.set_material(qi, mw);
    }
    let m = mat(&mut w, 0.45, 0.1);
    let side = (n as f32).cbrt().ceil() as usize;
    for i in 0..n {
        let ix = i % side;
        let iy = (i / side) % side;
        let iz = i / (side * side);
        add_sphere(
            &mut w,
            Vec3::new(
                (ix as f32 - (side as f32 - 1.0) / 2.0) * 0.7,
                0.4 + iy as f32 * 0.72,
                (iz as f32 - (side as f32 - 1.0) / 2.0) * 0.7,
            ),
            0.32,
            m,
            800.0,
        );
    }
    w
}
