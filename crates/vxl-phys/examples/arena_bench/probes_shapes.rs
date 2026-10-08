//! probes_shapes：PhysArena「碰撞形状」（`shapes.ts`）复刻批。
//!
//! `trimesh-terrain` 已在 `probes_b`（`scene_trimesh_terrain`），不在此重复。
use super::*;

/// arena `mixed-convex`（200 个随机不规则凸包）：窄相只能走 GJK/EPA 通用路径。
pub(crate) fn scene_mixed_convex(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.8);
    let mut r = rng32(20260915);
    let n = 200usize;
    let span = (n as f32).sqrt() * 0.9;
    for i in 0..n {
        let size = 0.28 + r() * 0.3;
        let x = (r() - 0.5) * span;
        let z = (r() - 0.5) * span;
        let nv = 8 + (r() * 8.0).floor() as usize;
        let pts = rock_points(size, &mut r, nv);
        spawn_hull(
            &mut w,
            pts,
            Vec3::new(x, 0.75 + i as f32 * 0.75, z),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.08),
            800.0,
        );
    }
    w
}

/// arena `shape-zoo`（140 体六形状混层）：一次看清各原始体。
pub(crate) fn scene_shape_zoo(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.8);
    let mut r = rng32(20260915);
    let n = 140usize;
    let span = (n as f32).sqrt() * 0.95;
    for i in 0..n {
        let x = (r() - 0.5) * span;
        let z = (r() - 0.5) * span;
        let p = Vec3::new(x, 0.7 + i as f32 * 0.9, z);
        match i % 6 {
            0 => {
                add_ball(&mut w, p, 0.3, s2(0.5, 0.2), 1000.0);
            }
            1 => {
                add_box_r(
                    &mut w,
                    p,
                    Vec3::splat(0.27),
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.55, 0.05),
                    1000.0,
                );
            }
            2 => {
                // arena 侧给了非单位四元数 [0.2, 0, 0, 0.98]（引擎自行归一）。
                add_capsule(
                    &mut w,
                    p,
                    0.2,
                    0.22,
                    vxl_phys_core::Quat {
                        x: 0.2,
                        y: 0.0,
                        z: 0.0,
                        w: 0.98,
                    },
                    s2(0.5, 0.05),
                    1000.0,
                );
            }
            3 => {
                add_cyl(
                    &mut w,
                    p,
                    0.28,
                    0.26,
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.55, 0.05),
                    1000.0,
                );
            }
            4 => {
                add_cone(
                    &mut w,
                    p,
                    0.3,
                    0.32,
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.5, 0.05),
                    1000.0,
                );
            }
            _ => {
                spawn_hull(
                    &mut w,
                    icosa_points(0.32),
                    p,
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.5, 0.05),
                    700.0,
                );
            }
        }
    }
    w
}

/// arena `compound-crates`（90 个复合体货箱：箱体 + 球顶 + 柱脚 + 板檐）。
pub(crate) fn scene_compound_crates(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let mut r = rng32(20260915);
    let n = 90usize;
    let cols = (n as f32).sqrt().ceil() as usize;
    for i in 0..n {
        let x = ((i % cols) as f32 - (cols as f32 - 1.0) / 2.0) * 1.9;
        let z = ((i / cols) as f32 - (cols as f32 - 1.0) / 2.0) * 1.9;
        let cw = 0.5 + r() * 0.25;
        let y = 1.2 * cw + 0.01 + ((i / (cols * cols)) as f32) * 1.7;
        let children = vec![
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::new(cw, cw * 0.7, cw),
                },
                offset: Vec3::ZERO,
                rot: vxl_phys_core::Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Sphere { radius: cw * 0.35 },
                offset: Vec3::new(0.0, cw * 0.95, 0.0),
                rot: vxl_phys_core::Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Cylinder {
                    half_height: cw * 0.5,
                    radius: cw * 0.18,
                },
                offset: Vec3::new(cw * 0.7, -cw * 0.7, 0.0),
                rot: vxl_phys_core::Quat::IDENTITY,
            },
            CompoundChild {
                shape: Shape::Box {
                    half: Vec3::new(cw * 0.9, 0.08, 0.12),
                },
                offset: Vec3::new(-cw * 0.2, -cw * 0.78, 0.0),
                rot: vxl_phys_core::Quat::IDENTITY,
            },
        ];
        spawn_compound(&mut w, children, Vec3::new(x, y, z), s2(0.65, 0.03), 600.0);
    }
    w
}

/// arena `capsule-rain`（300 个胶囊从天而降）。
pub(crate) fn scene_capsule_rain(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.75);
    let mut r = rng32(20260915);
    let n = 300usize;
    let span = (n as f32).sqrt() * 1.0;
    for i in 0..n {
        let rad = 0.16 + r() * 0.12;
        let x = (r() - 0.5) * span;
        let z = (r() - 0.5) * span;
        let y = 1.2 + (i % 24) as f32 * 1.15 + r() * 0.3;
        let hh = rad * (1.2 + r() * 1.4);
        add_capsule(
            &mut w,
            Vec3::new(x, y, z),
            rad,
            hh,
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.06),
            900.0,
        );
    }
    w
}

/// arena `sensor-field`（160 球横穿传感器区域）。
///
/// ⚠️ **降级**：本仓无 sensor（触发体只检测不响应）⇒ arena 的触发面板**已省略**，
/// 只保留球场的成本形态；本场景**不与 arena 的数字对表**（球不再穿过面板）。
pub(crate) fn scene_sensor_field(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.7);
    let mut r = rng32(20260915);
    let n = 160usize;
    for i in 0..n {
        let x = (r() - 0.5) * 6.0;
        let y = 6.0 + r() * 16.0 + i as f32 * 0.5;
        let z = (r() - 0.5) * 22.0;
        let vz = -1.0 - r() * 3.0;
        let b = add_ball(&mut w, Vec3::new(x, y, z), 0.28, s2(0.5, 0.1), 1000.0);
        w.bodies.set_linvel(b, Vec3::new(0.0, 0.0, vz));
    }
    w
}

/// arena `shape-zoo-hard`（圆锥 + 凸包 + 三角网一起下落）。
///
/// 三角网臂走**一等三角网形状**（`add_trimesh` + `Shape::TriMesh`，可动态）。
pub(crate) fn scene_shape_zoo_hard(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.85);
    let mut rnd = rng32(20260915);
    let n = 48usize;
    for i in 0..n {
        let x = (rnd() - 0.5) * 9.0;
        let z = (rnd() - 0.5) * 9.0;
        let y = 2.0 + (i as f32 / n as f32) * 12.0;
        let p = Vec3::new(x, y, z);
        match i % 3 {
            0 => {
                add_cone(
                    &mut w,
                    p,
                    0.55,
                    0.8,
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.6, 0.03),
                    1000.0,
                );
            }
            1 => {
                let pts = rock_points(0.62, &mut rnd, 14);
                spawn_hull(
                    &mut w,
                    pts,
                    p,
                    vxl_phys_core::Quat::IDENTITY,
                    s2(0.65, 0.02),
                    1000.0,
                );
            }
            _ => {
                let (verts, tris) =
                    heightfield_mesh(1.2, 3, |mx, mz| (mx * 3.0).sin() * (mz * 3.0).cos() * 0.16);
                let mesh = w.add_trimesh(verts, tris);
                let half = w.trimesh_half_extents(mesh);
                let m = mat(&mut w, 0.6, 0.05);
                let bi = w.bodies.len();
                w.add_dynamic(
                    Shape::TriMesh { mesh, half },
                    p,
                    vxl_phys_core::Quat::IDENTITY,
                    1000.0,
                );
                w.bodies.set_material(bi, m);
            }
        }
    }
    w
}

/// arena `shape-shells`（24 张 6 cm 薄板叠塔）：薄壳深度求解。
pub(crate) fn scene_shape_shells(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.9);
    let n = 24usize.clamp(4, 60);
    for i in 0..n {
        add_box_r(
            &mut w,
            Vec3::new(0.0, 0.06 + i as f32 * 0.14, 0.0),
            Vec3::new(1.4, 0.03, 1.4),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.75, 0.0),
            2400.0,
        );
    }
    w
}

/// arena `shape-slices`（140 张薄片从天飘落）：边角先触的姿态稳定性。
pub(crate) fn scene_shape_slices(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 180.0, 0.85);
    let mut rnd = rng32(20260915);
    let n = 140usize;
    for i in 0..n {
        let a = rnd() * std::f32::consts::TAU;
        let rad = rnd() * 5.0;
        let x = a.cos() * rad;
        let z = a.sin() * rad;
        // arena 侧是非单位四元数（三个分量各 +1 的 w），引擎自行归一。
        let rot = vxl_phys_core::Quat {
            x: rnd() - 0.5,
            y: rnd() - 0.5,
            z: rnd() - 0.5,
            w: 1.0,
        };
        add_box_r(
            &mut w,
            Vec3::new(x, 3.0 + (i as f32 / n as f32) * 14.0, z),
            Vec3::new(0.5, 0.02, 0.5),
            rot,
            s2(0.7, 0.0),
            1200.0,
        );
    }
    w
}
