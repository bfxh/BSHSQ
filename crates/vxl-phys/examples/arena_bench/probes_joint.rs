//! probes_joint：PhysArena「约束与关节」（`joints.ts`）复刻批 —— **约束链/软弹簧**这一半
//! （绳桥/布娃娃/曲柄滑块/车轮/弹簧网）；机构那半（塔吊/活塞/悬索桥/布娃娃堆/齿轮/剪叉/
//! 弹簧床）在 `probes_machine.rs`（2026-10-08 拆，避免单文件超 god 门 800 行）。
//!
//! `chain-hinge` / `hanging-tower` 已在 `probes_b`（640 步锚点分离版），不在此重复。
//! arena 的 `spring` 关节（软距离）在本仓按**刚性距离**落地（与 wasm 桥同口径），
//! 逐场景在注册表注记里写明。
use super::{
    add_ball, add_box_r, add_dyn_shape, add_static_box, ground, ground_mu, mat, rng32, rot_axis,
    s2, Joint, JointKind, PhysConfig, Shape, Vec3, World, ARENA_DEFAULT,
};

/// arena `rope-bridge`（18 跨桥面 + 悬索 + 5 载荷）：长约束链 + 二次接触。
pub(crate) fn scene_rope_bridge(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 160.0);
    let span = 24.0f32;
    let n = (70usize / 4).clamp(8, 64);
    let dx = span / n as f32;
    let mut pts: Vec<(f32, f32)> = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = i as f32 / n as f32;
        pts.push((
            -span / 2.0 + span * t,
            6.5 - (t * std::f32::consts::PI).sin() * 1.4,
        ));
    }
    let z_deck = -2.2f32;
    let zc_a = -1.4f32;
    let zc_b = 1.4f32;
    let left = add_static_box(
        &mut w,
        Vec3::new(pts[0].0, pts[0].1, z_deck),
        Vec3::splat(0.2),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    let mut centres: Vec<(f32, f32)> = Vec::with_capacity(n);
    let mut planks: Vec<usize> = Vec::with_capacity(n);
    let mut prev = left as u32;
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[i + 1];
        let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
        let ang = (y1 - y0).atan2(x1 - x0);
        let plank = add_box_r(
            &mut w,
            Vec3::new(cx, cy, z_deck),
            Vec3::new(dx * 0.5, 0.12, 1.6),
            rot_axis(Vec3::Z, ang),
            s2(0.7, 0.05),
            900.0,
        );
        centres.push((cx, cy));
        planks.push(plank);
        let anchor_a = if i == 0 {
            Vec3::ZERO
        } else {
            Vec3::new(dx * 0.5, 0.0, zc_a)
        };
        w.add_joint(Joint::new(
            JointKind::Spherical,
            prev,
            plank as u32,
            anchor_a,
            Vec3::new(-dx * 0.5, 0.0, zc_a),
        ));
        prev = plank as u32;
    }
    let right = add_static_box(
        &mut w,
        Vec3::new(pts[n].0, pts[n].1, z_deck),
        Vec3::splat(0.2),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    w.add_joint(Joint::new(
        JointKind::Spherical,
        prev,
        right as u32,
        Vec3::new(dx * 0.5, 0.0, zc_a),
        Vec3::ZERO,
    ));
    for i in 1..planks.len() {
        let (ax, ay) = centres[i - 1];
        let (bx, by) = centres[i];
        let rest = ((bx - ax) * (bx - ax) + (by - ay) * (by - ay)).sqrt();
        w.add_joint(
            Joint::new(
                JointKind::Distance,
                planks[i - 1] as u32,
                planks[i] as u32,
                Vec3::new(0.0, 0.0, zc_b),
                Vec3::new(0.0, 0.0, zc_b),
            )
            .with_rest(rest),
        );
    }
    let mut r = rng32(20260915);
    let load = 70usize / 12; // arena: Math.max(2, floor(70/12)) = 5（常量已满足）
    for i in 0..load {
        let pi = (r() * planks.len() as f32) as usize;
        let (px, py, pz) = {
            let p = w.bodies.position[planks[pi]];
            (p.x, p.y, p.z)
        };
        add_box_r(
            &mut w,
            Vec3::new(px, py + 0.6 + (i % 3) as f32 * 0.55, pz + (r() - 0.5) * 1.6),
            Vec3::splat(0.26),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            700.0,
        );
    }
    w
}

/// arena `ragdoll(builder, ox, oz, yaw)` 的复刻：11 刚体 + 10 关节的铰接人形。
///
/// 装配按 rig-local 地标 + 逐侧镜像组织（god 门 120 行线：地标在模块级常量、
/// 手臂/腿各一个 helper），关节序与 arena 逐字一致：肩×2 → 肘×2 → 髋×2 → 膝×2
/// → 脊柱 → 颈。
const RIG_HIPS: [f32; 3] = [0.0, 2.16, 0.0];
const RIG_TORSO: [f32; 3] = [0.0, 3.15, 0.0];
const RIG_HEAD: [f32; 3] = [0.0, 3.95, 0.0];
const RIG_HIP_L: [f32; 3] = [-0.2, 1.75, 0.0];
const RIG_THIGH_L: [f32; 3] = [-0.2, 1.33, 0.0];
const RIG_KNEE_L: [f32; 3] = [-0.2, 0.905, 0.0];
const RIG_SHIN_L: [f32; 3] = [-0.2, 0.5, 0.0];
const RIG_SHOULDER_L: [f32; 3] = [-0.3, 3.0, 0.0];
const RIG_UPPER_ARM_L: [f32; 3] = [-0.4, 3.0, 0.0];
const RIG_ELBOW_L: [f32; 3] = [-0.4, 2.65, 0.0];
const RIG_FORE_ARM_L: [f32; 3] = [-0.4, 2.3, 0.0];
const RIG_SPINE: [f32; 3] = [0.0, 2.565, 0.0];
const RIG_NECK: [f32; 3] = [0.0, 3.72, 0.0];

/// 布娃娃装配的共享上下文。
struct Rig<'a> {
    rot: vxl_phys_core::Quat,
    density: f32,
    to_world: &'a dyn Fn([f32; 3]) -> Vec3,
}

fn rig_sub(a: [f32; 3], from: [f32; 3]) -> Vec3 {
    Vec3::new(a[0] - from[0], a[1] - from[1], a[2] - from[2])
}

fn rig_mirror(p: [f32; 3], side: usize) -> [f32; 3] {
    if side == 0 {
        p
    } else {
        [-p[0], p[1], p[2]]
    }
}

/// 一侧手臂（上臂/前臂 + 肩球关节 + 肘转动关节限位 [0.05, 2.4]）。
fn ragdoll_arm(w: &mut World, rig: &Rig, side: usize, torso: usize) {
    let shoulder = rig_mirror(RIG_SHOULDER_L, side);
    let upper = rig_mirror(RIG_UPPER_ARM_L, side);
    let elbow = rig_mirror(RIG_ELBOW_L, side);
    let fore = rig_mirror(RIG_FORE_ARM_L, side);
    let upper_arm = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.22,
            radius: 0.13,
        },
        (rig.to_world)(upper),
        rig.rot,
        s2(0.7, 0.05),
        rig.density,
    );
    let fore_arm = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.22,
            radius: 0.12,
        },
        (rig.to_world)(fore),
        rig.rot,
        s2(0.7, 0.05),
        rig.density,
    );
    w.add_joint(Joint::new(
        JointKind::Spherical,
        torso as u32,
        upper_arm as u32,
        rig_sub(shoulder, RIG_TORSO),
        rig_sub(shoulder, upper),
    ));
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            upper_arm as u32,
            fore_arm as u32,
            rig_sub(elbow, upper),
            rig_sub(elbow, fore),
        )
        .with_axis(Vec3::X)
        .with_limits(0.05, 2.4),
    );
}

/// 一侧腿（大腿/小腿 + 髋球关节 + 膝转动关节限位 [0, 2.2]）。
fn ragdoll_leg(w: &mut World, rig: &Rig, side: usize, hips: usize) {
    let hip = rig_mirror(RIG_HIP_L, side);
    let thigh = rig_mirror(RIG_THIGH_L, side);
    let knee = rig_mirror(RIG_KNEE_L, side);
    let shin = rig_mirror(RIG_SHIN_L, side);
    let upper_leg = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.25,
            radius: 0.15,
        },
        (rig.to_world)(thigh),
        rig.rot,
        s2(0.8, 0.05),
        rig.density,
    );
    let lower_leg = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.25,
            radius: 0.13,
        },
        (rig.to_world)(shin),
        rig.rot,
        s2(0.8, 0.05),
        rig.density,
    );
    w.add_joint(Joint::new(
        JointKind::Spherical,
        hips as u32,
        upper_leg as u32,
        rig_sub(hip, RIG_HIPS),
        rig_sub(hip, thigh),
    ));
    w.add_joint(
        Joint::new(
            JointKind::Revolute,
            upper_leg as u32,
            lower_leg as u32,
            rig_sub(knee, thigh),
            rig_sub(knee, shin),
        )
        .with_axis(Vec3::X)
        .with_limits(0.0, 2.2),
    );
}

fn ragdoll(w: &mut World, ox: f32, oz: f32, yaw: f32) {
    let (s, c) = ((yaw / 2.0).sin(), (yaw / 2.0).cos());
    let rot = vxl_phys_core::Quat {
        x: 0.0,
        y: s,
        z: 0.0,
        w: c,
    };
    let to_world = |p: [f32; 3]| -> Vec3 {
        Vec3::new(ox + p[0] * c + p[2] * s, p[1], oz - p[0] * s + p[2] * c)
    };
    let rig = Rig {
        rot,
        density: 600.0,
        to_world: &to_world,
    };
    let hips = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.2,
            radius: 0.19,
        },
        to_world(RIG_HIPS),
        rot,
        s2(0.7, 0.05),
        rig.density,
    );
    let torso = add_dyn_shape(
        w,
        Shape::Capsule {
            half_height: 0.35,
            radius: 0.22,
        },
        to_world(RIG_TORSO),
        rot,
        s2(0.7, 0.05),
        rig.density,
    );
    let head = add_ball(w, to_world(RIG_HEAD), 0.22, s2(0.6, 0.05), rig.density);
    for side in 0..2 {
        ragdoll_arm(w, &rig, side, torso);
    }
    for side in 0..2 {
        ragdoll_leg(w, &rig, side, hips);
    }
    w.add_joint(Joint::new(
        JointKind::Spherical,
        hips as u32,
        torso as u32,
        rig_sub(RIG_SPINE, RIG_HIPS),
        rig_sub(RIG_SPINE, RIG_TORSO),
    ));
    w.add_joint(Joint::new(
        JointKind::Spherical,
        torso as u32,
        head as u32,
        rig_sub(RIG_NECK, RIG_TORSO),
        rig_sub(RIG_NECK, RIG_HEAD),
    ));
}

/// arena `ragdoll`（10 具人形）：每具 11 刚体 + 10 关节。
pub(crate) fn scene_ragdoll(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 140.0, 0.8);
    let mut r = rng32(20260915);
    let count = (100usize / 10).clamp(1, 50);
    let cols = (count as f32).sqrt().ceil() as usize;
    for i in 0..count {
        let x = ((i % cols) as f32 - (cols as f32 - 1.0) / 2.0) * 3.2;
        let z = ((i / cols) as f32 - (cols as f32 - 1.0) / 2.0) * 3.2;
        let yaw = r() * std::f32::consts::TAU;
        ragdoll(&mut w, x, z, yaw);
    }
    w
}

/// arena `slider-crank`（5 组曲柄滑块闭环，含马达与棱柱限位）。
pub(crate) fn scene_slider_crank(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let count = (20usize / 4).clamp(1, 8);
    let (y_mech, y_slider) = (2.3f32, 2.42f32);
    for cidx in 0..count {
        let ox = (cidx as f32 - (count as f32 - 1.0) / 2.0) * 6.0;
        let base = add_static_box(
            &mut w,
            Vec3::new(ox, 2.0, 0.0),
            Vec3::new(3.4, 0.1, 0.6),
            vxl_phys_core::Quat::IDENTITY,
            ARENA_DEFAULT,
        );
        let crank = add_box_r(
            &mut w,
            Vec3::new(ox - 1.5, y_mech, 0.0),
            Vec3::new(0.8, 0.1, 0.14),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.05),
            1200.0,
        );
        let rod = add_box_r(
            &mut w,
            Vec3::new(ox + 0.3, y_mech, 0.0),
            Vec3::new(1.0, 0.1, 0.14),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.4, 0.05),
            900.0,
        );
        let slider = add_box_r(
            &mut w,
            Vec3::new(ox + 1.3, y_slider, 0.0),
            Vec3::new(0.4, 0.32, 0.32),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.5, 0.05),
            2000.0,
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                base as u32,
                crank as u32,
                Vec3::new(-1.5, 0.3, 0.0),
                Vec3::ZERO,
            )
            .with_axis(Vec3::Z)
            .with_motor(6.0, 40000.0),
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                crank as u32,
                rod as u32,
                Vec3::new(0.8, 0.0, 0.0),
                Vec3::new(-1.0, 0.0, 0.0),
            )
            .with_axis(Vec3::Z),
        );
        w.add_joint(
            Joint::new(
                JointKind::Revolute,
                rod as u32,
                slider as u32,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, y_mech - y_slider, 0.0),
            )
            .with_axis(Vec3::Z),
        );
        w.add_joint(
            Joint::new(
                JointKind::Prismatic,
                base as u32,
                slider as u32,
                Vec3::new(1.3, y_slider - 2.0, 0.0),
                Vec3::ZERO,
            )
            .with_axis(Vec3::X)
            .with_limits(-3.0, 3.0),
        );
    }
    w
}

/// arena `motor-wheel`（8 辆车：底盘 + 双轮，轮毂马达 -14 rad/s）。
pub(crate) fn scene_motor_wheel(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground_mu(&mut w, 160.0, 0.95);
    let count = (24usize / 3).clamp(1, 8);
    let d90 = std::f32::consts::FRAC_PI_2;
    for cidx in 0..count {
        let ox = (cidx as f32 - (count as f32 - 1.0) / 2.0) * 2.4;
        let chassis = add_box_r(
            &mut w,
            Vec3::new(ox, 1.4, 0.0),
            Vec3::new(0.9, 0.18, 0.5),
            vxl_phys_core::Quat::IDENTITY,
            s2(0.6, 0.05),
            700.0,
        );
        for side in [-1.0f32, 1.0] {
            let wheel = add_dyn_shape(
                &mut w,
                Shape::Cylinder {
                    half_height: 0.1,
                    radius: 0.42,
                },
                Vec3::new(ox, 1.0, side * 0.65),
                rot_axis(Vec3::X, d90),
                s2(1.2, 0.05),
                2000.0,
            );
            w.add_joint(
                Joint::new(
                    JointKind::Revolute,
                    chassis as u32,
                    wheel as u32,
                    Vec3::new(0.0, -0.4, side * 0.65),
                    Vec3::ZERO,
                )
                .with_axis(Vec3::Z)
                .with_motor(-14.0, 8000.0),
            );
        }
    }
    w
}

/// arena `spring-net`（11×11 距离网格 + 3 载荷）：最柔的约束链形态。
pub(crate) fn scene_spring_net(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    ground(&mut w, 120.0);
    let side = ((120f32).sqrt().round() as usize).clamp(3, 20);
    let step = 0.6f32;
    let mut ids: Vec<Vec<usize>> = Vec::with_capacity(side);
    for j in 0..side {
        let mut row = Vec::with_capacity(side);
        for i in 0..side {
            let pos = Vec3::new(
                (i as f32 - (side as f32 - 1.0) / 2.0) * step,
                8.0 - j as f32 * step * 0.15,
                j as f32 * step,
            );
            let id = if j == 0 {
                let m = mat(&mut w, 0.4, 0.05);
                let i2 = w.bodies.len();
                w.add_static(
                    Shape::Sphere { radius: 0.13 },
                    pos,
                    vxl_phys_core::Quat::IDENTITY,
                );
                w.bodies.set_material(i2, m);
                i2
            } else {
                add_ball(&mut w, pos, 0.13, s2(0.4, 0.05), 500.0)
            };
            row.push(id);
        }
        ids.push(row);
    }
    let link = |w: &mut World, a: usize, b: usize, di: usize, dj: usize| {
        let rest = step * ((di * di + dj * dj) as f32).sqrt();
        w.add_joint(
            Joint::new(
                JointKind::Distance,
                a as u32,
                b as u32,
                Vec3::ZERO,
                Vec3::ZERO,
            )
            .with_rest(rest),
        );
    };
    for j in 0..side {
        for i in 0..side {
            if i + 1 < side {
                link(&mut w, ids[j][i], ids[j][i + 1], 1, 0);
            }
            if j + 1 < side {
                link(&mut w, ids[j][i], ids[j + 1][i], 0, 1);
            }
            if i + 1 < side && j + 1 < side {
                link(&mut w, ids[j][i], ids[j + 1][i + 1], 1, 1);
            }
        }
    }
    let mut r = rng32(20260915);
    let net_z = ((side as f32 - 1.0) / 2.0) * step;
    for i in 0..(120usize / 40) {
        let x = (r() - 0.5) * side as f32 * 0.5;
        let z = net_z + (r() - 0.5) * 1.5;
        add_ball(
            &mut w,
            Vec3::new(x, 12.0 + i as f32 * 1.2, z),
            0.35,
            s2(0.5, 0.05),
            3000.0,
        );
    }
    w
}
