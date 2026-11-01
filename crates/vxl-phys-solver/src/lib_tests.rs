//! `lib.rs` 的**迷你闭环单测**（搬出：`solver/lib.rs` 顶在 god 门文件行数棘轮，只准减）。
//!
//! 覆盖面：顺序冲量把下落盒收敛到静置高度 + 速度趋零（岛级休眠的端到端触发）。
use crate::ImpulseSolver;
use vxl_phys_core::{BodySet, PhysConfig, Quat, SerialJobSystem, Shape, Vec3};
use vxl_phys_narrow::{ContactPoint, Manifold};

#[test]
pub(crate) fn resting_box_velocities_damp_to_zero() {
    // 迷你闭环：每帧按当前位置重建流形（模拟窄相），验证顺序冲量把
    // 下落盒收敛到静置高度（y ≈ 1.0）且速度趋零、最终入睡。
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 2.5, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let g = b.push_static(
        Shape::Box {
            half: Vec3::new(10.0, 0.5, 10.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let cfg = PhysConfig::default();
    let mut solver = ImpulseSolver::new(cfg.contact_skin);
    let dt = cfg.dt;
    // 简化重力（真实管线经力场→积分器；此处直接并入速度积分）。
    let mut y = 2.5f32;
    for _ in 0..300 {
        // 引擎契约：积分器只对清醒体施加重力（沉睡体由岛冻结）。
        if b.awake[0] {
            b.linvel[0] += Vec3::new(0.0, -9.81 * dt, 0.0);
        }
        let depth = 1.0 - y;
        if depth > -cfg.contact_skin {
            let m = Manifold {
                a: 0,
                b: g,
                normal: Vec3::new(0.0, -1.0, 0.0),
                points: [ContactPoint {
                    point: Vec3::new(0.0, y - 0.5, 0.0),
                    depth,
                    feature: 0,
                }]
                .into(),
            };
            solver.solve(&mut b, &[m], &cfg, dt, &SerialJobSystem);
        } else {
            solver.solve(&mut b, &[], &cfg, dt, &SerialJobSystem);
        }
        y += b.linvel[0].y * dt;
        b.position[0].y = y;
    }
    assert!((y - 1.0).abs() < 0.05, "rest y = {y}");
    assert!(b.linvel[0].y.abs() < 0.05, "linvel {:?}", b.linvel[0]);
}
