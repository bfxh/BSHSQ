//! ang：关节**角行**（固定/棱柱锁 3 轴、转动锁 2 轴）——从 `joints/solve.rs` 拆出。
//!
//! **为什么单独一片**：本文件在 2026-10-10 加**姿态偏置**（相对旋转的旋转向量
//! 按 `β/dt·e` 进 RHS）时，`solve.rs` 受 god 门 file_lines 棘轮（只准减）而
//! 该改动净增行 ⇒ 把角行整片外迁，既让 `solve.rs` **净缩**、又给新逻辑一个
//! 自足的落点（同 `joints/linalg.rs` 的分片先例）。
use vxl_phys_core::{BodySet, Mat3, Quat, Vec3};

use super::linalg::{ang_k, apply_ang_pair, perp_basis, proj, solve3};
use super::types::{Joint, JointKind, JointParams};

/// 相对旋转（世界系，a→b）的**旋转向量**：`R_b·R_aᵀ` 的轴角形式，取**短弧**。
/// 固定/棱柱的目标是 `e = 0`；转动只约束 `e` 垂直于自由轴的两个分量。
fn angular_error(qa: Quat, qb: Quat) -> Vec3 {
    let mut q = qb * qa.conjugate();
    if q.w < 0.0 {
        q = Quat::new(-q.x, -q.y, -q.z, -q.w);
    }
    let v = Vec3::new(q.x, q.y, q.z);
    let s = v.length();
    if s < 1e-8 {
        v * 2.0
    } else {
        v * (2.0 * s.atan2(q.w) / s)
    }
}

/// 角行：固定/棱柱锁 3 轴、转动锁垂直于自由轴的 2 轴；返回更新后的最大修正。
///
/// **姿态偏置**：与线性行同族，把相对旋转的旋转向量 `e` 按 `β/dt·e` 加进 RHS
/// ⇒ 不只消相对角速度，还修正相对转角的**漂移**（此前只消角速度，转角误差只靠
/// 速度级间接收敛）。这是 `joints.rs` 头注里"要热启动必须先给角行加姿态偏置"
/// 的前置件。关节是 opt-in ⇒ 四哈希与金样 8 项（都不含关节）逐位不变。
pub(crate) fn angular_rows(
    j: &Joint,
    bodies: &mut BodySet,
    ai: usize,
    bi: usize,
    sp: &JointParams,
    mut max_dv: f32,
) -> f32 {
    let (_, qa) = bodies.pose(ai);
    let (_, qb) = bodies.pose(bi);
    let e_ang = angular_error(qa, qb);
    let k_ang = ang_k(bodies, ai, bi);
    match j.kind {
        JointKind::Fixed | JointKind::Prismatic => {
            let wrel = bodies.angvel(bi) - bodies.angvel(ai);
            if let Some(lambda) = solve3(k_ang, -(wrel + e_ang * sp.bias_inv_dt)) {
                apply_ang_pair(bodies, ai, bi, lambda);
                max_dv = max_dv.max(lambda.length());
            }
        }
        JointKind::Revolute => {
            let axis_w = Mat3::from_quat(qa).mul_vec3(j.axis_a).normalize();
            let [t1, t2] = perp_basis(axis_w);
            let wrel = bodies.angvel(bi) - bodies.angvel(ai);
            let (k11, k12, k22) = (
                proj(k_ang, t1, t1),
                proj(k_ang, t1, t2),
                proj(k_ang, t2, t2),
            );
            let det = k11 * k22 - k12 * k12;
            if det.abs() >= 1e-12 {
                let b = sp.bias_inv_dt;
                let r1 = -(wrel.dot(t1) + e_ang.dot(t1) * b);
                let r2 = -(wrel.dot(t2) + e_ang.dot(t2) * b);
                let l1 = (r1 * k22 - r2 * k12) / det;
                let l2 = (r2 * k11 - r1 * k12) / det;
                apply_ang_pair(bodies, ai, bi, t1 * l1 + t2 * l2);
                max_dv = max_dv.max(l1.abs().max(l2.abs()));
            }
        }
        _ => {}
    }

    max_dv
}

#[cfg(test)]
mod tests {
    use super::angular_error;
    use vxl_phys_core::{BodySet, PhysConfig as Cfg, Quat, Shape, Vec3};

    use super::super::types::{Joint, JointKind, JointSet};

    /// **姿态偏置的契约**：固定关节从**初始转角误差**出发必须把相对转角修回去。
    /// 两个体的锚点都取体心（偏移 = ZERO）⇒ 线性行只锁平移、不产生力臂
    /// ⇒ **只有角行**能修姿态。判据非空洞：去掉姿态偏置（只消相对角速度，而
    /// 初角速度 = 0）时相对转角会**停在初值**（≈0.2 rad），本断言即红。
    #[test]
    fn fixed_joint_corrects_initial_orientation_error() {
        let mut bodies = BodySet::new();
        bodies.push_static(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(0.0, 10.0, 0.0),
            Quat::IDENTITY,
        );
        bodies.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(0.0, 10.0, 0.0),
            Quat::from_axis_angle(Vec3::Z, 0.2),
            500.0,
        );
        let ang_err = |b: &BodySet| -> f32 {
            let (_, qa) = b.pose(0);
            let (_, qb) = b.pose(1);
            angular_error(qa, qb).length()
        };
        let mut set = JointSet::default();
        set.add(Joint::new(JointKind::Fixed, 0, 1, Vec3::ZERO, Vec3::ZERO));
        let cfg = Cfg::default();
        let dt = cfg.dt / cfg.substeps.max(1) as f32;
        let e0 = ang_err(&bodies);
        let mut hold = e0;
        for _ in 0..240 {
            set.solve(&mut bodies, &cfg, dt);
            // 半步积分（与 `joints.rs` 的 `integrate` 同一数学：位置 += v·dt、
            // 姿态走 `Quat::integrate_angular`）——让关节真的"受力保持"。
            for i in 0..bodies.len() {
                if !bodies.is_dynamic(i) || !bodies.awake[i] {
                    continue;
                }
                bodies.position[i] = bodies.position[i] + bodies.linvel[i] * dt;
                let q = bodies.rot(i).integrate_angular(bodies.angvel(i), dt);
                bodies.set_rot(i, q);
            }
            hold = ang_err(&bodies);
        }
        assert!(e0 > 0.15, "初始转角误差应真的存在：{e0:.4} rad");
        assert!(
            hold < 0.02,
            "姿态偏置没把初始转角误差修回去：{e0:.4} → {hold:.4} rad"
        );
    }
}
