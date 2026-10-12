//! **CCD 之后的位置级关节投影**（`OPEN-PROBLEMS.md` **P17** 的修法）—— `joints` 的子模块。
//!
//! **为什么需要**：CCD 扫掠命中后把体**钳位到上一安全采样点**，这一步只按**单体**位移算、
//! **不看关节约束** ⇒ 关节链的相邻节各自被钳位 ⇒ 锚点分离被拉大（实测 R6：10 节球链的
//! `sep_p99` 0.00966 → 0.12382，13×）。本模块在 CCD 之后按 **PBD 口径**把锚点拉回：
//!
//! `λ = K⁻¹·(−err)`（`K` = 与速度级**同源**的 3×3 点约束有效质量），
//! 然后**直接改位置/姿态**（镜像速度级 `apply_pair` 的符号）：
//! `x_a −= inv_m_a·λ`、`θ_a −= I_a⁻¹(r_a×λ)`；b 侧取反。
//!
//! **只改位置与姿态，不碰速度** ⇒ 不注入动能（CCD 的"清法向速度"语义保持不变），
//! 关节的速度级误差留给下一子步的 `JointSet::solve`。
//!
//! ⚠️ **两条实测教训**（第一版都没做，读数反升）：
//! 1. **必须沿关节连通分量传播**：只修"被钳的那一条"时，tip 的分离从 0.084 修到 0.003 ✓，
//!    但误差被**推给邻接**（0.007 → 0.08）⇒ 全链 p99 不降反升（0.12397 → 0.12605）；
//! 2. **迭代数要够走完一条链**（`PROJECT_ITERS = 8`）：闭包 + 8 遍后开档回到 `0.00984`。
//!
//! **零成本保证**：只处理"本子步被 CCD 标记过"的连通分量（`flagged` 空 ⇒ 首行短路）；
//! CCD 关档时 `flagged` 恒空 ⇒ 既有场景**逐位不变**。

use super::{point_k, solve3, JointSet};
use crate::{BodySet, Mat3, Vec3};

/// 位置级 Gauss-Seidel 的迭代遍数（链式结构的误差沿链传播，见模块头注的教训 2）。
const PROJECT_ITERS: u32 = 8;

impl JointSet {
    /// 把"被 CCD 钳过"的体所在**关节连通分量**的锚点位置级拉回（见模块头注）。
    pub fn project_after_ccd(&self, bodies: &mut BodySet, flagged: &[usize]) {
        if flagged.is_empty() || self.joints.is_empty() {
            return;
        }
        let n = bodies.len();
        let mut touched = vec![false; n];
        for &i in flagged {
            if i < n {
                touched[i] = true;
            }
        }
        self.propagate_touched(&mut touched, n);
        for _ in 0..PROJECT_ITERS {
            self.project_pass(bodies, &touched, n);
        }
    }

    /// **关节连通闭包**：从被钳体出发，把整条链标进 `touched`（确定性：关节序 + 升序索引；
    /// 遍数上限 = 关节数，链最长就是每遍往下传一节）。
    fn propagate_touched(&self, touched: &mut [bool], n: usize) {
        for _ in 0..self.joints.len() {
            let mut changed = false;
            for j in &self.joints {
                let (ai, bi) = (j.a as usize, j.b as usize);
                if ai >= n || bi >= n || touched[ai] == touched[bi] {
                    continue;
                }
                touched[ai] = true;
                touched[bi] = true;
                changed = true;
            }
            if !changed {
                return;
            }
        }
    }

    /// 一遍位置级投影（Gauss-Seidel，关节序固定；只碰 `touched` 里的关节）。
    fn project_pass(&self, bodies: &mut BodySet, touched: &[bool], n: usize) {
        for j in &self.joints {
            let (ai, bi) = (j.a as usize, j.b as usize);
            if ai >= n || bi >= n || (!touched[ai] && !touched[bi]) {
                continue;
            }
            if !bodies.is_dynamic_or_kinematic(ai) && !bodies.is_dynamic_or_kinematic(bi) {
                continue;
            }
            let ra = Mat3::from_quat(bodies.rot(ai)).mul_vec3(j.anchor_a);
            let rb = Mat3::from_quat(bodies.rot(bi)).mul_vec3(j.anchor_b);
            // err = b 侧锚点 − a 侧锚点（与速度级 `rel_vel` 同口径）。
            let err = (bodies.position[bi] + rb) - (bodies.position[ai] + ra);
            if err.length_squared() < 1e-12 {
                continue;
            }
            if let Some(lambda) = solve3(point_k(bodies, ai, bi, ra, rb), -err) {
                apply_pos_pair(bodies, ai, bi, ra, rb, lambda);
            }
        }
    }
}

/// 位置/姿态版的对施加：**镜像**速度级 `apply_pair` 的符号（a 侧减、b 侧加）。
#[inline]
fn apply_pos_pair(bodies: &mut BodySet, ai: usize, bi: usize, ra: Vec3, rb: Vec3, lambda: Vec3) {
    let (im_a, im_b) = (bodies.inv_mass[ai], bodies.inv_mass[bi]);
    bodies.position[ai] -= lambda * im_a;
    bodies.position[bi] += lambda * im_b;
    let da = bodies.apply_world_inv_inertia(ai, ra.cross(lambda));
    let db = bodies.apply_world_inv_inertia(bi, rb.cross(lambda));
    bodies.set_rot(ai, bodies.rot(ai).integrate_angular(-da, 1.0));
    bodies.set_rot(bi, bodies.rot(bi).integrate_angular(db, 1.0));
}
