//! world_step/aero：**面元气动**（T4；`World::set_aero` 显式开启，`Option` 槽默认 `None`
//! ⇒ 未开启时 `aero_pass` 首行短路 ⇒ 默认档逐位不变）。
//!
//! **口径**（与 `vxl_phys_aero` 文档头一致）：每三角面 `F = ½ρ·Cd·A·u·|u|`（Bridson 线化，
//! `u = v_wind − v_face`，**全相对速度**）；面元速度 = `linvel + angvel×(面心 − 体原点)`
//! （与接触点速度同款口径，§8.4.30）。力/力矩在 `substep()` 第 1 段施加——**逐子步**
//! （`bodies.force/torque` 是逐子步累加器，§8.4.28 的已钉契约；同 `medium_pass` 段位）。
//! **确定性**：体按索引序、三角按注册序、纯 f32 ⇒ 可复现。
//!
//! **仪器与被测同源**：`AeroState.forces/torques` 留本子步施加的那份快照
//! （判据 ③ 经 `World::aero_force/aero_torque` 读它，不是另算一份）。
use super::*;
// 风的下发（布/液两条受体腿）单开文件：本文件受 god 门 file_lines 棘轮（只准减）。
#[path = "wind.rs"]
mod wind;

impl World {
    pub(crate) fn aero_pass(&mut self, dt: f32) {
        let Some(st) = self.aero.as_mut() else {
            return; // 未开启 ⇒ 零成本短路（默认档逐位不变）
        };
        let cfg = st.cfg; // Copy
        st.forces.clear();
        st.torques.clear();
        for i in 0..self.bodies.len() {
            let (f_sum, t_sum) = match self.bodies.shape[i] {
                Shape::TriMesh { mesh, .. } => body_aero(&self.bodies, &self.narrow, mesh, i, &cfg),
                _ => (Vec3::ZERO, Vec3::ZERO),
            };
            st.forces.push(f_sum);
            st.torques.push(t_sum);
            // 经受体门写（`PLAN-COUPLING.md` §3.2）：静态/睡眠体原先"写进去、被积分器丢弃"
            // ⇒ C1 起统一为提前不写。**快照（`st.forces/torques`）不受影响**（判据 ③ 读它）。
            coupling::add_force(&mut self.bodies, i, f_sum, t_sum);
        }
        // 受体腿（布 × 风 / 液 × 风）在 `wind.rs`：本文件受 god 门棘轮，加行必须让最长函数变短。
        wind::config_cloths(&mut self.soft.cloths, cfg);
        wind::drive_liquids(&mut self.fluids, st, cfg.drag_coefficient, dt);
    }
}

/// **单个体的面元气动力/力矩**（`|n_full| = 2A`；退化三角跳过 ⇒ 与注册期过滤叠加兜底）。
/// 从 `aero_pass` 抽出（那是本文件最长函数；god 门"合法交换"要求最长函数**严格下降**）。
fn body_aero(
    bodies: &BodySet,
    narrow: &crate::world_step::narrow_tier::NarrowSlot,
    mesh: u32,
    i: usize,
    cfg: &vxl_phys_aero::AeroConfig,
) -> (Vec3, Vec3) {
    let wind = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
    let pos = bodies.position[i];
    let m = vxl_phys_core::Mat3::from_quat(bodies.rot(i));
    let v_body = bodies.linvel[i];
    let ang = bodies.angvel(i);
    let (mut f_sum, mut t_sum) = (Vec3::ZERO, Vec3::ZERO);
    let tris: &[[u32; 3]] = narrow.mesh_tris(mesh);
    let pts: &[Vec3] = narrow.mesh_points(mesh);
    for tri in tris {
        let (p0, p1, p2) = (
            pos + m.mul_vec3(pts[tri[0] as usize]),
            pos + m.mul_vec3(pts[tri[1] as usize]),
            pos + m.mul_vec3(pts[tri[2] as usize]),
        );
        let two_a = (p1 - p0).cross(p2 - p0).length(); // |n_full| = 2A
        if two_a <= 1e-12 {
            continue;
        }
        let center = (p0 + p1 + p2) * (1.0 / 3.0);
        let v_face = v_body + ang.cross(center - pos);
        let f = vxl_phys_aero::face_force(wind - v_face, two_a * 0.5, cfg);
        f_sum += f;
        t_sum += (center - pos).cross(f);
    }
    (f_sum, t_sum)
}
