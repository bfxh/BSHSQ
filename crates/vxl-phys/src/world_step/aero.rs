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

impl World {
    pub(crate) fn aero_pass(&mut self) {
        let Some(st) = self.aero.as_mut() else {
            return; // 未开启 ⇒ 零成本短路（默认档逐位不变）
        };
        let cfg = st.cfg; // Copy
        let wind = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
        st.forces.clear();
        st.torques.clear();
        for i in 0..self.bodies.len() {
            let Shape::TriMesh { mesh, .. } = self.bodies.shape[i] else {
                st.forces.push(Vec3::ZERO);
                st.torques.push(Vec3::ZERO);
                continue;
            };
            let pos = self.bodies.position[i];
            let rot = self.bodies.rot(i);
            let v_body = self.bodies.linvel[i];
            let w = self.bodies.angvel(i);
            let m = vxl_phys_core::Mat3::from_quat(rot);
            let mut f_sum = Vec3::ZERO;
            let mut t_sum = Vec3::ZERO;
            let tris: &[[u32; 3]] = self.narrow.mesh_tris(mesh);
            if !tris.is_empty() {
                let pts: &[Vec3] = self.narrow.mesh_points(mesh);
                for tri in tris {
                    let (p0, p1, p2) = (
                        pos + m.mul_vec3(pts[tri[0] as usize]),
                        pos + m.mul_vec3(pts[tri[1] as usize]),
                        pos + m.mul_vec3(pts[tri[2] as usize]),
                    );
                    let n_full = (p1 - p0).cross(p2 - p0); // |n_full| = 2A
                    let two_a = n_full.length();
                    if two_a <= 1e-12 {
                        continue; // 注册期已滤退化三角，此处兜底
                    }
                    let center = (p0 + p1 + p2) * (1.0 / 3.0);
                    let v_face = v_body + w.cross(center - pos);
                    let f = vxl_phys_aero::face_force(wind - v_face, two_a * 0.5, &cfg);
                    f_sum += f;
                    t_sum += (center - pos).cross(f);
                }
            }
            st.forces.push(f_sum);
            st.torques.push(t_sum);
            // 经受体门写（`PLAN-COUPLING.md` §3.2）：静态/睡眠体原先"写进去、被积分器丢弃"
            // ⇒ C1 起统一为提前不写。**快照（`st.forces/torques`）不受影响**（判据 ③ 读它）。
            coupling::add_force(&mut self.bodies, i, f_sum, t_sum);
        }
    }
}
