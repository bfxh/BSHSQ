//! **布片气动消费**（切片 T4 收尾）：`vxl_phys_aero::face_force` 的**消费方**。
//!
//! **落点**：每**三角面**用**面心相对风** `u = v_wind − v_face_center` 算
//! `F = ½·ρ·Cd·A·u·|u|`（Bridson 线化；`v_face_center` = 三角三顶点速度均值），`F/3` 分给三个
//! 顶点作**冲量** `Δv_i = (F/3)·inv_mass_i·h`（在 `predict` 里、与**重力同段位**逐子步施加）。
//!
//! **两个口径沿用 `vxl-phys-aero` 的既有约定**（不是本片新定的）：
//! - **`u` 是"全相对速度"**（含切向）⇒ 面元与风**平行**时也有力（`F ∥ u`，**不是** `F ∥ 法向`）
//!   ⇒ 判据里专门有一条钉它（"法向口径"会给出零力，两者差得很远）；
//! - **`u = 0 ⇒ F = 0` 逐位成立** ⇒ 零风场景与"气动关"**逐位相同**（金丝雀判据靠这条）。
//!
//! **边界（写清，不是漏）**：本档**不含**力矩（顶点力天然给出分布力矩，但没有单独立"面元力矩"
//! 这条量）、不含升力线斜率（`lift_slope` 在骨架里但 Bridson 线化式不用它）、面积退化面跳过
//! （`area ≤ 0` ⇒ 法向无定义，与 `project_constraints` 的 `len < 1e-9` 同惯例）。
use crate::cloth::ClothSheet;
use vxl_phys_aero::{face_force, AeroConfig};
use vxl_phys_core::Vec3;

/// **气动输入**（**默认关**）。
///
/// **默认关的意义**：`predict` 首行短路 ⇒ 既有场景**逐位不变**（三条冻结哈希与 `gold` 门不动）
/// ⇒ 与门面 `World::set_aero` 的 `Option` 槽、`angular_reaction` 的 `bool` 同款"0 = 关"先例。
#[derive(Clone, Copy, Debug, Default)]
pub struct ClothAero {
    /// `false`（默认）⇒ 不消费风（零成本短路）。
    pub enabled: bool,
    /// 气动配置（**风在 `AeroConfig::wind` 里**，与门面气动域**同一个骨架**）。
    pub cfg: AeroConfig,
}

impl ClothSheet {
    /// **逐子步施加气动力**（`predict` 里、重力之前）：逐三角面算面元气动力并**均分到三个顶点**。
    ///
    /// **必须"先算全部、再统一施加"（Jacobi）**：逐面**就地**施加会让**后面的面**读到已被
    /// 前面的面改过的顶点速度 ⇒ 相对风被虚假削弱 ⇒ 系统性地**偏小**（实测：法向风的总动量比
    /// 解析低 **0.63%**）。物理上各面元的力是**同时**作用的，所以这里先按**子步开始时的**顶点
    /// 速度把力全算出来（用 `v0` 快照），再一次性加到 `vel` 上。
    ///
    /// ⚠️ **`v0` 是局部 `Vec`（每子步一次分配）**：`ClothSheet` 的成员棘轮**已顶格 24/24**
    /// （见 `aero` 字段的注），放不下这个 scratch。代价只在 `enabled` 时发生（默认关 ⇒ 零成本）；
    /// **下一片腾出成员时应把它并进某个组结构**（本仓反"热路径分配"的惯例）。
    pub(crate) fn apply_aero(&mut self, h: f32) {
        let cfg = self.aero.cfg;
        let wind = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
        let v0 = self.vel.clone();
        for t in 0..self.tris.len() {
            let [ia, ib, ic] = self.tris[t];
            let (a, b, c) = (ia as usize, ib as usize, ic as usize);
            let (pa, pb, pc) = (self.pos[a], self.pos[b], self.pos[c]);
            let area = (pb - pa).cross(pc - pa).length() * 0.5;
            if area <= 0.0 {
                continue; // 退化面：法向无定义
            }
            let v_c = (v0[a] + v0[b] + v0[c]) * (1.0 / 3.0);
            let f3 = face_force(wind - v_c, area, &cfg) * (1.0 / 3.0);
            for i in [a, b, c] {
                if self.inv_mass[i] == 0.0 {
                    continue; // 钉住粒子不受力（也不该被推）
                }
                self.vel[i] += f3 * (self.inv_mass[i] * h);
            }
        }
    }
}
