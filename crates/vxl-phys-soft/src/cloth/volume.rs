//! **体积 / 气压约束**（`SPEC.md` §4.6「体积守恒：Müller 2007 压力约束（气压系数 k）」）——
//! 封闭三角网的**有向体积**约束，按 XPBD 口径投影（Müller et al. 2020，§4 的 compliance 形式）。
//!
//! 用途：气球 / 充气结构 / 封闭软壳。**开口网格**（体积无定义）不该开它；三角形**绕序须
//! 一致向外**，否则有向体积会自相抵消（这是调用方的几何契约，本模块不做朝向修复）。
//!
//! **默认关**：`k ≤ 0` ⇒ 首行短路 ⇒ 默认档逐位不变（与其它约束族同款）。
//!
//! **数学**（有向体积）：
//! `V = (1/6) Σ_tri (p_a × p_b) · p_c`、`C = V − V0`；
//! `∇_a C = (1/6) (p_b × p_c)`（`b`/`c` 轮换，1/6 计入梯度以保持 XPBD 的物理量纲）。
//! 投影：`λ = −C / (Σ_j w_j |∇_j C|² + α̃)`、`Δp_i = λ w_i ∇_i C`、`α̃ = (1/k)/h²`
//! ⇒ **k 越大，体积越贴近 `target`**（`k → ∞` = 刚性体积约束）。
use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

/// 气压 / 体积约束参数（`k ≤ 0` = 关；见模块头注的公式）。
#[derive(Clone, Default)]
pub struct VolumePressure {
    /// 气压刚度（`1/compliance` 口径；越大越贴近 [`Self::target`]；`≤ 0` = 关）。
    pub k: f32,
    /// 目标体积 `V0`；`≤ 0` ⇒ **首次投影时锁定为当时的体积**（注册态 = 零应变态）。
    pub target: f32,
    /// 逐顶点梯度 scratch（`resize` 复用 ⇒ 每子步零分配）。
    pub(crate) grad: Vec<Vec3>,
}

/// 封闭三角网的**有向体积**（绕序一致向外时为正；见模块头注的几何契约）。
pub fn mesh_volume(pos: &[Vec3], tris: &[[u32; 3]]) -> f32 {
    let mut six_v = 0.0f32;
    for &[a, b, c] in tris {
        let (pa, pb, pc) = (pos[a as usize], pos[b as usize], pos[c as usize]);
        six_v += pa.cross(pb).dot(pc);
    }
    six_v / 6.0
}

/// 一次 XPBD 体积投影（`k ≤ 0` / 空网格 / 非有限 `h` ⇒ 直接返回）。
pub(crate) fn project_volume(cs: &mut ClothSheet, h: f32, k: f32, target: f32) {
    // `k`/`h` 须为**有限正数**（NaN、0、负、∞ 一律视为关 ⇒ 与其它域的"非有限 = 关"同口径）。
    let ok_k = k.is_finite() && k > 0.0;
    let ok_h = h.is_finite() && h > 0.0;
    if !ok_k || !ok_h || cs.tris.is_empty() {
        return;
    }
    let n = cs.pos.len();
    let grad = &mut cs.volume.grad;
    grad.clear();
    grad.resize(n, Vec3::ZERO);
    let mut six_v = 0.0f32;
    for &[a, b, c] in &cs.tris {
        let (i, j, m) = (a as usize, b as usize, c as usize);
        let (pi, pj, pm) = (cs.pos[i], cs.pos[j], cs.pos[m]);
        six_v += pi.cross(pj).dot(pm);
        grad[i] += pj.cross(pm);
        grad[j] += pm.cross(pi);
        grad[m] += pi.cross(pj);
    }
    let vol = six_v / 6.0;
    // `target ≤ 0` ⇒ **首次调用锁定**（此后每子步都用同一 V0，否则约束恒为 0 = 空转）。
    let v0 = if target > 0.0 { target } else { vol };
    let c = vol - v0;
    let a_tilde = (1.0 / k) / (h * h);
    let inv6 = 1.0 / 6.0;
    let mut denom = 0.0f32;
    for (i, g) in grad.iter().enumerate() {
        denom += cs.inv_mass[i] * ((*g) * inv6).length_squared();
    }
    let lambda = -c / (denom + a_tilde);
    for (i, g) in grad.iter().enumerate() {
        cs.pos[i] += ((*g) * inv6) * (lambda * cs.inv_mass[i]);
    }
}

#[cfg(test)]
mod tests {
    use super::{mesh_volume, project_volume};
    use crate::cloth::ClothSheet;
    use crate::params::Stiffness;
    use vxl_phys_core::Vec3;

    /// 半径 `r` 的正八面体，**面绕序自动定向向外**（法线 · 面重心 ≥ 0）——
    /// 体积有解析值 `4/3·r³`，用它把"公式 + 朝向"一起钉住（绕序反了体积会变号）。
    fn octahedron(r: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
        let pos = vec![
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(-r, 0.0, 0.0),
            Vec3::new(0.0, r, 0.0),
            Vec3::new(0.0, -r, 0.0),
            Vec3::new(0.0, 0.0, r),
            Vec3::new(0.0, 0.0, -r),
        ];
        let raw = [
            [0u32, 2, 4],
            [0, 4, 3],
            [0, 3, 5],
            [0, 5, 2],
            [1, 4, 2],
            [1, 3, 4],
            [1, 5, 3],
            [1, 2, 5],
        ];
        let tris = raw
            .iter()
            .map(|&[a, b, c]| {
                let (pa, pb, pc) = (pos[a as usize], pos[b as usize], pos[c as usize]);
                let n = (pb - pa).cross(pc - pa);
                if n.dot((pa + pb + pc) * (1.0 / 3.0)) < 0.0 {
                    [a, c, b]
                } else {
                    [a, b, c]
                }
            })
            .collect();
        (pos, tris)
    }

    fn sheet(r: f32) -> ClothSheet {
        let (pos, tris) = octahedron(r);
        ClothSheet::new(pos, tris, 1000.0, 0.01, Stiffness::Hard)
    }

    /// ① 解析校验：八面体有向体积 = `4/3·r³`，且定向后为**正**。
    #[test]
    fn octahedron_volume_matches_analytic() {
        let (pos, tris) = octahedron(0.5);
        let v = mesh_volume(&pos, &tris);
        let want = 4.0 / 3.0 * 0.5f32.powi(3);
        assert!((v - want).abs() < 1e-6, "有向体积 {v} ≠ 解析值 {want}");
    }

    /// ② `k ≤ 0` = 关：投影**不动**粒子（默认档不被本域碰到）。
    #[test]
    fn disabled_leaves_positions_untouched() {
        let mut cs = sheet(0.5);
        cs.pos[0] *= 1.3;
        let p0 = cs.pos[0];
        let v0 = mesh_volume(&cs.pos, &cs.tris);
        project_volume(&mut cs, 1.0 / 60.0, 0.0, 0.0);
        assert_eq!(cs.pos[0], p0);
        assert_eq!(mesh_volume(&cs.pos, &cs.tris), v0);
    }

    /// ③ 单次投影把体积**拉向** `target`，且 `k` 越大残差越小（单调）。
    #[test]
    fn projection_pulls_volume_and_k_is_monotone() {
        let s = sheet(0.5);
        let target = mesh_volume(&s.pos, &s.tris);
        let mut prev = f32::INFINITY;
        for k in [1e2f32, 1e4, 1e6, 1e8] {
            let mut cs = sheet(0.5);
            for p in cs.pos.iter_mut() {
                *p *= 1.05; // 统一外推 5% ⇒ 体积变大，投影应把它拉回
            }
            project_volume(&mut cs, 1.0 / 60.0, k, target);
            let err = (mesh_volume(&cs.pos, &cs.tris) - target).abs();
            assert!(err < prev, "k={k} 的残差 {err} 未小于上一档 {prev}");
            prev = err;
        }
    }
}
