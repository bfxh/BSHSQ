//! **点-边对的自摩擦**（切向库仑锥）—— 与点-点自摩擦（`cloth_self_friction.rs`）**同一条物理**，
//! 差别只在"相对滑移"的定义：那边是两粒子，这边是**粒子 vs 边上投影点**（接触点的位移按
//! 重心参数 `u` 在两端点上插值）。
//!
//! **为什么是独立文件 + 自由函数**：唯一的调用点在 `cloth_self_collision/edge.rs::project`，
//! 而那个函数正卡在 god 门的**最长函数**棘轮上（改动的全部行数必须落在别处，调用点只留一行）。
//!
//! **口径**（逐条对齐 `cloth_self_friction::resist_slip`）：
//! 相对滑移 `rel = Δp_接触点 − Δp_粒子`（`Δp = pos − prev` = 本子步的实际位移），
//! 切向分量 `tan = rel − n̂(rel·n̂)`、模长 `slip = |tan|`；预算 = `μ·depth`
//! （`depth` = 该对的**相对**法向修正量 `d_c − len`）；锥内整段吃掉（静摩擦）、超出按动摩擦滑；
//! 分摊按逆质量，接触点那一侧再乘**杠杆权重** `(1−u)` / `u`。
//!
//! **默认 `μ = 0`**（`SelfCollision::friction` —— 规格书没规定自摩擦，本仓默认关）
//! ⇒ 首行短路 ⇒ 点-边的既有判据读数**逐位不变**。
use vxl_phys_core::Vec3;

/// 把「粒子 `i` × 边 `(j,k)`」的相对切向滑移按库仑锥扣掉。
///
/// 首参传**三个字段切片**而不是整张 `&mut ClothSheet`：调用点正借着
/// `self_contacts.edges.seen`（候选表）⇒ 整表可变借用会冲突，字段级借用才相容
/// （`pos` / `prev` / `inv_mass` 与 `self_contacts` 是不同的字段）。
///
/// `u` = 投影点在边上的重心参数；`w` = 该对的**有效逆质量**（法向那支用的同一个 `w`，
/// 见 `edge::project` 的杠杆权重）；`nrm` = 接触点指向粒子 `i` 的单位法线；
/// `depth` = 相对法向修正量 `d_c − len`。`μ ≤ 0` / 参数退化 ⇒ 直接返回（默认档零成本）。
pub(crate) fn resist_edge_slip(
    (pos, prev, inv_mass): (&mut [Vec3], &[Vec3], &[f32]),
    mu: f32,
    (i, j, k): (usize, usize, usize),
    (u, w): (f32, f32),
    (nrm, depth): (Vec3, f32),
) {
    if mu <= 0.0 || w <= 0.0 || depth <= 0.0 {
        return;
    }
    let dp_i = pos[i] - prev[i];
    let dp_c = (pos[j] - prev[j]) * (1.0 - u) + (pos[k] - prev[k]) * u;
    let rel = dp_c - dp_i;
    let tan = rel - nrm * rel.dot(nrm);
    let slip = tan.length();
    if slip <= 0.0 {
        return;
    }
    let k_frac = slip.min(mu * depth) / slip;
    let (w_i, w_j, w_k) = (inv_mass[i], inv_mass[j], inv_mass[k]);
    pos[i] += tan * (k_frac * (w_i / w));
    pos[j] -= tan * (k_frac * (w_j * (1.0 - u) / w));
    pos[k] -= tan * (k_frac * (w_k * u / w));
}

#[cfg(test)]
mod tests {
    use super::resist_edge_slip;
    use vxl_phys_core::Vec3;

    /// 最小场景：`i` = 粒子（自由）、`(j,k)` = 边的两端（**钉住** ⇒ `inv_mass = 0`）
    /// ⇒ `w = w_i`，摩擦修正全部落在粒子自己身上（判据读起来不需要摊派算术）。
    /// 边沿 `x` 从 −0.1 到 +0.1；法线取 `+y`（接触点在边中点、粒子在它上方）。
    fn scene(prev_i: Vec3, pos_i: Vec3) -> (Vec<Vec3>, Vec<Vec3>, Vec<f32>) {
        let a = Vec3::new(-0.1, 0.0, 0.0);
        let b = Vec3::new(0.1, 0.0, 0.0);
        (vec![pos_i, a, b], vec![prev_i, a, b], vec![2.0, 0.0, 0.0])
    }

    /// ① 纯切向滑移、预算够 ⇒ **完全粘住**（静摩擦：整段扣回）。
    #[test]
    fn tangential_slip_is_fully_removed_within_budget() {
        let nrm = Vec3::new(0.0, 1.0, 0.0);
        let (mut pos, prev, inv) = scene(Vec3::ZERO, Vec3::new(0.01, 0.0, 0.0));
        resist_edge_slip(
            (&mut pos, &prev, &inv),
            0.5,
            (0, 1, 2),
            (0.5, 2.0),
            (nrm, 0.03),
        );
        assert!(pos[0].x.abs() < 1e-6, "切向滑移没被扣回：{:?}", pos[0]);
        assert_eq!(pos[0].y, 0.0, "只该动切向，不该碰法向");
    }

    /// ② `μ = 0`（默认）⇒ **逐位不动**（默认档零成本承诺）。
    #[test]
    fn zero_friction_is_bitwise_noop() {
        let nrm = Vec3::new(0.0, 1.0, 0.0);
        let (mut pos, prev, inv) = scene(Vec3::ZERO, Vec3::new(0.01, 0.0, 0.0));
        let before = pos[0];
        resist_edge_slip(
            (&mut pos, &prev, &inv),
            0.0,
            (0, 1, 2),
            (0.5, 2.0),
            (nrm, 0.03),
        );
        assert_eq!(pos[0], before);
    }

    /// ③ 只扣**切向**：纯法向位移（切向分量为 0）完全不动。
    #[test]
    fn normal_motion_is_untouched() {
        let nrm = Vec3::new(0.0, 1.0, 0.0);
        let (mut pos, prev, inv) = scene(Vec3::ZERO, Vec3::new(0.0, 0.01, 0.0));
        let before = pos[0];
        resist_edge_slip(
            (&mut pos, &prev, &inv),
            0.5,
            (0, 1, 2),
            (0.5, 2.0),
            (nrm, 0.03),
        );
        assert_eq!(pos[0], before);
    }

    /// ④ 滑移超出预算 ⇒ 按**动摩擦**只扣 `μ·depth`，剩下的照滑。
    #[test]
    fn beyond_budget_removes_exactly_mu_times_depth() {
        let nrm = Vec3::new(0.0, 1.0, 0.0);
        let (slip, depth, mu) = (0.1f32, 0.03f32, 0.5f32);
        let (mut pos, prev, inv) = scene(Vec3::ZERO, Vec3::new(slip, 0.0, 0.0));
        resist_edge_slip(
            (&mut pos, &prev, &inv),
            mu,
            (0, 1, 2),
            (0.5, 2.0),
            (nrm, depth),
        );
        let want = slip - mu * depth;
        assert!(
            (pos[0].x - want).abs() < 1e-6,
            "剩余滑移 {} ≠ 期望 {want}",
            pos[0].x
        );
    }
}
