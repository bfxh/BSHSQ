//! **自由旋转的陀螺项**（欧拉方程 `ω̇ = I⁻¹·(τ − ω × (I·ω))`）—— 从 `lib.rs` 抽出的独立关注点：
//! 那里受 god 门的**文件行数 + 最长函数**双棘轮（`integrate_velocities` 正卡在上限），
//! 按 `mesh_mesh.rs` / `mesh_cross.rs` 的先例另起一个文件、调用点只多一行。
//!
//! **为什么必须有**（`SURVEY-RIGID-HARD-CASES §1.2` 列的"三处有意的近似"之首，2026-10-14 实修）：
//! 原来只做 `ω += I⁻¹_world·τ·dt` ⇒ 无接触自由旋转的**非球体**里 `ω` 在世界系恒定 ⇒ 体绕**固定轴**
//! 转，而 `I_world = R·I_local·Rᵀ` 随姿态变 ⇒ **`L = I_world·ω` 的矢量不守恒**。
//! 实测（`examples/free_rotation_probe.rs`，1200 步 / 20 s、无重力无接触）：
//!
//! | 用例 | 修前 `|ΔL|/|L|` | 修前进动率 | 解析值 |
//! |---|---|---|---|
//! | 不对称盒 2×1×0.5（非主轴自旋 ω = 6） | **0.622** | — | 极迹摆动（网球拍定理） |
//! | 对称陀螺（方盒 1×1×0.5，ω = (1,0,6)） | 0.123 | **0.000 rad/s** | `Ω = ω₃(I₃−I₁)/I₁` ≈ **3.6 rad/s** |
//!
//! ⚠️ **动能读数抓不到它**：`ω` 恒定 ⇒ KE = `½ω·L` 也恒定（实测两档 KE 漂移都 ~1e-6）⇒ 判据必须
//! 落在 **L 矢量**（或对称陀螺的进动率）上 —— 首版探针只量了 `|L|` 的模长，读数全 0（踩过，别再犯）。
//!
//! ⚠️ **推进格式：显式被实测否证 ⇒ 用隐式中点**。显式 `ωₙ = ω + dt·f(ω)` 对不对称体**发散**：
//! 同一个不对称盒 1200 步后动能 **+323%**（`ω₃` 摆到 −9.2…+8.7）、对称陀螺动能 **+13%**。
//! 隐式中点 `ωₙ = ω + dt·f((ω+ωₙ)/2)` 用**固定点迭代**解（3 遍足够：`dt·|ω| ≈ 0.05` ⇒ 迭代算子
//! 范数 ≪ 1），代价 = 每体每步 3 次 `I⁻¹` 与 3 次叉积。
//!
//! **逐位不变的两档**（换代面守门的核心）：`ω ≡ 0` ⇒ 陀螺项为 0（扣 0 是精确运算）；
//! **各向同性惯量**（球 / 正方体）⇒ `ω × (I·ω) = I·(ω × ω) = 0` ⇒ 一字不动。
use vxl_phys_core::{BodySet, Mat3, Vec3};

/// **角速度的一步积分**：`ω += I⁻¹·(τ − ω × (I·ω))·dt`（欧拉方程）+ **隐式中点**推进。
/// 收在这里是为了让 `lib.rs` 的调用点只有**一行**（那边的 `integrate_velocities` 卡在最长函数上限）。
pub(crate) fn integrate_angular(bodies: &mut BodySet, i: usize, dt: f32) {
    let inv = bodies.local_inv_inertia[i];
    // 早退①：**各向同性惯量**（球 / 立方体）⇒ `ω × (I·ω) = I·(ω×ω) = 0` ⇒ 与旧式逐位相同，
    // 且省掉整段迭代（球是最常见的一族）。
    // 早退②：退化体（`I⁻¹ = 0`）⇒ 陀螺项 0（也免得除零）。
    if (inv.x == inv.y && inv.y == inv.z) || inv.x <= 0.0 || inv.y <= 0.0 || inv.z <= 0.0 {
        let accel = bodies.apply_world_inv_inertia(i, bodies.torque[i]);
        bodies.set_angvel_raw(i, bodies.angvel(i) + accel * dt);
        return;
    }
    let w = bodies.angvel(i);
    // 早退③：**自由进动率快过 dt 能分辨的尺度 ⇒ 退回旧式并如实登记**（不是"正确"，是取舍）。
    // 量纲：`ω·dt·(I_max/I_min − 1)` = 每步里"体坐标系自转"的相位角；≳ 1 时无论显式还是隐式
    // 格式都解不动（隐式中点的固定点迭代在该处不再收缩）。实测（`examples/free_rotation_probe.rs`
    // 的细杆档 + `tests/ccd_energy.rs` ③）：细杆（`I_⊥/I_∥ ≈ 1.2e3`、ω = 40、dt = 1/120）该值为
    // **416**，此时"撞墙给的横向冲量"会把动能顶到 **×3.19**（远离主轴的姿态解不动）；
    // 而 2:1:0.5 的不对称盒在 ω = 6…40 该值只有 **0.03…0.2** ⇒ 正常进动（探针实测进动率与解析
    // `ω₃(I₃−I₁)/I₁` 差 < 0.1%）。
    let aniso = (inv.x.min(inv.y).min(inv.z), inv.x.max(inv.y).max(inv.z));
    let stiff = aniso.1 / aniso.0 - 1.0;
    if w.length() * dt * stiff > 1.0 {
        let accel = bodies.apply_world_inv_inertia(i, bodies.torque[i]);
        bodies.set_angvel_raw(i, w + accel * dt);
        return;
    }
    let tau = bodies.torque[i];
    let mut mid = w;
    for _ in 0..3 {
        mid = w + with_term(bodies, i, tau, mid) * (0.5 * dt);
    }
    bodies.set_angvel_raw(i, w + with_term(bodies, i, tau, mid) * dt);
}

/// `ω × (I_world·ω)`（世界系，取体当前 `ω`）—— 供**单元测试**直接读（生产路径只走
/// [`integrate_angular`]；这里 `cfg(test)` 是为了不欠一条 `dead_code` 豁免）。
#[cfg(test)]
pub(crate) fn spin_term(bodies: &BodySet, i: usize) -> Vec3 {
    inertia_sandwich(bodies, i, bodies.angvel(i))
}

/// `I⁻¹·(τ − ω × (I·ω))`（给定 `ω`；隐式迭代用）。
fn with_term(bodies: &BodySet, i: usize, tau: Vec3, w: Vec3) -> Vec3 {
    bodies.apply_world_inv_inertia(i, tau - inertia_sandwich(bodies, i, w))
}

/// `ω × (I_world·ω)`：`I_world·ω` 走**局部系**算（本仓局部惯量是**对角**：`local_inv_inertia` 存
/// `I⁻¹` 的分量 ⇒ 正向惯量 = 逐分量倒数）：`ω_local = Rᵀω`、`I·ω|_local = ω_local / I⁻¹_local`、
/// 再 `R·(·)` 回世界系。⚠️ 退化体（`inv_inertia` 有分量 0：质量 0 的兜底体）**返回 0**，否则除零造 NaN。
fn inertia_sandwich(bodies: &BodySet, i: usize, w: Vec3) -> Vec3 {
    let inv = bodies.local_inv_inertia[i];
    if w == Vec3::ZERO || inv.x <= 0.0 || inv.y <= 0.0 || inv.z <= 0.0 {
        return Vec3::ZERO;
    }
    let r = Mat3::from_quat(bodies.rot(i));
    let w_l = r.transpose_mul_vec3(w);
    let l_world = r.mul_vec3(Vec3::new(w_l.x / inv.x, w_l.y / inv.y, w_l.z / inv.z));
    w.cross(l_world)
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use super::{integrate_angular, spin_term};
    use vxl_phys_core::{BodySet, Quat, Shape, Vec3};

    fn box_body(half: Vec3, w: Vec3) -> (BodySet, usize) {
        let mut b = BodySet::new();
        let i = b.push_dynamic(Shape::Box { half }, Vec3::ZERO, Quat::IDENTITY, 1000.0) as usize;
        b.linvel.set_ang(i, w);
        (b, i)
    }

    /// 球（各向同性惯量）⇒ 陀螺项**恒 0**（换代面守门的核心一条）。
    #[test]
    fn isotropic_body_has_no_spin_term() {
        let mut b = BodySet::new();
        let i = b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        b.linvel.set_ang(i, Vec3::new(3.0, -1.0, 2.0));
        assert_eq!(spin_term(&b, i), Vec3::ZERO);
    }

    /// 退化体（`I⁻¹ = 0`）⇒ 0（不是 NaN）。
    #[test]
    fn degenerate_inertia_is_zero_not_nan() {
        let (mut b, i) = box_body(Vec3::splat(0.5), Vec3::new(1.0, 0.0, 0.0));
        b.local_inv_inertia[i] = Vec3::ZERO;
        assert_eq!(spin_term(&b, i), Vec3::ZERO);
    }

    /// 非主轴自旋的不对称盒 ⇒ 陀螺项非 0，且与 `ω` 正交（`(ω×Iω)·ω = 0`）。
    #[test]
    fn anisotropic_spin_term_is_orthogonal() {
        let (b, i) = box_body(Vec3::new(1.0, 0.5, 0.25), Vec3::new(2.0, 5.0, 0.8));
        let g = spin_term(&b, i);
        assert!(g.length() > 1.0, "非主轴自旋必须有陀螺项：{g:?}");
        assert!(g.dot(Vec3::new(2.0, 5.0, 0.8)).abs() < 1e-3, "必与 ω 正交");
    }

    /// 各向同性 + 纯力矩 ⇒ 一步积分与无陀螺项的解析值一致（守"换代面只在各向异性到期"）。
    #[test]
    fn isotropic_step_matches_analytic() {
        let mut b = BodySet::new();
        let i = b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        let dt = 1.0 / 60.0;
        let tau = Vec3::new(1.0, 0.0, 0.0);
        // 解析：ω = I⁻¹·τ·dt（球：I = 2/5·m·r²）
        let expected = b.apply_world_inv_inertia(i, tau) * dt;
        b.torque[i] = tau;
        integrate_angular(&mut b, i, dt);
        let got = b.angvel(i);
        assert!(
            (got - expected).length() < 1e-6,
            "got {got:?}, expected {expected:?}"
        );
    }
}
