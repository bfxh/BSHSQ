//! world_step/added_mass：**附加质量**（added mass）—— `PhysConfig::added_mass_coeff`（默认 0 = 关）。
//!
//! 口径（`SURVEY-RIGID-HARD-CASES §4.2②`）：浸没体在介质里**加速**时还要拖动一份介质，
//! 等效于惯性质量变大：`(m + m_a)·a = F_ext`，`m_a = C_m·ρ_med·V·frac_sub`
//! （球 `C_m = 0.5`、平板 ≈ 1、长柱 ≈ 0.2 ⇒ 本字段是**无量纲的 C_m**）。
//!
//! **实现走"缩放已累加的力"而不是改积分器**：`v ← v + F·dt/(m+m_a)` 与
//! `v ← v + (F·m/(m+m_a))·dt/m` 是同一个式子 ⇒ 把 `bodies.force[i]`（与力矩）乘
//! `m/(m+m_a)` 就等价于把惯性换成有效质量 —— 不必给 `Integrator` 加"每体有效质量"这条通道，
//! 也不动 god 门棘轮上的 `World`/`BodySet` 成员。
//!
//! ⚠️ **两条如实登记**：
//! 1. 本段只缩放**本段之前**已累加的力（重力 + 浮力 + 介质阻力）；`aero` / 2b 反作用在段后追加
//!    ⇒ 不进缩放。
//! 2. **接触/关节的冲量**走求解器（不是累加器）⇒ **不**含附加质量 ⇒ 被接触推着加速的那一档
//!    仍旧是干质量（"启停瞬态"的另一半）。
//!
//! 两条都只影响**瞬态**：稳态（`F_ext = 0`）与缩放无关 ⇒ 终端速度**不受影响**（缩放是正常数）。

/// 给一个体按附加质量缩放已累加的力/力矩（`C_m ≤ 0` / 静态体 / 未浸没 ⇒ 原样返回 `0`）。
///
/// 返回 `m_a`（kg；供探针/判据读数）；`m/(m+m_a)` 是施加的缩放因子。
pub(crate) fn scale(
    bodies: &mut vxl_phys_core::BodySet,
    i: usize,
    c_m: f32,
    rho_med: f32,
    vol: f32,
    frac_sub: f32,
) -> f32 {
    let inv_m = bodies.inv_mass[i];
    if c_m <= 0.0 || inv_m <= 0.0 || vol <= 0.0 || frac_sub <= 0.0 || rho_med <= 0.0 {
        return 0.0;
    }
    let m_a = c_m * rho_med * vol * frac_sub.clamp(0.0, 1.0);
    let m = 1.0 / inv_m;
    let s = m / (m + m_a);
    if s.is_nan() || s <= 0.0 || s >= 1.0 {
        return 0.0; // m_a 退化（0/NaN）⇒ 不动
    }
    bodies.force[i] *= s;
    bodies.torque[i] *= s;
    m_a
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use super::scale;
    use vxl_phys_core::{BodySet, Quat, Shape, Vec3};

    fn sphere(radius: f32, density: f32) -> BodySet {
        let mut b = BodySet::new();
        b.push_dynamic(
            Shape::Sphere { radius },
            Vec3::ZERO,
            Quat::IDENTITY,
            density,
        );
        b
    }

    /// 球在水中：`m_a = 0.5·ρV` ⇒ 缩放因子 = `m/(m+m_a)`；`C_m = 0` ⇒ 恒等（一个位都不动）。
    #[test]
    fn scale_is_m_over_m_plus_added() {
        let r = 0.5f32;
        let mut b = sphere(r, 1000.0);
        let vol = 4.0 / 3.0 * core::f32::consts::PI * r * r * r;
        let m = 1000.0 * vol;
        b.force[0] = Vec3::new(10.0, -20.0, 3.0);
        b.torque[0] = Vec3::new(1.0, 2.0, 3.0);
        let zero = b.force[0];
        assert_eq!(scale(&mut b, 0, 0.0, 1000.0, vol, 1.0), 0.0);
        assert_eq!(b.force[0], zero, "C_m = 0 必须恒等");
        // C_m = 0.5 ⇒ m_a = 0.5ρV = m/2 ⇒ 缩放 2/3
        let m_a = scale(&mut b, 0, 0.5, 1000.0, vol, 1.0);
        assert!(
            (m_a / (0.5 * m) - 1.0).abs() < 1e-4,
            "m_a 应 = 0.5ρV：{m_a}"
        );
        let want = 2.0 / 3.0;
        assert!(
            (b.force[0].x / zero.x - want).abs() < 1e-4,
            "力应缩放 m/(m+m_a) = {want}：{}",
            b.force[0].x / zero.x
        );
        assert!((b.torque[0].z / 3.0 - want).abs() < 1e-4, "力矩同缩放");
        // 半浸没 ⇒ 因子按 frac_sub 线性
        let mut c = sphere(r, 1000.0);
        c.force[0] = Vec3::new(10.0, 0.0, 0.0);
        let m_a_half = scale(&mut c, 0, 0.5, 1000.0, vol, 0.5);
        assert!((m_a_half / m_a - 0.5).abs() < 1e-4, "半浸 ⇒ m_a 减半");
    }

    /// 静态体 / 体积 0 / 介质密度 0 ⇒ 不动（防 NaN 与"给静态体塞力"）。
    #[test]
    fn degenerate_inputs_are_noops() {
        let r = 0.5f32;
        let vol = 4.0 / 3.0 * core::f32::consts::PI * r * r * r;
        let mut s = BodySet::new();
        s.push_static(Shape::Sphere { radius: r }, Vec3::ZERO, Quat::IDENTITY);
        assert_eq!(scale(&mut s, 0, 0.5, 1000.0, vol, 1.0), 0.0);
        let mut b = sphere(r, 1000.0);
        b.force[0] = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(scale(&mut b, 0, 0.5, 0.0, vol, 1.0), 0.0);
        assert_eq!(scale(&mut b, 0, 0.5, 1000.0, 0.0, 1.0), 0.0);
        assert_eq!(b.force[0], Vec3::new(1.0, 0.0, 0.0), "退化输入必须一字不动");
    }
}
