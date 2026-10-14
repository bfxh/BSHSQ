//! world_step/film：**油膜 / 润滑**（切向黏性接触）—— `PhysConfig::film_drag`（N·s/m，默认 0 = 关）。
//!
//! 口径（`SURVEY-RIGID-HARD-CASES §4.2③`）：把**切向黏性**从**库仑摩擦**里分出来 ——
//! 库仑给的是"上限 `μ·pn`"（与法向压力相关、与速度无关）；油膜给的是"与切向相对速度成正比"
//! （`F_t = −k·v_t`，**与压力无关**）⇒ 于是能表达"黏着油膜"：`μ_coulomb = 0` + 油膜仍能把体停住。
//!
//! **律（逐接触点）**：`J_t = −t̂ · (|v_t| · dt·k / (1 + dt·k·w))`，
//! `w` = 切向**有效逆质量**（两侧平动项 + 绕 `t̂` 的角向项，[`tangent_inv_mass`]）；
//! 半隐式分母 ⇒ **无条件稳定**，且不动点是 `|v_t| = F_ext/k`（线性阻力；显式力在 `dt·k·w ≫ 1`
//! 的档会振荡 —— 与 `world_step/viscous.rs` 同一个理由）。
//!
//! **段位**：解算之后（本子步的流形刚更新过）、位置积分之前 ⇒ 解算器先把接触解干净、
//! 油膜再按**黏性**收切向残余速度（睡眠计时器读的是收完的值 ⇒ 油膜帮助入睡）。
//! 只对 `depth > 0`（真接触）的接触点施加；`film_drag <= 0` ⇒ 整段不进（默认档逐位不变）。
use crate::world_step::coupling::is_receptor;
use crate::{Vec3, World};

/// **切向有效逆质量** `w(±t̂)`：`1/m_a + 1/m_b + (I⁻¹(r_a×t̂))·(r_a×t̂) + (…)`（两侧）。
/// 返回到 `t̂` 的 `w`；两侧都不可动（静态/静量奇异）⇒ 0（调用方跳过）。
fn tangent_inv_mass(
    bodies: &vxl_phys_core::BodySet,
    a: u32,
    b: u32,
    ra: Vec3,
    rb: Vec3,
    t: Vec3,
) -> f32 {
    let mut w = 0.0f32;
    for (i, r) in [(a, ra), (b, rb)] {
        let k = i as usize;
        w += bodies.inv_mass[k];
        // 角向项：t̂·(I⁻¹(r×t̂))×r —— 用引擎自己的世界逆惯量作用（静态体恒零）
        let lt = r.cross(t);
        w += bodies.apply_world_inv_inertia(k, lt).cross(r).dot(t);
    }
    w
}

/// 油膜段：逐流形 × 逐接触点施加**切向黏性**冲量（顺序 = 流形序 × 点序 ⇒ 确定性）。
///
/// 自由函数而不是 `impl World`：`World` 的**方法数是 god 门的登记债务（只准减）**
/// ⇒ 新段一律走自由函数（同 `destruction.rs` / `nan_guard::guard` 的形态）。
pub(crate) fn pass(w: &mut World, dt: f32) {
    let k = w.config.film_drag;
    if k <= 0.0 || w.manifolds.is_empty() {
        return;
    }
    for c in 0..w.manifolds.len() {
        let n = w.manifolds[c].normal;
        let npts = w.manifolds[c].points.len();
        for p in 0..npts {
            let point = w.manifolds[c].points[p].point;
            let depth = w.manifolds[c].points[p].depth;
            if depth <= 0.0 {
                continue; // 只算真接触（皮肤带内的"预期接触"不算油膜）
            }
            let (ia, ib) = (w.manifolds[c].a as usize, w.manifolds[c].b as usize);
            // 两侧都睡/静态 ⇒ 跳过（与介质段同一条受体口径：睡着的体不吃外力）
            if !is_receptor(&w.bodies, ia) && !is_receptor(&w.bodies, ib) {
                continue;
            }
            let (ra, rb) = (point - w.bodies.position[ia], point - w.bodies.position[ib]);
            let vrel = w.bodies.velocity_at(ia, ra) - w.bodies.velocity_at(ib, rb);
            let vt = vrel - n * vrel.dot(n);
            let sp = vt.length();
            if sp <= 1e-6 {
                continue; // 没有切向相对运动
            }
            let t = vt * (1.0 / sp);
            let wgt = tangent_inv_mass(&w.bodies, ia as u32, ib as u32, ra, rb, t);
            if wgt <= 1e-9 {
                continue; // 两侧都不可动
            }
            // `t̂` 是 a 相对 b 的**切向速度方向** ⇒ 阻尼逆着它施加（a 拿 −t̂·j、b 拿 +t̂·j）
            let j = sp * (dt * k / (1.0 + dt * k * wgt));
            w.bodies.apply_impulse(ia, t * (-j), point);
            w.bodies.apply_impulse(ib, t * j, point);
        }
    }
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use super::tangent_inv_mass;
    use vxl_phys_core::{Quat, Shape, Vec3};

    /// **切向有效逆质量**：自由质量 = `1/m`（无角向项）；球面上一点绕切向有角向项
    /// ⇒ 有效逆质量**大于** `1/m`（同样的冲量只换来更小的切向速度变化）。
    #[test]
    fn tangent_inv_mass_matches_the_rigid_body_formula() {
        let mut b = vxl_phys_core::BodySet::new();
        b.push_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1000.0,
        );
        let m = 1.0 / b.inv_mass[0];
        // 接触点在球心正下方（r = −y），切向取 +x ⇒ 角向项 = (r×t)·I⁻¹·(r×t)/… > 0
        let r = Vec3::new(0.0, -0.5, 0.0);
        let w = tangent_inv_mass(&b, 0, 0, r, r, Vec3::X);
        assert!(
            w > 1.0 / m,
            "角向项必须让有效逆质量 > 1/m：w = {w}、1/m = {}",
            1.0 / m
        );
        // 静态体（inv_mass = 0、惯量零）⇒ w = 0 ⇒ 调用方早退
        let mut s = vxl_phys_core::BodySet::new();
        s.push_static(Shape::Sphere { radius: 0.5 }, Vec3::ZERO, Quat::IDENTITY);
        assert_eq!(tangent_inv_mass(&s, 0, 0, r, r, Vec3::X), 0.0);
    }
}
