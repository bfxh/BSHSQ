//! world_step/viscous：**黏性介质阻力**（Stokes 线性，`FluidConfig::dynamic_viscosity`，Pa·s）。
//!
//! 链路：`FluidConfig::dynamic_viscosity` → `MediumSample::viscosity`（介质自己的物理属性）
//! → 本模块给体算**阻力 + 阻力矩**。默认 μ = 0 ⇒ 整段不进（`world_step.rs` 的调用点短路）
//! ⇒ 既有场景**逐位不变**。
//!
//! **口径**（`SURVEY-RIGID-HARD-CASES §4.2②`）：
//! - 平动：`F = −6πμ·r_eq·v_rel`（`r_eq` = **体积等效球半径** `(3V/4π)^{1/3}`；球 ⇒ **精确**
//!   的 Stokes 阻力，其它形状是标准等效近似），按浸没分数 `frac_sub` 缩放。
//! - 转动：`T = −8πμ·r_eq³·ω`（Stokes 转动阻力矩，球 ⇒ 精确），`I_eq = 2/5·m·r_eq²`
//!   ⇒ `τ_rot = m/(20πμ·r_eq)`（球) = `0.3·τ_trans` —— 同一个介质里球**转**得比**平动**收得快。
//!
//! ⚠️ **半隐式写法**：低 Re 的 μ 必然让 `τ ≲ dt`（例：`r_eq = 5 cm`、`μ = 2 Pa·s`、`m = 0.6 kg`
//! 已到 `τ = 1.3 s` ⇒ 窗口内够用；但沥青档 `μ = 1e3` 会到 `τ = 2.6 ms ≪ dt = 16.7 ms`）——
//! **显式**力在那一档会振荡（`v ← v(1 − dt/τ)` 当 `dt/τ > 2` 发散）。故这里用
//! `F = −(m/τ)·v_rel/(1 + dt/τ)`（等价于 `v ← (v + a·dt)/(1 + dt/τ)` 的半隐式更新）
//! ⇒ **无条件稳定**；代价是终端速度偏高 `(1 + dt/τ)` 倍 ⇒ 本档的判据在 `dt/τ ≤ 0.05`
//! （偏差 ≤ 5%）内与解析对拍，更黏的档等**半解析积分**那一刀。
use crate::Vec3;

/// Stokes 特征长度 = **体积等效球半径**（`(3V/4π)^{1/3}`）：球恰好等于 `r`
/// （⇒ `6πμr` 的精确 Stokes 口径），其它形状是标准等效近似。
pub(crate) fn stokes_radius(vol: f32) -> f32 {
    if vol <= 0.0 {
        return 0.0;
    }
    (3.0 * vol / (4.0 * core::f32::consts::PI)).cbrt()
}

/// 一个体的黏性阻尼输入（**打包成结构体** ⇒ 免 7 参函数门 `args-gate`）。
pub(crate) struct ViscInput {
    /// 逆质量（`1/m`；≤ 0 = 静态体 ⇒ 不施加）。
    pub inv_mass: f32,
    /// 角速度（转动阻力矩用）。
    pub angvel: Vec3,
    /// 形状体积（⇒ `r_eq`）。
    pub vol: f32,
    /// 浸没分数（0..1；阻力按它缩放 ⇒ 半浸时阻力减半）。
    pub frac_sub: f32,
    /// 介质的动力黏度（Pa·s）。
    pub mu: f32,
    /// 相对介质的速度（体速度 − 介质流速）。
    pub v_rel: Vec3,
    /// **本子步**的 dt（半隐式分母与 τ 同尺）。
    pub dt: f32,
}

/// 返回 `(阻力, 阻力矩)`；`μ ≤ 0` / 体积 0 / 静止受体 ⇒ 双零。
pub(crate) fn drag(x: &ViscInput) -> (Vec3, Vec3) {
    let r = stokes_radius(x.vol);
    if x.mu <= 0.0 || x.inv_mass <= 0.0 || x.frac_sub <= 0.0 || r <= 0.0 {
        return (Vec3::ZERO, Vec3::ZERO);
    }
    let m = 1.0 / x.inv_mass;
    // 平动：k = 6πμ·r_eq（N·s/m）⇒ τ = m/k
    let k_t = 6.0 * core::f32::consts::PI * x.mu * r * x.frac_sub;
    let inv = 1.0 / (1.0 + x.dt * k_t * x.inv_mass); // 半隐式分母 1 + dt/τ
    let f = x.v_rel * (-k_t * inv);
    // 转动：k = 8πμ·r_eq³ ⇒ τ_rot = I_eq/k（I_eq = 2/5·m·r_eq²）
    let i_eq = 0.4 * m * r * r;
    let k_r = 8.0 * core::f32::consts::PI * x.mu * r * r * r * x.frac_sub;
    let inv_r = if k_r > 0.0 {
        1.0 / (1.0 + x.dt * k_r / i_eq)
    } else {
        0.0
    };
    (f, x.angvel * (-k_r * inv_r))
}

/// **体上的一步黏性阻尼**（`world_step.rs` 段② 的调用口）：从体集读出质量/角速度，
/// 算 [`drag`] 并经**统一施加器**落力（C3 片 2 的"唯一落点"）。
pub(crate) fn apply_body(
    bodies: &mut vxl_phys_core::BodySet,
    i: usize,
    geom: &ViscGeom,
    v_rel: Vec3,
) {
    let x = ViscInput {
        inv_mass: bodies.inv_mass[i],
        angvel: bodies.angvel(i),
        vol: geom.vol,
        frac_sub: geom.frac_sub,
        mu: geom.mu,
        v_rel,
        dt: geom.dt,
    };
    let (f, t) = drag(&x);
    crate::world_step::coupling::add_force(bodies, i, f, t);
}

/// [`apply_body`] 的几何/介质参数（打包 ⇒ 免 7 参函数门 `args-gate`）。
pub(crate) struct ViscGeom {
    /// 形状体积（⇒ `r_eq`）。
    pub vol: f32,
    /// 浸没分数（0..1）。
    pub frac_sub: f32,
    /// 介质的动力黏度（Pa·s）。
    pub mu: f32,
    /// **本子步**的 dt。
    pub dt: f32,
}

/// 段②（2a 流体作介质）**一个体的收尾输入**（打包 ⇒ 免 7 参门）。
/// `add_buoy_drag = false` 表示被 2b 覆盖 ⇒ **不叠**浮力/二次阻力（让位语义不变）；
/// 但附加质量缩放照做（它缩放的是"已累加的力"，至少含重力）。
pub(crate) struct MediumArgs {
    /// 世界重力（浮力 `−g·ρV·frac_sub` 用）。
    pub g: Vec3,
    /// 介质密度（kg/m³）。
    pub rho: f32,
    /// 形状体积。
    pub vol: f32,
    /// 浸没分数。
    pub frac_sub: f32,
    /// 介质动力黏度（Pa·s；> 0 ⇒ 阻力走 Stokes 档）。
    pub mu: f32,
    /// 相对介质速度（体心 − 介质流速）。
    pub v_rel: Vec3,
    /// 本子步 dt。
    pub dt: f32,
    /// 附加质量系数 `C_m`（0 = 关）。
    pub c_m: f32,
    /// 迎风面积（旧的二次阻力口径用）。
    pub area: f32,
    /// 是否施加浮力 + 二次阻力（`false` = 2b 覆盖 ⇒ 让位）。
    pub add_buoy_drag: bool,
}

/// 段②收尾：浮力（阿基米德）+ 阻力（μ > 0 ⇒ Stokes；否则旧的 `Cd = 1` 二次阻力，
/// **冻结行为**）+ 附加质量缩放 ⇒ 全部经统一施加器落力。**μ = 0 且 `C_m` = 0 时与历史逐位一致**
/// （同一条算式、同一个调用序；附加质量缩放被 `C_m ≤ 0` 短路）。
pub(crate) fn apply_medium(bodies: &mut vxl_phys_core::BodySet, i: usize, a: &MediumArgs) {
    if a.add_buoy_drag {
        if a.mu > 0.0 {
            let geom = ViscGeom {
                vol: a.vol,
                frac_sub: a.frac_sub,
                mu: a.mu,
                dt: a.dt,
            };
            apply_body(bodies, i, &geom, a.v_rel);
        } else {
            // 无数理黏度 ⇒ 旧口径（`Cd = 1` 的二次阻力）：**冻结行为**，逐位不变。
            let mut f = -a.g * (a.rho * a.vol * a.frac_sub);
            let sp = a.v_rel.length();
            if a.area > 0.0 && sp > 1e-6 {
                f += a.v_rel * (-0.5 * a.rho * DRAG_CD * a.area * sp);
            }
            crate::world_step::coupling::add_force(bodies, i, f, Vec3::ZERO);
        }
    }
    // 附加质量：把**已累加的力**（重力 + 浮力 + 阻力）按 m/(m+m_a) 缩放（C_m = 0 ⇒ 一字不动）。
    crate::world_step::added_mass::scale(bodies, i, a.c_m, a.rho, a.vol, a.frac_sub);
}

/// 旧口径的阻力系数（与 `medium.rs` 段①同值；分开声明以免跨函数依赖）。
const DRAG_CD: f32 = 1.0;

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use super::{drag, stokes_radius, ViscInput};
    use crate::Vec3;

    /// `r_eq`：球必须**恰好**等于半径（⇒ Stokes 口径精确）；体积 0/负 ⇒ 0。
    #[test]
    fn stokes_radius_is_exact_for_a_sphere() {
        let r = 0.07f32;
        let vol = 4.0 / 3.0 * core::f32::consts::PI * r * r * r;
        assert!(
            (stokes_radius(vol) - r).abs() < 1e-5,
            "球的体积等效半径必须 = r：得 {}",
            stokes_radius(vol)
        );
        assert_eq!(stokes_radius(0.0), 0.0);
    }

    /// **终端速度 = `(m − ρ_med·V)·g/(6πμ·r)`**（球、低 Re）：把"重力 − 浮力"当外力 `a` 积分，
    /// 半隐式不动点应落在 `a·τ/(1 − …)` 的窗口内 —— 判据取 `dt/τ ≤ 0.05` 的偏差 ≤ 5%。
    #[test]
    fn terminal_velocity_matches_stokes_within_scheme_bias() {
        let (r, m, mu) = (0.05f32, 0.6f32, 2.0f32);
        let vol = 4.0 / 3.0 * core::f32::consts::PI * r * r * r;
        let dt = 1.0 / 240.0;
        let k = 6.0 * core::f32::consts::PI * mu * r; // 6πμr
        let tau = m / k;
        assert!(dt / tau <= 0.05, "本用例要求 dt/τ ≤ 0.05：得 {}", dt / tau);
        let a = 0.5f32; // 净加速度（m/s²）：这里只关心不动点，数值取 0.5 便于读
        let mut v = Vec3::ZERO;
        for _ in 0..2000 {
            let x = ViscInput {
                inv_mass: 1.0 / m,
                angvel: Vec3::ZERO,
                vol,
                frac_sub: 1.0,
                mu,
                v_rel: v,
                dt,
            };
            let (f, _) = drag(&x);
            v += (Vec3::new(0.0, a, 0.0) + f * (1.0 / m)) * dt;
        }
        let want = a * tau;
        let got = v.y;
        assert!(
            (got / want - 1.0).abs() <= 0.05,
            "终端速度应 ≈ a·τ = {want:.5}（偏差 ≤ 5%）实得 {got:.5}（比 {:.3}）",
            got / want
        );
    }

    /// **只耗散 + μ = 0/静止 = 恒等**：`v_rel = 0` ⇒ 阻力与阻力矩都是**精确 0**
    /// （不会给静止体塞进动量）；`μ = 0` / 静态体 / 体积 0 同样恒等。
    #[test]
    fn drag_is_exactly_zero_at_rest_and_without_viscosity() {
        let base = ViscInput {
            inv_mass: 1.0,
            angvel: Vec3::new(0.3, -0.2, 0.7),
            vol: 1e-3,
            frac_sub: 1.0,
            mu: 3.0,
            v_rel: Vec3::ZERO,
            dt: 1.0 / 240.0,
        };
        let (f0, t0) = drag(&base);
        assert_eq!(f0, Vec3::ZERO, "静止体的阻力必须是精确 0");
        // 转动阻力矩：ω ≠ 0 时必须**反向**（只耗散）
        assert!(t0.dot(base.angvel) < 0.0, "阻力矩必须与原角速度反向");
        let no_mu = drag(&ViscInput { mu: 0.0, ..base });
        assert_eq!(no_mu.0, Vec3::ZERO);
        assert_eq!(no_mu.1, Vec3::ZERO);
        let stat = drag(&ViscInput {
            inv_mass: 0.0,
            ..base
        });
        assert_eq!(stat.0, Vec3::ZERO);
    }

    /// **平动阻力只耗散**：`F·v_rel ≤ 0`（任意 μ/dt 组合），且比显式写法（未除 `1 + dt/τ`）
    /// 更弱 —— 半隐式只会**少**耗散，不会过冲。
    #[test]
    fn translational_drag_never_pumps_energy() {
        for (mu, dt) in [
            (1.0f32, 1.0f32 / 240.0),
            (100.0, 1.0 / 60.0),
            (1000.0, 1.0 / 60.0),
        ] {
            let x = ViscInput {
                inv_mass: 1.0,
                angvel: Vec3::ZERO,
                vol: 5e-4,
                frac_sub: 1.0,
                mu,
                v_rel: Vec3::new(1.5, 0.0, -0.5),
                dt,
            };
            let (f, _) = drag(&x);
            assert!(
                f.dot(x.v_rel) <= 0.0,
                "阻力必须与相对速度反向（μ={mu} dt={dt}）"
            );
            let k = 6.0 * core::f32::consts::PI * mu * stokes_radius(x.vol);
            assert!(
                f.length() <= k * x.v_rel.length() + 1e-6,
                "半隐式阻力必须 ≤ 显式阻力（μ={mu} dt={dt}）"
            );
        }
    }
}
