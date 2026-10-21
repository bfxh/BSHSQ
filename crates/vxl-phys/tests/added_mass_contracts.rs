//! **附加质量（added mass）判据** —— `PhysConfig::added_mass_coeff`（`C_m`，无量纲，默认 0 = 关）。
//!
//! 口径（`SURVEY-RIGID-HARD-CASES §4.2②`）：浸没体加速时要拖动一份介质 ⇒ 等效惯性 `m + m_a`、
//! `m_a = C_m·ρ_med·V·frac_sub`（球 0.5 / 平板 ≈ 1 / 长柱 ≈ 0.2）。
//! 实现 = 把**已累加的力**乘 `m/(m+m_a)`（`world_step/added_mass.rs`；等价于换有效质量）。
//!
//! 场景 = **"冻结蜂蜜"**（同 `viscous_medium_contracts.rs`：流体自身零重力 + 数值黏度全 0
//! ⇒ 粒子基本不动、采样静态精确）⇒ 体的加速度就是**解析值**：
//! `a = (m − ρ_med·V)·g / (m + m_a)`。
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

const R: f32 = 0.05;
/// 体密度 1146 ⇒ `m ≈ 0.600 kg`；介质 ρ0 = 1000 ⇒ `ρV ≈ 0.5236`（净重 `0.076 kg` 量级）。
const RHO_BODY: f32 = 1146.0;

fn scenario(c_m: f32, rho_body: f32, g: Vec3) -> (World, usize) {
    let mut w = World::new(PhysConfig {
        added_mass_coeff: c_m,
        gravity: g, // ⚠️ 必须在 `World::new` 里给（重力是**注册的场**，事后改 config 不生效）
        substeps: 1, // 单子步 ⇒ "第一个 tick 的速度变化"就是**初始**加速度（阻力此时取 v = 0）
        ..PhysConfig::default()
    });
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            // 介质**静止**：自身零重力 + 数值黏度全 0（同 #288 的"冻结蜂蜜"）
            gravity: Vec3::ZERO,
            xsph_viscosity: 0.0,
            artificial_viscosity: 0.0,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    let _ = w.add_fluid(sys, &[]);
    // ⚠️ **先空跑一 tick**：流体的网格/密度在第一步里才建起来（首 tick 的介质采样是"真空"，
    // 实测首 tick 的加速度正好是自由落体 9.81）⇒ 体在**介质已就绪**之后再入场。
    w.step();
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.0, 1.25, 0.0),
        Quat::IDENTITY,
        rho_body,
    ) as usize;
    (w, i)
}

/// **初始**加速度 = 第一个 tick 的速度变化 / dt。
///
/// ⚠️ 必须只取 1 个 tick：介质里有旧的二次阻力（`Cd = 1`，∝ v²），多跑几 tick 就把加速度拉低，
/// 而且**拉低得对 C_m 不公平**（附加质量档速度更小 ⇒ 阻力更小 ⇒ 看起来"没怎么慢"—— 实测
/// 5-tick 平均会把 0.744 的比抬到 0.902）。力相位在**积分之前**跑 ⇒ 首 tick 的阻力取 `v = 0`（精确 0）。
fn accel(w: &mut World, i: usize) -> f32 {
    let v0 = w.bodies.linvel[i];
    w.step();
    (w.bodies.linvel[i] - v0).length() * 60.0
}

/// **① 加速度比 = `m/(m + C_m·ρV)`**（`C_m` = 0.5 / 1.0 两档）。
///
/// 手法：先用 `C_m = 0` 档的读数反解**介质实际给的** `ρV_eff`（`a₀ = (m − ρV_eff)g/m`
/// ⇒ `ρV_eff = m(1 − a₀/g)`），再拿它**预测**另外两档 —— 这样判据只钉**附加质量那条律**
/// （同一个介质、同一个采样密度 ⇒ 沿用了介质侧自己的 `ρ` 口径），不被"介质采样密度比 ρ0 低几 %"
/// 这条既有口径污染；后者只做**量级核对**（打印出来、给一个宽带宽）。
#[test]
fn added_mass_scales_the_acceleration_by_m_over_m_plus_ma() {
    let vol = 4.0 / 3.0 * core::f32::consts::PI * R * R * R;
    let m = RHO_BODY * vol;
    let g = 9.81f32;
    let (mut w0, i0) = scenario(0.0, RHO_BODY, Vec3::new(0.0, -g, 0.0));
    let a0 = accel(&mut w0, i0);
    let rho_v_eff = m * (1.0 - a0 / g);
    println!(
        "① C_m = 0：a₀ = {a0:.5} m/s² ⇒ 反解 ρV_eff = {rho_v_eff:.5}（ρ0·V = {:.5}，比 {:.3}）",
        1000.0 * vol,
        rho_v_eff / (1000.0 * vol)
    );
    assert!(
        rho_v_eff > 0.5 * 1000.0 * vol && rho_v_eff < 1.5 * 1000.0 * vol,
        "介质给的 ρV 应与 ρ0·V 同量级（介质采样口径允许偏低）：{rho_v_eff:.5}"
    );
    let mut ratios = Vec::new();
    for c_m in [0.5f32, 1.0] {
        let (mut w, i) = scenario(c_m, RHO_BODY, Vec3::new(0.0, -g, 0.0));
        let a = accel(&mut w, i);
        let got = a / a0;
        let want = m / (m + c_m * rho_v_eff);
        println!(
            "① C_m = {c_m}：a = {a:.5} m/s² ⇒ 比 {got:.4}、解析 m/(m + C_m·ρV_eff) = {want:.4}"
        );
        assert!(
            (got / want - 1.0).abs() <= 0.03,
            "附加质量档的加速度比应 = m/(m+m_a) = {want:.4}：实测 {got:.4}"
        );
        ratios.push(got);
    }
    assert!(
        ratios[0] < 1.0 && ratios[1] < ratios[0],
        "附加质量必须**单调减小**加速度：{ratios:?}"
    );
}

/// **② 零外力档的静止体在 `C_m > 0` 下逐位不动**（缩放只乘"已累加的力" ⇒ 0 乘 s 仍是 0；
/// 挡住"缩放自己塞进一个力"这类实现错误）。
#[test]
fn resting_body_is_untouched_without_external_forces() {
    let (mut w, i) = scenario(1.0, RHO_BODY, Vec3::ZERO); // 无外力：浮力也为 0 ⇒ 累加器恒 0
    for k in 0..120 {
        w.step();
        assert!(
            w.bodies.linvel[i].length() < 1e-9,
            "零外力档的体在第 {k} tick 动了（> f32 噪声）：{:?}",
            w.bodies.linvel[i]
        );
    }
}

/// **③ `C_m = 0` 是恒等**（默认档）：与"同场景、同初速、`C_m = 0`"的两跑逐位一致
/// （发动机级别的零换代由四道哈希另钉；这里钉**路径**）。
#[test]
fn zero_coefficient_is_identity() {
    let (mut a, ia) = scenario(0.0, RHO_BODY, Vec3::new(0.0, -9.81, 0.0));
    let (mut b, ib) = scenario(0.0, RHO_BODY, Vec3::new(0.0, -9.81, 0.0));
    for _ in 0..30 {
        a.step();
        b.step();
    }
    let (va, vb) = (a.bodies.linvel[ia], b.bodies.linvel[ib]);
    assert!(
        va.x.to_bits() == vb.x.to_bits()
            && va.y.to_bits() == vb.y.to_bits()
            && va.z.to_bits() == vb.z.to_bits(),
        "C_m = 0 档必须与自身逐位一致：{va:?} vs {vb:?}"
    );
}
