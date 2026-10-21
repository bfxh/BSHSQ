//! **黏性介质（Pa·s）判据**（`SURVEY-RIGID-HARD-CASES §4.2②`：高黏介质里的刚体）。
//!
//! 档位 = `FluidConfig::dynamic_viscosity`（**物理**动力黏度：水 1.0e-3 / 甘油 1.4 / 蜂蜜 10
//! 同一把尺；默认 0 = 关）。消费方 = 介质通道的 Stokes 线性阻力 + 转动阻力矩
//! （`crates/vxl-phys/src/world_step/viscous.rs`；律本身的单元判据在那里的 `mod tests`）。
//!
//! **场景 = "冻结蜂蜜"**：流体自身 `gravity = 0` + 数值黏度（α / ε）全 0 ⇒ 粒子基本不动
//! （实测 `max|v| ≈ 1e-5 m/s`）⇒ 介质采样是**静态精确**的（`ρ ≈ ρ0`、`v_med ≈ 0`）
//! ⇒ 体的运动只由阻力律决定，**可以定量对拍解析解**。
//! 为什么不用常规水盆：`OPEN-PROBLEMS` P7 把 2a/2b 定性成"水被推动、轻物浮起、方向对"，
//! 定量档做不了（体把水推开 ⇒ ρ 爆 ⇒ 速度上限；见 `RECIPES.md` 的"别把体直接放进已有水格"）。
//!
//! 判据：① 自由衰减 **λ = 6πμr/m**；② **μ = 0 对照**是**二次**阻力（率 ∝ |v|）—— 同一个体
//! 两个速度窗的率之比 ~3.5×，而 Stokes 档两窗**一致**；③ **静止体金丝雀**（`v ≡ 0` ⇒ 不动）；
//! ④ 只耗散（动能单调不增）；⑤ 转动阻力矩 ⇒ 自旋按 τ_rot = m/(20πμr) 收、μ = 0 无力矩。
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

const R: f32 = 0.05;
/// 体**密度**（`add_dynamic` 的第 4 参是密度）：r = 0.05 ⇒ m ≈ 0.600 kg。
const RHO_BODY: f32 = 1146.0;
const M: f32 = 0.6;

fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// "冻结蜂蜜"场景：返回 `(World, 体号)`。体心在水块正中 `(0, 1.25, 0)`，初速/初旋由调用方设。
fn frozen_honey(mu: f32, spin: Vec3) -> (World, usize) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO, // 世界重力关掉 ⇒ 只剩阻力（浮力 = −g·… = 0）
        // 睡眠阈（|v| < 0.04）会把低速档的读数**直接归零** ⇒ 本判据要量到低速，关掉它。
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    });
    let v = tank(&mut w);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            dynamic_viscosity: mu,
            gravity: Vec3::ZERO,
            xsph_viscosity: 0.0,
            artificial_viscosity: 0.0,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    let _ = w.add_fluid(sys, &[v]);
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.0, 1.25, 0.0),
        Quat::IDENTITY,
        RHO_BODY,
    ) as usize;
    if spin != Vec3::ZERO {
        w.bodies.set_angvel(i, spin);
    }
    (w, i)
}

/// 平动自由衰减的**实测率**：把体按 `+x` 给 `v0`，扫过 `|v| ∈ [v_lo, v_hi]` 两个端点，
/// 返回 `ln(v_hi/v_lo)/Δt`（1/s）。扫不到 ⇒ 断言失败（场景参数写错了）。
fn decay_rate(w: &mut World, i: usize, v0: f32, v_hi: f32, v_lo: f32, ticks: usize) -> f32 {
    // 每窗都把体**摆回水块正中**（否则前一个窗的位移会让它撞上水槽内壁 ⇒ 速度被墙掐断）
    w.bodies.position[i] = Vec3::new(0.0, 1.25, 0.0);
    w.bodies.linvel[i] = Vec3::new(v0, 0.0, 0.0);
    let (mut t_hi, mut t_lo) = (f32::NAN, f32::NAN);
    for k in 0..ticks {
        w.step();
        let v = w.bodies.linvel[i].length();
        let t = (k + 1) as f32 / 60.0;
        if t_hi.is_nan() && v <= v_hi {
            t_hi = t;
        }
        if v <= v_lo {
            t_lo = t;
            break;
        }
    }
    assert!(
        t_hi.is_finite() && t_lo.is_finite() && t_lo > t_hi,
        "速度窗没扫到（t_hi = {t_hi}、t_lo = {t_lo}）"
    );
    (v_hi / v_lo).ln() / (t_lo - t_hi)
}

/// **① 自由衰减率 = 6πμr/m**（Stokes 线性阻力）。带按 20% 给：偏差来自（a）有效 `frac_sub`
/// 由**采样密度**决定（静止晶格的 ρ 略低于 ρ0）、（b）半隐式格式 `1/(1 + dt/τ)` ≈ −2%。
#[test]
fn stokes_drag_sets_the_free_decay_rate() {
    let mu = 5.0f32;
    let (mut w, i) = frozen_honey(mu, Vec3::ZERO);
    let want = 6.0 * core::f32::consts::PI * mu * R / M;
    // 两个速度窗都必须给同一个率（线性阻力 ⇒ 率与 |v| 无关）
    let fast = decay_rate(&mut w, i, 0.5, 0.45, 0.25, 400);
    let slow = decay_rate(&mut w, i, 0.5, 0.15, 0.05, 400);
    println!(
        "① μ = {mu} Pa·s：λ(0.25–0.45) = {fast:.3}、λ(0.05–0.15) = {slow:.3}、\
         解析 6πμr/m = {want:.3}（比 {:.3} / {:.3}）",
        fast / want,
        slow / want
    );
    assert!(
        (fast / want - 1.0).abs() <= 0.20,
        "快窗的衰减率应 ≈ 6πμr/m = {want:.3}（带 20%）：实测 {fast:.3}"
    );
    assert!(
        (slow / fast - 1.0).abs() <= 0.20,
        "线性阻力 ⇒ 两个速度窗的率必须一致：{fast:.3} vs {slow:.3}"
    );
}

/// **② μ = 0 对照**：无数理黏度 ⇒ 走**旧的**二次阻力（`Cd = 1`，率 ∝ |v|）
/// ⇒ 同一速度窗（0.25–0.45）里的率必须**显著小于** Stokes 档（实测 ~3× 差距）。
#[test]
fn without_viscosity_the_medium_uses_the_old_quadratic_drag() {
    let (mut w, i) = frozen_honey(5.0, Vec3::ZERO);
    let stokes = decay_rate(&mut w, i, 0.5, 0.45, 0.25, 400);
    let (mut w0, j) = frozen_honey(0.0, Vec3::ZERO);
    let old = decay_rate(&mut w0, j, 0.5, 0.45, 0.25, 400);
    println!(
        "② 同窗（0.25–0.45）：旧二次阻力 λ = {old:.3} /s vs Stokes λ = {stokes:.3} /s（比 {:.2}）",
        old / stokes
    );
    assert!(
        old > 0.0 && old < 0.6 * stokes,
        "μ = 0 档应显著慢于 Stokes 档（旧口径是二次阻力）：{old:.3} vs {stokes:.3}"
    );
}

/// **③ 静止体金丝雀**：`v ≡ 0` 且介质静止 ⇒ 阻力**精确为 0**（不许被介质噪声泵起、
/// 也不许漂位）。容差取介质自身的残余速度量级（实测 `max|v| ≈ 1e-5 m/s`）。
#[test]
fn body_at_rest_is_never_pumped_by_the_medium() {
    let (mut w, i) = frozen_honey(5.0, Vec3::ZERO);
    let p0 = w.bodies.position[i];
    let mut peak = 0.0f32;
    for _ in 0..600 {
        w.step();
        peak = peak.max(w.bodies.linvel[i].length());
    }
    let drift = (w.bodies.position[i] - p0).length();
    println!("③ 静止体：峰值 |v| = {peak:.2e} m/s、10 s 漂位 = {drift:.2e} m");
    assert!(peak <= 1e-4, "静止体不许被介质泵起：峰值 |v| = {peak:.2e}");
    assert!(drift <= 1e-3, "静止体不许漂位：{drift:.2e} m");
}

/// **④ 只耗散**：无外力档里动能**单调不增**（平动/转动两项都只减不加）。
#[test]
fn viscous_medium_only_dissipates() {
    let (mut w, i) = frozen_honey(5.0, Vec3::new(0.0, 3.0, 0.0));
    w.bodies.linvel[i] = Vec3::new(0.5, 0.0, 0.0);
    let m = 1.0 / w.bodies.inv_mass[i];
    let ke = |w: &World| {
        let lv = w.bodies.linvel[i].length_squared();
        let av = w.bodies.angvel(i).length_squared();
        0.5 * m * lv + 0.5 * 0.4 * m * R * R * av
    };
    let mut prev = ke(&w);
    for k in 0..300 {
        w.step();
        let now = ke(&w);
        assert!(
            now <= prev + 1e-9,
            "动能必须单调不增：tick {k} {prev:.6e} → {now:.6e}"
        );
        prev = now;
    }
    println!("④ 300 tick 后 KE = {prev:.3e} J");
    assert!(prev < 1e-3, "300 tick 后应当基本停住：KE = {prev:.3e}");
}

/// **⑤ 转动阻力矩**：自旋按 `τ_rot = m/(20πμ·r)` 收（球的 Stokes 转动阻力矩 `−8πμr³ω`、
/// `I = 2/5·m·r²`）；μ = 0 时**没有**力矩（旧口径不给力矩）⇒ 自旋保持。
#[test]
fn rotational_drag_stops_a_spinning_sphere() {
    let mu = 0.5f32;
    let tau_rot = M / (20.0 * core::f32::consts::PI * mu * R);
    let (mut w, i) = frozen_honey(mu, Vec3::new(0.0, 5.0, 0.0));
    let (mut t_hi, mut t_lo) = (f32::NAN, f32::NAN);
    for k in 0..200 {
        w.step();
        let t = (k + 1) as f32 / 60.0;
        let om = w.bodies.angvel(i).y;
        if t_hi.is_nan() && om <= 3.0 {
            t_hi = t;
        }
        if om <= 1.0 {
            t_lo = t;
            break;
        }
    }
    let got = (3.0f32).ln() / (t_lo - t_hi);
    let want = 1.0 / tau_rot;
    println!(
        "⑤ μ = {mu} Pa·s：实测转动率 = {got:.3} /s、解析 1/τ_rot = {want:.3} /s（τ_rot = {tau_rot:.4} s）"
    );
    assert!(
        (got / want - 1.0).abs() <= 0.25,
        "转动阻力率应 ≈ 1/τ_rot = {want:.3}（带 25%）：实测 {got:.3}"
    );
    // 对照：μ = 0 ⇒ 旧口径不施加力矩 ⇒ 自旋保持
    let (mut w0, j) = frozen_honey(0.0, Vec3::new(0.0, 5.0, 0.0));
    for _ in 0..30 {
        w0.step();
    }
    assert!(
        (w0.bodies.angvel(j).y - 5.0).abs() < 1e-3,
        "μ = 0 ⇒ 不该有力矩（旧口径）：ω_y = {}",
        w0.bodies.angvel(j).y
    );
}
