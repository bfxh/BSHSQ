//! **油膜 / 润滑（切向黏性接触）判据** —— `PhysConfig::film_drag`（N·s/m，默认 0 = 关）。
//!
//! 口径（`SURVEY-RIGID-HARD-CASES §4.2③`）：把**切向黏性**从**库仑摩擦**里分出来 ——
//! 库仑是"上限 `μ·pn`"（与法向压力相关、与速度无关 ⇒ 速度**线性**衰减）；
//! 油膜是"与切向相对速度成正比"（`F_t = −k·v_t`、**与压力无关** ⇒ 速度**指数**衰减）。
//! 两条判据的形状差异（指数 vs 线性）就是本档的**指纹**：同一个体、同一条滑道，只换旋钮。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 滑道：静态地板（顶面 y = 0）+ 一个 1×1×1 m 的滑块（密度 1000 ⇒ m = 1000 kg）躺在地上。
/// 返回 `(World, 滑块号)`；滑块初速 = `+x`。
fn slider(film: f32, mu: f32, v0: f32) -> (World, usize) {
    let mut w = World::new(PhysConfig {
        film_drag: film,
        ..PhysConfig::default()
    });
    w.add_static(
        Shape::Box {
            half: Vec3::new(10.0, 0.5, 10.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu },
        restitution: 0.0,
    });
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    w.bodies.set_linvel(i, Vec3::new(v0, 0.0, 0.0));
    (w, i)
}

/// 在速度窗 `[v_lo, v_hi]` 内测衰减率 `ln(v_hi/v_lo)/Δt`（1/s）。
fn decay_rate(w: &mut World, i: usize, v_hi: f32, v_lo: f32, ticks: usize) -> f32 {
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

/// **① 油膜 = 切向黏性（与压力无关）**：`μ_coulomb = 0` + `film_drag > 0` ⇒ 滑块**仍然停住**，
/// 且衰减是**指数**形状 —— 两个速度窗（2.0→1.5 / 0.8→0.4）的**率一致**（线性阻力指纹）。
#[test]
fn oil_film_stops_a_slider_exponentially() {
    let k = 500.0f32;
    let (mut w, i) = slider(k, 0.0, 2.0);
    let fast = decay_rate(&mut w, i, 2.0, 1.5, 4000);
    let (mut w2, j) = slider(k, 0.0, 2.0);
    let slow = decay_rate(&mut w2, j, 0.8, 0.4, 4000);
    println!(
        "① k = {k} N·s/m、μ_coulomb = 0：λ(1.5–2.0) = {fast:.3} /s、λ(0.4–0.8) = {slow:.3} /s（比 {:.3}）",
        slow / fast
    );
    assert!(fast > 0.0, "油膜必须让滑块减速（实测 λ = {fast:.3}）");
    assert!(
        (slow / fast - 1.0).abs() <= 0.15,
        "线性黏性 ⇒ 两个速度窗的率必须一致：{fast:.3} vs {slow:.3}"
    );
}

/// **② 库仑对照（film = 0）**：纯库仑摩擦的减速度 ≈ `μg`（常数）⇒ 率 **∝ |v|**
/// ⇒ 两个速度窗的率必须**明显不同**（慢窗更大）。这与 ① 的"两窗一致"正好互为反证。
#[test]
fn coulomb_friction_alone_decays_linearly() {
    let (mut w, i) = slider(0.0, 0.4, 2.0);
    let fast = decay_rate(&mut w, i, 2.0, 1.5, 4000);
    let (mut w2, j) = slider(0.0, 0.4, 2.0);
    let slow = decay_rate(&mut w2, j, 0.8, 0.4, 4000);
    println!(
        "② μ_coulomb = 0.4、film = 0：λ(1.5–2.0) = {fast:.3} /s、λ(0.4–0.8) = {slow:.3} /s（比 {:.2}）",
        slow / fast
    );
    assert!(
        slow > 1.4 * fast,
        "库仑是常力（率 ∝ |v|）⇒ 慢窗的率必须明显更大：{fast:.3} vs {slow:.3}"
    );
}

/// **③ 只耗散 + 金丝雀**：`v ≡ 0` 的静置滑块在 `film_drag > 0` 下不许被泵起
/// （油膜冲量与切向相对速度成正比 ⇒ 静止 ⇒ 精确 0），`film_drag = 0` 档也不许出现 NaN/漂移。
#[test]
fn resting_slider_is_never_pumped_by_the_film() {
    for k in [0.0f32, 500.0] {
        let (mut w, i) = slider(k, 0.0, 0.0);
        let p0 = w.bodies.position[i];
        for _ in 0..300 {
            w.step();
        }
        let v = w.bodies.linvel[i].length();
        let drift = (w.bodies.position[i] - p0).length();
        println!("③ k = {k}：静置 300 tick 后 |v| = {v:.2e}、漂位 = {drift:.2e} m");
        assert!(v < 1e-3, "静置滑块不许被油膜泵起：|v| = {v:.2e}");
        assert!(drift < 1e-3, "静置滑块不许漂位：{drift:.2e} m");
    }
}

/// **④ 动能单调不增**（无外力档：油膜+库仑都只耗散）。
/// 容差 0.5 J：接触求解器的**软接触抖动**在沉降期会有 ~0.3 J 量级的尖峰（`film = 0` 的同一场景
/// 实测 0.33 J、`film = 500` 档 0.06 J ⇒ 油膜顺带把它压小了），那是既有口径、不是本刀引入的。
#[test]
fn film_and_friction_only_dissipate() {
    let (mut w, i) = slider(500.0, 0.2, 2.0);
    let m = 1.0 / w.bodies.inv_mass[i];
    let ke = |w: &World| 0.5 * m * w.bodies.linvel[i].length_squared();
    let mut prev = ke(&w);
    for k in 0..600 {
        w.step();
        let now = ke(&w);
        assert!(
            now <= prev + 0.5,
            "动能必须单调不增：tick {k} {prev:.6e} → {now:.6e}"
        );
        prev = now;
    }
    println!(
        "④ 600 tick 后 KE = {prev:.3e} J（起点 {:.3e} J）",
        0.5 * m * 4.0
    );
    assert!(prev < 1e-2, "600 tick 后应当基本停住：KE = {prev:.3e}");
}
