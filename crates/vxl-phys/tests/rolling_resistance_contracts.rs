//! **滚动阻力**（`PhysConfig::roll_resistance`，单位米 = 等效力臂；0 = 关）。
//!
//! | 判据 | 钉住的行为 |
//! |---|---|
//! | R1 关档 | 球在倾角 θ 上**纯滚动**：`a = 5g·sinθ/7`（解析） |
//! | R2 开档 | 阻力矩 `τ = μ_r·N` ⇒ `a = 5g(sinθ − μ_r·cosθ/r)/7`（解析） |
//! | R3 缓坡停住 | `μ_r > r·tanθ` ⇒ 球停在坡上（关档时滚走，对照） |
//! | R4 只作用于"滚" | 开着阻力时：静置盒与**纯滑动**盒的角速度都保持 ≈ 0（不注入角冲量） |
//! | R5 扭转摩擦 = 有意不做 | 多接触点自转盒**已被点摩擦停住**（20 → 0 rad/s，与 `roll_resistance` 无关）； |
//! |   | 单点接触绕法向没有力臂 ⇒ 不建阻力行（物理事实，不是欠账） |
//!
//! 口径来源：R1/R2/R3 的解析式由"球纯滚动 + 阻力矩 `μ_r·N`"推出（见各判据头注）；
//! 实现见 `vxl-phys-solver/src/island.rs::solve_rolling`（与 Rapier 的 twist 行同构，轴取滚动方向）。

use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

/// θ 度的坡（绕 z 转；+x 为下坡方向）+ 坡上半径 `r` 的球；`roll` 是滚动阻力系数（米）。
fn slope_scene(theta_deg: f32, r: f32, roll: f32) -> (World, usize) {
    let cfg = PhysConfig {
        roll_resistance: roll,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(8.0, 0.5, 8.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::from_axis_angle(Vec3::Z, theta_deg.to_radians()),
    );
    let s = w.add_dynamic(
        Shape::Sphere { radius: r },
        Vec3::new(0.0, 0.6 + r, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    (w, s)
}

/// 从静止开始沿坡滚 `ticks`，返回 `(沿 下坡方向的加速度, 末速, 位移)`。
fn roll_down(theta_deg: f32, r: f32, roll: f32, ticks: usize) -> (f32, f32, Vec3) {
    let (mut w, s) = slope_scene(theta_deg, r, roll);
    for _ in 0..60 {
        w.step(); // 坐稳（坡上静止，摩擦抓得住）
    }
    let th = theta_deg.to_radians();
    // 下坡单位向量：坡面（绕 z 转 +θ）的法线是 `(−sinθ, cosθ, 0)` ⇒ 面内 y 随 +x 升高
    // ⇒ **下坡朝 −x**。
    let down = Vec3::new(-th.cos(), -th.sin(), 0.0);
    let p0 = w.bodies.position[s];
    let (v0, t) = (w.bodies.linvel[s].dot(down), ticks as f32 * w.config.dt);
    for _ in 0..ticks {
        w.step();
    }
    let v1 = w.bodies.linvel[s].dot(down);
    let d = w.bodies.position[s] - p0;
    ((v1 - v0) / t, v1, d)
}

/// **R1 关档**：纯滚动解析 `a = 5g·sinθ/7`（θ = 10° ⇒ 1.216 m/s²）。
#[test]
fn r1_zero_rolling_resistance_matches_pure_rolling() {
    let (a, v, _d) = roll_down(10.0, 0.3, 0.0, 60);
    let want = 5.0 / 7.0 * 9.81 * 10f32.to_radians().sin();
    println!("[R1 关档] θ=10°：a = {a:.4} m/s²（解析 {want:.4}）、v = {v:.4} m/s");
    assert!(
        (a - want).abs() < 0.1 * want,
        "纯滚动加速度应 ≈ {want}，实得 {a}"
    );
}

/// **R2 开档**：滚动阻力矩 `τ = μ_r·N` ⇒ `a = 5g(sinθ − μ_r·cosθ/r)/7`。
/// θ = 10°、μ_r = 0.02 m、r = 0.3 ⇒ 解析 0.756 m/s²（对比关档的 1.216）。
#[test]
fn r2_resistance_reduces_acceleration_per_analytic() {
    let (r, th) = (0.3f32, 10f32.to_radians());
    let mu = 0.02f32;
    let (a, _v, _d) = roll_down(10.0, r, mu, 60);
    let want = 5.0 / 7.0 * 9.81 * (th.sin() - mu * th.cos() / r);
    println!("[R2 开档] μ_r = {mu} m、r = {r}：a = {a:.4} m/s²（解析 {want:.4}）");
    assert!(
        (a - want).abs() < 0.15 * want,
        "含阻力的加速度应 ≈ {want}，实得 {a}"
    );
}

/// **R3 缓坡停住**：`μ_r > r·tanθ` ⇒ 球不起（θ = 1°、μ_r = 0.02 m、r = 0.3 ⇒ 0.0667 > 0.0175）。
/// 对照：同为 1° 但关档 ⇒ 球滚走（否则本判据没有分辨力）。
#[test]
fn r3_shallow_slope_is_held_by_resistance() {
    let (a_on, v_on, d_on) = roll_down(1.0, 0.3, 0.02, 120);
    let (_a_off, _v_off, d_off) = roll_down(1.0, 0.3, 0.0, 120);
    println!(
        "[R3 缓坡] 1° 坡：开档 a = {a_on:.4}、|Δ| = {:.4} m、|v| = {v_on:.4}｜关档 |Δx| = {:.4} m",
        d_on.length(),
        d_off.x
    );
    assert!(
        a_on.abs() < 0.05 && v_on.abs() < 0.05 && d_on.length() < 0.02,
        "缓坡上球应被阻力按住（a = {a_on}、v = {v_on}、|Δ| = {}）",
        d_on.length()
    );
    assert!(
        d_off.x < -0.05 || d_off.x > 0.05,
        "对照：关档时球应当滚走（Δx = {}）",
        d_off.x
    );
}

/// **R4 只作用于"滚"**：开着 `roll_resistance` 时，静置盒与被**纯滑动**的盒都不该获得角速度。
#[test]
fn r4_resistance_does_not_spin_sliding_or_resting_bodies() {
    let cfg = PhysConfig {
        roll_resistance: 0.5, // 放大到 0.5 m：若判据写错，这里必然显形
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.5, 6.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let rest = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(-3.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    ) as usize;
    let slide = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(3.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    ) as usize;
    for _ in 0..60 {
        w.step();
    }
    w.bodies.linvel[slide] = Vec3::new(3.0, 0.0, 0.0); // 纯滑动（无自转）
    for _ in 0..120 {
        w.step();
    }
    let (wr, ws) = (w.bodies.angvel(rest), w.bodies.angvel(slide));
    println!("[R4 只作用于滚] 静置盒 ω = {wr:?}｜纯滑动盒 ω = {ws:?}");
    assert!(wr.length() < 1e-3, "静置盒不该被转动（ω = {wr:?}）");
    assert!(ws.length() < 1e-3, "纯滑动盒不该被转动（ω = {ws:?}）");
}

/// **R5 扭转摩擦 = 有意不做**（文档化边界，同 `kinematic_contracts.rs` 的 C5 那一格）：
/// 多接触点的自转盒**已经被点摩擦停住**（角动量的力臂来自四个接触点，见下读数），
/// 而单点接触绕接触法向**没有力臂** ⇒ 按构造不可能有扭转阻力（这正是陀螺尖能长转的原因）。
/// 本判据把"停住"这件事钉住（含"与 `roll_resistance` 无关"），免得将来有人误以为缺一条。
#[test]
fn r5_twist_is_intentionally_not_modeled() {
    let run = |roll: f32| {
        let cfg = PhysConfig {
            roll_resistance: roll,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        w.add_static(
            Shape::Box {
                half: Vec3::new(6.0, 0.5, 6.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.5, 0.0),
            Quat::IDENTITY,
            500.0,
        ) as usize;
        for _ in 0..60 {
            w.step();
        }
        w.bodies.set_angvel(b, Vec3::new(0.0, 20.0, 0.0)); // 绕接触法向自转
        for _ in 0..300 {
            w.step();
        }
        w.bodies.angvel(b).length()
    };
    let (w_off, w_on) = (run(0.0), run(0.02));
    println!("[R5 扭转边界] 自转盒 300 tick 后 |ω|：关档 {w_off:.6}｜开档 {w_on:.6}");
    assert!(
        w_off < 0.5 && w_on < 0.5,
        "多接触点自转盒应被点摩擦停住（关档 {w_off}、开档 {w_on}）"
    );
}
