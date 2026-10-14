//! **油膜探针**（仪表，只打印不断言）—— `PhysConfig::film_drag`（N·s/m，默认 0 = 关）。
//!
//! 量什么（`SURVEY-RIGID-HARD-CASES §4.2③`）：把**切向黏性**与**库仑摩擦**分开后的三条读数
//! 1. **滑动衰减曲线**（k = 0 / 50 / 500；μ_coulomb = 0）⇒ 指数 ⇒ 拟合 λ 与解析 `m/(N·k)` 对拍；
//! 2. **库仑对照**（μ = 0.4、film = 0）⇒ 常力 ⇒ **线性**衰减（同一滑道、只换旋钮）；
//! 3. **斜坡蠕行**（μ_coulomb = 0 + 油膜）：稳态下滑速度 = `m·g·sinθ/k_总`
//!    ⇒ 油膜是"与压力无关"的黏性项：**没有静摩擦**、只有**蠕行终速**（这正是油膜与库仑的区别）。
//!
//! 跑法：
//! ```text
//! cargo run -q --release -p vxl-phys --example oil_film_probe
//! FILM_K=200 FILM_TICKS=240 cargo run -q --release -p vxl-phys --example oil_film_probe
//! ```
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 滑块（1 m 立方、密度 1000 ⇒ m = 1000 kg）；`Tilt` = 滑道倾角（弧度，绕 z 轴）。
struct Scene {
    w: World,
    body: usize,
}

fn scene(film: f32, mu: f32, tilt: f32) -> Scene {
    let mut w = World::new(PhysConfig {
        film_drag: film,
        ..PhysConfig::default()
    });
    // 滑道：半长 10×0.5×10 的静态盒，绕 z 轴倾斜 ⇒ 顶面法线偏 θ。
    let rot = if tilt == 0.0 {
        Quat::IDENTITY
    } else {
        Quat::from_axis_angle(Vec3::Z, tilt)
    };
    w.add_static(
        Shape::Box {
            half: Vec3::new(30.0, 0.5, 30.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        rot,
    );
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu },
        restitution: 0.0,
    });
    // 滑块**随坡面一起转**、沿坡面法线就位（留 1 cm 隙 ⇒ 落定后贴合）。
    let n = rot.rotate_vec3(Vec3::Y);
    let half = Vec3::splat(0.5);
    let i = w.add_dynamic(Shape::Box { half }, n * (half.y + 0.01), rot, 1000.0) as usize;
    w.bodies.set_material(i, m);
    Scene { w, body: i }
}

fn env_f32(name: &str, d: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(d)
}

/// 在 `[v_lo, v_hi]` 窗内拟合衰减率 `ln(v_hi/v_lo)/Δt`（1/s）；扫不到 ⇒ `NaN`。
fn rate(s: &mut Scene, v_hi: f32, v_lo: f32, ticks: usize) -> f32 {
    let (mut t_hi, mut t_lo) = (f32::NAN, f32::NAN);
    for k in 0..ticks {
        s.w.step();
        let v = s.w.bodies.linvel[s.body].length();
        let t = (k + 1) as f32 / 60.0;
        if t_hi.is_nan() && v <= v_hi {
            t_hi = t;
        }
        if v <= v_lo {
            t_lo = t;
            break;
        }
    }
    if t_hi.is_finite() && t_lo.is_finite() && t_lo > t_hi {
        (v_hi / v_lo).ln() / (t_lo - t_hi)
    } else {
        f32::NAN
    }
}

fn main() {
    let ticks: usize = env_f32("FILM_TICKS", 1200.0) as usize;
    println!("油膜探针（滑块 1 m 立方、m = 1000 kg；film_drag 单位 N·s/m）");

    // 1) 滑动衰减：k 轴（μ_coulomb = 0 ⇒ 只有油膜）。
    for k in [0.0f32, 50.0, 500.0] {
        let mut s = scene(k, 0.0, 0.0);
        s.w.bodies.set_linvel(s.body, Vec3::new(2.0, 0.0, 0.0));
        let fast = rate(&mut s, 2.0, 1.5, ticks);
        let mut s2 = scene(k, 0.0, 0.0);
        s2.w.bodies.set_linvel(s2.body, Vec3::new(2.0, 0.0, 0.0));
        let slow = rate(&mut s2, 0.8, 0.4, ticks);
        let n_pts = n_contact_points(&mut scene(k, 0.0, 0.0));
        let note = if k > 0.0 {
            format!(
                "有效总导纳 λ·m = {:.0} N·s/m（名义 N·k = {:.0} ⇒ 角向软化让每点只出 ~{:.0}%）",
                fast * 1000.0,
                n_pts as f32 * k,
                100.0 * (fast * 1000.0) / (n_pts as f32 * k)
            )
        } else {
            "无阻力（不衰减 ⇒ 窗口扫不到，NaN）".to_string()
        };
        println!(
            "  [油膜 k = {k:>5}] λ(1.5–2.0) = {fast:.3} /s、λ(0.4–0.8) = {slow:.3} /s｜\
             接触点数 = {n_pts}｜{note}"
        );
    }

    // 2) 库仑对照：常力 ⇒ 线性（率 ∝ |v|）。
    let mut s = scene(0.0, 0.4, 0.0);
    s.w.bodies.set_linvel(s.body, Vec3::new(2.0, 0.0, 0.0));
    let fast = rate(&mut s, 2.0, 1.5, ticks);
    let mut s2 = scene(0.0, 0.4, 0.0);
    s2.w.bodies.set_linvel(s2.body, Vec3::new(2.0, 0.0, 0.0));
    let slow = rate(&mut s2, 0.8, 0.4, ticks);
    println!(
        "  [库仑 μ=0.4, k=0] λ(1.5–2.0) = {fast:.3} /s、λ(0.4–0.8) = {slow:.3} /s\
         （比 {:.2}× ⇒ 慢窗更大 = 线性；解析 a = μg = {:.2} m/s²）",
        slow / fast,
        0.4 * 9.81
    );

    // 3) 斜坡蠕行（μ_coulomb = 0 + 油膜）：终速 = m·g·sinθ/k_总（**没有静摩擦** ⇒ 一直慢慢滑）。
    let tilt = env_f32("FILM_TILT_DEG", 10.0) * core::f32::consts::PI / 180.0;
    let k = env_f32("FILM_K", 500.0);
    let mut s = scene(k, 0.0, tilt);
    let mut line = String::new();
    for t in 1..=ticks.min(900) {
        s.w.step();
        if t % 120 == 0 {
            line += &format!(
                "  {:.0}s: {:.4}",
                t as f32 / 60.0,
                s.w.bodies.linvel[s.body].length()
            );
        }
    }
    let v = s.w.bodies.linvel[s.body].length();
    println!(
        "  [斜坡 θ = {:.1}°、k = {k}] 下滑速度随时间：{line}｜末态 {v:.4} m/s\
         ｜名义蠕行终速 m·g·sinθ/(N·k) = {:.3} m/s（用上面的**有效**总导纳 ≈½·N·k ⇒ ≈{:.2}）\
         ⇒ 油膜给的是「蠕行」而不是「静摩擦」",
        tilt * 180.0 / core::f32::consts::PI,
        1000.0 * 9.81 * tilt.sin() / (4.0 * k),
        2.0 * 1000.0 * 9.81 * tilt.sin() / (4.0 * k)
    );
}

/// 就位后第一个 tick 的接触点数（油膜的"总导纳 = 点数 × k"用）。
fn n_contact_points(s: &mut Scene) -> usize {
    for _ in 0..30 {
        s.w.step(); // 让它落定（1 cm 隙 ⇒ ~3 tick；多给几 tick 稳妥）
    }
    s.w.manifolds()
        .iter()
        .filter(|m| m.a as usize == s.body || m.b as usize == s.body)
        .map(|m| m.points.iter().filter(|p| p.depth > 0.0).count())
        .sum()
}
