//! **判据（§8.4.32）：开了角反作用，中心场景照旧托住** —— 也是"翻默认是否安全"的**前瞻守卫**
//! （谁弄坏了角那一支，这条会红）。
//!
//! **为什么需要"带转动的自扮引擎"**：只吃 `body_dv`/`body_dx` 的引擎**量不到角反作用**（那是 2c-3 之前
//! 所有对拍都看不见它、以及我上一版探针给出"ON/OFF 逐位相同"的原因）。
//!
//! 关键：自扮引擎必须**消费**角反作用（只吃 `body_dv`/`body_dx` 是量不到它的）：
//! `ω += I⁻¹·(Σ r.torque)`（与门面同口径：`r.torque` 是**整 tick** 的角冲量）、`q = q ⊕ ω·dt`。
//! 惯量**不硬编码**：从一个"同形状同密度"的临时 `World` 里读出 `inv_mass` / `local_inv_inertia`
//! ⇒ 与引擎同口径（本会话早前正是栽在"密度当质量"这类口径回代上）。
use vxl_phys_core::mass::mass_props;
use vxl_phys_core::{interop::NoProviders, Mat3, Quat, Shape, Vec3};
use vxl_phys_soft::{RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const HALF: Vec3 = Vec3::new(0.3, 0.05, 0.3);

/// 盒的 `(inv_mass, local_inv_inertia)` —— 走**引擎自己那支口径**（`core::mass::mass_props`，
/// 与 `BodySet::push_dynamic` 内部同一个函数），不硬编码。
fn box_props() -> (f32, Vec3) {
    let mp = mass_props(&Shape::Box { half: HALF }, 1.0 / 0.036);
    (mp.inv_mass, mp.local_inv_inertia)
}

/// `(末 y, 末 x, |x|max, |ω|, up.x)`
fn run(enable: bool, ticks: usize) -> (f32, f32, f32, f32, f32) {
    let (inv_mass, inv_i) = box_props();
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    r.angular_reaction = enable;
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &[]);
    }
    let shape = Shape::Box { half: HALF };
    let (mut pos, mut vel) = (Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    let (mut omega, mut q) = (Vec3::ZERO, Quat::IDENTITY);
    let mut x_max = 0.0f32;
    for _ in 0..ticks {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: q,
            linvel: vel,
            angvel: omega,
            local_inv_inertia: inv_i,
            inv_mass,
        };
        r.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        // 体自己积分（半隐式 + 姿态）
        vel += G * DT;
        pos += vel * DT;
        q = q.integrate_angular(omega, DT);
        // 反作用回填
        if let Some(dv) = r.body_dv.first() {
            vel += *dv;
        }
        if let Some(dx) = r.body_dx.first() {
            pos += *dx;
        }
        if enable {
            if let Some(re) = r.reactions.first() {
                let m = Mat3::from_quat(q);
                let tau_local = m.transpose_mul_vec3(re.torque);
                omega += m.mul_vec3(tau_local.mul_per_elem(inv_i));
            }
        }
        x_max = x_max.max(pos.x.abs());
    }
    (
        pos.y,
        pos.x,
        x_max,
        omega.length(),
        q.rotate_vec3(Vec3::Y).x,
    )
}

#[test]
fn angular_reaction_keeps_the_hold() {
    println!("\n=== 中心场景：带转动的自扮引擎（**消费**角反作用）===");
    for enable in [false, true] {
        let (y, x, xm, w, upx) = run(enable, 1800);
        println!(
            "角反作用={enable:5} | 末 y={y:+9.4} x={x:+.4} |x|max={xm:.4} |ω|={w:.4} up.x={upx:+.5}"
        );
        // 两条共同：盒子仍被托住（末态在绳高度、横向留在自身足迹内）。
        assert!(
            y > 0.5 && xm < 0.3,
            "角反作用={enable} 时盒子仍该被托住：实测 末 y={y:+.4} |x|max={xm:.4}"
        );
        if enable {
            // 开着角反作用 ⇒ 接触会给体一点力矩（**轻微摇晃**），但不许跑飞：
            // 实测 |ω|≈0.12 rad/s、倾 ≈3°；跑飞（|ω| 或倾角失控）说明角冲量的口径/符号坏了。
            assert!(
                w < 2.0 && upx.abs() < 0.25,
                "开着时该只是**轻微摇晃**而不是跑飞：实测 |ω|={w:.4} up.x={upx:+.5}"
            );
        } else {
            // **金丝雀**：关着时体一点都不该转（自扮引擎只在 `enable` 时消费 `r.torque`）。
            assert_eq!(
                w, 0.0,
                "关着角反作用时体不该获得任何角速度（实测 |ω|={w:.4}）"
            );
            assert_eq!(upx, 0.0, "关着时也不该有倾角（实测 up.x={upx:+.5}）");
        }
    }
}
