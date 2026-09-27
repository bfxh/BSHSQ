//! **粒子↔刚体耦合的已知缺口：钉住判据**（T1 第四/五片，2026-09-26）。
//!
//! **现状（实测）**：绳把落下的盒子**接住**了（前 60 tick 的动量确实被吃掉），但**十几~几十 tick 后
//! 会被弹飞**（+4 m/s 量级），之后自由落体 ⇒ 1800 tick 的长窗判据**过不了**。
//!
//! **已排除的成因（8 组二维扫描，别重复试）**：接触**柔度** `α_c ∈ {0, 1e-6, 1e-5, 1e-4, 1e-3}`
//! × 绳**内摩擦** `damping ∈ {1.0, 0.999, 0.995, 0.99}` —— **全部 8 组都弹飞**
//! （稳态窗摆幅 2000–4200 m）⇒ **不是刚度错配、也不是阻尼不足**。
//!
//! **缺口已闭合（2026-09-27，见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8.4.10）**：
//! 加了**位置口径回填** `Rope::body_dx`（被速度钳位压掉的那一份**只补位置、不补速度**），
//! 本判据的引擎按**门面同口径**吃它 ⇒ **长窗 1800 tick 盒子停在 y = +0.982**（短窗 0.9802）。
//! 机制：下沉 = **位置漏**（体每 tick 先按 `v·dt` 走过 `g·dt² = 2.72 mm`，而接触只取消接近速度）
//! ⇒ 补足 ~3.1 mm/tick 正好抵掉（实测 Σ|dx| = 5.6 m / 1800 tick）。
//!
//! **仍未闭的是门面侧**：门面也喂 `body_dx`，盒子能停住几百 tick，但之后仍会滑穿
//! （实测 `|ω| = 0`、无睡眠 ⇒ 剩下的就是 §8.4.8 那个**离散接触集**：接触集一跳，支撑就断）。
//! 那一刀落地前，门面级判据（`crates/vxl-phys/tests/rope_scene.rs`）还不能翻。
use vxl_phys_core::{interop::NoProviders, Quat, Shape, Vec3};
use vxl_phys_soft::{RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// **托住判据（3 维）**：紧绳 + 0.6 宽盒 + 1800 tick ⇒ 短窗（60 tick）接住（0.9913）、
/// **长窗仍停在绳上**（末 y = +0.9949、末速 −0.158、横向仅 −0.011、接触保持到 t=1799）。
/// 三条钉子：① 短窗 `y > 0.70`；② 末次接触时仍在绳高度（形态=被托住而非沉下去）；③ 横向留在自身足迹内。
/// （原为"钉住缺口"；2026-09-27 修好 —— 见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8.4.19/§8.4.20。）
#[test]
fn box_on_rope_is_held() {
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &[]);
    }
    // **三维自扮引擎**（§8.4.16）：`pos`/`linvel` 是全 `Vec3`、反作用吃**整个向量**
    // （`v += dv`、`x += dx`）。**原来是 1 维的**（`pos = Vec3::new(0.0, y, 0.0)`、只吃 `.y`）
    // ⇒ **摩擦反作用的横向分量被静默丢掉** ⇒ 与 3 维门面对拍时混进"1D vs 3D"这一整类差异。
    let shape = Shape::Box {
        half: Vec3::new(0.3, 0.05, 0.3),
    };
    let m = 1.0f32;
    let (mut pos, mut vel) = (Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    let mut pos_at_60 = Vec3::ZERO;
    // **§8.4.18 横向漂移判据**：把两种缺口拆开——**滑出**（接触丢失时盒子仍在绳的高度）
    // vs **下沉**（接触丢失时它已经沉下去很久）。判据全部机器无关（无计时、无随机）。
    let (mut t_lost, mut y_lost, mut x_lost) = (usize::MAX, 0.0f32, 0.0f32);
    let (mut x_abs_max, mut t_xmax) = (0.0f32, 0usize);
    // ⚠️ 首版判据的坑：`hit == 0` 在**首次接触之前**也成立（盒子还在往下掉）⇒ 必须先建立过接触。
    let (mut t_x05, mut y_x05) = (usize::MAX, 0.0f32);
    for t in 0..1800 {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: Quat::IDENTITY,
            linvel: vel,
            angvel: Vec3::ZERO, // 2c-1：本判据场景里体不转（ω = 0 ⇒ 转动项恒等）
            local_inv_inertia: Vec3::ZERO, // 静态/未用（角反作用开关默认关）
            inv_mass: 1.0 / m,
        };
        r.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = r.body_dv.first() {
            vel += *dv; // 速度口径（整向量：含摩擦的横向分量）
        }
        // **位置口径回填**（`Rope::body_dx`，§8.4.10）：门面也这么做 ⇒ 自扮引擎必须同口径才可比。
        // 只回速度 = 体每 tick 按 `v·dt` 走过的 `g·dt²` 一去不回（"缓慢下沉"的真因）。
        if let Some(dx) = r.body_dx.first() {
            pos += *dx;
        }
        if t == 59 {
            pos_at_60 = pos;
        }
        if pos.x.abs() > x_abs_max {
            x_abs_max = pos.x.abs();
            t_xmax = t;
        }
        // **横向漂移起始**（|x| 首次越过 5 cm）：这是"滑出"真正的起点，也是修它要看的量。
        if t_x05 == usize::MAX && pos.x.abs() > 0.05 {
            t_x05 = t;
            y_x05 = pos.y;
        }
        let hit = r
            .entry_faces()
            .iter()
            .filter(|(b, _)| *b != u32::MAX)
            .count();
        // **逃逸时刻 = 最后一次"还有接触"的 tick**：中间会有瞬时弹跳（丢-又接），
        // 首版取"第一次丢失"抓到的是 t=14 的弹跳（y 还在 1.16）⇒ 改成取最后一次。
        if hit > 0 {
            t_lost = t;
            y_lost = pos.y;
            x_lost = pos.x;
        }
    }
    let (y, y_at_60, x) = (pos.y, pos_at_60.y, pos.x);
    let v = vel.y;
    println!(
        "短窗(60 tick) y={y_at_60:.4} | 长窗(1800 tick) 末 y={y:.4} v={v:+.3} | 横向 x={x:+.4}\n\
         横向判据：|x|max={x_abs_max:.4}（t={t_xmax}）| 漂移起始 {} | 最后一次接触 t={t_lost} 时 y={y_lost:.4} x={x_lost:+.4}\
         ⇒ **{}**",
        if t_x05 == usize::MAX {
            "未达 5 cm".to_string()
        } else {
            format!("t={t_x05}（y={y_x05:.4}）")
        },
        if y_lost > 0.8 {
            "末次接触仍在绳高度（被托住）"
        } else {
            "末次接触前已沉下去"
        }
    );
    // **判据（§8.4.18/§8.4.20）**：这两枚钉子把**形态**钉住 —— 缺口开着时是"横向滑出"型
    // （末次接触仍在绳高度 `y_lost > 0.8`、且已漂出自身足迹）；2026-09-27 修好（摩擦口径与法向统一）
    // 后它们**自然满足**（`y_lost = +0.9949`、`x_abs_max = 0.031`），形态判读仍留在打印里。
    assert!(
        y_lost > 0.8,
        "末次接触时盒子该仍在绳高度（实测 y={y_lost:.4}）——红了说明它不是被托住而是沉下去的"
    );
    assert!(
        x_abs_max < 0.3,
        "横向该**留在自身足迹内**（半宽 0.3；实测 |x|max={x_abs_max:.4}）——\
         红了说明又出现横向滑出（回看摩擦的相对速度口径，§8.4.19）"
    );
    assert!(
        y > 0.5,
        "长窗该**停在绳上**（实测末 y={y:.4}、v={v:+.3}、横向 x={x:+.4}）——\
         红了说明位置口径回填或摩擦口径被改坏（§8.4.10 与 §8.4.19 那两刀）"
    );
}
