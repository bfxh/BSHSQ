//! **绳↔刚体耦合的鲁棒性判据**（§8.4.26）：把"绳托住盒子"这件事放在**整条参数轴**上验，
//! 而不是只看一组参数——本仓刚栽过"只在 1 维/单点参数上成立"的跟头（§8.4.16/§8.4.24.1）。
//!
//! **判据（机器无关：无计时、无随机）**：14 格（质量 ×4 / 盒半宽 ×3 / 摩擦 ×3 / 子步 ×2 / 时长 ×2 + 基准）
//! **全部满足** `末 y > 0.5`（仍在绳高度）**且** `|x|max < 0.3`（横向留在自身足迹内）。
//!
//! **为什么这几格**：
//! - **质量 0.25 kg** 是历史失败格（旧的锥口径 `μ·depth` 下巡航逃逸到 `+271 m`）；
//! - **摩擦 μ ∈ {0.1, 0.25, 1}** 守住"库仑锥取冲量口径 `μ·w_p·λ`"（§8.4.25）没把摩擦语义改坏；
//! - **子步 4/16** 守住子步粒度；
//! - **3600 tick** 守住长窗（缓慢下沉/横向蠕变都在这条上现形）。
//!
//! ⚠️ 修耦合三件（`Rope::body_dx` 位置口径 / 角反作用停用 / 摩擦含 `body_dv`）与锥口径时要跑它；
//! 三件的说明见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8.4.20 与 §8.4.25。
use vxl_phys_core::{interop::NoProviders, Quat, Shape, Vec3};
use vxl_phys_soft::{RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

struct Cfg {
    mass: f32,
    half_x: f32,
    mu: f32,
    substeps: u32,
    ticks: usize,
}

/// 跑一格，返回 `(末 y, |x|max, 末速 y)`。
fn run(c: &Cfg) -> (f32, f32, f32) {
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    r.friction = c.mu;
    r.substeps = c.substeps;
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &[]);
    }
    let shape = Shape::Box {
        half: Vec3::new(c.half_x, 0.05, c.half_x),
    };
    let (mut pos, mut vel) = (Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    let mut x_max = 0.0f32;
    for _ in 0..c.ticks {
        // 3 维自扮引擎（§8.4.16：**只积 `y` 会让"横向逃逸"整类现象隐形**）。
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: Quat::IDENTITY,
            linvel: vel,
            angvel: Vec3::ZERO, // 2c-1：本判据场景里体不转（ω = 0 ⇒ 转动项恒等）
            inv_mass: 1.0 / c.mass,
        };
        r.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = r.body_dv.first() {
            vel += *dv; // 速度口径（整向量）
        }
        if let Some(dx) = r.body_dx.first() {
            pos += *dx; // 位置口径（整向量）
        }
        x_max = x_max.max(pos.x.abs());
    }
    (pos.y, x_max, vel.y)
}

#[test]
fn rope_holds_box_across_parameters() {
    let base = || Cfg {
        mass: 1.0,
        half_x: 0.3,
        mu: 0.5,
        substeps: 8,
        ticks: 1800,
    };
    let mut cells: Vec<(String, Cfg)> = vec![("基准 1kg/0.6宽/μ0.5".into(), base())];
    for m in [0.25f32, 0.5, 2.0, 6.0] {
        cells.push((format!("质量 {m} kg"), Cfg { mass: m, ..base() }));
    }
    for hx in [0.15f32, 0.2, 0.45] {
        cells.push((
            format!("盒半宽 {hx}"),
            Cfg {
                half_x: hx,
                ..base()
            },
        ));
    }
    for mu in [0.1f32, 0.25, 1.0] {
        cells.push((format!("摩擦 μ={mu}"), Cfg { mu, ..base() }));
    }
    for ss in [4u32, 16] {
        cells.push((
            format!("子步 {ss}"),
            Cfg {
                substeps: ss,
                ..base()
            },
        ));
    }
    cells.push((
        "基准 × 3600 tick".into(),
        Cfg {
            ticks: 3600,
            ..base()
        },
    ));

    let mut bad = Vec::new();
    for (tag, c) in &cells {
        let (y, x_max, v) = run(c);
        println!("{tag:26} | 末 y={y:+9.4} |x|max={x_max:6.3} v={v:+8.3}");
        if !(y > 0.5 && x_max < 0.3) {
            bad.push(format!("{tag}: y={y:+.4} |x|max={x_max:.3}"));
        }
    }
    assert!(
        bad.is_empty(),
        "这些格该**全部托住**（末 y > 0.5 且 |x|max < 0.3）：{bad:?}\n\
         ⇒ 红了先查耦合三件（`body_dx` 位置口径 / 角反作用停用 / 摩擦含 `body_dv`）与锥口径（`μ·w_p·λ`）"
    );
}
