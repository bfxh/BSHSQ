//! **P19 ⑦ 体侧隧道的取证探针**（`OPEN-PROBLEMS.md` P19 的收口块）。
//!
//! 逐 tick 读「盒 × 三角网体」这一对：**有没有流形**、**在不在配对表里**、两边醒没醒、
//! 以及双方的位置/速度（`Σp` 用来核动量账）。
//!
//! 为什么必须有它（2026-10-14 的翻案）：上一轮只看了"盒末态 x"，于是把"盒没停住"读成
//! "流形没到求解器"。逐 tick 打印**流形表 + 配对表**一看就清楚：修前是 `pair=true` 而
//! **流形表恒空**（该对整场零候选），修后是"贴而不穿"的两体**同漂**（自由薄板的正常物理）。
//!
//! 跑法（`PROBE_SHAPE` 选盒/球/胶囊、`PROBE_V` 改初速、`PROBE_NO_CCD` 关 CCD、
//! `PROBE_FIXED_PLATE=1` 把薄板设成撞不动）：
//! ```text
//! cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! PROBE_SHAPE=sphere PROBE_FIXED_PLATE=1 cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! PROBE_FIXED_PLATE=1 cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! PROBE_V=61 PROBE_NO_CCD=1 cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! ```
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

const HALF: f32 = 0.5;

fn main() {
    let no_ccd = std::env::var_os("PROBE_NO_CCD").is_some();
    let mut cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    if no_ccd {
        cfg.ccd_speed_threshold = f32::INFINITY;
    }
    println!(
        "cfg: ccd_threshold={} extent_ratio={} max_steps={} substeps={} skin={}",
        cfg.ccd_speed_threshold,
        cfg.ccd_extent_ratio,
        cfg.ccd_max_steps,
        cfg.substeps,
        cfg.contact_skin
    );
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    let mesh = w.add_trimesh(pts, vec![[0u32, 1, 2], [0, 2, 3]]);
    let plate = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 1.0e5, 0.02) as usize;
    // PROBE_FIXED_PLATE=1：把薄板变成"撞不动"（inv_mass = 0）—— 金丝雀原意（当障碍用）。
    if std::env::var_os("PROBE_FIXED_PLATE").is_some() {
        w.bodies.inv_mass[plate] = 0.0;
        w.bodies.local_inv_inertia[plate] = Vec3::ZERO;
    }
    let shape = probe_shape();
    let b = w.add_dynamic(shape, Vec3::new(-5.0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    let v0: f32 = std::env::var("PROBE_V")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(60.0);
    w.bodies.linvel[b] = Vec3::new(v0, 0.0, 0.0);
    println!("v0={v0}");
    println!("plate={plate} box={b}");
    for k in 0..400u32 {
        w.step();
        let x = w.bodies.position[b].x;
        let vx = w.bodies.linvel[b].x;
        let man: Vec<String> = w
            .manifolds()
            .iter()
            .filter(|m| {
                let (a, c) = (m.a as usize, m.b as usize);
                a == plate || c == plate || a == b || c == b
            })
            .map(|m| {
                let d = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
                format!(
                    "({},{}) n=({:.2},{:.2},{:.2}) pts={} dmax={:.4}",
                    m.a,
                    m.b,
                    m.normal.x,
                    m.normal.y,
                    m.normal.z,
                    m.points.len(),
                    d
                )
            })
            .collect();
        let in_pairs = w.pairs().iter().any(|&(p, q)| {
            let (p, q) = (p as usize, q as usize);
            (p == plate && q == b) || (p == b && q == plate)
        });
        if k < 40 || !man.is_empty() || in_pairs {
            println!(
                "t={k:>3} x={x:>10.4} vx={vx:>9.4} plate_x={:>9.4} plate_vx={:>9.4} awake(p,b)=({},{}) pair={in_pairs} man=[{}]",
                w.bodies.position[plate].x,
                w.bodies.linvel[plate].x,
                w.bodies.awake[plate],
                w.bodies.awake[b],
                man.join(" | ")
            );
        }
    }
    println!("FINAL x = {:.4}", w.bodies.position[b].x);
}

/// 撞击体形状（`PROBE_SHAPE`：box（默认）/ sphere / capsule）——半长/半径都取 `HALF` 量级，
/// 这样"停在壳面上"的期望位置都是 `-HALF`（球 = 半径；胶囊的轴是 +Y ⇒ 那个方向的伸距只有半径）。
fn probe_shape() -> Shape {
    match std::env::var("PROBE_SHAPE").unwrap_or_default().as_str() {
        "sphere" => Shape::Sphere { radius: HALF },
        "capsule" => Shape::Capsule {
            half_height: HALF * 0.5,
            radius: HALF * 0.5,
        },
        _ => Shape::Box {
            half: Vec3::splat(HALF),
        },
    }
}
