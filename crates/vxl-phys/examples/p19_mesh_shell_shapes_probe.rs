//! **P19 ⑥ 形状矩阵的取证探针**：同一面 4×4 m 薄壳（**provider 路**，`add_mesh`）、同一速度，
//! 逐形状看「到底有没有流形 / 有没有进配对表 / 扫掠有没有命中」。
//!
//! 背景（`OPEN-PROBLEMS.md` P19 ⑥）：盒被拦住（`mesh_cross.rs` 的交叉通道），
//! **球/胶囊/圆柱/锥全穿**（末态 x = 395 m）。本探针把"穿"这件事拆到这几层。
//!
//! 跑法（`PROBE_V` 改初速、`PROBE_NO_CCD` 关 CCD、`PROBE_SHAPES=sphere,capsule` 选形状）：
//! ```text
//! cargo run -q --release -p vxl-phys --example p19_mesh_shell_shapes_probe
//! PROBE_V=61 cargo run -q --release -p vxl-phys --example p19_mesh_shell_shapes_probe
//! ```
use vxl_phys::{
    ccd_sweep_stats_take, BodyBuildExt, DiagExt, NarrowTierExt, PhysConfig, Quat, Shape, Vec3,
    World,
};

const TICKS: u32 = 60;

fn scene(shape: Shape, ccd: bool) -> (World, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ccd_speed_threshold: if ccd { 10.0 } else { f32::INFINITY },
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(
        pts,
        vec![[0u32, 1, 2], [0, 2, 3]],
    ));
    let b = w.add_dynamic(shape, Vec3::new(-5.0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    let v0: f32 = std::env::var("PROBE_V")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(60.0);
    w.bodies.linvel[b] = Vec3::new(v0, 0.0, 0.0);
    (w, b)
}

fn run(name: &str, shape: Shape, ccd: bool) {
    let (mut w, b) = scene(shape, ccd);
    let _ = ccd_sweep_stats_take();
    println!("=== {name}（ccd={ccd}）===");
    for k in 0..TICKS {
        w.step();
        let x = w.bodies.position[b].x;
        let man: Vec<String> = w
            .manifolds()
            .iter()
            .filter(|m| m.a as usize == b || m.b as usize == b)
            .map(|m| {
                let d = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
                format!(
                    "n=({:+.2},{:+.2},{:+.2}) pts={} dmax={d:+.4}",
                    m.normal.x,
                    m.normal.y,
                    m.normal.z,
                    m.points.len()
                )
            })
            .collect();
        let in_pairs = w
            .pairs()
            .iter()
            .any(|&(p, q)| p as usize == b || q as usize == b);
        if x.abs() < 2.0 {
            println!(
                "t={k:>3} x={x:>9.4} vx={:>9.4} pair={in_pairs} man=[{}]",
                w.bodies.linvel[b].x,
                man.join(" | ")
            );
        }
    }
    let (samples, hits, refine, deep, seg) = ccd_sweep_stats_take();
    println!(
        "FINAL x = {:.4} | 扫掠：采样 {samples} 命中 {hits} 二分 {refine} 最深穿透 {deep:.5} 段号和 {seg}",
        w.bodies.position[b].x
    );
}

fn main() {
    let no_ccd = std::env::var_os("PROBE_NO_CCD").is_some();
    let only = std::env::var("PROBE_SHAPES").unwrap_or_default();
    let want = |n: &str| only.is_empty() || only.split(',').any(|s| s.trim() == n);
    let cases: [(&str, Shape); 5] = [
        (
            "box",
            Shape::Box {
                half: Vec3::splat(0.4),
            },
        ),
        ("sphere", Shape::Sphere { radius: 0.4 }),
        (
            "capsule",
            Shape::Capsule {
                half_height: 0.4,
                radius: 0.2,
            },
        ),
        (
            "cylinder",
            Shape::Cylinder {
                half_height: 0.4,
                radius: 0.4,
            },
        ),
        (
            "cone",
            Shape::Cone {
                half_height: 0.4,
                radius: 0.4,
            },
        ),
    ];
    for (name, shape) in cases {
        if !want(name) {
            continue;
        }
        run(name, shape, !no_ccd);
    }
}
