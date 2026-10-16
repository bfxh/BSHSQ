//! **深穿透恢复探针**（`SURVEY-RIGID-HARD-CASES §6.3①` 的判据：把球**埋进**地板半米 ⇒
//! 允许顶出、**能量不增**、不抖）。
//!
//! 五条**不同体积通道**各来一档（同一场景、同一埋深、同一半径）：`mesh`（三角网提供者）、
//! `voxel`（体素体积）、`splat`（高斯喷溅 SDF 场）、`hull`（凸体外壳）、`box`（静态盒，走
//! `prims::sphere_convex_ab` 的解析腿）。用途：§6.2① 说"体素/SDF 都当**表面**用，深穿透明确不保证"
//! —— 这条把它**量出来**。
//!
//! ⚠️ **`box` 与 `hull` 两档在埋 0.2 / 0.5 m 时会走"球心恰好落在顶面平面"那一 tick**（f32 累加
//! 正好落在 `y = 0`）—— P22 ② 的侧滑就是在那一 tick 塞进来的（斜法线 / 壳角接触点）。
//! 判据在 `tests/deep_penetration.rs`（⑤ 用 μ = 0 把摩擦这条腿关掉）。
//!
//! 跑法：
//! ```text
//! cargo run -q --release -p vxl-phys --example deep_penetration_probe
//! PROBE_DEPTH=0.5 PROBE_TICKS=600 cargo run -q --release -p vxl-phys --example deep_penetration_probe
//! ```
//!
//! 旋钮（`PROBE_*` 环境变量）：`PROBE_DEPTH`（埋深，默认 0.5）、`PROBE_TICKS`（默认 400）、
//! `PROBE_MU`（摩擦系数，默认 0.9；**置 0** 可把横向速度/自旋全归到接触口径上）、
//! `PROBE_SUBSTEPS`（≥1 时覆盖子步数，默认用 `PhysConfig::default`）、`PROBE_TRACE` +
//! `PROBE_TRACE_N`（逐 tick 打印位置/速度/流形，法线 5 位小数）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

/// 球半径（与 `rest_height_contract.rs` 的陷网档一致）。
const R: f32 = 0.45;

fn env_f32(name: &str, d: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(d)
}

/// 平网格（8 m 见方、y = 0）。
fn flat_mesh() -> vxl_phys_terrain::mesh::TriMesh {
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=2u32 {
        for ix in 0..=2u32 {
            verts.push(Vec3::new(
                -4.0 + 4.0 * ix as f32,
                0.0,
                -4.0 + 4.0 * iz as f32,
            ));
        }
    }
    for iz in 0..2u32 {
        for ix in 0..2u32 {
            let a = iz * 3 + ix;
            tris.push([a, a + 3, a + 1]);
            tris.push([a + 1, a + 3, a + 4]);
        }
    }
    vxl_phys_terrain::mesh::TriMesh::new(verts, tris)
}

/// 平体素地板（顶面 y = 1.0）。
fn flat_voxel() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

/// 平喷溅场：y = 0 面上按 0.25 m 铺各向同性核（σ = 0.5、幅值 1.0、iso 0.5）—— 与
/// `rest_height_contract.rs::flat_splat` 同配方。
fn flat_splat() -> vxl_phys_splat::GaussianSplatField {
    let mut f = vxl_phys_splat::GaussianSplatField::new(0.5);
    for ix in -3..=3 {
        for iz in -3..=3 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(ix as f32 * 0.25, 0.0, iz as f32 * 0.25),
                0.5,
                1.0,
            ));
        }
    }
    f
}

/// 一圈刚体（球）的读数：`(末态 y, 峰值 KE, 末态 KE, 末窗 |v| 均值, 升到地面之上的刻度)`。
struct Reading {
    y_end: f32,
    ke_peak: f32,
    ke_end: f32,
    v_tail: f32,
}

fn ke(w: &World, i: usize) -> f32 {
    let m = 1.0 / w.bodies.inv_mass[i];
    0.5 * m * w.bodies.linvel[i].length_squared()
}

/// 搭台：子步覆盖 + 世界 + 材质 + 埋在 `surface_y - depth` 的球；返回（世界, 球索引）。
fn build_world(surface_y: f32, depth: f32, build: impl Fn(&mut World)) -> (World, usize) {
    let subs = env_f32("PROBE_SUBSTEPS", 0.0);
    let cfg = if subs >= 1.0 {
        PhysConfig {
            substeps: subs as u32,
            ..PhysConfig::default()
        }
    } else {
        PhysConfig::default()
    };
    let mut w = World::new(cfg);
    build(&mut w);
    let mu = env_f32("PROBE_MU", 0.9);
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.4, surface_y - depth, -0.4),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    (w, i)
}

/// 首 tick 的接触摘要：配对 + 各流形的法线与最深点。
fn first_tick_note(w: &World, i: usize) -> String {
    let man: Vec<String> = w
        .manifolds()
        .iter()
        .filter(|mf| mf.a as usize == i || mf.b as usize == i)
        .map(|mf| {
            let d = mf.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
            format!(
                "n=({:+.2},{:+.2},{:+.2}) pts={} dmax={d:+.4}",
                mf.normal.x,
                mf.normal.y,
                mf.normal.z,
                mf.points.len()
            )
        })
        .collect();
    let in_pairs = w
        .pairs()
        .iter()
        .any(|&(p, q)| p as usize == i || q as usize == i);
    format!("｜首 tick：配对 {in_pairs}、流形 [{}]", man.join(" | "))
}

/// `PROBE_TRACE` 的逐 tick 一行（位置/速度/流形）。
fn trace_line(w: &World, i: usize, k: usize) -> String {
    let man: Vec<String> = w
        .manifolds()
        .iter()
        .filter(|mf| mf.a as usize == i || mf.b as usize == i)
        .map(|mf| {
            let d = mf.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
            let p0 = mf
                .points
                .first()
                .map(|p| format!("{:+.2},{:+.2},{:+.2}", p.point.x, p.point.y, p.point.z))
                .unwrap_or_default();
            format!(
                "({},{}) n=({:+.5},{:+.5},{:+.5}) d={d:+.5} p=({p0})",
                mf.a, mf.b, mf.normal.x, mf.normal.y, mf.normal.z,
            )
        })
        .collect();
    format!(
        "    t={k:>3} y={:+.4} vy={:+.4} x={:+.4} vx={:+.4} wz={:+.3} man=[{}]",
        w.bodies.position[i].y,
        w.bodies.linvel[i].y,
        w.bodies.position[i].x,
        w.bodies.linvel[i].x,
        w.bodies.angvel(i).z,
        man.join(" | ")
    )
}

fn run(
    name: &str,
    surface_y: f32,
    build: impl Fn(&mut World),
    depth: f32,
    ticks: usize,
) -> Reading {
    let (mut w, i) = build_world(surface_y, depth, build);
    let (mut ke_peak, mut v_tail, mut n) = (0.0f32, 0.0f32, 0.0f32);
    let mut first_tick = String::new();
    for k in 0..ticks {
        w.step();
        if k == 0 {
            first_tick = first_tick_note(&w, i);
        }
        if std::env::var_os("PROBE_TRACE").is_some() && k < env_f32("PROBE_TRACE_N", 60.0) as usize
        {
            println!("{}", trace_line(&w, i, k));
        }
        ke_peak = ke_peak.max(ke(&w, i));
        if k + 60 >= ticks {
            v_tail += w.bodies.linvel[i].length();
            n += 1.0;
        }
    }
    let r = Reading {
        y_end: w.bodies.position[i].y,
        ke_peak,
        ke_end: ke(&w, i),
        v_tail: v_tail / n,
    };
    println!(
        "  [{name}] 埋深 {depth:.2} m ⇒ 末态 y = {:+.4}（球心应 ≈ {:.2}）、峰值 KE = {:.3} J、\
         末态 KE = {:.3e} J、末窗 |v| 均 = {:.5} m/s{first_tick}",
        r.y_end,
        surface_y + R,
        r.ke_peak,
        r.ke_end,
        r.v_tail,
    );
    r
}

/// `hull` 档夹具：4×4×4 m 盒壳（顶面 y = 0）的 8 点凸体外壳。
fn add_hull_floor(w: &mut World) {
    // 8 点盒壳：**几何中心在原点**（点 ∈ ±2）⇒ 位姿下移 2 m 让顶面正好在 y = 0，
    // 且 `half` 与点集匹配（宽相 AABB 才不会错）。
    let mut pts = Vec::new();
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                pts.push(Vec3::new(sx * 2.0, sy * 2.0, sz * 2.0));
            }
        }
    }
    let hull = w.add_hull(pts);
    w.add_static(
        Shape::ConvexHull {
            hull,
            half: Vec3::new(2.0, 2.0, 2.0),
        },
        Vec3::new(0.0, -2.0, 0.0),
        Quat::IDENTITY,
    );
}

/// `box` 档夹具：同一个 4×4×4 m 静态盒（顶面 y = 0），走解析腿。
fn add_box_floor(w: &mut World) {
    w.add_static(
        Shape::Box {
            half: Vec3::new(2.0, 2.0, 2.0),
        },
        Vec3::new(0.0, -2.0, 0.0),
        Quat::IDENTITY,
    );
}

fn main() {
    let depth = env_f32("PROBE_DEPTH", 0.5);
    let ticks: usize = env_f32("PROBE_TICKS", 400.0) as usize;
    println!("深穿透恢复探针（球 r = {R}、埋深 {depth} m、{ticks} tick）");
    run(
        "mesh ",
        0.0,
        |w| {
            w.add_mesh(flat_mesh());
        },
        depth,
        ticks,
    );
    run(
        "voxel",
        1.0,
        |w| {
            w.add_voxel(flat_voxel());
        },
        depth,
        ticks,
    );
    run(
        "splat",
        0.0,
        |w| {
            w.add_splat_field(flat_splat());
        },
        depth,
        ticks,
    );
    run("hull ", 0.0, add_hull_floor, depth, ticks);
    // 静态**盒**（同一个 4×4×4 m 盒、顶面 y = 0）—— 走 `prims::sphere_convex_ab` 的"球心在凸体内部"分支。
    run("box  ", 0.0, add_box_floor, depth, ticks);
}
