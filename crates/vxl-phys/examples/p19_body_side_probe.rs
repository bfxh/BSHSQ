//! **P19 ⑦ 体侧隧道的取证探针**（`OPEN-PROBLEMS.md` P19 的三段收口）。
//!
//! 场景：4×4 m 单页薄板（`Shape::TriMesh` **体**，x = 0 平面、环绕法线 +X）+ 从 −5 m 沿 +X
//! 撞来的一个体（盒/球/胶囊/圆柱/圆锥）。逐 tick 读「该对**有没有流形**、**在不在配对表里**、
//! 两边醒没醒」以及双方的位置/速度。
//!
//! 为什么必须有它（2026-10-14 的翻案）：只看"末态 x"会把"**盒没停住**"读成"**流形没到求解器**"；
//! 逐 tick 打印**流形表 + 配对表**一看就清楚：修前是 `pair=true` 而**流形表恒空**
//! （该对整场零候选），修后是"贴而不穿"的两体**同漂**（自由薄板的正常物理）。
//!
//! 跑法（`PROBE_SHAPE` 选形状、`PROBE_V` 改初速、`PROBE_NO_CCD` 关 CCD、
//! `PROBE_FIXED_PLATE=1` 把薄板设成撞不动、`PROBE_SCAN=1` 走相位扫描）：
//! ```text
//! cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! PROBE_SHAPE=cylinder PROBE_FIXED_PLATE=1 cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! PROBE_V=9 PROBE_SCAN=1 PROBE_FIXED_PLATE=1 cargo run -q --release -p vxl-phys --example p19_body_side_probe
//! ```
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 撞击体的"沿 +X 尺寸"基线（盒半长 = 球半径 = 圆柱/锥半径；胶囊的轴是 +Y ⇒ 只有半径）。
const HALF: f32 = 0.5;
const TICKS: u32 = 400;

fn main() {
    let v0 = env_f32("PROBE_V", 60.0);
    let fixed = std::env::var_os("PROBE_FIXED_PLATE").is_some();
    if std::env::var_os("PROBE_SCAN").is_some() {
        scan_phases(fixed, v0);
        return;
    }
    let (mut w, plate, b) = scene(fixed, probe_shape(), -5.0, v0);
    println!("v0={v0} plate={plate} body={b} fixed={fixed}");
    for k in 0..TICKS {
        w.step();
        let man = manifolds_of(&w, b);
        let in_pairs = w
            .pairs()
            .iter()
            .any(|&(p, q)| p as usize == b || q as usize == b);
        if k < 40 || !man.is_empty() || in_pairs {
            println!(
                "t={k:>3} x={:>9.4} vx={:>9.4} plate_x={:>9.4} plate_vx={:>9.4} awake(p,b)=({},{}) pair={in_pairs} man=[{}]",
                w.bodies.position[b].x,
                w.bodies.linvel[b].x,
                w.bodies.position[plate].x,
                w.bodies.linvel[plate].x,
                w.bodies.awake[plate],
                w.bodies.awake[b],
                man
            );
        }
    }
    println!("FINAL x = {:.4}", w.bodies.position[b].x);
}

/// **相位扫描**（`PROBE_SCAN=1`）：起始 x 扫过**一个子步位移**（= 离散窄相的采样周期）⇒ 覆盖全部
/// 相位。用途：判"体侧窗口 `2·skin` vs 子步位移"这条缺口是**真的**（存在穿过去的相位）还是
/// **只是理论上的**（体自身的几何把有效窗口撑宽了）。
fn scan_phases(fixed: bool, v: f32) {
    let sub = v / 120.0; // 子步位移（dt = 1/60、默认 2 子步）
    let mut worst = (f32::MIN, 0usize);
    let mut thru = 0usize;
    for k in 0..64 {
        let x0 = -5.0 + sub * k as f32 / 64.0;
        let (mut w, _plate, b) = scene(fixed, probe_shape(), x0, v);
        for _ in 0..TICKS {
            w.step();
        }
        let x = w.bodies.position[b].x;
        thru += usize::from(x > HALF);
        if x > worst.0 {
            worst = (x, k);
        }
    }
    println!(
        "相位扫描（v={v}、子步 {sub:.5}、64 档）：穿过 {thru} 档；最坏末态 x = {:.4}（相位 {}）",
        worst.0, worst.1
    );
}

/// 场景（`main` 与相位扫描共用）：4×4 m 单页薄板（`Shape::TriMesh` **体**，x = 0 平面、
/// 环绕法线 +X）+ 从 `x0` 以 `v0` 沿 +X 撞来的 `shape`。`fixed` = 板撞不动（`inv_mass = 0`）。
fn scene(fixed: bool, shape: Shape, x0: f32, v0: f32) -> (World, usize, usize) {
    let mut cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    if std::env::var_os("PROBE_NO_CCD").is_some() {
        cfg.ccd_speed_threshold = f32::INFINITY;
    }
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    let mesh = w.add_trimesh(pts, vec![[0u32, 1, 2], [0, 2, 3]]);
    let plate = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 1.0e5, 0.02) as usize;
    if fixed {
        w.bodies.inv_mass[plate] = 0.0; // 只当障碍：金丝雀原意（密度大 ⇒ 撞不动）
        w.bodies.local_inv_inertia[plate] = Vec3::ZERO;
    }
    let b = w.add_dynamic(shape, Vec3::new(x0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    w.bodies.linvel[b] = Vec3::new(v0, 0.0, 0.0);
    (w, plate, b)
}

/// 撞击体形状（`PROBE_SHAPE`：box（默认）/ sphere / capsule / cylinder / cone）——**沿 +X 的伸距**
/// 除胶囊外都取 `HALF`（胶囊的轴是 +Y ⇒ 伸距只有半径 `HALF/2`）⇒ "停在壳面上"的期望位置 = `−伸距`。
fn probe_shape() -> Shape {
    match std::env::var("PROBE_SHAPE").unwrap_or_default().as_str() {
        "sphere" => Shape::Sphere { radius: HALF },
        "capsule" => Shape::Capsule {
            half_height: HALF * 0.5,
            radius: HALF * 0.5,
        },
        "cylinder" => Shape::Cylinder {
            half_height: HALF * 0.5,
            radius: HALF,
        },
        "cone" => Shape::Cone {
            half_height: HALF * 0.5,
            radius: HALF,
        },
        _ => Shape::Box {
            half: Vec3::splat(HALF),
        },
    }
}

/// 与本体相关的流形（一行一条：法线 / 点数 / 最深）。
fn manifolds_of(w: &World, b: usize) -> String {
    w.manifolds()
        .iter()
        .filter(|m| m.a as usize == b || m.b as usize == b)
        .map(|m| {
            let d = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
            format!(
                "({},{}) n=({:+.2},{:+.2},{:+.2}) pts={} dmax={d:+.4}",
                m.a,
                m.b,
                m.normal.x,
                m.normal.y,
                m.normal.z,
                m.points.len()
            )
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

fn env_f32(name: &str, dflt: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(dflt)
}
