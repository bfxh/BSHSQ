//! **离线对比台场景复刻 + 相位剖析**（不依赖浏览器/前端）。
//!
//! 用 PhysArena 的同一批场景（同体数/同尺寸/同出生位姿/同材质）在原生侧跑
//! 固定步长基准，输出每步 p50/p95 与**相位分解**（宽相/窄相/求解/积分/CCD），
//! 用于引擎侧优化迭代。
//!
//! 运行：`cargo run --release -p vxl-phys --example arena_bench [场景] [--iters N]`
//! 场景：pyramid（默认）/ wall / ballpit / all

use vxl_phys::*;
use vxl_phys_core::{FrictionModel, Material, PhysConfig, Shape, Vec3};

pub(crate) const WARMUP: usize = 30;
pub(crate) const MEASURE: usize = 180;

// ── 按域拆出的子模块（子目录 arena_bench/）
mod probes_a;
mod probes_b;
mod probes_c;
pub(crate) use self::{probes_a::*, probes_b::*, probes_c::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

/// 从余下参数解析 `--iters` / `--substeps` / `--inner` 三条旋钮（默认用 `PhysConfig::default`）。
fn parse_config(rest: &[String]) -> PhysConfig {
    let mut cfg = PhysConfig::default();
    // 允许 --iters N 做迭代数敏感性实验（默认 16）。
    if let Some(pos) = rest.iter().position(|a| a == "--iters") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.velocity_iterations = v;
        }
    }
    if let Some(pos) = rest.iter().position(|a| a == "--substeps") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.substeps = v;
        }
    }
    if let Some(pos) = rest.iter().position(|a| a == "--inner") {
        if let Some(v) = rest.get(pos + 1).and_then(|s| s.parse::<u32>().ok()) {
            cfg.normal_inner = v;
        }
    }
    cfg
}

/// 「打印轨迹」型专用场景（不走 `bench()` 的统计口径）；命中即返回 true。
fn dispatch_scene(s: &str, cfg: &PhysConfig) -> bool {
    // slide / approach 是"打印轨迹"型基准，不走 bench() 的统计口径。
    if s == "slide" {
        scene_slide((*cfg).clone(), 120);
        return true;
    }
    if s == "approach" {
        scene_approach((*cfg).clone());
        return true;
    }
    if s == "voxel_land" {
        scene_voxel_land((*cfg).clone());
        return true;
    }
    if s == "wall_provider" {
        scene_wall_provider((*cfg).clone());
        return true;
    }
    if s == "mesh_land" {
        scene_mesh_land((*cfg).clone());
        return true;
    }
    if s == "fidelity" {
        scene_bounce((*cfg).clone());
        scene_incline((*cfg).clone());
        return true;
    }
    if s == "joints" {
        scene_joint_probes((*cfg).clone());
        return true;
    }
    if s == "joint_chains" {
        println!(
            "配置：joint_iterations={} substeps={}",
            cfg.joint_iterations, cfg.substeps
        );
        scene_joint_chains((*cfg).clone());
        return true;
    }
    false
}

/// 三条对照旋钮（`--serial` / `--mu` / `--steps`）+ 场景构建 ⇒ 直接跑 `bench`。
fn run_scene(s: &str, rest: &[String], cfg: &PhysConfig) {
    // `--serial X`：串行作业系统（对照"每步开线程"的开销）。
    // ⚠️ 2026-09-27 清理：此处原本还有一段 `if let Some(pos) = …position("--serial")` 的块，
    // 内容是 `let mut w_cfg = cfg.clone(); w_cfg.velocity_iterations = cfg.velocity_iterations;
    // cfg = w_cfg;`——**对 cfg 的空操作**，且那时 `World` 还没建 ⇒ 它不可能影响任何行为。
    // 真正生效的是这里：抓到标志后把 `w.jobs` 换成 `SerialJobSystem`（见下方 `if serial`）。
    let serial = rest.iter().any(|a| a == "--serial");
    // `--mu X`：金字塔场景的摩擦覆盖（判定"堆不入睡"的能量源）。
    let mu_ovr = rest
        .iter()
        .position(|a| a == "--mu")
        .and_then(|pos| rest.get(pos + 1))
        .and_then(|s| s.parse::<f32>().ok());
    let mut w = match s {
        "pyramid" => match mu_ovr {
            Some(mu) => scene_pyramid_mu((*cfg).clone(), mu),
            None => scene_pyramid((*cfg).clone()),
        },
        "wall" => scene_wall((*cfg).clone()),
        "ballpit" => scene_ballpit((*cfg).clone()),
        "trimesh" => scene_trimesh_terrain((*cfg).clone()),
        other => {
            eprintln!("未知场景 {other}（pyramid / wall / ballpit / all）");
            return;
        }
    };
    if serial {
        w.jobs = Box::new(vxl_phys_core::SerialJobSystem);
    }
    let extra = rest
        .iter()
        .position(|a| a == "--steps")
        .and_then(|pos| rest.get(pos + 1))
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    bench(s, w, extra);
}

pub(crate) fn main() {
    let mut args = std::env::args().skip(1);
    let which = args.next().unwrap_or_else(|| "pyramid".to_string());
    let rest: Vec<String> = args.collect();
    let cfg = parse_config(&rest);
    println!(
        "配置：iterations={} inner={} substeps={} contact_skin={}",
        cfg.velocity_iterations, cfg.normal_inner, cfg.substeps, cfg.contact_skin
    );
    let scenes: Vec<&str> = if which == "all" {
        vec!["pyramid", "wall", "ballpit"]
    } else {
        vec![which.as_str()]
    };
    for s in scenes {
        if dispatch_scene(s, &cfg) {
            continue;
        }
        run_scene(s, &rest, &cfg);
    }
}
