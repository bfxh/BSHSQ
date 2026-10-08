//! run：arena_bench 的入口与分派（**纯搬移**自 `main.rs`，2026-10-08；`main.rs` 只留一行装配）。
use super::*;

pub(crate) fn run() {
    let mut args = std::env::args().skip(1);
    let which = args.next().unwrap_or_else(|| "pyramid".to_string());
    let rest: Vec<String> = args.collect();
    let cfg = apply_flags(&rest);
    println!(
        "配置：iterations={} inner={} substeps={} contact_skin={}",
        cfg.velocity_iterations, cfg.normal_inner, cfg.substeps, cfg.contact_skin
    );
    if which == "--list" {
        registry::list();
        return;
    }
    let scenes: Vec<&str> = if which == "--all-arena" {
        registry::all_ids()
    } else if which == "all" {
        vec!["pyramid", "wall", "ballpit"]
    } else {
        vec![which.as_str()]
    };
    for s in scenes {
        run_one(s, &cfg, &rest);
    }
}

/// `--iters N`（迭代数敏感性）/ `--substeps N` / `--inner N` 的配置覆盖。
fn apply_flags(rest: &[String]) -> PhysConfig {
    let mut cfg = PhysConfig::default();
    let pick = |name: &str| -> Option<u32> {
        rest.iter()
            .position(|a| a == name)
            .and_then(|pos| rest.get(pos + 1))
            .and_then(|s| s.parse::<u32>().ok())
    };
    if let Some(v) = pick("--iters") {
        cfg.velocity_iterations = v;
    }
    if let Some(v) = pick("--substeps") {
        cfg.substeps = v;
    }
    if let Some(v) = pick("--inner") {
        cfg.normal_inner = v;
    }
    cfg
}

/// 单场景：打印型探针走各自路径；其余建 `World` → 通用 `bench()` 或专用窗。
fn run_one(s: &str, cfg: &PhysConfig, rest: &[String]) {
    // slide / approach 等是"打印轨迹"型基准，不走 bench() 的统计口径。
    match s {
        "slide" => return scene_slide(cfg.clone(), 120),
        "approach" => return scene_approach(cfg.clone()),
        "voxel_land" => return scene_voxel_land(cfg.clone()),
        "wall_provider" => return scene_wall_provider(cfg.clone()),
        "mesh_land" => return scene_mesh_land(cfg.clone()),
        "fidelity" => {
            scene_bounce(cfg.clone());
            return scene_incline(cfg.clone());
        }
        "joints" => return scene_joint_probes(cfg.clone()),
        "joint_chains" => {
            println!(
                "配置：joint_iterations={} substeps={}",
                cfg.joint_iterations, cfg.substeps
            );
            return scene_joint_chains(cfg.clone());
        }
        _ => {}
    }
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
    let mut impact_spec: Option<registry::ImpactSpec> = None;
    let mut w = match s {
        "pyramid" => match mu_ovr {
            Some(mu) => scene_pyramid_mu(cfg.clone(), mu),
            None => scene_pyramid(cfg.clone()),
        },
        "wall" => scene_wall(cfg.clone()),
        "ballpit" => scene_ballpit(cfg.clone()),
        "trimesh" => scene_trimesh_terrain(cfg.clone()),
        // 注册表场景（PhysArena 复刻批 + 本仓域批）：id 同名同参，见 registry.rs
        other => match registry::build(other, cfg.clone()) {
            Some(r) => {
                impact_spec = r.impact;
                r.world
            }
            None => {
                eprintln!("未知场景 {other}（std + 注册表全量见 `arena_bench --list`）");
                return;
            }
        },
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
    match impact_spec {
        Some(spec) => bench_impact(s, w, spec.provider, spec.threshold, spec.density),
        None => bench(s, w, extra),
    }
}
