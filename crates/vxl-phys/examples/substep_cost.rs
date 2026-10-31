//! **每子步开销占比**（M1 性能档：先量成本结构，再决定"让检测更便宜"往哪投）。
//!
//! 用法：
//! `cargo run --release -p vxl-phys --example substep_cost -- [tower|pile5|m1pile] [ticks] [iters] [substeps,csv] [threads]`
//! 默认 `m1pile 120 3 1,4 1`。每档跑两遍（顺序交错），便于看出热漂移。
//!
//! 口径：`World::timings()` 的**引擎侧**累计（不含外层墙钟），单位 ms/tick；
//! `wall` 是 `Instant` 实测，两者之差 = 引擎没计的其它开销。本工具只读，不改任何行为。

use std::time::Instant;

use vxl_phys::{BodyBuildExt, HeightField, PhysConfig, Quat, Shape, Vec3, World};

const WARMUP: u32 = 10;

struct Args {
    mode: String,
    ticks: u32,
    iters: u32,
    substeps: Vec<u32>,
    threads: usize,
}

fn parse_args() -> Args {
    let mut a = std::env::args().skip(1);
    let mode = a.next().unwrap_or_else(|| "m1pile".to_string());
    let ticks = a.next().and_then(|s| s.parse().ok()).unwrap_or(120);
    let iters = a.next().and_then(|s| s.parse().ok()).unwrap_or(3);
    let substeps = a
        .next()
        .map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_else(|| vec![1, 4]);
    let threads = a.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    Args {
        mode,
        ticks,
        iters,
        substeps,
        threads,
    }
}

/// 场景与 M1 的验收跑轮对齐：`m1pile` = `m1_pile` 的 25×20×20；`tower25`/`pile5` = §3.5 的两组。
fn build(mode: &str, iters: u32, substeps: u32, threads: usize) -> World {
    let cfg = PhysConfig {
        velocity_iterations: iters,
        substeps,
        threads,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    if mode == "m1pile" {
        w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    } else {
        w.add_static(
            Shape::Box {
                half: Vec3::new(6.0, 0.1, 6.0),
            },
            Vec3::new(0.0, -0.1, 0.0),
            Quat::IDENTITY,
        );
    }
    let (layers, side) = match mode {
        "tower" => (25usize, 1usize),
        "pile5" => (5, 20),
        _ => (25, 20),
    };
    for l in 0..layers {
        for iz in 0..side {
            for ix in 0..side {
                let x = (ix as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                let z = (iz as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(x, 0.25 + 0.52 * l as f32, z),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

struct Row {
    awake: u32,
    contacts: u32,
    islands: u32,
    groups: u32,
    manifolds: u32,
    wall_ms: f64,
    fields: f64,
    broad: f64,
    narrow: f64,
    solve: f64,
    integ: f64,
    ccd: f64,
}

fn measure(mode: &str, ticks: u32, iters: u32, substeps: u32, threads: usize) -> Row {
    let mut w = build(mode, iters, substeps, threads);
    for _ in 0..WARMUP {
        w.step();
    }
    w.reset_timings();
    let t0 = Instant::now();
    for _ in 0..ticks {
        w.step();
    }
    let wall_ms = t0.elapsed().as_secs_f64() * 1000.0 / ticks as f64;
    let t = w.timings();
    let h = w.health();
    let d = &w.solver.island_diag;
    let us = |v: u64| v as f64 / 1000.0 / ticks as f64;
    Row {
        awake: h.awake_bodies,
        contacts: h.contacts,
        islands: d.islands,
        groups: d.g_count,
        manifolds: d.manifolds,
        wall_ms,
        fields: us(t.fields_us),
        broad: us(t.broadphase_us),
        narrow: us(t.narrowphase_us),
        solve: us(t.solve_us),
        integ: us(t.integrate_vel_us + t.integrate_pos_us),
        ccd: us(t.ccd_us),
    }
}

fn report(a: &Args, substeps: u32, pass: u32, r: &Row) {
    let engine = r.fields + r.broad + r.narrow + r.solve + r.integ + r.ccd;
    let pct = |v: f64| 100.0 * v / engine.max(1e-9);
    println!(
        "[{}] pass{} ss={} iters={} threads={} | awake {} contacts {} islands {} groups {} manifolds {} | wall {:7.2} ms/tick | engine {:7.2} \
         | broad {:6.2} ({:5.1}%) narrow {:6.2} ({:5.1}%) solve {:6.2} ({:5.1}%) \
         integ {:5.2} ({:4.1}%) fields {:5.2} ({:4.1}%) ccd {:5.2} ({:4.1}%)",
        a.mode,
        pass,
        substeps,
        a.iters,
        a.threads,
        r.awake,
        r.contacts,
        r.islands,
        r.groups,
        r.manifolds,
        r.wall_ms,
        engine,
        r.broad,
        pct(r.broad),
        r.narrow,
        pct(r.narrow),
        r.solve,
        pct(r.solve),
        r.integ,
        pct(r.integ),
        r.fields,
        pct(r.fields),
        r.ccd,
        pct(r.ccd),
    );
}

fn main() {
    let a = parse_args();
    // 两遍顺序交错（1,4,1,4）：单跑不可信，热漂移要看得见。
    for pass in 1..=2 {
        for &ss in &a.substeps {
            let r = measure(&a.mode, a.ticks, a.iters, ss, a.threads);
            report(&a, ss, pass, &r);
        }
    }
}
