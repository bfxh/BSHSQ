//! **卡上求解档接进 `World` 之后的第一读数**（`PLAN-gpu.md` §30 B 片；B1+B3+B4 走求解位置，
//! 窄相/宽相/分岛/休眠全留主机）。
//!
//! 回答两个问题：
//! ① **整 tick 多少钱**——与 CPU 默认档**交错 A/B**（热漂移 ±25%，不交错读数不可比）；
//! ② **这堆还撑不撑得住**——同一场景同一预热，两侧各跑满 `ticks`，比 `awake`/`deep`/最大穿透/沉降。
//!
//! ⚠️ 口径：卡上 = Jacobi + 无暖启动 + 无锚点回收 ⇒ **不与默认档比逐位**，只比质量与耗时。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example solve_tier_tick -- [ticks] [rounds] [sweeps]`
use std::time::Instant;

use vxl_phys::{BodyBuildExt, DiagExt, HeightField, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_gpu::solve_tier::{prof_snapshot, SolveTier};

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;
/// m1 配方（`16 4 8 600 2 2 8 4`，逐字照 §3.8 的那台机器上的基准档）。
fn m1_config() -> PhysConfig {
    PhysConfig {
        velocity_iterations: 16,
        substeps: 4,
        threads: 8,
        shock_iterations: 2,
        stabilization_iterations: 2,
        wake_gate_k: 8,
        settled_hold_iterations: 4,
        ..PhysConfig::default()
    }
}

/// 与 `m1_pile::build` 同式的 10k 体堆 + 静默地形。
fn build_scene() -> World {
    let mut w = World::new(m1_config());
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let off = (SIDE as f32 - 1.0) * 0.5 * SPACING;
    for layer in 0..LAYERS {
        let y = 0.25 + layer as f32 * SPACING;
        for row in 0..SIDE {
            for col in 0..SIDE {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * SPACING - off, y, row as f32 * SPACING - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

/// 质量读数（末态）：入睡/深穿透由 `health()`，最大穿透与沉降自己扫。
#[derive(Default)]
struct Quality {
    awake: u32,
    deep: u32,
    max_depth: f32,
    sink: f32,
    contacts: usize,
}

fn measure(w: &World) -> Quality {
    let h = w.health();
    let mut q = Quality {
        awake: h.awake_bodies,
        deep: h.deep_penetrations,
        contacts: w.manifolds().len(),
        ..Default::default()
    };
    for m in w.manifolds() {
        for cp in &m.points {
            q.max_depth = q.max_depth.max(cp.depth);
        }
    }
    // 沉降 = 全堆 y 均值相对初始（初值 = (0.25 + 层均值) 的等距堆）。
    let mut sum = 0.0f32;
    let mut n = 0u32;
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            sum += w.bodies.position[i].y;
            n += 1;
        }
    }
    if n > 0 {
        let expect = 0.25 + (LAYERS as f32 - 1.0) * 0.5 * SPACING;
        q.sink = sum / n as f32 - expect;
    }
    q
}

/// 一段 `ticks` 个 tick 的墙钟（ms/tick）。
fn run(w: &mut World, ticks: u32) -> f64 {
    let t = Instant::now();
    for _ in 0..ticks {
        w.step();
    }
    t.elapsed().as_secs_f64() * 1000.0 / ticks as f64
}

/// 中位数（交错 A/B 之后用它压掉个别抖动轮）。
fn median(v: &[f64]) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.total_cmp(b));
    s[s.len() / 2]
}

struct Args {
    ticks: u32,
    rounds: u32,
    sweeps: u32,
    warm: u32,
}

fn parse_args() -> Args {
    let mut a = std::env::args().skip(1);
    Args {
        ticks: a.next().and_then(|s| s.parse().ok()).unwrap_or(120),
        rounds: a.next().and_then(|s| s.parse().ok()).unwrap_or(5),
        sweeps: a.next().and_then(|s| s.parse().ok()).unwrap_or(12),
        // 第 4 个参数 = 预热 tick 数（默认真配方 120；定位崩溃时给个小值快速复现）。
        warm: a.next().and_then(|s| s.parse().ok()).unwrap_or(120),
    }
}

fn main() {
    let arg = parse_args();
    // 两个**同样预热**的世界：预热段都走 CPU 默认档 ⇒ 进对比段时体态一致（此后才开始分叉）。
    let mut cpu = build_scene();
    let mut gpu = build_scene();
    let t_warm = run(&mut cpu, arg.warm);
    let t_warm2 = run(&mut gpu, arg.warm);
    println!(
        "预热 {warm} tick：CPU {t_warm:.1} ms/tick、GPU 侧同场景 {t_warm2:.1} ms/tick",
        warm = arg.warm
    );
    gpu.solver
        .set_tier(Some(Box::new(open_tier(&gpu))), arg.sweeps);
    let name = gpu.solver.tier_name().unwrap_or("未注册").to_string();
    println!("档：{name} · sweeps={} · 容量按当前体数×2 给", arg.sweeps);

    // 交错 A/B：每轮先 CPU 段再 GPU 段，段长 = ticks。
    let (mut a_ms, mut b_ms) = (Vec::new(), Vec::new());
    for r in 0..arg.rounds {
        println!("轮 {r}：CPU 段 {} tick…", arg.ticks);
        let x = run(&mut cpu, arg.ticks);
        println!(
            "轮 {r}：卡上段 {} tick…（sweeps={}）",
            arg.ticks, arg.sweeps
        );
        let y = run(&mut gpu, arg.ticks);
        println!(
            "轮 {r}：两段都跑完，进档 {} 次",
            gpu.solver.tier_stats().calls
        );
        a_ms.push(x);
        b_ms.push(y);
        println!(
            "轮 {r}：CPU {x:7.1} ms/tick | 卡上档 {y:7.1} ms/tick | 比 {:.1}×",
            x / y.max(0.001)
        );
    }
    let (am, bm) = (median(&a_ms), median(&b_ms));
    println!("----");
    println!(
        "中位：CPU {am:.1} ms/tick | 卡上档 {bm:.1} ms/tick ⇒ **{:.2}×**（同机交错，轮数 {}）",
        am / bm.max(0.001),
        arg.rounds
    );
    report_tier(&gpu);
    println!(
        "---- 质量（各跑满 {} + {} tick）----",
        arg.warm,
        arg.ticks * arg.rounds
    );
    let qc = measure(&cpu);
    let qg = measure(&gpu);
    println!(
        "CPU   ：awake {} | deep {} | 接触 {} | max深度 {:.4} | 沉降 {:+.4}",
        qc.awake, qc.deep, qc.contacts, qc.max_depth, qc.sink
    );
    println!(
        "卡上档：awake {} | deep {} | 接触 {} | max深度 {:.4} | 沉降 {:+.4}",
        qg.awake, qg.deep, qg.contacts, qg.max_depth, qg.sink
    );
    let err = gpu.solver.tier_error();
    println!(
        "回退：{} 趟（其中后端报错 {} 次）｜最近报错：{}",
        gpu.solver.tier_stats().fallbacks,
        gpu.solver.tier_stats().errors,
        err.unwrap_or("无")
    );
}

/// 建后端：容量 = 当前体数 × 2 / 接触数上限 40 万（§3.23 的容量坑：给小了整趟静默回退）。
fn open_tier(w: &World) -> SolveTier {
    let cap_b = (w.bodies.len() as u32) * 2 + 16;
    // 接触上限要按**压实后的宽相对数**给：默认档堆稳后每体平均 6 个接触点，
    // 而求解一旦失稳（穿透）接触数会自我放大（实测 660k）⇒ 给小了会撞护栏、把质量读数糊成回退读数。
    match SolveTier::new(0, cap_b, 1_500_000) {
        Ok(t) => t,
        Err(e) => {
            println!("❌ 卡上档建不起来：{e}（本读数作废，下面全是回退 CPU）");
            std::process::exit(1);
        }
    }
}

/// 分项读数：门面侧（打包/CSR/后端/回写/休眠）+ 后端内部（差量/上传/提交/回读）。
fn report_tier(w: &World) {
    let s = w.solver.tier_stats();
    let n = s.calls.max(1);
    println!("---- 卡上求解档分项（{} 趟，µs/趟）----", s.calls);
    println!(
        "打包 {:.0} | CSR {:.0} | 后端 {:.0} | 回写 {:.0} | 分岛休眠 {:.0} | 均接触 {:.0} | 均体 {:.0}",
        s.pack_us / n,
        s.csr_us / n,
        s.run_us / n,
        s.writeback_us / n,
        s.sleep_us / n,
        s.contacts_sum / n,
        s.bodies_sum / n
    );
    let p = prof_snapshot();
    let c = p[4].max(1);
    println!(
        "后端内：差量+增量打包 {:.0} | write_buffer {:.0} | 编码+提交+等 {:.0} | 映射+解码 {:.0}",
        p[0] / c,
        p[1] / c,
        p[2] / c,
        p[3] / c
    );
}
