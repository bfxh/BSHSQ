//! **B5 基线**：M1 的 4 子步配置下，窄相卡上档每 tick 到底几趟、同步往返占多少。
//!
//! 两条链只差"谁跑窄相"：A = 默认 CPU 窄相；B = 注册卡上档（`NarrowTier`）。
//! 读数口径：`World` 累计 + `NarrowTierStats` 分项 + GPU 档内 `prof`（`[4]` = 提交+映射+等）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_roundtrip_baseline -- [ticks]`
use vxl_phys::{
    BodyBuildExt, HeightField, NarrowTierExt, PhysConfig, Quat, Shape, Vec3, World,
};
use vxl_phys_gpu::narrow::NarrowTier;

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;

fn cfg() -> PhysConfig {
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

fn scene() -> World {
    let mut w = World::new(cfg());
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

fn main() {
    let ticks: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);
    // A：默认 CPU 窄相。
    let mut a = scene();
    for _ in 0..20 {
        a.step();
    }
    a.reset_timings();
    let t0 = std::time::Instant::now();
    for _ in 0..ticks {
        a.step();
    }
    let a_ms = t0.elapsed().as_secs_f64() * 1000.0 / ticks as f64;
    let a_narrow = a.timings().narrowphase_us as f64 / 1000.0 / ticks as f64;

    // B：卡上窄相档。
    let mut b = scene();
    for _ in 0..20 {
        b.step();
    }
    // ⚠️ 容量要按**宽相对数**给：m1 10k 体的候选对可达十万级，给 65536 会整趟回退（实测）。
    let (cap_b, cap_p) = (b.bodies.len() as u32 + 8, 400_000u32);
    let tier = match NarrowTier::new(0, cap_b, cap_p, cfg().contact_skin) {
        Ok(t) => t,
        Err(e) => {
            println!("⚠️ 建卡上档失败（{e}）");
            return;
        }
    };
    b.set_narrow_tier(Box::new(tier));
    // 预热两 tick（建常驻表 + 编译）。
    for _ in 0..2 {
        b.step();
    }
    // 诊断：逐 tick 看这一档进了几次（4 子步 ⇒ 期望 4）。
    {
        let mut prev = b.narrow_tier_stats().calls;
        for t in 0..6 {
            b.step();
            let now = b.narrow_tier_stats().calls;
            println!("   [诊断] tick {t}: 本 tick 进档 {} 次 · 常驻 {}", now - prev, b.narrow_resident_active());
            prev = now;
        }
    }
    vxl_phys_gpu::narrow::prof::reset();
    b.reset_timings();
    let before = b.narrow_tier_stats();
    let t1 = std::time::Instant::now();
    for _ in 0..ticks {
        b.step();
    }
    let b_ms = t1.elapsed().as_secs_f64() * 1000.0 / ticks as f64;
    let b_narrow = b.timings().narrowphase_us as f64 / 1000.0 / ticks as f64;
    let after = b.narrow_tier_stats();
    let prof = vxl_phys_gpu::narrow::prof_snapshot();

    let calls = (after.calls - before.calls) as f64;
    let per = |x: u64, y: u64| (y - x) as f64 / 1000.0; // µs → ms
    let pass = |x: u64, y: u64| per(x, y) / calls.max(1.0);
    println!("场景 m1pile {LAYERS}×{SIDE}² · 4 子步 · {ticks} tick");
    println!("A 默认（CPU 窄相）：{a_ms:.2} ms/tick（窄相 {a_narrow:.3} ms/tick）");
    println!(
        "B 卡上档：{b_ms:.2} ms/tick（窄相 {b_narrow:.3} ms/tick）· 常驻 {}",
        if b.narrow_resident_active() {
            "已生效 ✓"
        } else {
            "未生效 ✗"
        }
    );
    let pairs_per = (after.pairs_sum - before.pairs_sum) as f64 / calls.max(1.0);
    println!(
        "   窄相趟数 {:.1} 趟/tick（{calls:.0} 趟 / {ticks} tick，均 {pairs_per:.0} 对/趟）⇒ **每趟一次同步往返**",
        calls / ticks as f64
    );
    println!(
        "   分项（每趟 ms）：打包 {:.3} · 比对 {:.3} · 上传 {:.3} · 后端 {:.3} · 回填+装配 {:.3}",
        pass(before.pack_us, after.pack_us),
        pass(before.diff_us, after.diff_us),
        pass(before.upload_us, after.upload_us),
        pass(before.run_us, after.run_us),
        pass(before.assemble_us, after.assemble_us)
    );
    println!(
        "   后端内（每趟 ms）：写 {:.3} · 参数+对表 {:.3} · 编码 {:.3} · **提交+映射+等 {:.3}** · 解码 {:.3}",
        pass(0, prof[1]),
        pass(0, prof[2]),
        pass(0, prof[3]),
        pass(0, prof[4]),
        pass(0, prof[5])
    );
    let rt_tick = per(0, prof[4]) / ticks as f64;
    println!(
        "   ⇒ 往返 {rt_tick:.3} ms/tick（占窄相卡上档 {:+.0}%）",
        100.0 * rt_tick / b_narrow.max(1e-9)
    );
    println!(
        "   ⇒ 卡上档窄相比默认 CPU 慢 {:.2}×（{:.3} vs {:.3} ms/tick）",
        b_narrow / a_narrow.max(1e-9),
        b_narrow,
        a_narrow
    );
    println!("   （参照：两链整 tick {:.2} vs {:.2} ms）", a_ms, b_ms);
}
