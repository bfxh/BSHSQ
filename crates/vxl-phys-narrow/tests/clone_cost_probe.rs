//! **窄相并行路径的 `clone()` 成本**（成本探针，只报数不断言；2026-09-27 立）。
//!
//! ## 为什么量它
//!
//! `crates/vxl-phys-narrow/src/entry.rs` 的并行分支对**每个块**做一次
//! `let mut np = this.clone();`（`DefaultNarrowPhase` 是 `#[derive(Clone)]`），
//! 即 **每 tick 每子步 × 块数** 次深拷贝（块数 = `min(threads, pairs/2048)`）。
//! 而该结构体里装的**不只是 scratch**：
//!
//! | 字段 | 是什么 | collide 期间 |
//! |---|---|---|
//! | `hulls: HullStore{ hulls: Vec<ConvexHull> }` | **全部注册外壳的点云**（每个 `ConvexHull` 内含 `Vec<Vec3>`） | **只读** |
//! | `compounds: CompoundStore{ items: Vec<Vec<CompoundChild>> }` | **全部复合体子形状表** | **只读** |
//! | `polys` / `poly_index` | 世界多面体**缓存**（`poly_for` 会 push/insert） | 读写 |
//! | `axes` / `clip_in` / `clip_out` / `cand` / `kept_buf` / `ref_v` / `poly_a` / `poly_b` / `kids_buf` / `hull_pts` | scratch | 读写 |
//!
//! ⇒ **前两类（只读注册表）被逐块复制在结构上是浪费**，且成本随**注册的外壳/复合体
//! 总量**增长，而非随本子步要处理的对数增长。它一直没被注意到的原因很可能是：
//! 主力性能场景（`m1_scale` 的 10 万盒）**一个外壳都没注册** ⇒ clone 近乎免费；
//! 只有外壳密集场景（`arena_bench` 三角网探针：200 体里约 1/3 是 34 点外壳）才会显现。
//!
//! ## 口径（照 `KNOWLEDGE.md` 的读数纪律）
//!
//! - 每档**预热 1 次**（丢弃，首次会触碰新堆页），再取 **3 轮的最小值 + 极差比**。
//! - `std::hint::black_box` 防优化器把 clone 消掉。
//! - **三组读数**：(a) 单个 clone；(b) **一次 8 个**（= 默认 8 线程下每趟 collide 的真实
//!   形状：8 个 clone 同时存活再一起释放）；(c) 先跑一遍 `collide` 把 `polys` 缓存填起来，
//!   看缓存那份额外贡献。
//!
//! ⚠️ **本机读数的可信度**：`KNOWLEDGE.md` 记过本机单跑漂移 ±5%（并发时 ±25%），而
//! 这个量是**小对象分配**主导 ⇒ 实测出现过**非单调**（256 外壳 60.7µs > 1024 外壳 34.8µs，
//! 极差 ×2.25）。**结论按"量级"读，别按单点定论**；要拿它当判据须在安静机上成批交错 A/B
//! （`scripts/ab_perf.sh`）。
//!
//! 运行：`cargo test --release -p vxl-phys-narrow --test clone_cost_probe -- --nocapture`

use std::hint::black_box;
use std::time::Instant;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Shape, Vec3};
use vxl_phys_narrow::{DefaultNarrowPhase, NarrowPhase};

/// 复刻 `arena_bench` 里"圆柱降级为凸包"的点云：双环 16 段 + 两端心 = 34 点。
fn cylinder_like_hull(r: f32, hh: f32) -> Vec<Vec3> {
    let mut pts = Vec::with_capacity(34);
    for ring in [-1.0f32, 1.0] {
        for k in 0..16 {
            let a = k as f32 / 16.0 * std::f32::consts::TAU;
            pts.push(Vec3::new(a.cos() * r, ring * hh, a.sin() * r));
        }
    }
    pts.push(Vec3::new(0.0, -hh, 0.0));
    pts.push(Vec3::new(0.0, hh, 0.0));
    pts
}

/// 3 轮取最小 + 极差比。`body` 是一次被计时的动作（可含多个 clone）。
fn best_us(rounds: usize, mut body: impl FnMut()) -> (f64, f64, f64) {
    body(); // 预热（丢弃）
    let mut best = f64::INFINITY;
    let mut worst = 0.0f64;
    for _ in 0..rounds {
        let t = Instant::now();
        body();
        let us = t.elapsed().as_secs_f64() * 1e6;
        best = best.min(us);
        worst = worst.max(us);
    }
    (best, worst, worst / best.max(1e-9))
}

/// **单个** clone 的 µs（`reps` 次取均值）。
fn one_clone_us(np: &DefaultNarrowPhase, reps: usize) -> (f64, f64, f64) {
    let (b, w, s) = best_us(3, || {
        for _ in 0..reps {
            let c = np.clone();
            black_box(&c);
        }
    });
    (b / reps as f64, w / reps as f64, s)
}

/// **引擎真实形状**：一次 8 个 clone 同时存活再一起释放（= 8 线程下每趟 collide 的形状），
/// 返回**每趟**的 µs。
fn batch8_us(np: &DefaultNarrowPhase, reps: usize) -> (f64, f64, f64) {
    let (b, w, s) = best_us(3, || {
        for _ in 0..reps {
            let batch: Vec<DefaultNarrowPhase> = (0..8).map(|_| np.clone()).collect();
            black_box(&batch);
        }
    });
    (b / reps as f64, w / reps as f64, s)
}

/// 只注册 `n` 个外壳（不跑 collide）。
fn with_hulls(n: usize) -> DefaultNarrowPhase {
    let mut np = DefaultNarrowPhase::new(0.01);
    for _ in 0..n {
        np.add_hull(cylinder_like_hull(0.3, 0.34));
    }
    np
}

/// 注册 `n` 个外壳 + 建 `n` 个外壳体 + 一串相邻对，并**跑一遍 collide**
/// （把 `polys`/`poly_index` 世界多面体缓存填起来）。
fn with_hulls_and_cache(n: usize) -> DefaultNarrowPhase {
    let mut np = DefaultNarrowPhase::new(0.01);
    let mut b = BodySet::new();
    let half = Vec3::new(0.3, 0.34, 0.3);
    for i in 0..n {
        let hull = np.add_hull(cylinder_like_hull(0.3, 0.34));
        // 相邻体紧贴（间距 0.5 < 直径 0.6）⇒ 真的产生接触、真的填缓存
        b.push_dynamic(
            Shape::ConvexHull { hull, half },
            Vec3::new(i as f32 * 0.5, 0.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
    }
    let pairs: Vec<(u32, u32)> = (0..n.saturating_sub(1))
        .map(|i| (i as u32, i as u32 + 1))
        .collect();
    let mut out = Vec::new();
    if !pairs.is_empty() {
        np.collide(&b, &pairs, &[], &NoProviders, &mut out, &SerialJobSystem);
    }
    println!(
        "    （已跑一遍 collide：{} 对 → {} 条流形 ⇒ `polys` 缓存已填）",
        pairs.len(),
        out.len()
    );
    np
}

#[test]
fn narrow_phase_clone_cost() {
    const REPS: usize = 200;
    println!("\n== 窄相并行路径的 `clone()` 成本 ==（外壳注册量 → µs）");
    println!("   口径：预热 1 次 + 3 轮 × {REPS} 次取最小；`spread` = 3 轮极差比（>2 说明噪声大）");
    println!("\n   --- (a) 单个 clone ---");
    println!("   外壳数      单个 clone µs      spread");
    let mut singles: Vec<(usize, f64)> = Vec::new();
    for n in [0usize, 64, 256, 1024] {
        let np = with_hulls(n);
        let (best, _worst, spread) = one_clone_us(&np, REPS);
        println!("   {n:6}      {best:12.3}        ×{spread:.2}");
        singles.push((n, best));
    }

    println!("\n   --- (b) 引擎真实形状：一次 8 个 clone（8 线程 = 每趟 collide 的形状）---");
    println!("   外壳数      每趟 µs（8 clone）   spread");
    let mut batches: Vec<(usize, f64)> = Vec::new();
    for n in [0usize, 64, 256, 1024] {
        let np = with_hulls(n);
        let (best, _worst, spread) = batch8_us(&np, REPS / 8);
        println!("   {n:6}      {best:12.3}        ×{spread:.2}");
        batches.push((n, best));
    }

    println!("\n   --- (c) 含 world-polytope 缓存（先跑一遍 collide）---");
    println!("   外壳数      每趟 µs（8 clone）   spread");
    for n in [256usize, 1024] {
        let np = with_hulls_and_cache(n);
        let (best, _worst, spread) = batch8_us(&np, REPS / 8);
        println!("   {n:6}      {best:12.3}        ×{spread:.2}");
    }

    println!("\n   --- 外推：每 tick = substeps 2 × 每子步一趟（= 2 × 上表 b 列）---");
    for (n, us) in &batches {
        println!("   外壳 {n:5} 个  → {:7.3} ms/tick", us * 2.0 / 1000.0);
    }
    println!(
        "\n   参照（本仓已录读数）：`m1_scale` 窄相 均 14.62 ms/tick；`arena_bench` 三角网探针 ≈22 ms\n   \
         读法：拿外推列比参照列 ⇒ 得 clone 占该场景窄相的比例。\n   \
         ⚠️ 本机该量是**小对象分配**主导 ⇒ 读数可能非单调（见表中 spread），**按量级读**。\n"
    );

    // 仪器自检（不是判据）：clone 后注册表必须一致。
    let np = with_hulls(64);
    assert_eq!(np.clone().hull_points(63).len(), 34, "clone 应保住注册表");
}
