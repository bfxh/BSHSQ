//! **窄相并行 clone：`MeshStore` 那一份账**（成本探针，只报数不断言；2026-10-10 立）。
//!
//! ## 为什么单独一片
//!
//! `PERF-REVIEW-2026-09-27.md` §1 量的是 `DefaultNarrowPhase::clone()` 的成本，但成本表
//! **只登记了 `HullStore` / `CompoundStore`**，漏了同一只读面上的 `MeshStore`（三角网：
//! 布片/薄板/碎片/体素提取产物，每个内含 `Vec<Vec3>` 点云 + 三角表）。本探针把那份账补上
//! ⇒ §1 的触发阈值（"≳400 外壳才值得做"）按**注册几何总量**重读，不是一个只数外壳的数。
//!
//! 实测（本机 release，2026-10-10）：三角网每项约是外壳的 **3×** —— 64 网 333.7 µs/趟、
//! 1024 网 4256.6 µs/趟（对照 §1 表：64 壳 109.3 µs/趟、1024 壳 1416.2 µs/趟）。
//! 而 400 个网格的碎片/提取场景在**注册期**就能出现 ⇒ 该杠杆按 §1.4 的补丁形状（三仓库
//! 改 `Arc`）落地后，本探针的读数掉到亚 µs（量级读，见下）。
//!
//! ## 口径（照 `KNOWLEDGE.md` 的读数纪律）
//!
//! 预热 1 次 + 3 轮取最小；`std::hint::black_box` 防优化器消掉 clone；量的是**一次 8 个
//! 并存**（= 默认 8 线程下每趟 collide 的真实形状）。
//! ⚠️ 本机该量由**小对象分配**主导 ⇒ 读数可能非单调（表中 spread 会 >1.2），**按量级读**。
//!
//! 运行：`cargo test --release -p vxl-phys-narrow --test clone_cost_probe_meshes -- --nocapture`

use std::hint::black_box;
use std::time::Instant;

use vxl_phys_core::Vec3;
use vxl_phys_narrow::DefaultNarrowPhase;

/// 一个"碎片尺度"的三角网：`(nx+1)×(nz+1)` 个点的规则网格三角化（2·nx·nz 个三角）。
fn fragment_mesh(nx: usize, nz: usize) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    for iz in 0..=nz {
        for ix in 0..=nx {
            pts.push(Vec3::new(ix as f32 * 0.1, 0.0, iz as f32 * 0.1));
        }
    }
    let w = (nx + 1) as u32;
    let mut tris = Vec::new();
    for iz in 0..nz as u32 {
        for ix in 0..nx as u32 {
            let a = iz * w + ix;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    (pts, tris)
}

/// 只注册 `n` 个三角网（碎片/布片规模：81 点 / 128 三角）。
fn with_meshes(n: usize) -> DefaultNarrowPhase {
    let mut np = DefaultNarrowPhase::new(0.01);
    let (p, t) = fragment_mesh(8, 8);
    for _ in 0..n {
        np.add_mesh(p.clone(), t.clone());
    }
    np
}

/// 一次 8 个 clone 同时存活再一起释放（= 8 线程下每趟 collide 的形状），返回**每趟**的 µs。
fn batch8_us(np: &DefaultNarrowPhase, reps: usize) -> (f64, f64, f64) {
    let body = || {
        for _ in 0..reps {
            let batch: Vec<DefaultNarrowPhase> = (0..8).map(|_| np.clone()).collect();
            black_box(&batch);
        }
    };
    body(); // 预热（丢弃）
    let mut best = f64::INFINITY;
    let mut worst = 0.0f64;
    for _ in 0..3 {
        let t = Instant::now();
        body();
        let us = t.elapsed().as_secs_f64() * 1e6;
        best = best.min(us);
        worst = worst.max(us);
    }
    (
        best / reps as f64,
        worst / reps as f64,
        worst / best.max(1e-9),
    )
}

#[test]
fn narrow_phase_clone_cost_meshes() {
    const REPS: usize = 50;
    println!("\n== 窄相 clone：**三角网注册量** → 每趟（8 个并存）µs ==");
    println!("   口径同 `clone_cost_probe`；每网 = 81 点 / 128 三角");
    println!("   网格数      8 个并存/趟 µs     spread");
    for n in [0usize, 64, 256, 1024] {
        let np = with_meshes(n);
        let (best, _worst, spread) = batch8_us(&np, REPS);
        println!("   {n:6}      {best:12.3}        ×{spread:.2}");
    }
}
