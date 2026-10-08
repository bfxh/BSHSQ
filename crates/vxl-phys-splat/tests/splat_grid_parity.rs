//! **候选网格 vs 全扫：逐位一致 + 加速读数**（2026-10-08）。
//!
//! 压力/黏性的对循环走 `candidate_ids`（有网格 = 只取所在格的登记表；无网格 = 全扫）。
//! 网格**只裁候选、不改求和序** ⇒ 两条路必须逐位一致；不建网格时每核对全扫 = O(n²)。
//!
//! 判据分两层（与仓库既有先例同款）：
//! ① **逐位一致**（机器无关，进 CI）；
//! ② 微基准**只报数不判**（时间类断言在 CI 上不稳）。
//!
//! ⚠️ **否证留档（2026-10-08）**：曾试过"压力/黏性对循环前强制 `rebuild_grid()`"，release 档扫
//! 核数后**不成立** —— n = 200/400/800/1600 的加速比 = **0.63 / 1.31 / 1.00 / 0.93×**：
//! 本仓的 bin 边长 = `3·max_scale·√cut`（为**铺开**的核云调的），随机云场景里常退化成"一格装下
//! 全部核" ⇒ 只留下建表开销。⇒ 该改动**已回退**；要真做加速得先改 **bin 口径**（按局部密度/
//! 核半径分布自适应），并重跑本文件的扫描。判据（逐位一致）保留 —— 对循环若吃到**已有**网格，
//! 必须与全扫同结果。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::pressure::{apply_self_pressure, SelfPressure};
use vxl_phys_splat::viscosity::{apply_self_viscosity, SelfViscosity};
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 三核/多核场：`grid_min_splats` 决定"建不建网格"（0 = 总建；`usize::MAX` = 永不建 = 全扫）。
fn field(n: usize, grid_min_splats: usize) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.medium_density = 1000.0;
    f.grid_min_splats = grid_min_splats;
    let mut st = 0x1234_5678u32;
    let mut next = move || {
        st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (st >> 8) as f32 / (1u32 << 24) as f32
    };
    for _ in 0..n {
        f.push(Splat {
            center: Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0),
            scale: Vec3::splat(0.3),
            rot: Mat3::IDENTITY,
            opacity: 0.5 + next(),
            color: [1.0, 1.0, 1.0],
        });
    }
    f
}

/// 跑 `steps` 步「压力 + 黏性」，返回逐核速度（位模式比较用）。
fn run(n: usize, grid_min_splats: usize, steps: usize) -> Vec<u32> {
    let mut f = field(n, grid_min_splats);
    let v0: Vec<Vec3> = (0..n)
        .map(|k| {
            let t = k as f32 * 0.37;
            Vec3::new(t.sin(), t.cos(), -t.sin())
        })
        .collect();
    assert!(f.set_kernel_velocities(&v0));
    for _ in 0..steps {
        apply_self_pressure(&mut f, 1.0 / 120.0, SelfPressure::default());
        apply_self_viscosity(&mut f, 1.0 / 120.0, SelfViscosity::default());
        // 模拟每子步位置变化：网格置脏 ⇒ 下一次调用必须自己重建（否则退回全扫）
        f.translate(Vec3::new(0.001, 0.0, 0.0));
    }
    f.kernel_velocities()
        .iter()
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

#[test]
fn grid_and_brute_force_agree_bitwise() {
    let grid = run(120, 0, 8);
    let brute = run(120, usize::MAX, 8);
    assert_eq!(grid.len(), brute.len());
    assert_eq!(grid, brute, "候选网格与全扫必须逐位一致");
    assert!(grid.iter().any(|b| *b != 0), "用例非平凡：速度确实变了");
}

#[test]
fn grid_speedup_microbench_report_only() {
    // 只报数不判（时间类断言在 CI 上不稳）；**扫核数**是为了找到"建网格 vs 全扫"的拐点。
    let steps = 10usize;
    for n in [200usize, 400, 800, 1600] {
        let t0 = std::time::Instant::now();
        let _ = run(n, 0, steps);
        let grid = t0.elapsed().as_secs_f64();
        let t1 = std::time::Instant::now();
        let _ = run(n, usize::MAX, steps);
        let brute = t1.elapsed().as_secs_f64();
        println!(
            "n={n:5} × {steps} 步（压力+黏性）：网格 {:8.1} ms | 全扫 {:8.1} ms | 加速 {:.2}×",
            grid * 1e3,
            brute * 1e3,
            brute / grid
        );
    }
}
