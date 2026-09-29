//! **测试共享件**（`two_block_order.rs` / `two_block_tables.rs` / `two_class_real_scene.rs` 共用）：
//! 2b 场景构造 + 与 `grid.wgsl` 逐字同式的 CPU 计数排序。
//!
//! **为什么是子目录模块**：`tests/*.rs` 各自是独立测试 crate（顶层 `.rs` 才是目标 ⇒
//! `support/mod.rs` 不是目标、只被 `mod support;` 引入）。god 门要求"新代码进新文件"，
//! 而这三条判据本来要各带一份同样的 helper ⇒ 抽到一处（顺带让两个既有文件变小）。
//!
//! `dead_code` 豁免：三个测试 crate 各取所需（不是每个 helper 在每个 crate 里都被用到）。
#![allow(dead_code)]
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

pub const N: usize = 16;
pub const SPACING: f32 = 0.05;

/// 2b 场景：晶格 + 5 趟静置 + 一块地板；**之后再推一个子步** ⇒ 引擎的格表覆盖全量粒子（含边界）。
pub fn scene_2b() -> FluidSystem {
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.5,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let half = N as f32 * SPACING * 0.5 + 4.0 * h;
    let bodies = vec![(
        0u32,
        Shape::Box {
            half: Vec3::new(half, 2.0 * SPACING, half),
        },
        BodyPose {
            pos: Vec3::new(0.0, -2.0 * h, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    )];
    assert!(f.set_boundary_particles(&bodies) > 0, "地板没造出边界粒子");
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    f
}

/// 对 `[lo, hi)` 这段粒子做计数排序：返回 `(每格起点, 按格分组的粒子下标)`（格内 = 索引升序）。
/// 与 `grid.wgsl` 的四步同义（分箱 → 前缀和 → 按索引升序占位 ⇒ 与 `canon` 的规范结果一致）。
pub fn count_sort_range(f: &FluidSystem, lo: usize, hi: usize) -> (Vec<u32>, Vec<u32>) {
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);
    let bin = |p: Vec3| -> usize {
        let ax = |o: f32, v: f32, n: u32| -> u32 {
            (((v - o) * gd.inv).floor().max(0.0) as u32).min(n - 1)
        };
        let idx = (ax(gd.min.x, p.x, nx) * ny + ax(gd.min.y, p.y, ny)) * nz + ax(gd.min.z, p.z, nz);
        (idx as usize).min(total - 1)
    };
    let apos = f.raw_particles().0;
    let mut counts = vec![0u32; total + 1];
    for p in &apos[lo..hi] {
        counts[bin(*p) + 1] += 1;
    }
    for c in 0..total {
        counts[c + 1] += counts[c];
    }
    let start = counts.clone();
    let mut cur = counts;
    let mut perm = vec![0u32; hi - lo];
    for (i, p) in apos[lo..hi].iter().enumerate() {
        let c = bin(*p);
        perm[cur[c] as usize] = (i + lo) as u32;
        cur[c] += 1;
    }
    (start, perm)
}

/// 引擎当前的箱参数 → `GridParams`（GPU 探针/判据用；`n_fluid`/`class_lo` 取纯流体缺省，
/// 调用方按需覆写）。
pub fn params_of(f: &FluidSystem) -> vxl_phys_gpu::grid::GridParams {
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let np = f.raw_particles().0.len();
    vxl_phys_gpu::grid::GridParams {
        gmin: [gd.min.x, gd.min.y, gd.min.z],
        inv: gd.inv,
        nx,
        ny,
        nz,
        n: np as u32,
        total: nx * ny * nz,
        cap: 512,
        n_fluid: np as u32,
        class_lo: 0,
    }
}
