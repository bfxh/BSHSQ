//! **§26.1 第 1 步的 CPU oracle（纯 CPU，CI 可跑）**：两类格表的**布局语义**自洽，且从它拼出的
//! 每格序列与平铺**逐条相同**、两类**槽位不重叠** —— GPU 侧 `grid.wgsl` 四入口要抄的就是这套语义。
//!
//! **布局（`PLAN-gpu.md` §26.1/§26.2 定稿）**：类 `c` 的表在 `[c·(total+1), …)`，**表里存的是
//! 类内前缀**（0 基）；**全局槽位 = `base_c + 类内值`**（`base_0 = 0`、`base_1 = n_fluid`）——
//! 平移只在 `place`/`canon` 处施加。**为什么不用"表基址 = 粒子基址"**：那样表数组要
//! `n_fluid + total + 1` 项，10M 档（`n_fluid ≈ 10M`、`total ≈ 1.3M`）比 `2·(total+1)` 多 4.5×；
//! 本布局下**分配 `2·(total+1)` 恒够**（与 `n_fluid` 无关）。
//!
//! 本文件与 `two_block_order.rs` **同源**（`scene_2b` / `count_sort_range` 各带一份副本）：
//! god 门要求"新代码进新文件"（既有文件受行数/最长函数棘轮），故不共享模块。
//!
//! ✅ 不需要适配器 ⇒ **CI 上也真跑**。
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

const N: usize = 16;
const SPACING: f32 = 0.05;

/// 2b 场景：晶格 + 5 趟静置 + 一块地板；**之后再推一个子步** ⇒ 引擎的格表覆盖全量粒子（含边界）。
fn scene_2b() -> FluidSystem {
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
fn count_sort_range(f: &FluidSystem, lo: usize, hi: usize) -> (Vec<u32>, Vec<u32>) {
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

/// 两类方案的装置读数：两张类内表 + 两个类内置换 + 全局 `items`。
struct TwoBlock {
    tbl_size: usize,
    total: usize,
    np: usize,
    n_fluid: usize,
    /// 类 0 / 类 1 的**类内**表（长度各 `tbl_size`）。
    fstart: Vec<u32>,
    bstart: Vec<u32>,
    /// 全局槽位置换（流体块 ‖ 边界块）。
    items: Vec<u32>,
}

impl TwoBlock {
    /// **全局段**：类 `c` 的第 `g` 格在 `items` 里的区间（= `base_c + 类内值`）。
    fn seg(&self, c: usize, g: usize) -> (usize, usize) {
        let tbl = if c == 0 { &self.fstart } else { &self.bstart };
        let base = if c == 0 { 0 } else { self.n_fluid };
        (base + tbl[g] as usize, base + tbl[g + 1] as usize)
    }
}

fn two_block(f: &FluidSystem) -> TwoBlock {
    let (_, _, _, n_fluid) = f.raw_particles();
    let np = f.raw_particles().0.len();
    let (nx, ny, nz) = f.neighbor_grid().dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);
    let (fstart, perm_f) = count_sort_range(f, 0, n_fluid);
    let (bstart, perm_b) = count_sort_range(f, n_fluid, np);
    let mut items = vec![0u32; np];
    items[..n_fluid].copy_from_slice(&perm_f);
    items[n_fluid..].copy_from_slice(&perm_b);
    TwoBlock {
        tbl_size: total + 1,
        total,
        np,
        n_fluid,
        fstart,
        bstart,
        items,
    }
}

/// **判据 A（表布局）**：每类表首 = 0、表尾 = 该类粒数、单调不减；**槽位不重叠**（类 0 段的值
/// 全 < `n_fluid`、类 1 段全 ≥ `n_fluid`）；分配 `2·(total+1)` 恒够。
#[test]
fn two_class_tables_are_self_consistent_and_base_shifted_at_use() {
    let f = scene_2b();
    let b = two_block(&f);
    let (tb, np, nf) = (b.tbl_size, b.np, b.n_fluid);

    assert_eq!(b.fstart[0], 0, "类 0 表首 = 0");
    assert_eq!(b.bstart[0], 0, "类 1 表首 = 0（**类内**值）");
    assert_eq!(b.fstart[b.total] as usize, nf, "类 0 表尾 = 流体粒数");
    assert_eq!(b.bstart[b.total] as usize, np - nf, "类 1 表尾 = 边界粒数");
    for k in 0..b.total {
        assert!(
            b.fstart[k] <= b.fstart[k + 1] && b.bstart[k] <= b.bstart[k + 1],
            "每类表须单调不减（格 {k}）"
        );
    }
    println!(
        "  表：类 0 [0, {tb}) 尾 {} | 类 1 [{tb}, {}) 尾 {} ⇒ 分配 {} 项（与 n_fluid = {nf} 无关）",
        b.fstart[b.total],
        2 * tb,
        b.bstart[b.total],
        2 * tb
    );

    for c in 0..2 {
        for g in 0..b.total {
            let (a, z) = b.seg(c, g);
            for v in &b.items[a..z] {
                if c == 0 {
                    assert!((*v as usize) < nf, "类 0 段出现边界粒子（{v}）⇒ 槽位串了");
                } else {
                    assert!((*v as usize) >= nf, "类 1 段出现流体粒子（{v}）⇒ 槽位串了");
                }
            }
        }
    }
    println!("  ⇒ ✅ 判据 A：表自洽 + 槽位不重叠（类 0 ∈ [0,{nf})、类 1 ∈ [{nf},{np})）。");
}

/// **判据 B（枚举序）**：由表（经平移）+ 共享 `items` 拼出的每格序列 == 平铺序列。
#[test]
fn two_class_tables_reproduce_the_flat_enumeration_order() {
    let f = scene_2b();
    let b = two_block(&f);
    let (lstart, perm_all) = count_sort_range(&f, 0, b.np);
    let mut bad = 0usize;
    let mut bins = 0usize;
    for c in 0..b.total {
        let flat = &perm_all[lstart[c] as usize..lstart[c + 1] as usize];
        if flat.is_empty() {
            continue;
        }
        bins += 1;
        let (fa, fb) = b.seg(0, c);
        let (ba, bb) = b.seg(1, c);
        let two: Vec<u32> = b.items[fa..fb]
            .iter()
            .chain(b.items[ba..bb].iter())
            .copied()
            .collect();
        if two != flat {
            bad += 1;
        }
    }
    println!(
        "== 两类表 oracle（{} 粒：流体 {} + 边界 {}；{bins} 个非空格）==",
        b.np,
        b.n_fluid,
        b.np - b.n_fluid
    );
    println!("  逐格对拍（经由表 + 平移）：不一致 {bad} / {bins} 格");
    assert_eq!(
        bad, 0,
        "由两类表拼出的每格序列必须与平铺逐条相同（实得 {bad} 格不同）"
    );
    println!("  ⇒ ✅ 判据 B：两类表布局语义成立（经由表复现平铺序）。");
}
