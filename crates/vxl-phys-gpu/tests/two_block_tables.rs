//! **§26.1 第 1 步的 CPU oracle（纯 CPU，CI 可跑）**：两类格表的**布局语义**自洽，且从它拼出的
//! 每格序列与平铺**逐条相同**、两类**槽位不重叠** —— GPU 侧 `grid.wgsl` 四入口要抄的就是这套语义。
//!
//! **布局（`PLAN-gpu.md` §26.1/§26.2 定稿）**：类 `c` 的表在 `[c·(total+1), …)`，**表里存的是
//! 类内前缀**（0 基）；**全局槽位 = `base_c + 类内值`**（`base_0 = 0`、`base_1 = n_fluid`）——
//! 平移只在 `place`/`canon` 处施加。**为什么不用"表基址 = 粒子基址"**：那样表数组要
//! `n_fluid + total + 1` 项，10M 档（`n_fluid ≈ 10M`、`total ≈ 1.3M`）比 `2·(total+1)` 多 4.5×；
//! 本布局下**分配 `2·(total+1)` 恒够**（与 `n_fluid` 无关）。
//!
//! 本文件与 `two_block_order.rs` / `two_class_real_scene.rs` **共用 `support` 模块**的
//! `scene_2b` / `count_sort_range`（三条判据同源同口径）。
//!
//! ✅ 不需要适配器 ⇒ **CI 上也真跑**。
mod support;

use support::{count_sort_range, scene_2b};
use vxl_phys_fluid::FluidSystem;

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
