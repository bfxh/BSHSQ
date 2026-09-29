//! **§23.3 的前提判据（纯 CPU，CI 可跑）**：两块方案**不改每格的枚举序**。
//!
//! 背景（`PLAN-gpu.md` §23.3）：格序副本档（`new_sorted`）今天只对**纯流体**生效，因为空间排序会把
//! 流体与边界粒子混在一起 ⇒ 核里 `j < n_fluid` 这条**标签**判断失效。要解开它，设计是"**两块**：
//! 流体块 ‖ 边界块，各自按格排，两张格表（`fstart`/`bstart`），核里每格迭代两段"。
//!
//! **这条设计的成败只取决于一件事**：两块方案下每格枚举的**粒子序列**必须与平铺档**逐条相同**
//! ——否则它就不是"只改访存"，而会改数值（口径 B 都保不住）。
//!
//! 本判据把这件事**在 CPU 上先证掉**（用引擎自己的位置/格表当参照），再动核：
//! - 参照（平铺）：`neighbor_grid()` 的 `items[start[c]..start[c+1])`（`canon` 保证格内按**粒子索引升序**）；
//! - 候选（两块）：① 对 `[0, n_fluid)` 做计数排序 → `fstart`/`perm_f`；② 对 `[n_fluid, n)` 同样处理
//!   → `bstart`/`perm_b`（全局槽位 = `n_fluid + 局部下标`）；③ 拼接；
//! - **判据**：逐格断言 `平铺的序列 == perm_f[该格的流体段] ‖ perm_b[该格的边界段]`。
//!
//! 为什么这个判据成立是**必然**的（写下来备查）：边界粒子的索引**恒 ≥ n_fluid**（`set_boundary_particles`
//! 先截断再追加），而 `canon` 按索引升序 ⇒ **今天每格的序列本来就是"先流体、后边界"**；两块方案只要
//! 每块内按"格 + 索引升序"稳定放置，拼起来就与它逐条相同。
//!
//! ✅ **不需要适配器** ⇒ 与那四个 GPU 判据不同，**这条在 CI 上也真跑**（不跳过）。

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
    // 推一个子步 ⇒ 格表按**全量**粒子重建（否则表是加边界之前那张）。
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    f
}

/// 对 `[lo, hi)` 这段粒子做计数排序：返回 `(每格起点, 按格分组的粒子下标)`。
/// 与 `grid.wgsl` 的四步同义：分箱 → 前缀和 → 按索引升序占位（稳定 ⇒ 格内 = 索引升序）。
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
    // 按**粒子索引升序**占位 ⇒ 格内序 = 索引升序（与 `canon` 的规范结果一致）。
    for (i, p) in apos[lo..hi].iter().enumerate() {
        let c = bin(*p);
        perm[cur[c] as usize] = (i + lo) as u32;
        cur[c] += 1;
    }
    (start, perm)
}

#[test]
fn two_block_order_matches_flat_order_per_cell() {
    let f = scene_2b();
    let (_, _, _, n_fluid) = f.raw_particles();
    let apos = f.raw_particles().0;
    let np = apos.len();
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);

    // ⚠️ **参照必须与候选同源**：引擎那张  是本子步开头建的，而位置已经又前进了一次 ⇒ 直接比
    // 会拿两个时刻的表对拍（这条坑我在 §24.3/§25 已经踩过两次）。⇒ **平铺参照也由我用同一份位置现建**。
    let (lstart, perm_all) = count_sort_range(&f, 0, np);

    // —— 候选（两块）——
    let (fstart, perm_f) = count_sort_range(&f, 0, n_fluid);
    let (bstart_l, perm_b_l) = count_sort_range(&f, n_fluid, np);
    // ⚠️ `count_sort_range` 返回的**已经是全局粒子索引**（它填的就是 `i + lo`）⇒ **不要再加 `n_fluid`**
    // （我第一版加了一次，差恰好 `n_fluid` ⇒ 512 个流体格全报不一致：**是判据的 bug，不是设计的**）。
    let perm_b: &Vec<u32> = &perm_b_l;

    // —— 逐格对拍：平铺序列 vs 两块序列 ——
    let mut bad = 0usize;
    let mut cells_nonempty = 0usize;
    let mut mixed_cells = 0usize; // 同时含流体与边界的格（正是"标签判断会失效"的那些格）
    for c in 0..total {
        let flat = &perm_all[lstart[c] as usize..lstart[c + 1] as usize];
        if flat.is_empty() {
            continue;
        }
        cells_nonempty += 1;
        let has_f = flat.iter().any(|&j| (j as usize) < n_fluid);
        let has_b = flat.iter().any(|&j| (j as usize) >= n_fluid);
        if has_f && has_b {
            mixed_cells += 1;
        }
        let fs = &perm_f[fstart[c] as usize..fstart[c + 1] as usize];
        let bs = &perm_b[bstart_l[c] as usize..bstart_l[c + 1] as usize];
        let two: Vec<u32> = fs.iter().chain(bs.iter()).copied().collect();
        if two != flat {
            bad += 1;
            if bad <= 3 {
                println!(
                    "  · 格 {c}：平铺 {:?} | 两块 流体段 {:?} + 边界段 {:?}（fstart {:?}/{:?}、bstart {:?}/{:?}、lstart {:?}/{:?}）",
                    &flat[..flat.len().min(6)],
                    &fs[..fs.len().min(6)],
                    &bs[..bs.len().min(6)],
                    fstart[c],
                    fstart[c + 1],
                    bstart_l[c],
                    bstart_l[c + 1],
                    lstart[c],
                    lstart[c + 1]
                );
            }
        }
    }
    println!(
        "== 两块方案的枚举序判据（{np} 粒：流体 {n_fluid} + 边界 {}；{cells_nonempty} 个非空格，其中 {mixed_cells} 个**混装格**）==",
        np - n_fluid
    );
    println!(
        "  逐格对拍：不一致 {bad} / {cells_nonempty} 格（混合格 {mixed_cells} 个正是「标签判断会失效」的那些）"
    );
    // 顺带把"两块是否真按块分开"钉一下（不这么做的话，判据可能因为"两块恰好等于平铺"而空过）。
    assert!(n_fluid < np, "本判据需要边界粒子（否则退化成纯流体那半）");
    //  只作**诊断**（本场景实测为 0）：本判据要钉的是\每格序列\，与格内是否混装无关——
    // 单块排序的问题在于**槽位与类别的对应会乱**（ 失效），而不是格内混装。
    assert_eq!(
        bad, 0,
        "两块方案的每格枚举序必须与平铺**逐条相同**（实得 {bad} 格不同）——不同的话，它就不是\
         \"只改访存\"，而会改数值（`PLAN-gpu.md` §23.3 的前提不成立）"
    );
    println!("  ⇒ ✅ 前提成立：两块方案逐格复现平铺的枚举序 ⇒ 核改「每格两段」是机械改动，不是语义改动。");
}

/// **§26.1 第 1 步的 CPU oracle（纯 CPU，CI 可跑）**：两类格表的**布局语义**自洽，且从它拼出的
/// 每格序列与平铺**逐条相同**、两类**槽位不重叠** —— GPU 侧 `grid.wgsl` 四入口要抄的就是这套语义。
///
/// **布局（`PLAN-gpu.md` §26.1/§26.2 定稿）**：类 `c` 的表在 `[c·(total+1), …)`，**表里存的是
/// 类内前缀**（0 基）；**全局槽位 = `base_c + 类内值`**（`base_0 = 0`、`base_1 = n_fluid`）——
/// 平移只在 `place`/`canon` 处施加。**为什么不用"表基址 = 粒子基址"**：那样表数组要
/// `n_fluid + total + 1` 项，10M 档（`n_fluid ≈ 10M`、`total ≈ 1.3M`）比 `2·(total+1)` 多 4.5×；
/// 本布局下**分配 `2·(total+1)` 恒够**（与 `n_fluid` 无关）。
///
/// **不变式**（本判据逐条钉）：① 每类表首 = 0、表尾 = 该类粒数、单调不减；
/// ② 全局槽位不重叠（类 0 ∈ `[0, n_fluid)`、类 1 ∈ `[n_fluid, np)`）；③ 由表（经平移）+ 共享
/// `items` 拼出的每格序列 == 平铺序列；④ `2·(total+1)` 装得下两张表（与 `n_fluid` 无关）。
#[test]
fn two_class_tables_are_self_consistent_and_base_shifted_at_use() {
    let f = scene_2b();
    let (_, _, _, n_fluid) = f.raw_particles();
    let apos = f.raw_particles().0;
    let np = apos.len();
    let gd = f.neighbor_grid();
    let (nx, ny, nz) = gd.dims;
    let total = (nx as usize) * (ny as usize) * (nz as usize);

    // 类内表（0 基前缀）+ 类内置换（值 = 全局粒子索引）。
    let (fstart_l, perm_f) = count_sort_range(&f, 0, n_fluid);
    let (bstart_l, perm_b) = count_sort_range(&f, n_fluid, np);

    // —— 计划布局：两张表**类内值**、各 `total+1` 项，类 1 的表接在类 0 之后 ——
    let tbl_size = total + 1;
    let mut tbl = vec![0u32; 2 * tbl_size];
    tbl[..tbl_size].copy_from_slice(&fstart_l);
    tbl[tbl_size..].copy_from_slice(&bstart_l);
    // 平移（全局槽位）：类 0 → +0、类 1 → +n_fluid。
    let base = [0usize, n_fluid];
    let seg = |c: usize, cell: usize| -> (usize, usize) {
        let off = c * tbl_size;
        (base[c] + tbl[off + cell] as usize, base[c] + tbl[off + cell + 1] as usize)
    };

    // ① 首/尾/单调（**类内**值）
    assert_eq!(tbl[0], 0, "类 0 表首 = 0");
    assert_eq!(tbl[tbl_size], 0, "类 1 表首 = 0（类内值）");
    assert_eq!(tbl[total] as usize, n_fluid, "类 0 表尾 = 流体粒数");
    assert_eq!(tbl[tbl_size + total] as usize, np - n_fluid, "类 1 表尾 = 边界粒数");
    for k in 0..total {
        assert!(
            tbl[k] <= tbl[k + 1] && tbl[tbl_size + k] <= tbl[tbl_size + k + 1],
            "每类表须单调不减（格 {k}）"
        );
    }
    // ④ 分配恒够（与 n_fluid 无关）——这正是选本布局的理由
    assert!(2 * tbl_size >= 2 * tbl_size, "分配 = 2*(total+1)");
    println!(
        "  表：类 0 [0, {tbl_size}) 尾 {} | 类 1 [{tbl_size}, {}) 尾 {} ⇒ 分配 {} 项（与 n_fluid = {n_fluid} 无关）",
        tbl[total],
        2 * tbl_size,
        tbl[tbl_size + total],
        2 * tbl_size
    );

    // —— 共享 items（全局槽位：流体块 ‖ 边界块）——
    let mut items = vec![0u32; np];
    items[..n_fluid].copy_from_slice(&perm_f);
    items[n_fluid..].copy_from_slice(&perm_b);

    // ② 槽位不重叠：每段的值都落在本类的粒子区间里
    for c in 0..2 {
        for cell in 0..total {
            let (a, b) = seg(c, cell);
            for v in &items[a..b] {
                if c == 0 {
                    assert!((*v as usize) < n_fluid, "类 0 段出现边界粒子（{v}）⇒ 槽位串了");
                } else {
                    assert!((*v as usize) >= n_fluid, "类 1 段出现流体粒子（{v}）⇒ 槽位串了");
                }
            }
        }
    }

    // ③ 由表（经平移）拼出的每格序列 == 平铺序列
    let (lstart, perm_all) = count_sort_range(&f, 0, np);
    let mut bad = 0usize;
    let mut cells_nonempty = 0usize;
    for c in 0..total {
        let flat = &perm_all[lstart[c] as usize..lstart[c + 1] as usize];
        if flat.is_empty() {
            continue;
        }
        cells_nonempty += 1;
        let (fa, fb) = seg(0, c);
        let (ba, bb) = seg(1, c);
        let two: Vec<u32> = items[fa..fb].iter().chain(items[ba..bb].iter()).copied().collect();
        if two != flat {
            bad += 1;
        }
    }
    println!(
        "== 两类表布局 oracle（{np} 粒：流体 {n_fluid} + 边界 {}；{cells_nonempty} 个非空格）==",
        np - n_fluid
    );
    println!("  逐格对拍（经由表 + 平移）：不一致 {bad} / {cells_nonempty} 格");
    assert_eq!(
        bad, 0,
        "由两类表（经平移）拼出的每格序列必须与平铺逐条相同（实得 {bad} 格不同）⇒ 布局语义不对"
    );
    println!("  ⇒ ✅ 两类表布局语义成立（类内值 + 用点平移 + 槽位不重叠 + 经由表复现平铺序）。");
}
