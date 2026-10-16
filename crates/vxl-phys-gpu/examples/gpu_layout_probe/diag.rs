//! 排布判别探针的**判据与诊断**：`bin_of` 对引擎旧表的吻合度 / 越界计数 / 表自洽 /
//! 四条前提断言（self_checks）/ 局部性衰减曲线 / 两档 repeats 解 `c` 与 `R`（solve）/
//! 逐位一致的接线判据 / 读数表。
//! （主文件 `main.rs` 只留编排与打印；本文件被 `mod diag;` 收进来 ⇒ 条目对父模块可见即可。）

use super::scene::{bin_of, build_scene, locality_diag, Arrangement, Box3, Layout};

use vxl_phys_core::grid::{grid_box, GRID_MAX_BINS};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::NeighborGrid;

/// **与时机无关**的 `bin_of` 判据：引擎那张表对应的是**上一个子步的位置**（`substep()` 开头建、
/// `step()` 末尾又积分了一次；`neighbor_grid()` 只导出缓存、不重建）⇒ 不能比"同格"，只能比
/// "引擎放在哪一格"与"我按当前位置算哪一格"**差几格**。返回 `(同格%, 差 ≤1 格%)`。
fn bin_agreement(stale: &[u32], gd: &NeighborGrid<'_>, np: usize) -> (f64, f64) {
    let (nx, ny, nz) = (gd.dims.0 as i32, gd.dims.1 as i32, gd.dims.2 as i32);
    let total = (nx * ny * nz) as usize;
    // 引擎表 → 每粒子的格下标
    let mut eng_cell = vec![u32::MAX; np];
    for c in 0..total {
        for k in gd.start[c]..gd.start[c + 1] {
            let p = gd.items[k as usize] as usize;
            if p < np {
                eng_cell[p] = c as u32;
            }
        }
    }
    let split = |c: u32| -> (i32, i32, i32) {
        let z = (c % gd.dims.2) as i32;
        let y = ((c / gd.dims.2) % gd.dims.1) as i32;
        let x = (c / (gd.dims.2 * gd.dims.1)) as i32;
        (x, y, z)
    };
    let mut exact = 0u64;
    let mut within1 = 0u64;
    let mut n = 0u64;
    for k in 0..np {
        let e = eng_cell[k];
        if e == u32::MAX {
            continue;
        }
        n += 1;
        let (ex, ey, ez) = split(e);
        let (mx, my, mz) = split(stale[k]);
        if (ex - mx).abs() <= 1 && (ey - my).abs() <= 1 && (ez - mz).abs() <= 1 {
            within1 += 1;
            if e == stale[k] {
                exact += 1;
            }
        }
    }
    let d = n.max(1) as f64;
    (100.0 * exact as f64 / d, 100.0 * within1 as f64 / d)
}

/// 越界槽位数：与 `bin_of` 的钳位同式但**去掉** `max(0)`/`min(n-1)`。
/// 非 0 ⇒ 有粒子在格盒外 ⇒ 27 格 stencil 不再覆盖全邻域 ⇒ 实验前提作废。
fn oob_count(b: &Box3, pos: &[Vec3]) -> u64 {
    let f = |o: f32, v: f32| -> f32 { ((v - o) * b.inv).floor() };
    pos.iter()
        .filter(|p| {
            f(b.min.x, p.x) < 0.0
                || f(b.min.x, p.x) >= b.dims.0 as f32
                || f(b.min.y, p.y) < 0.0
                || f(b.min.y, p.y) >= b.dims.1 as f32
                || f(b.min.z, p.z) < 0.0
                || f(b.min.z, p.z) >= b.dims.2 as f32
        })
        .count() as u64
}

/// 自检 ④：喂给核的那张表必须自洽——每格 `[start[c], start[c+1])` 里的槽位，其 `bins` 必须就是 `c`
/// （`count_sort` 的正向验证；表不自洽的话三档比的就不是"同一件事"）。
/// 顺带返回 `sorted` 档的 `items[m] == m` 命中率（那正是"理想档"的定义：槽位序 = 空间序）。
fn table_consistent(bins: &[u32], start: &[u32], items: &[u32], total: usize) -> (u64, u64) {
    let mut bad = 0u64;
    let mut identity = 0u64;
    for c in 0..total {
        for k in start[c]..start[c + 1] {
            let m = k as usize;
            if bins[items[m] as usize] != c as u32 {
                bad += 1;
            }
            if items[m] as usize == m {
                identity += 1;
            }
        }
    }
    (bad, identity)
}

/// **长跑下的局部性衰减**（回答"10M 档那个 226 ms/tick 能不能代表跑久了的场景"）：
/// 同一条场景往前推，在给定的 tick 检查点上重算两条诊断（`warp 邻域并集` 与 `相邻槽步距`）。
/// 索引↔空间的关联一旦衰减，warp 邻域并集就会从 ~5 往 27 爬 ⇒ 代价往 `random` 档那侧跑。
pub(super) fn diag_curve(n: usize, cpu_threads: usize, checkpoints: &[usize]) {
    let mut f = build_scene(n, cpu_threads);
    let mut pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    println!("== 局部性随时间的衰减（{np} 粒；诊断口径：32 槽/组 × 2000 组采样）==");
    println!("  tick | warp 邻域并集（格/线程）| 相邻槽步距 m | 格边 m");
    let mut done = 5usize; // `build_scene` 已经推了 5 趟
    for &s in checkpoints {
        for _ in done..s {
            f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
        }
        done = done.max(s);
        pos = f.positions().to_vec();
        let (lo, hi) = pos
            .iter()
            .fold((pos[0], pos[0]), |(l, h), p| (l.min(*p), h.max(*p)));
        let (min, bin, dims) = grid_box(lo, hi, f.config().smoothing_radius, GRID_MAX_BINS);
        let b = Box3 {
            min,
            inv: 1.0 / bin,
            dims: (dims[0], dims[1], dims[2]),
        };
        let total = b.total();
        let bins: Vec<u32> = pos.iter().map(|&p| bin_of(&b, p, total)).collect();
        let (nbr, step_m) = locality_diag(&b, &pos, &bins);
        println!("  {s:>4} | {nbr:>24.2} | {step_m:>12.4} | {bin:.4}");
    }
    let _ = np;
}

/// 四条**前提判据**（任一不过就不许读数）：① 三档格表逐项同 `start`；② 候选对数三档相同；
/// ③ `bin_of` 与引擎同式（与时机无关的判据，见 `bin_agreement`）+ 越界 0；④ 表自洽 + `sorted`
/// 档必须是恒等置换（"槽位序 = 空间序"的定义）。
pub(super) fn self_checks(
    arrs: &[(Layout, Arrangement)],
    gd: &NeighborGrid<'_>,
    pos: &[Vec3],
    b: &Box3,
    np: usize,
    total: usize,
) {
    let base_start = &arrs[0].1.grid.start;
    for (l, a) in arrs {
        assert_eq!(&a.grid.start, base_start, "{:?} 的 start 不同", l.name());
    }
    let p0 = arrs[0].1.pairs;
    for (l, a) in arrs {
        assert_eq!(a.pairs, p0, "{:?} 的候选对数不同", l.name());
    }
    let stale: Vec<u32> = pos.iter().map(|&p| bin_of(b, p, total)).collect();
    let (exact, within1) = bin_agreement(&stale, gd, np);
    let oob = oob_count(b, pos);
    println!(
        "  自检：三档 `start` 逐项同（{} 项）、候选对数同为 {p0}（{:.1}/粒）",
        base_start.len(),
        p0 as f64 / np as f64
    );
    println!(
        "        `bin_of` 对引擎（旧一子步的）表：同格 {exact:.2}%、差 ≤1 格 {within1:.4}%；越界槽位 {oob}"
    );
    assert_eq!(oob, 0, "有粒子落在格盒外 ⇒ 27 格 stencil 不再覆盖全邻域");
    assert!(within1 > 99.9, "bin_of 与引擎公式不一致（差 >1 格）");
    for (l, a) in arrs {
        let (bad, ident) = table_consistent(&a.grid.bins, &a.grid.start, &a.grid.items, total);
        assert_eq!(bad, 0, "{:?} 的表不自洽：{bad} 项", l.name());
        if l.name() == Layout::Sorted.name() {
            assert_eq!(ident as usize, np, "sorted 档不是恒等置换（{ident}/{np}）");
        }
        println!(
            "        表的自洽性 {:<7}：错位 {bad} 项；`items[m] == m` 命中 {ident}/{np}",
            l.name()
        );
    }
}

/// 读数表 + `†` 脚注 + 标定提示。
pub(super) fn report_table(arrs: &[(Layout, Arrangement)], best: &[Meas]) {
    println!("\n== 读数（密度+力两相位合计，每子步口径；`c` 已扣掉与排布无关的固定项 `R`）==");
    println!(
        "  档      | 每轮 c ms | 相对 natural | p1/p2 ms   | R ms  | warp 邻域并集 | 相邻槽步距 m"
    );
    let cn = best[0].c;
    for (i, (l, a)) in arrs.iter().enumerate() {
        let m = best[i];
        println!(
            "  {:<7} | {:>8.2}{} | {:>11.2}× | {:>6.1}/{:<6.1} | {:>5.1} | {:>13.2} | {:.4}",
            l.name(),
            m.c,
            if m.ok { " " } else { "†" },
            m.c / cn,
            m.p1,
            m.p2,
            m.r,
            a.nbr_per_warp,
            a.step_dist_m
        );
    }
    if best.iter().any(|m| !m.ok) {
        println!(
            "  † 该档的 `R` 解不出来（两档读数反过来 ⇒ 长跑降频）⇒ 表里给的是**两档原始值的较大者**，\n    即 c 的**上界**（保守侧）。"
        );
    }
}

/// 一档排布的一次测量结果。
#[derive(Clone, Copy)]
pub(super) struct Meas {
    /// 每轮稳态（两相位合计，已扣固定项 `R`）；`ok == false` 时是**上界**。
    pub(super) c: f64,
    /// 固定项（同步回读 + 首轮预热）；`ok == false` 时不可信。
    pub(super) r: f64,
    pub(super) p1: f64,
    pub(super) p2: f64,
    /// `R` 是否解得出来（见 `solve` 的护栏）。
    pub(super) ok: bool,
}

/// 两档 repeats 解 `t = c + R/r`：`c` = 每轮稳态（两相位合计），`R` = 固定项（同步回读 + 首轮预热）。
///
/// **护栏**：`R` 与排布无关，必须**为正**，且它在 `r2` 档读数里的占比要小（否则"解出 c"等于在噪声里
/// 外推）。`random` 档在 10M 会慢到 3.4 s/子步 ⇒ 一发跑满上百秒 ⇒ **降频/时钟漂移**，实测两档反过来
/// （p1 < p2 ⇒ 解出负 R）。此时**不报解**，改报"两档原始值的较大者"当上界（`ok = false`，表里带 †）。
pub(super) fn solve(r1: usize, p1: f64, r2: usize, p2: f64) -> (f64, f64, bool) {
    let (a, b) = (r1 as f64, r2 as f64);
    let c = (p1 * a - p2 * b) / (a - b);
    let r = p1 * a - c * a;
    let ok = r > 0.0 && r < 0.5 * p2 * b;
    (if ok { c } else { p1.max(p2) }, r, ok)
}

/// f32 的单调位序键（ulp 比较用；负数区要翻转）。
fn ukey(x: f32) -> i32 {
    let b = x.to_bits() as i32;
    if b < 0 {
        i32::MIN.wrapping_sub(b)
    } else {
        b
    }
}

/// **接线判据（逐位一致）**：`<档>[k]` 必须与 `natural[perm[k]]` **逐位相同**。
///
/// 为什么这条是接线的判据：生产上要把两核改成"读格序副本"（`x_c[m] = x[perm[m]]`）——那时每个粒子的
/// **邻域枚举序列**必须一字不变，否则就不是"只改访存"，而会动默认档的冻结值与哈希。所以接线前先在这里
/// 把"逐位"立住。
///
/// ⚠️ **判据必须能否证**：`random` 档的槽位序与粒子索引序无关 ⇒ 它格内枚举的是**另一批粒子的序列**
/// ⇒ 它**必须**报出不符。若三档都报 0 不符，说明这条判据没有分辨力（是坏的），不许当绿灯用。
pub(super) fn equivalence_check(arrs: &[(Layout, Arrangement)]) {
    let nat = &arrs[0].1;
    println!("\n== 接线判据：逐位一致（`<档>[k]` vs `natural[perm[k]]`）==");
    for (l, a) in arrs.iter().skip(1) {
        let n = a.out.len() / 7; // 拼条口径：dens[n] ‖ acc[3n] ‖ xsph[3n]
        let mut parts: Vec<String> = Vec::new();
        for (name, base, per) in [("dens", 0usize, 1usize), ("acc", n, 3), ("xsph", 4 * n, 3)] {
            let len = n * per;
            let mut bad = 0usize;
            let mut maxu = 0u32;
            for t in 0..len {
                // `per` = 每粒几个 f32：槽位下标 = t / per，分量 = t % per
                let j = a.perm[t / per] as usize * per + (t % per);
                let (x, y) = (a.out[base + t], nat.out[base + j]);
                if x.to_bits() != y.to_bits() {
                    bad += 1;
                }
                maxu = maxu.max(ukey(x).abs_diff(ukey(y)));
            }
            parts.push(format!("{name} 不符 {bad}/{len} maxulp {maxu}"));
        }
        println!("  {:<7}{}", l.name(), parts.join(" | "));
    }
}
