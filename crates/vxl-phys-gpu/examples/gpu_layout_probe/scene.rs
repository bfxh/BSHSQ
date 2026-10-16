//! 排布判别探针的**场景与排布构造**：箱三件套 / 三种「槽位 → 空间位置」映射 / 格表重建 /
//! 局部性诊断量的采样 / 场景与三档排布的装配（prepare，含过四条自检）。
//! （主文件 `main.rs` 只留编排与打印；本文件被 `mod scene;` 收进来 ⇒ 条目对父模块可见即可。）
//!
//! ⚠️ 本文件里的算式是**与引擎同式**的复刻（判据的口径来源）：改任何一个字面量 = 改判据的输入。

use super::diag::self_checks;

use vxl_phys_core::grid::{grid_box, GRID_MAX_BINS};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem, NeighborGrid};

/// 箱子三件套（`min`/`inv`/`dims`）——与管线**同一时刻**的那一套（见 `main` 里的说明）。
#[derive(Clone, Copy)]
pub(super) struct Box3 {
    pub(super) min: Vec3,
    pub(super) inv: f32,
    pub(super) dims: (u32, u32, u32),
}

impl Box3 {
    pub(super) fn total(&self) -> usize {
        self.dims.0 as usize * self.dims.1 as usize * self.dims.2 as usize
    }
    pub(super) fn split(&self, c: u32) -> (i32, i32, i32) {
        let z = (c % self.dims.2) as i32;
        let y = ((c / self.dims.2) % self.dims.1) as i32;
        let x = (c / (self.dims.2 * self.dims.1)) as i32;
        (x, y, z)
    }
    pub(super) fn cell_of(&self, x: i32, y: i32, z: i32) -> usize {
        ((x * self.dims.1 as i32 + y) * self.dims.2 as i32 + z) as usize
    }
}

/// 三种「槽位 → 空间位置」映射。
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Layout {
    /// 槽位 k ← 原粒子 k（今天的路径：`cell_items` 是任意置换）。
    Natural,
    /// 槽位 k ← 格序第 k 个粒子（按格重排的理想档）。
    Sorted,
    /// 槽位 k ← 随机置换（无局部性下界）。
    Random,
}

impl Layout {
    pub(super) fn name(self) -> &'static str {
        match self {
            Layout::Natural => "natural",
            Layout::Sorted => "sorted",
            Layout::Random => "random",
        }
    }
    pub(super) fn all() -> [Layout; 3] {
        [Layout::Natural, Layout::Sorted, Layout::Random]
    }
}

/// 一份排布：槽位化的输入数组 + 重建的网格表 + 解释用诊断量。
/// 一张排布的格表（三张捆一起：`Arrangement` 的成员数是棘轮门看着的量）。
pub(super) struct Grid {
    pub(super) start: Vec<u32>,
    pub(super) items: Vec<u32>,
    /// 逐槽位的格下标（自检用）。
    pub(super) bins: Vec<u32>,
}

pub(super) struct Arrangement {
    pub(super) pos_flat: Vec<f32>,
    pub(super) vel_flat: Vec<f32>,
    pub(super) press: Vec<f32>,
    pub(super) grid: Grid,
    /// 采样：连续 32 槽的邻域并集 / 32（1.0 = 整 warp 共享同一片格 ⇒ 理想局部性）。
    pub(super) nbr_per_warp: f64,
    /// 采样：槽 k 与 k+1 的空间距离均值（米）。
    pub(super) step_dist_m: f64,
    /// 候选对数（三档必须相同 ⇒ "工作量相同"这条前提被守住）。
    pub(super) pairs: u64,
    /// 槽位 k ← 原粒子 `perm[k]`（跨档逐位对拍的桥）。
    pub(super) perm: Vec<u32>,
    /// 两相位输出**拼成一条**：`dens[n] ‖ acc[3n] ‖ xsph[3n]`（拼一条同上——成员数有上限）。
    pub(super) out: Vec<f32>,
}

/// 与 `vxl-phys-fluid/src/grid.rs::bin_index_of` **同式**（格下标 + 越界钳位）。
pub(super) fn bin_of(b: &Box3, p: Vec3, total: usize) -> u32 {
    let f =
        |o: f32, v: f32, n: u32| -> u32 { (((v - o) * b.inv).floor().max(0.0) as u32).min(n - 1) };
    let idx = (f(b.min.x, p.x, b.dims.0) * b.dims.1 + f(b.min.y, p.y, b.dims.1)) * b.dims.2
        + f(b.min.z, p.z, b.dims.2);
    (idx as usize).min(total - 1) as u32
}

/// 计数排序（与 `UniformGrid::rebuild` 同式）：逐槽位分箱 → 前缀和 → 占位。
fn count_sort(bins: &[u32], total: usize) -> (Vec<u32>, Vec<u32>) {
    let mut counts = vec![0u32; total + 1];
    for &c in bins {
        counts[c as usize + 1] += 1;
    }
    for c in 0..total {
        counts[c + 1] += counts[c];
    }
    let start = counts.clone();
    let mut cur = counts;
    let mut items = vec![0u32; bins.len()];
    for (k, &c) in bins.iter().enumerate() {
        let slot = cur[c as usize] as usize;
        items[slot] = k as u32;
        cur[c as usize] += 1;
    }
    (start, items)
}

/// 逐槽位的「27 格邻域里的候选数」之和（减自身）⇒ 全局候选对数。
fn candidate_pairs(b: &Box3, bins: &[u32], start: &[u32]) -> u64 {
    let (nx, ny, nz) = (b.dims.0 as i32, b.dims.1 as i32, b.dims.2 as i32);
    let mut pairs = 0u64;
    for &bi in bins {
        let (x, y, z) = b.split(bi);
        let mut s = 0u64;
        for dz in -1..=1i32 {
            if z + dz < 0 || z + dz >= nz {
                continue;
            }
            for dy in -1..=1i32 {
                if y + dy < 0 || y + dy >= ny {
                    continue;
                }
                for dx in -1..=1i32 {
                    if x + dx < 0 || x + dx >= nx {
                        continue;
                    }
                    let ci = b.cell_of(x + dx, y + dy, z + dz);
                    s += (start[ci + 1] - start[ci]) as u64;
                }
            }
        }
        pairs += s.saturating_sub(1);
    }
    pairs
}

/// 把 27 格 stencil 的格下标压进 `buf`（采样用）。
fn stencil_into(b: &Box3, c: u32, buf: &mut Vec<u32>) {
    let (nx, ny, nz) = (b.dims.0 as i32, b.dims.1 as i32, b.dims.2 as i32);
    let (x, y, z) = b.split(c);
    for dz in -1..=1i32 {
        if z + dz < 0 || z + dz >= nz {
            continue;
        }
        for dy in -1..=1i32 {
            if y + dy < 0 || y + dy >= ny {
                continue;
            }
            for dx in -1..=1i32 {
                if x + dx < 0 || x + dx >= nx {
                    continue;
                }
                buf.push(b.cell_of(x + dx, y + dy, z + dz) as u32);
            }
        }
    }
}

/// 采样两条解释性诊断：warp 内邻域并集（格数/线程）、相邻槽的空间步距。
pub(super) fn locality_diag(b: &Box3, pos: &[Vec3], bins: &[u32]) -> (f64, f64) {
    let np = bins.len();
    let mut buf: Vec<u32> = Vec::with_capacity(32 * 27);
    let groups = 2000usize.min(np / 32).max(1);
    let stride = (np / 32 / groups).max(1);
    let mut sum_cells = 0usize;
    let mut used = 0usize;
    for g in 0..groups {
        let k0 = g * stride * 32;
        if k0 + 32 > np {
            break;
        }
        buf.clear();
        for t in 0..32 {
            stencil_into(b, bins[k0 + t], &mut buf);
        }
        buf.sort_unstable();
        buf.dedup();
        sum_cells += buf.len();
        used += 1;
    }
    let nbr_per_warp = sum_cells as f64 / (used.max(1) * 32) as f64;
    // 相邻槽步距（抽样，避免大档 O(n) 的整段扫描）
    let step = (np / 200_000).max(1);
    let mut acc = 0.0f64;
    let mut m = 0usize;
    let mut k = 0usize;
    while k + 1 < np {
        acc += (pos[k + 1] - pos[k]).length() as f64;
        m += 1;
        k += step;
    }
    (nbr_per_warp, acc / m.max(1) as f64)
}

/// 建一档排布：按 `perm` 把位置/速度/压力搬到槽位，再重建网格表。
fn build_arrangement(
    b: &Box3,
    layout: Layout,
    gd: &NeighborGrid<'_>,
    f: &FluidSystem,
) -> Arrangement {
    let pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    let total = b.total();
    // 槽位 k ← 原粒子 perm[k]
    let perm: Vec<u32> = match layout {
        Layout::Natural => (0..np as u32).collect(),
        // 格序：槽位 k 放"引擎表里第 k 个"粒子 ⇒ 槽位序 = 空间序。
        Layout::Sorted => gd.items.to_vec(),
        Layout::Random => {
            // 固定种子的 xorshift 洗牌（可复现；不引外部 crate）。
            let mut s = 0x9E37_79B9_7F4A_7C15u64;
            let mut v: Vec<u32> = (0..np as u32).collect();
            for i in (1..np).rev() {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                let j = (s % (i as u64 + 1)) as usize;
                v.swap(i, j);
            }
            v
        }
    };
    let vel: Vec<Vec3> = f.velocities().to_vec();
    let press: Vec<f32> = f.pressures().to_vec();
    let mut slot_pos: Vec<Vec3> = Vec::with_capacity(np);
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel_flat: Vec<f32> = Vec::with_capacity(np * 3);
    let mut pr: Vec<f32> = Vec::with_capacity(np);
    for &p in &perm {
        let p = p as usize;
        slot_pos.push(pos[p]);
        pos_flat.extend_from_slice(&[pos[p].x, pos[p].y, pos[p].z]);
        vel_flat.extend_from_slice(&[vel[p].x, vel[p].y, vel[p].z]);
        pr.push(press[p]);
    }
    let bins: Vec<u32> = slot_pos.iter().map(|&p| bin_of(b, p, total)).collect();
    let (cell_start, cell_items) = count_sort(&bins, total);
    let pairs = candidate_pairs(b, &bins, &cell_start);
    let (nbr_per_warp, step_dist_m) = locality_diag(b, &slot_pos, &bins);
    Arrangement {
        pos_flat,
        vel_flat,
        press: pr,
        grid: Grid {
            start: cell_start,
            items: cell_items,
            bins,
        },
        nbr_per_warp,
        step_dist_m,
        pairs,
        perm,
        out: Vec::new(),
    }
}

pub(super) fn build_scene(n: usize, cpu_threads: usize) -> FluidSystem {
    let spacing = 0.05f32;
    let cfg = FluidConfig {
        threads: cpu_threads,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    // 与 `gpu_tick_probe` **同一套**：零重力 + 5 趟静置 + 剪切初速（零重力下完美晶格会全程静止）。
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    f
}

/// 建场景 + **按当前位置现算箱子** + 建三档排布 + 过四条自检（`main` 的前半段）。
///
/// 箱子必须现算、不能直接借引擎那份：引擎的格表是 `substep()` **开头**建的、`step()` 末尾又积分了
/// 一次 ⇒ 它对应**上一个子步的位置**，照它分箱会有一批粒子落在盒外（实测 1.8%），27 格 stencil
/// 就不再覆盖全邻域。顺带这也让 `natural` 档与管线的访问模式逐条对齐。
pub(super) fn prepare(
    n: usize,
    cpu_threads: usize,
) -> (FluidSystem, Box3, [u32; 3], Vec<(Layout, Arrangement)>) {
    let f = build_scene(n, cpu_threads);
    let gd = f.neighbor_grid();
    let pos: Vec<Vec3> = f.positions().to_vec();
    let np = pos.len();
    let (lo, hi) = pos
        .iter()
        .fold((pos[0], pos[0]), |(l, h), p| (l.min(*p), h.max(*p)));
    let (min, bin, dims) = grid_box(lo, hi, f.config().smoothing_radius, GRID_MAX_BINS);
    let b = Box3 {
        min,
        inv: 1.0 / bin,
        dims: (dims[0], dims[1], dims[2]),
    };
    println!(
        "== 排布判别（{np} 粒；箱 {dims:?} = {} 格 ⇒ 格边 {bin:.4} m，h={}）==",
        b.total(),
        f.config().smoothing_radius
    );
    let arrs: Vec<(Layout, Arrangement)> = Layout::all()
        .into_iter()
        .map(|l| (l, build_arrangement(&b, l, &gd, &f)))
        .collect();
    self_checks(&arrs, &gd, &pos, &b, np, b.total());
    (f, b, dims, arrs)
}
