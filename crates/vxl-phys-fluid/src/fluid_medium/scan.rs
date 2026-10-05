//! **介质采样的核口径**（`fluid_medium.rs` 的子模块）：`sample` 与 `deposit` 的
//! **单一** 27 邻域遍历 —— 只采流体粒子（边界粒子是固体侧代表粒子，计进来会把
//! "体内部"读成"满水位"），逐粒回调 `(索引, poly6 权重)`。
//!
//! **为什么是自由函数而非 `FluidSystem` 的方法**：方法数是 god 门的**已登记债务**
//! （只准减），新能力不占方法位（同 `world_step` 的 `splat_body_drag` 先例）。
//! **为什么逐字段传参**：`deposit` 要同时借 `&mut self.vel` 与 `&self.grid`/`&self.pos`
//! —— 拆成字段后两处借用不相交，编译器放行（整借 `&FluidSystem` 会拒绝）。
use crate::grid::UniformGrid;
use crate::system::FluidSystem;
use vxl_phys_core::Vec3;

/// 遍历 `x` 所在格的 3×3×3 邻域，对每个**格内**流体粒子回调 `f(索引, W(r))`。
/// 格与格内顺序 = 均匀网格的既定次序（确定性；与 `sample` 逐位同序）。
// 单行签名 + `#[rustfmt::skip]`：args-gate 按"逗号数+1"计形参，**竖排 + 尾逗号会多算一个**
// （7 个形参被读成 8 ⇒ 撞 soft 阈值）。本仓既有同款先例。
#[rustfmt::skip]
pub(super) fn for_each_neighbor(grid: &UniformGrid, pos: &[Vec3], x: Vec3, h2: f32, k6: f32, n_fluid: usize, mut f: impl FnMut(usize, f32)) {
    if grid.nz == 0 {
        return;
    }
    let (bx, by, bz) = grid.bin_of(x);
    let (nx, ny, nz) = (grid.nx, grid.ny, grid.nz);
    for dx in -1i64..=1 {
        for dy in -1i64..=1 {
            for dz in -1i64..=1 {
                let (ix, iy, iz) = (bx as i64 + dx, by as i64 + dy, bz as i64 + dz);
                if ix < 0 || iy < 0 || iz < 0 || ix >= nx as i64 || iy >= ny as i64 || iz >= nz as i64
                {
                    continue;
                }
                let idx = ((ix as usize) * ny as usize + iy as usize) * nz as usize + iz as usize;
                for &j in grid.bin_items(idx) {
                    let j = j as usize;
                    if j >= n_fluid {
                        continue;
                    }
                    let d = pos[j] - x;
                    let r2 = d.length_squared();
                    if r2 > h2 {
                        continue;
                    }
                    let t = h2 - r2;
                    f(j, k6 * t * t * t);
                }
            }
        }
    }
}

/// 27 邻域 poly6 累加（只读）：返回 `(Σw, ρ, Σ w·v)` —— `sample` 的三个矩一次算完。
pub(super) fn poly6_scan(sys: &FluidSystem, x: Vec3) -> (f32, f32, Vec3) {
    let (mut wsum, mut rho) = (0.0f32, 0.0f32);
    let mut vsum = Vec3::ZERO;
    for_each_neighbor(
        &sys.grid,
        &sys.pos,
        x,
        sys.h2,
        sys.k6,
        sys.n_fluid,
        |j, w| {
            wsum += w;
            rho += sys.mass * w;
            vsum += sys.vel[j] * w;
        },
    );
    (wsum, rho, vsum)
}
