//! 均匀晶格 / 档位 / 邻居表：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::still_block;
use crate::{FluidConfig, FluidFamily};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;

/// #1 静止晶格密度 ≈ ρ0（±2%，核截断的边界粒子除外）。
#[test]
fn rest_lattice_density_matches_rest_density() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders);
    let rho0 = f.config().rest_density;
    // 内部判定：晶格坐标（i+½）落在 [2.4, 5.6] 之外即距边不足 2 格。
    let origin = Vec3::new(-0.1, 0.2, -0.1);
    let inv_sp = 1.0 / 0.05;
    let mut checked = 0;
    for (i, p) in f.positions().iter().enumerate() {
        let q = (*p - origin) * inv_sp;
        if q.x < 2.4 || q.x > 3.6 || q.y < 2.4 || q.y > 3.6 || q.z < 2.4 || q.z > 3.6 {
            continue;
        }
        let rel = (f.densities()[i] - rho0).abs() / rho0;
        assert!(rel < 0.02, "粒子 {i} 密度相对偏差 {rel:.6}");
        checked += 1;
    }
    assert_eq!(checked, 8, "5³ 块内部应恰有 2³=8 个粒子");
}

/// #6 网格邻居 = 全扫暴力邻居（集合相等，排序后比对）。
#[test]
fn grid_neighbors_equal_brute_force() {
    let mut f = still_block();
    f.step(1.0 / 60.0, &NoProviders); // 建格（零重力下位置不动）
    let n = f.len();
    let mut grid_sets: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (i, set) in grid_sets.iter_mut().enumerate() {
        f.for_neighbors(i, |j, _d, _r2| set.push(j as u32));
        set.sort_unstable();
    }
    let mut brute: Vec<Vec<u32>> = vec![Vec::new(); n];
    let h2 = f.dom.kern.h2;
    let ps = f.positions();
    for i in 0..n {
        for j in 0..n {
            if j != i && (ps[i] - ps[j]).length_squared() <= h2 {
                brute[i].push(j as u32);
            }
        }
    }
    assert_eq!(grid_sets, brute);
}

/// 既有口径：默认配置仍是 CPU SPH 族。
#[test]
fn default_is_cpu_sph() {
    let c = FluidConfig::default();
    assert_eq!(c.family, FluidFamily::CpuSph);
    assert_eq!(c.rest_density, 1000.0);
    assert_eq!(c.substeps, 4);
}
