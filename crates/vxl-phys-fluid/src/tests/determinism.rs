//! 确定性：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::{still_pose, FloorY, Tank};
use crate::{FluidConfig, FluidSystem};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Shape, Vec3};

/// #4 确定性：同输入两次运行 600 子步，位置逐位相等。
#[test]
fn deterministic_bitwise() {
    let run = || {
        let cfg = FluidConfig {
            substeps: 2,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.0, -0.1), [5, 5, 5], 0.05);
        f.set_boundaries(&[0]);
        for _ in 0..300 {
            f.step(1.0 / 60.0, &FloorY);
        }
        f.positions()
            .iter()
            .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run(), "两次运行必须逐位一致");
}

/// #14 **确定性**（2b 在场）：同场景两跑，位置/密度/反作用**逐位一致**。
#[test]
fn boundary_coupling_is_deterministic() {
    let digest = || {
        let cfg = FluidConfig {
            xsph_viscosity: 0.05,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
        f.set_boundaries(&[0]);
        let body = (
            7u32,
            Shape::Box {
                half: Vec3::splat(0.06),
            },
            still_pose(Vec3::new(0.0, 0.12, 0.0)),
        );
        for _ in 0..60 {
            let _ = f.set_boundary_particles(std::slice::from_ref(&body));
            f.step(1.0 / 60.0, &Tank);
        }
        (
            f.positions()
                .iter()
                .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                .collect::<Vec<u32>>(),
            f.densities()
                .iter()
                .map(|d| d.to_bits())
                .collect::<Vec<u32>>(),
            f.boundary_reactions()
                .iter()
                .flat_map(|r| [r.1.x.to_bits(), r.1.y.to_bits(), r.2.x.to_bits()])
                .collect::<Vec<u32>>(),
            f.boundary_count(),
        )
    };
    assert_eq!(digest(), digest(), "两次运行应逐位一致");
}

/// **并行 = 串行逐位一致**（`FluidConfig::threads`；2026-09-23 规模档并行化的守门）：
/// 同一场景跑 `threads = 1` 与 `threads = 4`，末态位置/速度/密度**逐位相同**。
/// 覆盖**两条路径**：① 纯流体（无边界粒子）；② **含边界粒子**（2b）——后者是风险点，
/// 因为 `bforce` 是唯一的跨粒子累加量（并行路径用**串行补趟**保序）。
#[test]
fn parallel_equals_serial_bitwise() {
    let digest = |threads: usize, with_boundary: bool| {
        let cfg = FluidConfig {
            threads,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
        if with_boundary {
            let floor = (
                0u32,
                Shape::Box {
                    half: Vec3::new(0.3, 0.05, 0.3),
                },
                still_pose(Vec3::new(0.0, -0.05, 0.0)),
            );
            let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
        }
        for _ in 0..40 {
            if with_boundary {
                let floor = (
                    0u32,
                    Shape::Box {
                        half: Vec3::new(0.3, 0.05, 0.3),
                    },
                    still_pose(Vec3::new(0.0, -0.05, 0.0)),
                );
                let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
            }
            f.step(1.0 / 60.0, &NoProviders);
        }
        let mut d: Vec<u32> = Vec::new();
        for p in f.positions() {
            d.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        }
        for v in f.velocities() {
            d.extend([v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
        }
        for r in f.densities() {
            d.push(r.to_bits());
        }
        for r in f.boundary_reactions() {
            d.extend([r.1.x.to_bits(), r.1.y.to_bits(), r.2.z.to_bits()]);
        }
        d
    };
    assert_eq!(
        digest(1, false),
        digest(4, false),
        "纯流体：并行与串行应逐位一致"
    );
    assert_eq!(
        digest(1, true),
        digest(4, true),
        "含边界粒子（2b）：并行与串行应逐位一致（bforce 走串行补趟）"
    );
}
