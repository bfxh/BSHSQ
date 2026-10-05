//! **扫描场景起步（splat → 初始粒子）** —— `ROUTE.md` §4 复核块最后一条。
//!
//! 这条链现在**两端都通了**：`GaussianSplatField::export_splats` 给出扫描件的核中心，
//! `vxl_phys_fluid::fluid_access::from_positions` 按**给定位置**建流体系统（质量仍按静止晶格
//! `ρ0 / Σ_lattice W` 标定 ⇒ 与 `FluidSystem::new` 的晶格块**同一套质量口径**）。
//! 判据：① 粒子数 = 扫描点数；② 每个粒子**逐值**落在对应扫描点上；③ 装进 `World` 推进后健康。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_fluid::fluid_access::from_positions;
use vxl_phys_fluid::FluidConfig;
use vxl_phys_splat::{export_splats, GaussianSplatField, Splat};

/// 三颗高斯核的"扫描件"（各向同性、opacity 1）。
fn scan() -> GaussianSplatField {
    let mut f = GaussianSplatField::new(4.0);
    f.push(Splat::isotropic(Vec3::new(0.0, 1.0, 0.0), 0.05, 1.0));
    f.push(Splat::isotropic(Vec3::new(0.1, 1.0, 0.0), 0.05, 1.0));
    f.push(Splat::isotropic(Vec3::new(0.0, 1.1, 0.05), 0.05, 1.0));
    f
}

#[test]
fn splat_centers_become_initial_fluid_particles() {
    let field = scan();
    let mut out = Vec::new();
    export_splats(&field, &mut out);
    assert_eq!(out.len(), 3, "扫描件应有 3 颗核");

    // 反向导入：把扫描点云当流体的**初始粒子**（`spacing` 只用于质量标定）。
    let centers: Vec<Vec3> = out.iter().map(|t| t.0).collect();
    let sys = from_positions(FluidConfig::default(), centers, 0.05);
    assert_eq!(sys.len(), out.len(), "粒子数应等于扫描点数");
    for (i, p) in sys.positions().iter().enumerate() {
        assert_eq!(*p, out[i].0, "第 {i} 个粒子应逐值落在扫描点上");
    }

    // 装进 `World` 推进：健康、粒子数不变（链路端到端可用）。
    let mut w = World::new(PhysConfig::default());
    w.add_fluid(sys, &[]);
    for _ in 0..10 {
        w.step();
    }
    let f = &w.fluids()[0].0;
    assert_eq!(f.len(), 3, "推进后粒子数不变");
    assert!(
        f.positions().iter().all(|p| p.y.is_finite()),
        "推进后位置应保持有限"
    );
}
