//! **Voronoi 预断裂的碎块预算**（安全审计 F-14）。
//!
//! 审计读数：公开的 `World::fracture_voronoi` 传大量种子时**一次调用生成 30 403 个刚体**
//! （冲击破坏路径受 `FragmentBudget::B1K` 约束，这条公开路径不受），耗时近平方
//! （2k 种子 0.15 s / 8k 1.1 s / 32k 16.5 s / 200k 超时）。
//!
//! 处置：**种子数与碎块数都封顶 `B1K` 的容量（1024）**，被截断的格留在体素里（不丢几何），
//! 返回值如实给本次生成数。判据四条：① 超量输入下**顶满**预算（非空洞）；② 生成体数 ==
//! 返回值；③ 截断后换一组种子**还能**提取（几何没丢、可分片）；④ 空种子集不造体。

use vxl_phys::{BodyBuildExt, DestructionOpsExt, PhysConfig, Vec3, World};

/// 32³ 格的实心体素块（域 = 8 m 立方、格 0.25 m）——**足够大**，让 1024 个种子也远超预算。
fn solid_volume() -> vxl_phys_terrain::voxel::VoxelVolume {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.25, 32, 32, 32);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 4.0, 4.0));
    vol
}

/// 域（8 m 立方）。
const LO: Vec3 = Vec3::new(-4.0, 0.0, -4.0);
const HI: Vec3 = Vec3::new(4.0, 4.0, 4.0);

#[test]
fn voronoi_fracture_is_budgeted() {
    let mut w = World::new(PhysConfig::default());
    let vol = solid_volume();
    let filled0 = vol.filled_count();
    w.add_voxel(vol);
    // 4000 个种子：旧实现把它们全用上（审计的 30 403 体就是这样来的）。
    let seeds = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
        Vec3::new(-3.5, 0.5, -3.5),
        Vec3::new(3.5, 3.5, 3.5),
        4000,
        0.6,
    );
    assert_eq!(seeds.len(), 4000, "场景没造出足够种子，判据会空洞");
    let before = w.bodies.len();
    let n = w.fracture_voronoi(0, LO, HI, &seeds, 1000.0);
    assert_eq!(n, 1024, "这个场景必须顶满预算，否则判据空洞（实际 {n}）");
    assert_eq!(
        w.bodies.len() - before,
        n,
        "生成的体数必须等于返回的碎块数（如实上报）"
    );
    // ③ 截断不丢几何：换一组种子再调一次，剩下的格照样能提取（分片可用）。
    let seeds2 = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
        Vec3::new(-3.5, 0.5, -3.5),
        Vec3::new(3.5, 3.5, 3.5),
        4000,
        0.4,
    );
    let n2 = w.fracture_voronoi(0, LO, HI, &seeds2, 1000.0);
    println!("fracture: filled0={filled0} 第一次 {n} 块 / 第二次 {n2} 块（预算 1024）");
    assert!(
        n2 > 0,
        "超预算的盒被回填成体素 ⇒ 第二次调用还能提取（实际 {n2}）"
    );
    assert!(n2 <= 1024, "第二次同样受预算约束（实际 {n2}）");
    // ④ 边界：空种子集不得造体
    let n3 = w.fracture_voronoi(0, LO, HI, &[], 1000.0);
    assert_eq!(n3, 0, "空种子集不得生成碎块");
}

/// 小规模（远不到预算）时不被截断：8 个种子的碎块全部提取。
#[test]
fn small_fracture_is_not_truncated() {
    let mut w = World::new(PhysConfig::default());
    w.add_voxel(solid_volume());
    let seeds = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
        Vec3::new(-3.5, 0.5, -3.5),
        Vec3::new(3.5, 3.5, 3.5),
        8,
        0.6,
    );
    let n = w.fracture_voronoi(0, LO, HI, &seeds, 1000.0);
    assert!(n > 0 && n < 1024, "8 个种子远不到预算（实际 {n}）");
}
