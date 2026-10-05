//! **刚体/流体 × 提供者（体素/高度场）的就位判据** —— `tests` 的子模块。
//!
//! 2026-10-05 从 967 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use super::{cube_hull_points, ground_world};
use crate::{PhysConfig, Quat, Shape, Vec3, World};

#[test]
fn box_falls_and_rests_on_heightfield() {
    let mut w = ground_world();
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 3.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..240 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在 y ≈ 0.5 + 少许穿透修正余量。
    assert!(y > 0.45 && y < 0.62, "y = {y}");
    assert!(w.health().is_clean());
}

/// **M2 贯通切片**（ROUTE §7）：**刚体 ↔ 体素**——盒经 `Shape::Provider` 路径
/// 落在体素地面上并入睡（跨域唯一通道 `ProviderColliders` 的第一条端到端用例）。
#[test]
fn box_falls_and_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
    let marker = w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 2.5, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：y ≈ 1.5 + 少许穿透修正余量。
    assert!(y > 1.42 && y < 1.60, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "盒应已入睡（静置 10s）");
    assert!(w.health().is_clean());
    // marker 体（provider）保持静止：位置零漂移。
    assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
}

/// **M0.3 液体域**（ROUTE §7）：流体块经门面落入**体素盆**（地板+四壁，
/// 一并覆盖体素 provider 的顶面/内壁/角点接触路径）并停驻。
/// `add_fluid` + `fluid_pass` 端到端；单向耦合，marker 体不受扰动。
/// （不用悬浮板：驻留投影不消耗切向速度，冲击横流会沿板面滑出板缘——
/// 那是正确物理，但场景里板外无物，跑出者永远下坠，断言无从谈起。）
#[test]
fn fluid_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    // 体素盆：外廓 0.8×0.8m、格边 0.2；地板层顶面 y=0.2，壁高到 y=0.8，
    // 内腔 0.4×0.4（与 fluid 域 Tank 测试同腔口）。
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-0.4, 0.0, -0.4), 0.2, 4, 4, 4);
    vol.fill_box(Vec3::new(-0.4, 0.0, -0.4), Vec3::new(0.4, 0.2, 0.4)); // 地板
    vol.fill_box(Vec3::new(0.2, 0.2, -0.4), Vec3::new(0.4, 0.8, 0.4)); // +x 壁
    vol.fill_box(Vec3::new(-0.4, 0.2, -0.4), Vec3::new(-0.2, 0.8, 0.4)); // −x 壁
    vol.fill_box(Vec3::new(-0.2, 0.2, 0.2), Vec3::new(0.2, 0.8, 0.4)); // +z 壁
    vol.fill_box(Vec3::new(-0.2, 0.2, -0.4), Vec3::new(0.2, 0.8, -0.2)); // −z 壁
    let marker = w.add_voxel(vol);
    assert!(
        w.provider_id_of(marker).is_some(),
        "marker 应是 provider 体"
    );
    let Some(vid) = w.provider_id_of(marker) else {
        return;
    };
    // 铸装近平衡块（8×8 贴腔口 + 5 层 ≈ 实测静水充高 0.44，320 粒）。
    // 不用方块自落：任何带落差的方块入盆，WCSPH 驻留瞬态（底部镜像
    // 鬼影密度尾 → 压实波在块顶心聚焦）都会把顶心粒子以近钳制速度
    // （实测 ~9 m/s）垂直喷过敞口壁顶——0.8 m 重落、0.1 m 轻落、触底
    // 就位皆复现，是 PLAN-0.3 §4 已记录的求解器瞬态而非边界失效；
    // 边界本身在全部场景中零穿壁。铸装后瞬态消失，本测试只验驻留与
    // 边界。
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.175, 0.25, -0.175),
        [8, 8, 5],
        0.05,
    );
    let fid = w.add_fluid(sys, &[vid]);
    for _ in 0..300 {
        w.step();
    }
    let f = &w.fluids()[fid].0;
    for (i, p) in f.positions().iter().enumerate() {
        assert!(p.y > 0.15, "粒子 {i} 穿透盆底：y = {}", p.y);
        assert!(p.y < 0.9, "粒子 {i} 飞出：y = {}", p.y);
        assert!(
            p.x.abs() < 0.45 && p.z.abs() < 0.45,
            "粒子 {i} 越出盆壁：({}, {})",
            p.x,
            p.z
        );
    }
    // 单向耦合：marker 体（provider）保持静止。
    assert_eq!(w.bodies.position[marker as usize], Vec3::ZERO);
    assert!(w.health().is_clean());
}

/// **L1**：外壳落在高度场上（顶点采样；此前不受理 ⇒ 直接穿地）。
#[test]
fn hull_rests_on_heightfield() {
    let mut w = ground_world(); // 平地高度场（y = 0）
    let hull = w.add_hull(cube_hull_points(0.5));
    let b = w.spawn_hull_body(hull, Vec3::new(0.1, 3.0, -0.1), Quat::IDENTITY, 1.0);
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在平地（y=0）上方：半长 0.5 ⇒ y ≈ 0.5
    assert!(y > 0.40 && y < 0.62, "y = {y}");
    assert!(w.health().is_clean());
}

/// **M3 多边形域**：凸体外壳落在体素地面上（外壳 × 提供者 = 顶点采样多点流形）。
#[test]
fn hull_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0)); // 顶面 y = 1.0
    w.add_voxel(vol);
    let hull = w.add_hull(cube_hull_points(0.5));
    let b = w.spawn_hull_body(hull, Vec3::new(0.1, 2.5, -0.1), Quat::IDENTITY, 1.0);
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：外壳半长 0.5 ⇒ y ≈ 1.5
    assert!(y > 1.42 && y < 1.70, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "外壳应已入睡");
    assert!(w.health().is_clean());
}

/// **M3 凸体切割**：立方体外壳 ✕ 8 种子 ⇒ 8 个凸碎块，逐个落在体素地面上。
#[test]
fn fractured_hull_pieces_rest_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    let hull = w.add_hull(cube_hull_points(0.5));
    let seeds: Vec<Vec3> = [
        Vec3::new(-0.3, -0.25, -0.35),
        Vec3::new(-0.3, -0.25, 0.25),
        Vec3::new(-0.3, 0.35, -0.35),
        Vec3::new(-0.3, 0.35, 0.25),
        Vec3::new(0.3, -0.25, -0.35),
        Vec3::new(0.3, -0.25, 0.25),
        Vec3::new(0.3, 0.35, -0.35),
        Vec3::new(0.3, 0.35, 0.25),
    ]
    .to_vec();
    let pieces = w.spawn_hull_pieces(hull, &seeds, Vec3::new(0.0, 2.2, 0.0), Quat::IDENTITY, 1.0);
    assert_eq!(pieces.len(), 8, "应有 8 块");
    for _ in 0..900 {
        w.step();
    }
    // 全体落到地面带内且干净
    for &p in &pieces {
        let y = w.bodies.position[p as usize].y;
        assert!(y > 0.9 && y < 2.0, "碎块 y = {y} 不在带内");
    }
    assert!(w.health().is_clean());
}

/// M2 provider 通道扩到**球**：球经 SDF 解析接触（`depth = r − sdf(c)`）
/// 落在体素地面上并入睡；顺带覆盖「斜坡不穿透」。
#[test]
fn sphere_rests_on_voxel_provider() {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 2, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.4 },
        Vec3::new(0.25, 2.0, 0.25),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 静置在体素顶面（y=1.0）上方：y ≈ 1.4
    assert!(y > 1.32 && y < 1.50, "y = {y}");
    assert!(!w.bodies.awake[b as usize], "球应已入睡");
    assert!(w.health().is_clean());
}
