//! **三角网地面 + 挖掘/冲击破坏判据** —— `tests` 的子模块。
//!
//! 2026-10-05 从 967 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use crate::{PhysConfig, Quat, Shape, Vec3, World};

/// **网格域（M3 扩展）**：盒落在三角网格地面上并停住（薄壳接触；
/// 8 顶点 + 6 面心采样 ⇒ 底四角多点接触）。
#[test]
fn box_rests_on_mesh_ground() {
    let mut w = World::new(PhysConfig::default());
    // 4×4 米网格地面（y = 0，朝上）
    let ground =
        vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, Vec3::X * 2.0, Vec3::Z * 2.0, Vec3::Y);
    w.add_mesh(ground);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.3, 2.0, -0.2),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    assert!(y > 0.42 && y < 0.70, "y = {y}"); // 静置在网格面上方（半长 0.5）
    assert!(w.health().is_clean());
}

/// **网格域**：球在斜网格上按**面法线**接触并沿坡下滑（球无滚阻必下滚——
/// 因此断言「接触带内 + 法向速度被抑制 + 沿坡下滑」，而非「停在坡上」）。
#[test]
fn sphere_contacts_sloped_mesh_along_face_normal() {
    let mut w = World::new(PhysConfig::default());
    // 斜面：沿 +u 抬升（面法线 n 偏向 −X）
    let slope = 0.25f32;
    let n = Vec3::new(-slope, 1.0, 0.0).normalize();
    let u = Vec3::new(1.0, slope, 0.0).normalize() * 8.0;
    let v = Vec3::Z * 8.0;
    let ground = vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, u, v, n);
    w.add_mesh(ground);
    let ball = w.add_dynamic(
        Shape::Sphere { radius: 0.4 },
        Vec3::new(0.0, 1.5, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    // 触面后约 0.5 秒：仍在接触带内、法向速度被压制、沿 −u 下滑
    for _ in 0..90 {
        w.step();
    }
    let p = w.bodies.position[ball as usize];
    let vel = w.bodies.linvel[ball as usize];
    let dist = p.dot(n);
    let v_n = vel.dot(n);
    let down = vel.dot(u.normalize());
    assert!(
        dist > 0.30 && dist < 0.75,
        "应在接触带内（半径 0.4）：沿法线 {dist}"
    );
    assert!(v_n.abs() < 1.0, "法向速度应被接触抑制：{v_n}");
    assert!(down < -0.05, "应沿坡下滑（−u 方向）：v·u = {down}");
    assert!(w.health().is_clean());
}

/// **M3 破坏切片**：体素柱被「切掉顶部」⇒ 顶部转成刚体碎块，落在余柱上停驻；
/// 余柱（仍在体素体里）与碎块共同构成确定性可继续推进的场景。
#[test]
fn carve_top_spawns_debris_resting_on_column() {
    let mut w = World::new(PhysConfig::default());
    // 柱：X/Z ∈ [−0.5,0.5]、Y ∈ [0,4)，格边长 0.5（8 层 × 2×2）
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 8, 8);
    vol.fill_box(Vec3::new(-0.5, 0.0, -0.5), Vec3::new(0.5, 4.0, 0.5));
    w.add_voxel(vol);
    // 切掉顶部 1m（Y ∈ [3,4)）⇒ 碎块（2×2×2 格 ⇒ 贪心合并为 1 个 1m 立方）
    let n = w.spawn_box_debris(
        0,
        Vec3::new(-0.5, 3.0, -0.5),
        Vec3::new(0.5, 4.0, 0.5),
        1000.0,
    );
    assert_eq!(n, 1, "顶部 8 格应合并为 1 个碎块盒");
    for _ in 0..600 {
        w.step();
    }
    let h = w.health();
    assert!(h.is_clean(), "无 NaN / 无深穿透");
    // 碎块落在余柱顶面（y=3.0）上方：中心 ≈ 3.5
    let mut top = 0.0f32;
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            top = top.max(w.bodies.position[i].y);
        }
    }
    assert!(top > 3.3 && top < 3.7, "碎块应停在余柱上，实际 top={top}");
}

/// **M3 冲击破坏**：高速盒撞体素墙 ⇒ 接触点处挖洞并产出碎块；墙体素减少、
/// 场景干净，且**整轮可复现**（同一构造两次 → 碎块数/末态哈希一致）。
#[test]
fn impact_carves_wall_and_spawns_debris() {
    let run = || -> (usize, usize, usize, u128) {
        let mut w = World::new(PhysConfig::default());
        // 地板（整幅 1 层）+ 墙（X ∈ [0,0.5]、Y ∈ [0.5,2.5)、Z ∈ [−2,2)）
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
        vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 0.5, 4.0));
        vol.fill_box(Vec3::new(0.0, 0.5, -2.0), Vec3::new(0.5, 2.5, 2.0));
        let filled0 = vol.filled_count();
        w.add_voxel(vol);
        // 炮弹：半 0.4 的盒，以 12 m/s 冲墙
        let bullet = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(-3.0, 1.0, 0.0),
            Quat::IDENTITY,
            2000.0,
        );
        w.bodies.linvel[bullet as usize] = Vec3::new(12.0, 0.0, 0.0);
        let mut debris_total = 0usize;
        for _ in 0..240 {
            w.step();
            debris_total += w.apply_impact_destruction(0, 3.0, 1000.0);
        }
        let vol_after = w.providers.voxel(0);
        assert!(vol_after.is_some(), "提供者仍应存在");
        let filled1 = vol_after.map_or(0, |v| v.filled_count());
        let bodies = w.bodies.len();
        (debris_total, filled0 - filled1, bodies, w.state_hash())
    };
    let (debris, carved, _bodies, hash1) = run();
    assert!(debris > 0, "应触发冲击破坏（产出碎块）");
    assert!(carved > 0, "墙体素应减少（挖洞）carved={carved}");
    let (debris2, carved2, _b, hash2) = run();
    assert_eq!((debris, carved), (debris2, carved2), "破坏应可复现（计数）");
    assert_eq!(hash1, hash2, "破坏应可复现（末态哈希逐位一致）");
}
