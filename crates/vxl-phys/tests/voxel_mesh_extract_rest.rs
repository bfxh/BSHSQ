//! **I3 静置对拍 + 零丢弃**（`PLAN-CONVERSION.md` §4）：同一块体素地板
//! （8×2×8、边长 0.5、顶面 y=1.0）分别以 ① 原生体素 provider 与
//! ② `voxel::surface_mesh` 提取网格 → `add_mesh`（三角网 provider，薄壳口径）
//! 两种表示立世，同一只盒落下 ⇒ 静置高度对拍；并断言提取产物经注册**零丢弃**。
//!
//! 两路口径不同（体素 `depth = skin − sdf` vs 网格薄壳 `depth = skin − dist`），
//! 差值阈值 = 首测值 + 余量（先量后写，PLAN I3；首测读数见断言注释）。

use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::mesh::TriMesh;
use vxl_phys_terrain::voxel::{surface_mesh, VoxelVolume};

const TICKS: usize = 600;

fn floor() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

/// 建世界（地板由调用方加）→ 落同一只盒（半高 0.5、μ=0.9）→ 推进 → 末态 y。
fn rest_height(add_floor: impl FnOnce(&mut World)) -> f32 {
    let mut w = World::new(PhysConfig::default());
    add_floor(&mut w);
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.25, 2.0, 0.25),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    for _ in 0..TICKS {
        w.step();
    }
    let (pos, _) = w.bodies.pose(i);
    pos.y
}

#[test]
fn extracted_mesh_floor_rests_like_voxel_floor() {
    // 零丢弃：提取产物注册进 provider 后，顶点/三角数不变（MeshStore/ TriMesh
    // 会静默丢非法三角——提取器必须产合法网格，这里把"静默"变"显式"）
    let vol = floor();
    let mesh = surface_mesh(&vol);
    let mut w = World::new(PhysConfig::default());
    let marker = w.add_mesh(TriMesh::new(mesh.points.clone(), mesh.tris.clone()));
    let pid = w
        .provider_id_of(marker)
        .expect("网格 marker 体应带 provider id");
    let tm = w.providers().mesh(pid).expect("网格 provider 应已注册");
    assert_eq!(tm.tris().len(), mesh.tris.len(), "注册不得丢弃三角");
    assert_eq!(tm.verts().len(), mesh.points.len(), "注册不得丢弃顶点");

    // 静置对拍：同一只盒，两种表示的地板
    let y_vox = rest_height(|w| {
        w.add_voxel(floor());
    });
    let y_mesh = rest_height(|w| {
        let m = surface_mesh(&floor());
        w.add_mesh(TriMesh::new(m.points, m.tris));
    });
    println!("  体素地板末态 y = {y_vox:.4}（期望 1.5）");
    println!("  网格地板末态 y = {y_mesh:.4}（期望 1.5）");
    // 首测锚（先量后写，2026-10-02）：体素 1.4995 / 网格 1.4997 / 差 0.0002
    // ⇒ 钉 5 mm 窗（≈10× 首测余量）。
    assert!(
        (y_vox - 1.5).abs() <= 0.005,
        "体素路径静置高度异常：y={y_vox}"
    );
    assert!(
        (y_mesh - 1.5).abs() <= 0.005,
        "网格路径静置高度异常：y={y_mesh}"
    );
    assert!(
        (y_vox - y_mesh).abs() <= 0.005,
        "两表示静置高度差超阈：vox={y_vox} mesh={y_mesh}"
    );
}
