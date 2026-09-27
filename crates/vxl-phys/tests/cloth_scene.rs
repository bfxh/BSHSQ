use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_soft::ClothSheet;
use vxl_phys_terrain::mesh::TriMesh;

fn mesh() -> TriMesh {
    TriMesh::new(
        vec![
            Vec3::new(-4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, -4.0),
            Vec3::new(-4.0, 0.0, 4.0),
            Vec3::new(4.0, 0.0, 4.0),
        ],
        vec![[0, 2, 1], [1, 2, 3]],
    )
}

fn hash(cloth: &ClothSheet) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for p in &cloth.pos {
        for value in [p.x, p.y, p.z] {
            for b in value.to_bits().to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

fn scene(with_mesh: bool) -> (f32, f32, f32, Vec3, u64, u64) {
    let mut world = World::new(PhysConfig::default());
    if with_mesh {
        world.add_mesh(mesh());
    }
    let body = world.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(1.5, 1.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let mut falling = ClothSheet::grid(
        Vec3::new(-0.2, 0.5, -0.2),
        Vec3::new(0.1, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.1),
        5,
        5,
    );
    falling.damping = 0.99;
    assert_eq!(world.add_cloth(falling), 0);
    let mut hanging = ClothSheet::grid(
        Vec3::new(-0.2, 1.5, 0.6),
        Vec3::new(0.1, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.1),
        5,
        5,
    );
    for col in 0..5 {
        hanging.set_pinned(col, true);
    }
    hanging.damping = 0.99;
    assert_eq!(world.add_cloth(hanging), 1);
    for _ in 0..180 {
        world.step();
    }
    assert_eq!(world.cloths().len(), 2);
    assert!(world.cloth(2).is_none());
    let land = world.cloth(0).unwrap();
    let min_y = land.pos.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let max_y = land
        .pos
        .iter()
        .map(|p| p.y)
        .fold(f32::NEG_INFINITY, f32::max);
    let hang = world.cloth(1).unwrap();
    for col in 0..5 {
        assert_eq!(hang.pos[col], Vec3::new(-0.2 + col as f32 * 0.1, 1.5, 0.6));
    }
    (
        min_y,
        max_y,
        world.bodies.position[body].y,
        hang.pos[22],
        hash(land),
        hash(hang),
    )
}

#[test]
fn cloth_steps_in_world_against_registered_mesh_beside_rigid_body() {
    let (min_y, max_y, body_y, tip, land_hash, hang_hash) = scene(true);
    let (without_floor, _, _, _, _, _) = scene(false);
    let repeat = scene(true);
    println!("cloth world floor y={min_y:.6}..{max_y:.6} no_floor={without_floor:.3} body={body_y:.3} tip={tip:?} hashes={land_hash:016x}/{hang_hash:016x}");
    assert!(min_y >= 0.017, "cloth must not penetrate mesh");
    assert!(max_y < 0.04, "cloth must settle on mesh");
    assert!(
        without_floor < -1.0,
        "mesh contact must come from world provider"
    );
    assert!((body_y - 0.2).abs() < 0.05, "rigid body must still settle");
    assert!(tip.y < 1.45, "pinned sheet must sag");
    assert_eq!((min_y, max_y, body_y, tip, land_hash, hang_hash), repeat);
    assert_eq!(land_hash, 0x9b91_24d9_1d44_2d05);
    assert_eq!(hang_hash, 0xab2c_6992_fcff_999a);
}
