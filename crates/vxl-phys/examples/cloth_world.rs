use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_soft::ClothSheet;
use vxl_phys_terrain::mesh::TriMesh;

fn main() {
    let mut world = World::new(PhysConfig::default());
    world.add_mesh(TriMesh::new(
        vec![
            Vec3::new(-2.0, 0.0, -2.0),
            Vec3::new(2.0, 0.0, -2.0),
            Vec3::new(-2.0, 0.0, 2.0),
            Vec3::new(2.0, 0.0, 2.0),
        ],
        vec![[0, 2, 1], [1, 2, 3]],
    ));
    let mut sheet = ClothSheet::grid(
        Vec3::new(-0.4, 1.0, -0.4),
        Vec3::new(0.1, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.1),
        9,
        9,
    );
    sheet.damping = 0.99;
    let cloth = world.add_cloth(sheet);
    for _ in 0..180 {
        world.step();
    }
    let sheet = world.cloth(cloth).expect("registered cloth");
    let min_y = sheet.pos.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    println!(
        "tick={} cloth={} vertices={} triangles={} min_y={min_y:.4}",
        world.tick,
        cloth,
        sheet.pos.len(),
        sheet.triangles.len(),
    );
}
