use vxl_phys_core::{interop::NoProviders, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

fn main() {
    let mut cloth = ClothSheet::grid(
        Vec3::new(-0.5, 2.0, 0.0),
        Vec3::new(0.125, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.125),
        9,
        9,
    );
    for col in 0..9 {
        cloth.set_pinned(col, true);
    }
    cloth.constraints.structural = Stiffness::Hard;
    cloth.damping = 0.99;
    for _ in 0..180 {
        cloth.step(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0), &NoProviders, 0);
    }
    println!(
        "cloth: {} vertices, {} triangles, edges {:?}",
        cloth.pos.len(),
        cloth.triangles.len(),
        cloth.edge_counts()
    );
    println!(
        "free corner {:?}, structural residual {:.6} m",
        cloth.pos[80],
        cloth.max_edge_error(0)
    );
}
