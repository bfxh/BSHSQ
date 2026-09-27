use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_soft::ClothSheet;

fn cloth() -> ClothSheet {
    let mut s = ClothSheet::grid(Vec3::ZERO, Vec3::X, Vec3::Z, 2, 2);
    s.substeps = 1;
    for i in 1..4 {
        s.set_pinned(i, true);
    }
    s
}

fn run() -> (u32, u32, u32, u32) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.2 },
        Vec3::new(0.0, -0.15, 0.0),
        Quat::IDENTITY,
        30.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::Y;
    w.add_cloth(cloth());
    w.step();
    let c = w.cloth(0).unwrap();
    let m = 1.0 / w.bodies.inv_mass[b];
    assert!(c.pos[0].y > 0.0);
    assert!(w.bodies.position[b].y < -0.15);
    assert!((c.vel[0].y + m * (w.bodies.linvel[b].y - 1.0)).abs() < 1e-4);
    println!(
        "cloth_y={:.6} sphere_y={:.6} impulse={:.6}",
        c.pos[0].y,
        w.bodies.position[b].y,
        m * w.bodies.linvel[b].y
    );
    (
        c.pos[0].y.to_bits(),
        w.bodies.position[b].y.to_bits(),
        c.vel[0].y.to_bits(),
        w.bodies.linvel[b].y.to_bits(),
    )
}

#[test]
fn world_cloth_returns_equal_opposite_impulse_to_dynamic_sphere() {
    let result = run();
    assert_eq!(result, run());
    assert_eq!(result, (1026684572, 3191130631, 1056986818, 1056986819));
}

#[test]
fn later_sheet_sees_sphere_correction_from_earlier_sheet() {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.2 },
        Vec3::new(0.0, -0.15, 0.0),
        Quat::IDENTITY,
        30.0,
    ) as usize;
    w.add_cloth(cloth());
    w.add_cloth(cloth());
    w.step();
    let first = w.cloth(0).unwrap().pos[0].y;
    let second = w.cloth(1).unwrap().pos[0].y;
    assert!(second > 0.0 && second < first);
    assert!(w.bodies.position[b].y > -0.21);
    assert_eq!(w.bodies.linvel[b], Vec3::ZERO);
}

#[test]
fn world_cloth_contacts_static_box_without_moving_it() {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let box_id = w.add_static(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(0.0, -0.2, 0.0),
        Quat::IDENTITY,
    ) as usize;
    w.add_cloth(cloth());
    w.step();
    assert!(w.cloth(0).unwrap().pos[0].y >= 0.019);
    assert_eq!(w.bodies.position[box_id], Vec3::new(0.0, -0.2, 0.0));
}
