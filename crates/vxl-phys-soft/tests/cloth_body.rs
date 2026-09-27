use vxl_phys_core::{interop::NoProviders, Quat, Shape, Vec3};
use vxl_phys_soft::{ClothSheet, RigidProxy};

fn sheet() -> ClothSheet {
    let mut s = ClothSheet::grid(Vec3::ZERO, Vec3::X, Vec3::Z, 2, 2);
    s.substeps = 1;
    for i in 1..4 {
        s.set_pinned(i, true);
    }
    s
}

fn proxy(shape: Shape, inv_mass: f32) -> RigidProxy {
    RigidProxy {
        body: 7,
        shape,
        pos: Vec3::new(0.0, -0.15, 0.0),
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        inv_mass,
    }
}

#[test]
fn static_sphere_and_box_separate_cloth_without_reaction() {
    for shape in [
        Shape::Sphere { radius: 0.2 },
        Shape::Capsule {
            half_height: 0.2,
            radius: 0.2,
        },
        Shape::Box {
            half: Vec3::splat(0.2),
        },
    ] {
        let mut s = sheet();
        s.step_with_bodies(0.1, Vec3::ZERO, &NoProviders, 0, &[proxy(shape, 0.0)]);
        assert!(s.pos[0].y > 0.0);
        assert_eq!(s.pos[1], Vec3::X);
        assert_eq!(s.body_dv, [Vec3::ZERO]);
        assert_eq!(s.body_dx, [Vec3::ZERO]);
    }
}

#[test]
fn dynamic_sphere_shares_penetration_and_equal_opposite_impulse() {
    let mut s = sheet();
    let mut moving = proxy(Shape::Sphere { radius: 0.2 }, 1.0);
    moving.linvel = Vec3::Y;
    s.step_with_bodies(0.1, Vec3::ZERO, &NoProviders, 0, &[moving]);
    assert!((s.pos[0].y - 0.035).abs() < 1e-6);
    assert!((s.body_dx[0].y + 0.035).abs() < 1e-6);
    assert!((s.body_dv[0].y + s.vel[0].y).abs() < 1e-6);
    assert_eq!(s.pos[1], Vec3::X);
}

#[test]
fn resting_overlap_does_not_create_momentum() {
    let mut s = sheet();
    s.step_with_bodies(
        0.1,
        Vec3::ZERO,
        &NoProviders,
        0,
        &[proxy(Shape::Sphere { radius: 0.2 }, 1.0)],
    );
    assert!(s.pos[0].y > 0.0);
    assert!(s.body_dx[0].y < 0.0);
    assert_eq!(s.vel[0], Vec3::ZERO);
    assert_eq!(s.body_dv[0], Vec3::ZERO);
}

#[test]
fn unsupported_dynamic_box_is_skipped() {
    let mut s = sheet();
    s.step_with_bodies(
        0.1,
        Vec3::ZERO,
        &NoProviders,
        0,
        &[proxy(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            1.0,
        )],
    );
    assert_eq!(s.pos[0], Vec3::ZERO);
    assert_eq!(s.body_dv, [Vec3::ZERO]);
}
