use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_soft::{ClothSheet, ClothWind};

fn hash(sheet: &ClothSheet) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for p in &sheet.pos {
        for value in [p.x, p.y, p.z] {
            for b in value.to_bits().to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

fn run(windy_first: bool) -> (u64, u64, f32) {
    let config = PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    };
    let mut world = World::new(config);
    let make = |wind: bool| {
        let mut sheet = ClothSheet::grid(
            Vec3::ZERO,
            Vec3::new(0.125, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.125),
            9,
            9,
        );
        for col in 0..9 {
            sheet.set_pinned(col, true);
        }
        sheet.wind = wind.then_some(ClothWind {
            velocity: Vec3::Y * 5.0,
            density: 1.2,
            drag: 1.1,
        });
        sheet.damping = 0.99;
        sheet
    };
    let (windy, calm) = if windy_first {
        (world.add_cloth(make(true)), world.add_cloth(make(false)))
    } else {
        let calm = world.add_cloth(make(false));
        let windy = world.add_cloth(make(true));
        (windy, calm)
    };
    for _ in 0..60 {
        world.step();
    }
    let active = world.cloth(windy).unwrap();
    let passive = world.cloth(calm).unwrap();
    for col in 0..9 {
        assert_eq!(active.pos[col], passive.pos[col]);
    }
    (hash(active), hash(passive), active.pos[76].y)
}

#[test]
fn world_wind_is_per_sheet_and_registration_order_independent() {
    let a = run(true);
    let b = run(false);
    let c = run(true);
    println!(
        "world wind hash={:016x}, calm={:016x}, tip_y={:.6}",
        a.0, a.1, a.2
    );
    assert_eq!(a, b);
    assert_eq!(a, c);
    assert!(a.2 > 0.003, "wind must lift the free edge");
    assert_ne!(a.0, a.1);
    assert_eq!(a.0, 0x86a5_7a37_1df5_9fa4);
    assert_eq!(a.1, 0xc8d7_fece_43fb_2175);
}
