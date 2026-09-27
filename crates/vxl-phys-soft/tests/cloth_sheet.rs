use vxl_phys_core::{interop::NoProviders, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};
use vxl_phys_terrain::mesh::TriMesh;

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

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

#[test]
fn grid_topology_and_independent_constraints() {
    let mut sheet = ClothSheet::grid(Vec3::ZERO, Vec3::X, Vec3::Z, 4, 3);
    assert_eq!(sheet.pos.len(), 12);
    assert_eq!(sheet.triangles.len(), 12);
    assert_eq!(sheet.triangles[0], [0, 4, 1]);
    let [a, b, c] = sheet.triangles[0].map(|i| sheet.pos[i as usize]);
    assert!((b - a).cross(c - a).y > 0.0);
    assert_eq!(sheet.edge_counts(), [17, 12, 10]);
    assert_eq!(sheet.max_edge_error(0), 0.0);
    sheet.set_pinned(0, true);
    sheet.constraints.structural = Stiffness::Custom(0.0);
    sheet.constraints.shear = Stiffness::Soft;
    sheet.constraints.bending = Stiffness::Jelly;
    sheet.step(DT, G, &NoProviders, 0);
    assert_eq!(sheet.pos[0], Vec3::ZERO);
    assert!(sheet.pos[11].y < 0.0);
    assert_eq!(sheet.edge_counts(), [17, 12, 10]);
}

#[test]
fn compliance_families_act_independently() {
    let make = |structural: f32, shear: f32| {
        let mut s = ClothSheet::grid(Vec3::ZERO, Vec3::X, Vec3::Z, 2, 2);
        s.pos[3] += Vec3::new(0.4, 0.0, 0.4);
        s.set_pinned(0, true);
        s.substeps = 1;
        s.iterations = 4;
        s.constraints.structural = Stiffness::Custom(structural);
        s.constraints.shear = Stiffness::Custom(shear);
        s.constraints.bending = Stiffness::Custom(1e9);
        s.step(DT, Vec3::ZERO, &NoProviders, 0);
        s
    };
    let hard_struct = make(0.0, 1e9);
    let soft_struct = make(1.0, 1e9);
    let hard_shear = make(1e9, 0.0);
    let soft_shear = make(1e9, 1.0);
    assert!(hard_struct.max_edge_error(0) < soft_struct.max_edge_error(0));
    assert!(hard_shear.max_edge_error(1) < soft_shear.max_edge_error(1));
    assert_ne!(soft_struct.pos[3], soft_shear.pos[3]);
}

fn hang(iterations: u32) -> ClothSheet {
    let mut sheet = ClothSheet::grid(
        Vec3::new(0.0, 2.0, 0.0),
        Vec3::new(0.125, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.125),
        9,
        9,
    );
    for col in 0..9 {
        sheet.set_pinned(col, true);
    }
    sheet.constraints.structural = Stiffness::Custom(0.0);
    sheet.constraints.shear = Stiffness::Custom(0.0);
    sheet.constraints.bending = Stiffness::Custom(0.0);
    sheet.iterations = iterations;
    sheet.damping = 0.99;
    for _ in 0..180 {
        sheet.step(DT, G, &NoProviders, 0);
    }
    sheet
}

#[test]
fn hanging_sheet_keeps_anchors_and_converges_with_iterations() {
    let loose = hang(1);
    let tight = hang(8);
    let again = hang(8);
    let e1 = loose.max_edge_error(0);
    let e8 = tight.max_edge_error(0);
    println!(
        "cloth residual 1={e1:.6} 8={e8:.6}, tip_y={:.6}, hash={:016x}",
        tight.pos[76].y,
        hash(&tight)
    );
    for col in 0..9 {
        assert_eq!(tight.pos[col], Vec3::new(col as f32 * 0.125, 2.0, 0.0));
    }
    assert!(tight.pos[76].y < 1.8, "hanging cloth should sag");
    assert!(
        e8 < e1 && e8 < 0.025,
        "XPBD iterations must reduce stretch: {e1} -> {e8}"
    );
    assert_eq!(hash(&tight), hash(&again));
    assert_eq!(hash(&tight), 0xa988_74a3_7349_3667);
}

fn floor() -> TriMesh {
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

#[test]
fn sheet_lands_on_real_terrain_provider() {
    let make = || {
        ClothSheet::grid(
            Vec3::new(-0.3, 0.5, -0.3),
            Vec3::new(0.1, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.1),
            7,
            7,
        )
    };
    let mut land = make();
    let mut fall = make();
    land.damping = 0.99;
    fall.damping = 0.99;
    let terrain = floor();
    for _ in 0..180 {
        land.step(DT, G, &terrain, 1);
        fall.step(DT, G, &NoProviders, 0);
    }
    let min_y = land.pos.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let max_y = land
        .pos
        .iter()
        .map(|p| p.y)
        .fold(f32::NEG_INFINITY, f32::max);
    println!(
        "cloth floor min_y={min_y:.6} max_y={max_y:.6} no_provider_y={:.3} hash={:016x}",
        fall.pos[24].y,
        hash(&land)
    );
    assert!(
        min_y >= land.radius - 0.003,
        "provider must prevent penetration: {min_y}"
    );
    assert!(
        max_y < land.radius + 0.01,
        "sheet must settle on terrain: {max_y}"
    );
    assert!(
        fall.pos[24].y < -1.0,
        "without provider it must keep falling"
    );
    assert_eq!(hash(&land), 0x55b6_58e7_1a72_700d);
}
