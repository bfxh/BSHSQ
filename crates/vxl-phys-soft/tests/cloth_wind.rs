use vxl_phys_core::{interop::NoProviders, Vec3};
use vxl_phys_soft::{ClothSheet, ClothWind};

const H: f32 = 1.0 / 60.0;

fn sheet(wind: Option<ClothWind>) -> ClothSheet {
    let mut sheet = ClothSheet::grid(Vec3::ZERO, Vec3::X, Vec3::Z, 2, 2);
    sheet.wind = wind;
    sheet.substeps = 1;
    sheet
}

fn step(sheet: &mut ClothSheet) {
    sheet.step(H, Vec3::ZERO, &NoProviders, 0);
}

fn wind(velocity: Vec3) -> ClothWind {
    ClothWind {
        velocity,
        density: 2.0,
        drag: 1.0,
    }
}

#[test]
fn wind_loads_triangle_normal_but_not_tangent() {
    let mut calm = sheet(None);
    let mut tangential = sheet(Some(wind(Vec3::X * 3.0)));
    let mut front = sheet(Some(wind(Vec3::Y * 3.0)));
    let mut back = sheet(Some(wind(Vec3::Y * -3.0)));
    step(&mut calm);
    step(&mut tangential);
    step(&mut front);
    step(&mut back);
    assert_eq!(calm.pos, tangential.pos);
    assert_eq!(calm.pos, [Vec3::ZERO, Vec3::X, Vec3::Z, Vec3::X + Vec3::Z]);
    let mean_v = front.vel.iter().map(|v| v.y).sum::<f32>() / 4.0;
    println!(
        "wind mean_v={mean_v:.7} front_y={:?}",
        front.pos.iter().map(|p| p.y).collect::<Vec<_>>()
    );
    assert!((mean_v - 0.75 * H).abs() < 1e-4);
    for (f, b) in front.pos.iter().zip(&back.pos) {
        assert!(f.y > 0.0 && b.y < 0.0);
        assert!((f.y + b.y).abs() < 1e-6);
    }
}

#[test]
fn pinned_nodes_ignore_wind_and_zero_drag_retains_default_path() {
    let mut calm = sheet(None);
    let mut zero = sheet(Some(ClothWind {
        velocity: Vec3::Y * 4.0,
        density: 2.0,
        drag: 0.0,
    }));
    calm.set_pinned(0, true);
    zero.set_pinned(0, true);
    for _ in 0..10 {
        step(&mut calm);
        step(&mut zero);
    }
    assert_eq!(calm.pos, zero.pos);
    assert_eq!(zero.pos[0], Vec3::ZERO);
    let mut gust = sheet(Some(wind(Vec3::Y * 3.0)));
    gust.set_pinned(0, true);
    for _ in 0..10 {
        step(&mut gust);
    }
    assert_eq!(gust.pos[0], Vec3::ZERO);
    assert!(gust.pos[3].y > 0.0);
}

#[test]
fn wind_scales_with_density_and_ignores_degenerate_faces() {
    let mut base = sheet(Some(wind(Vec3::Y * 3.0)));
    let mut double = sheet(Some(ClothWind {
        density: 4.0,
        ..wind(Vec3::Y * 3.0)
    }));
    step(&mut base);
    step(&mut double);
    let base_v = base.vel.iter().map(|v| v.y).sum::<f32>();
    let double_v = double.vel.iter().map(|v| v.y).sum::<f32>();
    assert!((double_v - 2.0 * base_v).abs() < 1e-4);
    let mut folded = sheet(Some(wind(Vec3::Y * 3.0)));
    folded.pos.fill(Vec3::ZERO);
    step(&mut folded);
    assert!(folded.pos.iter().all(|p| p.is_finite()));
}
