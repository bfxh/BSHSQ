//! **scratch 世界级验证**（不提交）：布片落在**被地面支撑的动态盒**上——引擎真实管线里
//! `w_b>0` 且体有地方"卸力"（地面）时，布片托得住吗？质量比怎么扫？
//! 对照：同一几何盒改静态（`add_static`）。
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// 地面（大网格，y = 0）+ 一枚盒（`density = 0` 表示静态对照），盒顶面上方 0.2 m 放布片。
fn run(density: f32, ticks: usize) -> (f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let ground =
        vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, Vec3::X * 2.0, Vec3::Z * 2.0, Vec3::Y);
    w.add_mesh(ground);
    let half = Vec3::new(0.3, 0.05, 0.3);
    let start = Vec3::new(0.0, half.y + 0.02, 0.0);
    let b = if density > 0.0 {
        w.add_dynamic(Shape::Box { half }, start, Quat::IDENTITY, density) as usize
    } else {
        w.add_static(Shape::Box { half }, start, Quat::IDENTITY) as usize
    };
    // 盒先落定（动态档）——静态档无所谓
    for _ in 0..120 {
        w.step();
    }
    let box_y0 = w.bodies.position[b].y;
    // 布片（0.5 m 见方）从盒顶上方 0.2 m 落下
    let (pts, tris) = plate(4, 0.25);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += box_y0 + half.y + 0.1 + 0.2;
    }
    w.add_cloth(sheet);
    for _ in 0..ticks {
        w.step();
    }
    let c = w.cloth(0).expect("just added");
    let mean_y = c.pos.iter().map(|p| p.y).sum::<f32>() / c.pos.len() as f32;
    let min_y = c.pos.iter().map(|p| p.y).fold(f32::MAX, f32::min);
    let top = w.bodies.position[b].y + half.y;
    (mean_y, min_y, top)
}

#[test]
fn scratch_cloth_on_ground_supported_dynamic_box() {
    println!("世界级：地面 + 盒（半高 0.05，<density>=0 ⇒ 静态对照）+ 0.5 m 布片落顶");
    // 盒 0.6×0.1×0.6 = 0.036 m³ ⇒ density 50/200/800 ⇒ 1.8 / 7.2 / 28.8 kg
    for (density, label) in [
        (0.0f32, "static"),
        (50.0, "1.8kg"),
        (200.0, "7.2kg"),
        (800.0, "28.8kg"),
    ] {
        let (mean_y, min_y, top) = run(density, 600);
        let gap = mean_y - top;
        println!(
            "  {label:>8}  布片 mean_y={mean_y:+.4} min_y={min_y:+.4}  盒顶={top:+.4}  gap(mean−顶)={gap:+.4}  {}",
            if gap > -0.02 && min_y > top - 0.05 { "✅ 托住" } else { "❌ 下沉/穿" }
        );
        // 再跑长窗
        let (mean_y2, min_y2, top2) = run(density, 1800);
        let gap2 = mean_y2 - top2;
        println!(
            "    长窗 1800：mean_y={mean_y2:+.4} min_y={min_y2:+.4} 盒顶={top2:+.4} gap={gap2:+.4}  {}",
            if gap2 > -0.02 && min_y2 > top2 - 0.05 { "✅ 托住" } else { "❌ 下沉/穿" }
        );
    }
}
