//! **判据（世界级）：布片落在「被地面支撑的动态盒」上 ⇒ 应贴面托住**（长窗零漂移、质量比无关）。
//!
//! **为什么单列一条**（2026-10-01，P11 调查收口；SURVEY §8.4.52）：
//! - 软体侧自扮引擎的动态代理判据（`cloth_reaction.rs`）是**盒压布**（布片四周钉住）；本条补的是
//!   **反过来的载荷路径**——布片压在体顶，且体在**引擎真实管线**里是动态但有支撑的（地面 = 网格
//!   提供者）。这正是 "held 体" 在引擎里的真实表示（对照：休眠/静态体进软体域时 `inv_mass=0`，
//!   §8.4.27 ⇒ 走静态路径）。
//! - 实测（P11）：把动态体**冻结但保留 `w_b > 0`**（自扮夹具）会让布片穿落——那是**夹具语义**、
//!   不是耦合缺陷：两体口径假定体自由，"冻结" = 算两体响应再把体那份丢掉。给地面支撑后全档托住。
//!   另两个反事实（钳位改用子步初速度 / 钳位对 `dv` 全盲）都不改变穿落 ⇒ GS 自我限幅是承重的。
//!
//! **判据（机器无关：无计时、无随机）**：静态对照 + 动态 1.8/7.2/28.8 kg（`w_b` 跨 16×）：
//! ① 贴面托住：`gap = mean_y − 盒顶 ∈ [0, 0.03]`；② 长窗零漂移：600 与 1800 tick 的 `mean_y`
//! 差 ≤ 5e-3（mm/tick 级的漏会远超）；③ 动态盒稳定坐在地面（底 ≈ 0）。
//! （显式导入，不用 `use vxl_phys::*`——glob-gate：新文件零通配。）
use vxl_phys::{Quat, Shape, World};
use vxl_phys_core::{PhysConfig, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 平铺网格（n×n 格、跨度 ±size、y = 0 平面；与 `cloth_reaction_scene.rs` 同款）。
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

fn cloth_mean_y(w: &World) -> f32 {
    match w.cloth(0) {
        Some(c) => c.pos.iter().map(|p| p.y).sum::<f32>() / c.pos.len().max(1) as f32,
        None => f32::NAN, // 不可能注册失败；由下面的断言兜底
    }
}

/// 地面（网格，y = 0）+ 一枚盒（`density = 0` ⇒ 静态对照，其余为动态、坐在地面上）；
/// 布片（0.5 m 见方）从盒顶上方 0.3 m 落下。
/// 返回（600 tick 的 mean_y、1800 tick 的 mean_y、盒顶 y、盒底 y）。
fn run(density: f32) -> (f32, f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let ground =
        vxl_phys_terrain::mesh::TriMesh::quad(Vec3::ZERO, Vec3::X * 2.0, Vec3::Z * 2.0, Vec3::Y);
    w.add_mesh(ground);
    let half = Vec3::new(0.3, 0.05, 0.3);
    // 动态档从静置高度上方 2 cm 出生（落定后底 ≈ 0）；静态档直接落在静置高度。
    let start = Vec3::new(0.0, half.y + if density > 0.0 { 0.02 } else { 0.0 }, 0.0);
    let b = if density > 0.0 {
        w.add_dynamic(Shape::Box { half }, start, Quat::IDENTITY, density) as usize
    } else {
        w.add_static(Shape::Box { half }, start, Quat::IDENTITY) as usize
    };
    for _ in 0..120 {
        w.step(); // 盒落定（静态档无变化）
    }
    let (pts, tris) = plate(4, 0.25);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for p in &mut sheet.pos {
        p.y += w.bodies.position[b].y + half.y + 0.3;
    }
    w.add_cloth(sheet);
    let mut mean_600 = 0.0f32;
    for t in 1..=1800 {
        w.step();
        if t == 600 {
            mean_600 = cloth_mean_y(&w);
        }
    }
    let top = w.bodies.position[b].y + half.y;
    (
        mean_600,
        cloth_mean_y(&w),
        top,
        w.bodies.position[b].y - half.y,
    )
}

#[test]
fn cloth_is_held_by_a_ground_supported_dynamic_box() {
    // 盒 0.6×0.1×0.6 = 0.036 m³ ⇒ density 50/200/800 ⇒ 1.8 / 7.2 / 28.8 kg。
    for (density, label) in [
        (0.0f32, "static"),
        (50.0, "1.8kg"),
        (200.0, "7.2kg"),
        (800.0, "28.8kg"),
    ] {
        let (m600, m1800, top, bottom) = run(density);
        let gap = m1800 - top;
        let drift = (m1800 - m600).abs();
        println!("{label:>8}: gap={gap:+.4} drift(600→1800)={drift:.2e} 盒底={bottom:+.4}");
        assert!(m1800.is_finite(), "{label}: 出现非有限值");
        assert!(
            (0.0..=0.03).contains(&gap),
            "{label}: 布片应贴面托住（gap={gap:+.4}）——红了说明布片下沉/穿透"
        );
        assert!(
            drift <= 5e-3,
            "{label}: 长窗应零漂移（drift={drift:.2e}）——红了说明慢漏"
        );
        assert!(
            bottom.abs() < 5e-3,
            "{label}: 盒应稳定坐在地面（底={bottom:+.4}）"
        );
    }
}
