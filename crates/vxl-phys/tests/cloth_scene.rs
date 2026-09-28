//! **门面级布料接线冒烟判据**（软体切片 1）：`World::add_cloth` + `cloth_pass`（每 tick 一次）
//! 把软体域的布片推进接进 `step()` 管线。深判据在软体侧 `cloth_minimal.rs`；
//! 这里只钉**接线**：不注册布片 ⇒ 逐位不变（默认档）；注册 ⇒ 与软体侧单跑同结果（确定性）。

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

/// 注册布片（两角钉住）⇒ 门面 `step()` 推进 ⇒ 与软体侧同场景**同末态**（哈希级确定性）。
#[test]
fn cloth_in_world_matches_standalone_and_empty_world_is_unchanged() {
    let build = || {
        let (pts, tris) = plate(2, 0.5);
        let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
        sheet.set_pinned(0, true);
        sheet.set_pinned(2, true);
        sheet
    };
    // A：门面（World 里只有一个布片 ⇒ 无接触对 ⇒ 应与单跑逐位同）。
    let mut w = World::new(PhysConfig::default());
    let cloth = build();
    let id = w.add_cloth(cloth);
    for _ in 0..600 {
        w.step();
    }
    // B：软体侧单跑（同一推进口径：**直接读 `PhysConfig::default()` 的 dt/gravity**，
    //    别手抄数值——默认重力是 −9.81 不是 −9.8，手抄差 0.01 会在 2 s 悬垂里放大成 0.68 m）。
    //    本场景没注册提供者 ⇒ 两侧都走"零接触"（切片 2 的 `provider_count = 0` 路径）。
    let cfg = PhysConfig::default();
    let mut sheet = build();
    for _ in 0..600 {
        sheet.step(cfg.dt, cfg.gravity, &vxl_phys_core::interop::NoProviders, 0);
    }
    let a = &w.cloth(id).expect("just added").pos;
    let b = &sheet.pos;
    let mut worst = 0.0f32;
    for (pa, pb) in a.iter().zip(b.iter()) {
        worst = worst.max((*pa - *pb).length());
    }
    println!(
        "[冒烟] 门面 vs 单跑最大差 = {worst:.3e}（应 ≈ 0）；末态中心 y = {:+.4}",
        a[4].y
    );
    assert!(
        worst < 1e-6,
        "门面推进应与软体侧单跑一致（最大差 {worst:.3e}）⇒ cloth_pass 的 dt/gravity 口径接错"
    );
    assert!(a[4].y < -0.1, "布片应在门面里下垂（y = {:+.4}）", a[4].y);
}
