//! **旗飘（布 × 风）贯通场景** —— `ROUTE.md` §4「必须有贯通示例」里那一格。
//!
//! **接线**：`World::set_aero(cfg)` 把同一份风配置**下发**到每张布（`cloth.aero`），
//! 布的 `predict` 逐子步按面心施加 Bridson 线化面元力。此前 `cloth.aero` 只能由用户手动填，
//! 门面不接 ⇒ 本判据钉住"门面一开风，旗子就飘"。
//!
//! **场景**：竖直旗面（面法向 = `x`）、**顶边钉在横杆上**、重力 `−y`。
//! 判据：① **无风对照**（不调 `set_aero`）⇒ 自由节点的平均 `x` 恒 ≈ 0 且 `aero.enabled == false`
//! （零成本关档）；② **有风**（`wind = +x`）⇒ 旗面被吹向 `+x`；③ 开启后 `aero.enabled == true`。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_aero::AeroConfig;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 竖直旗面（`n×n` 格、宽 `2·size`、挂在 `y = 0` 的横杆上、**面法向 = `x`**）。
/// 返回 `(点, 三角, 顶边点索引)`。
fn flag(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>, Vec<usize>) {
    let (mut pts, mut tris, mut rail) = (Vec::new(), Vec::new(), Vec::new());
    let s = 2.0 * size / n as f32;
    for iy in 0..=n {
        for iz in 0..=n {
            pts.push(Vec3::new(0.0, -(iy as f32) * s, -size + s * iz as f32));
            if iy == 0 {
                rail.push(pts.len() - 1);
            }
        }
    }
    for iy in 0..n as u32 {
        for iz in 0..n as u32 {
            let a = iy * (n as u32 + 1) + iz;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris, rail)
}

/// 自由节点（非钉扎）的平均 `x`。
fn free_mean_x(sc: &ClothSheet) -> (f32, usize) {
    let (mut sum, mut n) = (0.0f32, 0usize);
    for (i, p) in sc.pos.iter().enumerate() {
        if sc.inv_mass[i] != 0.0 {
            sum += p.x;
            n += 1;
        }
    }
    (sum / n.max(1) as f32, n)
}

/// 同一条旗、同一窗口；`wind` 为 `None` ⇒ **不调** `set_aero`（走零成本关档）。
/// 返回 `(自由节点平均 x, 布的气动开关)`。
fn run(wind: Option<[f32; 3]>, ticks: usize) -> (f32, bool) {
    let mut w = World::new(PhysConfig::default());
    let (pts, tris, rail) = flag(3, 0.3);
    let mut sc = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Standard);
    for i in rail {
        sc.set_pinned(i, true);
    }
    w.add_cloth(sc);
    if let Some(wind) = wind {
        w.set_aero(AeroConfig {
            wind,
            ..AeroConfig::default()
        });
    }
    for _ in 0..ticks {
        w.step();
    }
    match w.cloth(0) {
        Some(c) => {
            let (m, _) = free_mean_x(c);
            (m, c.aero.enabled)
        }
        None => (f32::NAN, false),
    }
}

/// 门面一开风，旗子就飘；不开风 ⇒ 一字不动（`cloth.aero.enabled` 保持 `false`）。
#[test]
fn wind_blows_the_flag_off_the_mast() {
    let (x_calm, on_calm) = run(None, 60);
    let (x_wind, on_wind) = run(Some([20.0, 0.0, 0.0]), 60);
    println!("无风 mean_x = {x_calm:.6e}（aero={on_calm}）| 有风 mean_x = {x_wind:.6e}（aero={on_wind}）");
    assert!(!on_calm, "没调 `set_aero` ⇒ 布的气动必须是关档（零成本）");
    assert!(on_wind, "`set_aero` 应把风配置下发到布上");
    assert!(
        x_calm.abs() < 1e-3,
        "无风对照不该有横向位移，实得 {x_calm:.3e}"
    );
    assert!(
        x_wind > 0.05,
        "风沿 +x ⇒ 自由节点应被吹向 +x，实得 {x_wind:.3e}"
    );
}
