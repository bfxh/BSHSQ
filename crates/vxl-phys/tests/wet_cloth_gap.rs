//! **落水布（布 × 液）的端到端判据** —— `ROUTE.md` §4 兼容矩阵
//! 「软体/布 × 液体 = 湿布（质量 + 阻力，双向）」那一格。
//!
//! **本片（2026-10-05）落地的是「阻力」那半**（单向：介质 → 布）：门面在水推进后按**面心**
//! 采样 `FluidSystem` 的 `MediumField`（2a 采样侧）填进 `cloth.medium`，`predict` 里由
//! `cloth_medium::inject` 施加 **Bridson 线化阻力** `F = ½·ρ·Cd·A·u·|u|`（与 `cloth_aero` 同式）。
//! ⚠️ **力必须在 `predict` 内**：XPBD 是位置式，晚于 `prev = pos` 的速度级注入会被
//! `write_back` 的 `(pos − prev)/h` 重算吞掉（2026-10-05 试刀负结果，见 `ROUTE.md` §4）。
//!
//! **判据**（同一条布、同一窗口，唯一变量 = 场内有无水）：
//! ① **无液对照**：落到槽底、穿液面处接近自由落体（`|vy| > 1.5`）；
//! ② **有液**：90 tick 处的均值高度**明显高于**对照（被水拖住），仍整体向下、且**不穿透槽底**。
//!
//! **仍缺（登记，别当已做）**：③ **湿质量**（吸水后有效质量上升）；④ **双向**（水获得布的动量 ——
//! 要接流体的边界粒子腿）；⑤ 介质采样**每 tick 一次**（子步内复用），per-substep 采样属后续片。
//!
//! ⚠️ 场景**铸装**（`PLAN-0.3.md` §4.2）：水块按沉降后几何直接就位，避免"带落差入盆"的顶心喷泉。
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 `y = y0` 平面）。
fn plate(n: usize, size: f32, y0: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, y0, -size + s * iz as f32));
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

/// 水槽：5×5 格（外沿 2.5 m）地板 + **中心一格**围堰 ⇒ 内腔 0.5×0.5 m（同 `fluid_boundary.rs`）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// 铸装水块 `[8,8,8]@0.05`（占据 y ∈ [1.05, 1.45]，液面 ≈ 1.45）。
fn water() -> FluidSystem {
    FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

/// 布的均值高度。
fn mean_y(sc: &ClothSheet) -> f32 {
    let n = sc.pos.len().max(1) as f32;
    sc.pos.iter().map(|p| p.y).sum::<f32>() / n
}

/// 同一条布、同一个落下窗口；`with_fluid` 决定场内有没有水（其余逐字相同）。
/// 返回 `(穿液面处的均值 vy, **落地 tick**（首次 y ≤ 1.02）, 180 tick 处的均值 y)`。
fn drop_cloth(with_fluid: bool) -> (f32, usize, f32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    if with_fluid {
        w.add_fluid(water(), &[v]);
    }
    // 布：0.4×0.4（内腔 0.5 宽 ⇒ 放得下），静置水面之上 0.17 m
    let (pts, tris) = plate(2, 0.2, 1.62);
    let sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    w.add_cloth(sheet);
    assert!(w.cloth(0).is_some(), "布片应已注册");
    let mut vy_cross: Option<f32> = None;
    let mut landing = usize::MAX;
    let mut y_end = 0.0f32;
    for tick in 0..180 {
        w.step();
        let Some(sc) = w.cloth(0) else {
            return (0.0, usize::MAX, 0.0);
        };
        let y = mean_y(sc);
        let vy = sc.vel.iter().map(|v| v.y).sum::<f32>() / sc.vel.len().max(1) as f32;
        y_end = y;
        if landing == usize::MAX && y <= 1.02 {
            landing = tick;
        }
        if vy_cross.is_none() && y < 1.40 {
            vy_cross = Some(vy);
        }
    }
    (vy_cross.unwrap_or(0.0), landing, y_end)
}

/// **同一条布、同一窗口，唯一变量 = 场内有无水**：干对照自由落体、湿布被拖住（落地明显更晚），
/// 两者都不得穿透槽底。
#[test]
fn water_drag_holds_the_cloth_back_versus_the_dry_control() {
    let (vy_dry, landing_dry, y_end_dry) = drop_cloth(false);
    let (vy_wet, landing_wet, y_end_wet) = drop_cloth(true);
    // 真空自由落体从 1.62 落到 1.40（0.22 m）⇒ |vy| ≈ √(2·9.81·0.22) ≈ 2.08 m/s。
    assert!(vy_dry < -1.5, "无液对照应接近自由落体，实得 vy={vy_dry:.3}");
    assert!(landing_dry < 100, "无液对照应早落地，实得 {landing_dry}");
    assert!(
        landing_wet > landing_dry + 8,
        "有液应被拖住：落地 tick wet={landing_wet} vs dry={landing_dry}"
    );
    assert!(
        vy_wet < 0.0,
        "湿布仍应整体向下（无浮力那半）：vy={vy_wet:.3}"
    );
    assert!(
        y_end_dry > 0.9 && y_end_wet > 0.9,
        "两者都不得穿透槽底：dry={y_end_dry:.3} wet={y_end_wet:.3}"
    );
}
