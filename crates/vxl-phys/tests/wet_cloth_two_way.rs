//! **落水布的双向那半（布 → 液）**的门面判据（2026-10-05）：牛顿第三定律。
//!
//! 场景（重力关掉，排除落体干扰）：水块静置，一块**水平布以 `+x` 初速穿过它**。
//! 布受的介质阻力沿 `−x` ⇒ 反作用 `−F·dt` 沉积进流场 ⇒ **水的总 `x` 动量应为正**。
//!
//! **判据**：① 无布对照 ⇒ 水的 `x` 动量**保持 0**（没有横向驱动源）；② 有布 ⇒ 显著为正；
//! ③ 有布时布自己被拖慢（`|vx|` 小于初速）—— 两半必须同时成立（只推水不拖布 = 凭空造动量）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 水平布 `n×n` 格（跨度 `±size`、落在 `y = y0` 平面）。
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

/// 水块 `[8,8,8]@0.05`（跨 ±0.175）、**零重力**、初始静止。
fn water() -> FluidSystem {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    FluidSystem::new(cfg, Vec3::splat(-0.2), [8, 8, 8], 0.05)
}

/// 水的总 `x` 动量 `Σ m·vx`。
fn water_momentum_x(w: &World) -> f32 {
    let slot = match w.fluids().first() {
        Some(s) => s,
        None => return f32::NAN,
    };
    slot.0.velocities().iter().map(|v| v.x).sum::<f32>() * slot.0.particle_mass()
}

/// 布的平均 `vx`。
fn cloth_vx(w: &World) -> f32 {
    match w.cloth(0) {
        Some(c) => c.vel.iter().map(|v| v.x).sum::<f32>() / c.vel.len().max(1) as f32,
        None => f32::NAN,
    }
}

/// 同一初始水块、同一窗口；`with_cloth` 决定布在不在（其余逐字相同）。跑 `ticks` 后返回
/// `(水的 x 动量, 布的均值 vx)`。
fn run(with_cloth: bool, ticks: usize) -> (f32, f32) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    w.add_fluid(water(), &[]);
    if with_cloth {
        let (pts, tris) = plate(2, 0.15, 0.0);
        let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
        for v in sheet.vel.iter_mut() {
            *v = Vec3::new(1.0, 0.0, 0.0);
        }
        w.add_cloth(sheet);
    }
    for _ in 0..ticks {
        w.step();
    }
    (water_momentum_x(&w), cloth_vx(&w))
}

#[test]
fn dragging_cloth_pushes_the_water_along_it() {
    let (px_dry, _) = run(false, 15);
    let (px_wet, vx_wet) = run(true, 15);
    println!("control Σm·vx = {px_dry:.6e} | wet Σm·vx = {px_wet:.6e} | cloth vx = {vx_wet:.4}");
    assert!(
        px_dry.abs() < 1e-4,
        "无布对照不该有横向驱动源，实得 Σm·vx={px_dry:.3e}"
    );
    assert!(
        px_wet > 10.0 * px_dry.abs().max(1e-6),
        "布沿 +x 拖水 ⇒ 水应获得 +x 动量，实得 {px_wet:.3e}"
    );
    assert!(
        vx_wet < 1.0,
        "同一时间布自己必须被拖慢（否则是凭空造动量）：vx={vx_wet:.4}"
    );
}
