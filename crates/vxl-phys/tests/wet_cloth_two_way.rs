//! **落水布的双向那半（布 → 液）**的门面判据（2026-10-05）：牛顿第三定律。
//!
//! 场景（重力关掉，排除落体干扰）：水块静置，一块**水平布以 `+x` 初速穿过它**⇒ 水获得 `+x` 动量。
//! **定量判据**：重力关 + 无接触 ⇒ `Δp布 + Δp水 ≈ 0`（实测残差 1.2e-5 / Δp 0.806；干对照恒 0）
//! —— 只推水不拖布、或反作用取"tick 末重算"的近似，这条都会红。
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
            tris.push([a, a + n as u32 + 1, a + 1]);
            tris.push([a + 1, a + n as u32 + 1, a + n as u32 + 2]);
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

/// 造"以 `+1 m/s` 穿过水"的那张布；返回 `(布, 初始总 x 动量)`。
fn sheet() -> (ClothSheet, f32) {
    let (pts, tris) = plate(2, 0.15, 0.0);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for v in s.vel.iter_mut() {
        *v = Vec3::new(1.0, 0.0, 0.0);
    }
    let p0 = s.vel.iter().zip(&s.mass).map(|(v, m)| v.x * m).sum();
    (s, p0)
}

/// `(水的总 x 动量, 布的总 x 动量)`（各自 `Σ m·vx`；水粒同质 ⇒ 总动量 = `m·Σvx`）。
fn momentum_x(w: &World) -> (f32, f32) {
    let pw = match w.fluids().first() {
        Some(s) => s.0.velocities().iter().map(|v| v.x).sum::<f32>() * s.0.particle_mass(),
        None => f32::NAN,
    };
    let pc = match w.cloth(0) {
        Some(c) => c.vel.iter().zip(&c.mass).map(|(v, m)| v.x * m).sum(),
        None => f32::NAN,
    };
    (pw, pc)
}

/// 同一初始水块、同一窗口；`with_cloth` 决定布在不在（其余逐字相同）。返回
/// `(水的 x 动量, 布的 x 动量, 布的初始 x 动量)`；无布时后两项为 `NAN`。
fn run(with_cloth: bool, ticks: usize) -> (f32, f32, f32) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    w.add_fluid(water(), &[]);
    let mut pc0 = f32::NAN;
    if with_cloth {
        let (s, p0) = sheet();
        pc0 = p0;
        w.add_cloth(s);
    }
    for _ in 0..ticks {
        w.step();
    }
    let (pw, pc) = momentum_x(&w);
    (pw, pc, pc0)
}

/// 布沿 `+x` 拖水：两半必须同时成立，且**动量账闭合**。
#[test]
fn dragging_cloth_pushes_the_water_along_it() {
    let (px_dry, _, _) = run(false, 15);
    let (px_wet, pc_wet, pc0) = run(true, 15);
    let (d_cloth, residual) = (pc_wet - pc0, pc_wet - pc0 + px_wet);
    println!("control Σm·vx={px_dry:.3e} | wet Σm·vx={px_wet:.6e} | Δp布={d_cloth:.6e} | 残差={residual:.3e}");
    assert!(
        px_dry.abs() < 1e-4,
        "无布对照不该有横向驱动源，实得 {px_dry:.3e}"
    );
    assert!(
        px_wet > 0.5,
        "布沿 +x 拖水 ⇒ 水应获得可观动量，实得 {px_wet:.3e}"
    );
    assert!(
        residual.abs() <= 1e-4 * d_cloth.abs(),
        "牛顿第三定律：Δp布 + Δp水 应为 0，实得 {residual:.3e}（Δp布={d_cloth:.3e}）"
    );
}
