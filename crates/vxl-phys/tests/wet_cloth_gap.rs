//! **落水布（布 × 液）的现状判据（缺口登记）** —— `ROUTE.md` §4 兼容矩阵里
//! 「软体/布 × 液体 = 湿布（质量 + 阻力，双向）」那一格**还没落地**（同文件 §4 的复核块已登记）。
//!
//! **现状**（本文件钉住）：布片与液体之间**没有任何通道** —— `vxl-phys-soft` 只挂
//! `core` + `vxl-phys-aero`（气动消费），液体的 `MediumField`（`fluid_medium.rs`，2a 采样侧）
//! 只被门面的**刚体**介质段消费（`world_step/medium.rs` 走 splat 场）⇒ 布落进水池后
//! **不受任何拖曳**，以接近真空自由落体的速度穿过液面、直接落到槽底。
//!
//! **目标判据**（本仓铁律：先写验收再写实现，落地后本文件**翻面**）：
//! ① **阻力**：布穿过液面（y ≈ 液面 − 5 cm）时的竖直速度应**明显慢于**真空自由落体
//!    （当前实测 ≈ −2 m/s；湿布档应显著更小）；
//! ② **质量**：吸水后布的有效质量上升 ⇒ 同样的外力下加速度更小（实现时按 `wet_ratio` 记账）；
//! ③ **双向**：液体获得布的动量（水中出现与布运动同向的流速）——需接流体的边界粒子腿（另立片）；
//! ④ 布**不得穿透槽底**（与槽底接触面高度落在容差内——这条由 `cloth × 提供者` 已有路径保证）。
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

/// 布的均值高度与均值竖直速度。
fn mean_pos_vel(sc: &ClothSheet) -> (f32, f32) {
    let n = sc.pos.len().max(1) as f32;
    let y = sc.pos.iter().map(|p| p.y).sum::<f32>() / n;
    let vy = sc.vel.iter().map(|v| v.y).sum::<f32>() / n;
    (y, vy)
}

/// **现状**：布以接近真空自由落体的速度穿过液面（液面对布零作用）。
#[test]
fn cloth_falls_through_water_at_free_fall_speed_today() {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    w.add_fluid(water(), &[v]);
    // 布：0.4×0.4（内腔 0.5 宽 ⇒ 放得下），静置水面之上 0.17 m
    let (pts, tris) = plate(2, 0.2, 1.62);
    let sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    w.add_cloth(sheet);
    assert!(w.cloth(0).is_some(), "布片应已注册");

    // 逐 tick 找"穿过液面（≈1.45）后 5 cm"那一刻的竖直速度。
    let mut vy_cross: Option<f32> = None;
    let mut y_end = 0.0f32;
    for _ in 0..180 {
        w.step();
        let Some(sc) = w.cloth(0) else {
            return;
        };
        let (y, vy) = mean_pos_vel(sc);
        y_end = y;
        if vy_cross.is_none() && y < 1.40 {
            vy_cross = Some(vy);
        }
    }
    assert!(vy_cross.is_some(), "布应已穿过液面（y < 1.40）——场景没跑成");
    let Some(vy) = vy_cross else {
        return;
    };
    // 真空自由落体从 1.62 落到 1.40（0.22 m）⇒ |vy| ≈ √(2·9.81·0.22) ≈ 2.08 m/s。
    // 现状是"液面**零作用**"⇒ 实测应贴住这个量级；湿布落地后这一行**翻面**成"明显更小"。
    assert!(
        vy < -1.5,
        "现状：穿过液面时应仍接近自由落体（≈ −2 m/s），实得 vy={vy:.3} —— \
         若这里变了，说明布×液耦合已落地/被打断，请按文件头的目标判据翻面"
    );
    // 落到槽底（内腔地板 y = 1.0）附近而不是继续下坠。
    assert!(
        y_end > 0.9,
        "布不得穿透槽底（内腔地板 y=1.0），实得 y_end={y_end:.3}"
    );
}
