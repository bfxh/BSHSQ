//! **面元气动判据**（T4，`PLAN-triangle-first-class.md` 判据 ③）：`vxl-phys-aero` 从
//! 28 行纯配置变成**消费方**——`World::set_aero` 开启后，`aero_pass` 逐子步对**每张三角面**
//! 施加 Bridson 线化气动力 `F = ½ρ·Cd·A·u·|u|`（`u` = 风 − 面元速度，**全相对速度**）。
//!
//! 判据三条（口径见 `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md` §10）：
//! ① **静力读数 = 解析式**：静态薄板 ⊥ 风，面元速度恒 0 ⇒ 读数 = `½ρv²A·Cd` **精确**
//!    （逐面离散和 = 连续积分，同薄壳质量那条的推理）；仪器与被测同源
//!    （`aero_force` 读的就是施加的那份，不是另算一份）；
//! ② **零风金丝雀**：`u = 0 ⇒ F = 0` **逐位精确**（`0.5·ρ·|0|·0 = 0`）；
//! ③ **行为**：零重力 + 顺风漂移（松界）+ 零风对照不漂。
//!
//! **Cd 取值来源**：`AeroConfig::default()` 沿用 crate 既有默认 `1.0`（平板量级；
//! Hoerner《Fluid-Dynamic Drag》方板 ≈ 1.17、圆盘 ≈ 1.12——判据用默认值自洽即可，
//! 换文献值不改判据形状）。

use vxl_phys::*;
use vxl_phys_aero::AeroConfig;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 **y = 0 平面**；与支持矩阵判据同款三角化）。
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

/// **① 静力读数与解析对拍**：静态薄板（1 m²）+ 风 `(0,−5,0)`（⊥ 板）⇒
/// 读数 = `½·ρ·Cd·A·v² = ½·1.225·1.0·1·25 ≈ 15.3125 N`，方向 = 风向（−Y）。
fn static_sheet_with_wind(wind: [f32; 3]) -> (World, u32) {
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts, tris);
    let half = w.trimesh_half_extents(mesh);
    let b = w.add_static(Shape::TriMesh { mesh, half }, Vec3::ZERO, Quat::IDENTITY);
    let cfg = AeroConfig {
        wind,
        ..AeroConfig::default()
    };
    w.set_aero(cfg);
    (w, b)
}

#[test]
fn static_force_matches_analytic_perpendicular_plate() {
    let (mut w, b) = static_sheet_with_wind([0.0, -5.0, 0.0]);
    w.step();
    let got = w.aero_force(b);
    let want = 0.5 * 1.225 * 1.0 * 1.0 * 25.0; // ½ρ·Cd·A·v²
    println!("[判据③] 静板读数 = {got:?}（解析 |F| = {want:.4} N，方向 −Y）");
    assert!(
        (got.length() - want).abs() / want < 1e-4,
        "静板气动合力 {:#?} 应 = ½ρv²A·Cd = {want:.4}（相对误差须 < 1e-4）",
        got
    );
    assert!(
        got.x.abs() < 1e-5 && got.z.abs() < 1e-5 && got.y < 0.0,
        "方向应沿风向（−Y），实得 {got:?}"
    );
    // 面对称 ⇒ 净力矩 ≈ 0（对称面对 force 求和的力臂成对抵消；f32 非结合 ⇒ 留 1e-3 容差）。
    let tau = w.aero_torque(b);
    assert!(
        tau.length() < 1e-3,
        "对称薄板 + 均匀风 ⇒ 净力矩应 ≈ 0，实得 {tau:?}"
    );
}

/// **② 零风金丝雀**：`u = 0 ⇒ F = 0` **逐位精确**（`0.5·ρ·|0|·0 = 0.0`，无累积误差）。
#[test]
fn zero_wind_gives_exact_zero_force() {
    let (mut w, b) = static_sheet_with_wind([0.0, 0.0, 0.0]);
    w.step();
    let f = w.aero_force(b);
    let tau = w.aero_torque(b);
    println!("[金丝雀] 零风：force = {f:?}，torque = {tau:?}");
    assert!(
        f == Vec3::ZERO && tau == Vec3::ZERO,
        "零风必须给**逐位零**（实得 force {f:?} / torque {tau:?}）⇒ 出现非零说明公式或求和被改"
    );
}

/// **③ 行为**：零重力 + 顺风漂移——水平薄板、风 `+X` 5 m/s：全相对速度口径下面元受**顺风**
/// 切向力 ⇒ 板沿 +X 漂移（连续解析 ≈ 2.2 m @ 2 s；Euler 离散 ⇒ 松界 [1, 3.5]）；
/// **零风对照**：同场景不漂（|x| < 1e-4）。
#[test]
fn sheet_drifts_downwind_and_zero_wind_does_not() {
    let cfg = || PhysConfig {
        gravity: Vec3::ZERO, // 隔离气动（重力会让板下落、混入读数）
        ..PhysConfig::default()
    };
    let mut drift = None;
    for wind in [[5.0f32, 0.0, 0.0], [0.0, 0.0, 0.0]] {
        let (pts, tris) = plate(2, 0.5);
        let mut w = World::new(cfg());
        let mesh = w.add_trimesh(pts, tris);
        let b = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 1000.0, 0.01);
        let c = AeroConfig {
            wind,
            ..AeroConfig::default()
        };
        w.set_aero(c);
        for _ in 0..120 {
            w.step();
        }
        let x = w.bodies.position[b as usize].x;
        println!("[判据③-漂移] wind={wind:?} ⇒ x = {x:+.4}");
        if wind[0] > 0.0 {
            drift = Some(x);
        } else {
            assert!(x.abs() < 1e-4, "零风对照不应漂移（x = {x:+.6}）");
        }
    }
    let x = drift.expect("顺风场景必跑");
    assert!(
        (1.0..3.5).contains(&x),
        "顺风漂移应落在连续解析 ≈ 2.2 m 的离散邻域（实得 {x:+.3}；符号/量级不对 ⇒ 公式或口径被改）"
    );
}
