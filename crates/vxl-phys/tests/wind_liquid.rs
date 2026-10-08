//! **风 × 液（表面驱动）门面场景**（`ROUTE.md` §4 那一格的第一片，2026-10-08）。
//!
//! 场景：零重力 8³ 水块（`set_aero` 显式开启）+ 60Hz 步进。判据（A/B + 组间对照，全部同场景
//! 同 tick 数；组别按**起始密度**分：`ρ < 0.9·ρ0` = 自由表面）：
//!
//! ① **顺风 vs 零风**：顺风档（`wind = +4x`）自由表面平均 Δx 明显为正，零风档必须≈0
//!    （金丝雀：测的是风，不是别的漂移）；
//! ② **表面领先内部**：顺风档 `表面均值 / 内部均值 ≥ 1.25`（实测 t=20：0.010932 / 0.006660
//!    = 1.64；t=60 掉到 1.31 ⇒ 取 20 tick 窗口）。
//!    ⚠️ 内部**也会**被带动 —— 那是压力/黏性把动量传下去（真实物理）；"内部不直接受风"由
//!    `crates/vxl-phys-fluid/tests/wind_surface.rs` 的**逐位**判据守。
//! ③ 关档（不 `set_aero`）⇒ 默认档逐位不变由 `determinism`/金样门守（本文件不重复跑）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys::{PhysConfig, Vec3, World};
use vxl_phys_aero::AeroConfig;
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// 零重力 8³ 水块（没有重力/边界 ⇒ 不引入别的位移源，只留表面驱动这一个自由度）。
fn scene(wind: [f32; 3]) -> World {
    let mut w = World::new(PhysConfig::default());
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    let sys = FluidSystem::new(cfg, Vec3::splat(-0.2), [8, 8, 8], 0.05);
    w.add_fluid(sys, &[]);
    w.set_aero(AeroConfig {
        air_density: 1.225,
        drag_coefficient: 1.0,
        lift_slope: 5.0,
        wind,
    });
    w
}

/// `settle` tick 后记基准（表面/内部按密度分组），再跑 `ticks` tick ⇒ `(表面均值 Δx, 内部均值 Δx)`。
fn run(wind: [f32; 3], settle: usize, ticks: usize) -> (f32, f32) {
    let mut w = scene(wind);
    for _ in 0..settle {
        w.step();
    }
    let (pos0, dens0, surface) = {
        let f = &w.fluids()[0].0;
        (
            f.positions().to_vec(),
            f.densities().to_vec(),
            0.9 * f.config().rest_density, // 与 `SurfaceDrag::default().surface_ratio` 同口径
        )
    };
    for _ in 0..ticks {
        w.step();
    }
    let f = &w.fluids()[0].0;
    let (mut surf, mut inner, mut ns, mut ni) = (0.0f32, 0.0f32, 0usize, 0usize);
    for i in 0..f.len() {
        let dx = f.positions()[i].x - pos0[i].x;
        if dens0[i] < surface {
            surf += dx;
            ns += 1;
        } else {
            inner += dx;
            ni += 1;
        }
    }
    assert!(
        ns > 0 && ni > 0,
        "两组都必须非空（场景分辨力金丝雀）：ns={ns} ni={ni}"
    );
    (surf / ns as f32, inner / ni as f32)
}

#[test]
fn wind_drives_the_liquid_surface() {
    let (surf_still, _) = run([0.0, 0.0, 0.0], 5, 20);
    let (surf_wind, inner_wind) = run([4.0, 0.0, 0.0], 5, 20);
    assert!(
        surf_still.abs() < 1e-4,
        "零风档不许漂移（金丝雀）：{surf_still}"
    );
    assert!(
        surf_wind > 0.005,
        "顺风档的表面必须被推着走：{surf_wind}（实测 0.0109）"
    );
    assert!(
        surf_wind > 1.25 * inner_wind,
        "表面必须领先内部（表面驱动 ≠ 整体拖拽）：surf={surf_wind} inner={inner_wind}"
    );
}
