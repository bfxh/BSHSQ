//! **液面表面驱动判据**（`vxl_phys_fluid::fluid_access::wind::surface_drag`，2026-10-08）。
//!
//! 判据链：
//! ① **单粒解析对拍**：孤立粒子必是自由表面 ⇒ 从零速度出发，`Δv = ½ρ_air·Cd·A·|u|·u·dt/m`
//!    （面积 = `spacing²`，`u = v_air`），横向分量**逐位为 0**；返回值 = 净注入动量；
//! ② **表面 / 内部**：8³ 水块里"最空"的粒子被驱动、"最实"（`ρ ≥ ρ0`）的粒子**逐位不动**；
//! ③ **反作用**：介质侧吸收量 = −流体侧净注入动量（逐位）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{MediumField, MediumSample, NoProviders};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::fluid_access::wind::{surface_drag, SurfaceDrag};
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// 均匀空气夹具（判据里密度/风速都是显式常量，不依赖 aero crate）。
struct Air {
    v: Vec3,
    rho: f32,
    absorbed: Vec3,
}

impl MediumField for Air {
    fn sample(&self, _x: Vec3) -> MediumSample {
        MediumSample {
            density: self.rho,
            velocity: self.v,
            viscosity: 0.0,
            temperature: 0.0,
            occupied: 1.0,
        }
    }
    fn deposit(&mut self, _x: Vec3, momentum: Vec3, _mass: f32, _pressure_work: f32) {
        self.absorbed += momentum;
    }
}

fn sys(dims: [usize; 3], spacing: f32) -> FluidSystem {
    FluidSystem::new(FluidConfig::default(), Vec3::splat(-0.2), dims, spacing)
}

#[test]
fn isolated_particle_gains_the_analytic_impulse() {
    let mut f = sys([1, 1, 1], 0.05);
    f.step(1.0 / 60.0, &NoProviders); // 让密度快照非零（单粒必是密度亏 = 自由表面）
    assert_eq!(f.len(), 1);
    assert!(
        f.densities()[0] < f.config().rest_density,
        "孤立粒子必须落在自由表面一侧：ρ={}",
        f.densities()[0]
    );
    f.set_velocities(&[Vec3::ZERO]);

    let mut air = Air {
        v: Vec3::new(2.0, 0.0, 0.0),
        rho: 1.25,
        absorbed: Vec3::ZERO,
    };
    let cfg = SurfaceDrag {
        cd: 0.8,
        surface_ratio: 0.9,
    };
    let dt = 1.0f32 / 240.0;
    let injected = surface_drag(&mut f, &mut air, cfg, dt);

    let m = f.particle_mass();
    let area = f.particle_spacing() * f.particle_spacing();
    // F = ½·ρ·Cd·A·|u|·u ⇒ |F| = ½·ρ·Cd·A·|u|²（u = v_air，粒子从零速出发）
    let want_v = 0.5 * air.rho * cfg.cd * area * air.v.length_squared() * dt / m;
    let got = f.velocities()[0];
    assert_eq!(got.y, 0.0, "风沿 +x ⇒ 横向分量必须逐位为 0");
    assert_eq!(got.z, 0.0);
    let rel = ((got.x - want_v) / want_v).abs();
    assert!(
        rel < 1e-5,
        "单粒阻力冲量应解析可算：rel={rel:e}（got={got:?}）"
    );
    let rel_p = ((injected.x - got.x * m) / (got.x * m)).abs();
    assert!(rel_p < 1e-5, "返回值 = m·Δv（相对容差 {rel_p:e}）");
    assert_eq!(air.absorbed, -injected, "反作用等大反向（逐位）");
    assert!(injected.x > 0.0, "用例非平凡：确实注入了 +x 动量");
}

#[test]
fn only_free_surface_particles_are_driven() {
    let mut f = sys([8, 8, 8], 0.05);
    f.step(1.0 / 60.0, &NoProviders);
    let surface = 0.9 * f.config().rest_density; // 与 `SurfaceDrag::default().surface_ratio` 同口径
    let d = f.densities();
    let mut imax = 0usize;
    let mut imin = 0usize;
    for i in 1..d.len() {
        if d[i] > d[imax] {
            imax = i;
        }
        if d[i] < d[imin] {
            imin = i;
        }
    }
    assert!(
        d[imax] >= surface,
        "区块内部必须被邻居补齐（否则场景无分辨力）：max ρ={}，阈值={surface}",
        d[imax]
    );
    assert!(
        d[imin] < surface,
        "角点必须是密度亏的自由表面：ρ={}，阈值={surface}",
        d[imin]
    );

    f.set_velocities(&vec![Vec3::ZERO; f.len()]);
    let mut air = Air {
        v: Vec3::new(1.0, 0.0, 0.0),
        rho: 1.25,
        absorbed: Vec3::ZERO,
    };
    surface_drag(&mut f, &mut air, SurfaceDrag::default(), 1.0 / 240.0);
    assert!(
        f.velocities()[imin].x > 0.0,
        "自由表面粒子应被风驱动：v={:?}",
        f.velocities()[imin]
    );
    assert_eq!(
        f.velocities()[imax],
        Vec3::ZERO,
        "内部粒子（ρ ≥ ρ0）必须逐位不动"
    );
    assert!(air.absorbed.x < 0.0, "大气记到反作用（方向相反）");
}
