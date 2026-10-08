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
    /// 动量槽的等效空气质量（0 = 运动学背景：吸收不改风速）。
    mass: f32,
}

impl MediumField for Air {
    fn sample(&self, _x: Vec3) -> MediumSample {
        let mut velocity = self.v;
        if self.mass > 0.0 {
            velocity += self.absorbed * (1.0 / self.mass);
        }
        MediumSample {
            density: self.rho,
            velocity,
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
        mass: 0.0,
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
        mass: 0.0,
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

/// ④ **双向账**（动量槽 `mass > 0`）：一块静止水被有限质量的风吹 ⇒
/// ① 动量闭合 `ΣΔp_水 + M·Δv_空气 = 0`；② 总动能**不增**（阻力只耗散）——
/// 这条正是"继承半速被否"那类能量源的通用判据。
#[test]
fn two_way_ledger_conserves_momentum_and_does_not_create_energy() {
    let mut f = sys([8, 8, 8], 0.05);
    f.step(1.0 / 60.0, &NoProviders); // 让密度快照就绪（自由表面判据）
    f.set_velocities(&vec![Vec3::ZERO; f.len()]);
    let surface = 0.9 * f.config().rest_density;
    let n_surface = f.densities().iter().filter(|d| **d < surface).count();
    assert!(n_surface > 0, "场景必须真的有自由表面（否则用例无意义）");

    let m = f.particle_mass();
    let mut air = Air {
        v: Vec3::new(2.0, 0.0, 0.0),
        rho: 1.25,
        absorbed: Vec3::ZERO,
        mass: 8.0,
    };
    let fluid_ke = |f: &FluidSystem| {
        let mut e = 0.0f32;
        for v in f.velocities() {
            e += 0.5 * m * v.length_squared();
        }
        e
    };
    let air_ke = |a: &Air| 0.5 * a.mass * a.sample(Vec3::ZERO).velocity.length_squared();
    let ke0 = fluid_ke(&f) + air_ke(&air);

    for _ in 0..4 {
        surface_drag(&mut f, &mut air, SurfaceDrag::default(), 1.0 / 240.0);
    }
    let ke1 = fluid_ke(&f) + air_ke(&air);

    let mut dp_water = Vec3::ZERO;
    for v in f.velocities() {
        dp_water += *v * m;
    }
    let dp_air = air.absorbed; // 大气吸收的动量 = −水拿到的
    assert!(
        dp_water.x > 0.0,
        "用例非平凡：水确实被推走（Δp={dp_water:?}）"
    );
    let resid = (dp_water + dp_air).length();
    let scale = dp_water.length().max(1e-12);
    assert!(
        resid / scale < 1e-5,
        "动量账应闭合：resid/scale={:.2e}",
        resid / scale
    );
    assert!(
        ke1 <= ke0 + 1e-9,
        "阻力只耗散、不造能：ΔKE={:.3e}",
        ke1 - ke0
    );
    assert!(
        air.sample(Vec3::ZERO).velocity.x < 2.0,
        "大气沿 +x 变慢（动量槽生效）"
    );
}
