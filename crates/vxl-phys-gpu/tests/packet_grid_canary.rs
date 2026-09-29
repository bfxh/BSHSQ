//! **反空跑判据（GPU；需适配器）**：`Packet` 的 SPH 管线**真在算**——判据用"**地板托住流体**"。
//!
//! **为什么需要它**（2026-09-30 实测教训）：`Packet` 层既有的 GPU 判据（`sorted_copies_boundary`）
//! 比的是"**两次重跑逐位相同**"——那对"整条管线空转"是**假绿**（全零轨迹逐位可复现）。
//! 本判据补"**物理有效性**"这一半：**网格四入口若空转**（`n_fluid`/`class_lo` 参数错位 ⇒ `cls_hi=0`），
//! 密度/力恒 0 ⇒ 流体失去压力与边界支撑 ⇒ **穿地板自由落体**；正常时被地板托住。
//!
//! 读数：末态**最低 y**（`y_min`）与**自由落体对照**（`½g t²`）。地板面在 `y ≈ 0`。
//! ⚠️ 无适配器 ⇒ 跳过（打印 `[SKIP]`，与其余 GPU 判据同款）。
//!
//! ⚠️ **不要用"过密晶格自膨胀"当判据**（首版就栽在这）：过密态压力极大，端到端读数是
//! **整体刚性平移**（实测 Δpos 全粒一致 0.10900、rms 一字不变）⇒ 该量对"管线是否在算"没有分辨力。
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

const N: usize = 8;
const SPACING: f32 = 0.05;
const SUB: usize = 4;
const TICKS: usize = 60;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 小场景：晶格 + 一块地板（提供 2b 边界粒子）+ 重力（用 CPU 配置的重力值）。
fn scene() -> FluidSystem {
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.5,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    let half = N as f32 * SPACING * 0.5 + 4.0 * h;
    let bodies = vec![(
        0u32,
        Shape::Box {
            half: Vec3::new(half, 2.0 * SPACING, half),
        },
        BodyPose {
            pos: Vec3::new(0.0, -2.0 * h, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    )];
    assert!(f.set_boundary_particles(&bodies) > 0, "地板没造出边界粒子");
    // 推一步 ⇒ `neighbor_grid()` 才有内容（否则 `total = 0` ⇒ 分派 0 组）。
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    f
}

fn cfg_for(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let gd = f.neighbor_grid();
    let (apos, _, _, n_fluid) = f.raw_particles();
    PacketCfg {
        n: apos.len() as u32,
        n_fluid: n_fluid as u32,
        total: gd.dims.0 * gd.dims.1 * gd.dims.2,
        gmin: [gd.min.x, gd.min.y, gd.min.z],
        inv: gd.inv,
        dims: [gd.dims.0, gd.dims.1, gd.dims.2],
        cap: 512,
        h,
        h2: h * h,
        k6: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)),
        w0: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)) * h.powi(6),
        ks: 45.0 / (std::f32::consts::PI * h.powi(6)),
        mass: f.particle_mass(),
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        // **重力开**：这是本判据的动力源（空跑 ⇒ 无支撑 ⇒ 自由落体）。
        gravity: [0.0, -9.81, 0.0],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box: false,
        grid_bins_cap: gd.dims.0 * gd.dims.1 * gd.dims.2,
    }
}

/// **流体段**（前 `n_fluid` 粒）的最低 y。
fn fluid_y_min(pos: &[f32], n_fluid: usize) -> f32 {
    let mut m = f32::MAX;
    for k in 0..n_fluid {
        m = m.min(pos[k * 3 + 1]);
    }
    m
}

#[test]
fn packet_sph_holds_the_fluid_on_the_floor() {
    if !have_adapter() {
        println!("[SKIP] 无可用适配器 ⇒ 本判据不跑（CI 常态）");
        return;
    }
    let f = scene();
    let pc = cfg_for(&f);
    let (apos, avel, apmass, n_fluid) = f.raw_particles();
    let mut pos: Vec<f32> = Vec::new();
    let mut vel: Vec<f32> = Vec::new();
    for k in 0..apos.len() {
        pos.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        vel.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    let y0 = fluid_y_min(&pos, n_fluid);
    let mut pk = Packet::new(0, pc, &pos, &vel, apmass).expect("建包失败");
    pk.run(&pc, TICKS, SUB, false);
    let (pos1, _) = pk.snapshot();
    let y1 = fluid_y_min(&pos1, n_fluid);
    // **CPU 同源对照**：新造一份同样的场景（确定性 ⇒ 与上面 `f` 的初值逐位相同）继续推 60 tick。
    // 判"Packet 对不对"要看**引擎自己的答案**，不能拿"我以为的物理"当期望。
    let mut fc = scene();
    fc.set_velocities(avel);
    for _ in 0..TICKS {
        fc.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let (cpos, _, _, cnf) = fc.raw_particles();
    let mut cy_min = f32::MAX;
    for p in cpos.iter().take(cnf) {
        cy_min = cy_min.min(p.y);
    }
    let t = TICKS as f32 / 60.0;
    let free_fall = 0.5 * 9.81 * t * t;
    println!(
        "== 反空跑判据（晶格 {N}³ + 地板；重力开；{TICKS} tick × {SUB} 子步）==\n\
         \x20  网格 dims {:?} = total {} | 粒数 {}（流体 {n_fluid}）\n\
         \x20  流体最低 y：初 {y0:+.5} → 卡 {y1:+.5} | **CPU 同源 {cy_min:+.5}** | 自由落体对照 −{free_fall:.3} m",
        pc.dims, pc.total, pc.n
    );
    assert!(
        y1.is_finite() && cy_min.is_finite(),
        "出现非有限值（卡 {y1} / CPU {cy_min}）"
    );
    assert!(
        (y1 - cy_min).abs() < 0.3,
        "卡的末态该与 **CPU 引擎同源读数**同量级（卡 {y1:+.5} vs CPU {cy_min:+.5}，差 {:.3}）——\
         差大说明网格/边界接线没生效（`n_fluid`/`class_lo` 写错时实测：卡侧自由落体、CPU 侧被托住）",
        (y1 - cy_min).abs()
    );
}
