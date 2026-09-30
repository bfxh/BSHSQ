//! **诊断（手动跑，`#[ignore]`）**：`recompute_box`（箱跟随）在 > 4.19M 粒时的
//! `Parent device is lost` 定位（`PLAN-gpu.md` §28.1 的"三条查法"之①）。
//!
//! **场景与 `gpu_tick_probe --tank --box=follow` 逐字同款**（零重力 + 地板四壁 5 体 + 剪切初速
//! 在边界之后加）——第一版诊断用了"地板 + 开重力"的简化场景 ⇒ **不炸**（留档：它把机制遮掉了）。
//!
//! 跑法：`cargo test -p vxl-phys-gpu --test follow_bisect -- --ignored --nocapture`
//! （需适配器；n=162 的 CPU 建场 ≈ 25 s。）
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

/// 过阈值（4 194 240）的规模（探针 n=162：流体 4 251 528 + 边界 580 592 = 4 832 120 总粒）。
const N: usize = 162;
const SPACING: f32 = 0.05;

/// 与探针 `tank_bodies` 逐字同款（地板 + 四壁）。
fn tank_bodies(n: usize, spacing: f32, h: f32) -> Vec<(u32, Shape, BodyPose)> {
    let half = n as f32 * spacing * 0.5;
    let t = 2.0 * spacing;
    let hgt = (n as f32 * spacing) + 2.0 * h;
    let tip = 0.5 + hgt * 0.5;
    let pose = |pos: Vec3| BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    };
    let span = half + 2.0 * t;
    let wall_x = |x: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(t * 0.5, hgt * 0.5, span),
            },
            pose(Vec3::new(x, tip, 0.0)),
        )
    };
    let wall_z = |z: f32, id: u32| {
        (
            id,
            Shape::Box {
                half: Vec3::new(span, hgt * 0.5, t * 0.5),
            },
            pose(Vec3::new(0.0, tip, z)),
        )
    };
    vec![
        (
            0u32,
            Shape::Box {
                half: Vec3::new(span, t * 0.5, span),
            },
            pose(Vec3::new(0.0, 0.5 - t * 0.5, 0.0)),
        ),
        wall_x(half + t * 0.5, 1),
        wall_x(-(half + t * 0.5), 2),
        wall_z(half + t * 0.5, 3),
        wall_z(-(half + t * 0.5), 4),
    ]
}

/// 与探针 `build_fluid` 逐字同款：**零重力**、5 体边界、剪切初速在边界**之后**加。
fn scene_with(n: usize) -> FluidSystem {
    let cfg0 = FluidConfig {
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    let h = cfg0.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg0,
        Vec3::new(
            -(n as f32) * SPACING * 0.5,
            0.5,
            -(n as f32) * SPACING * 0.5,
        ),
        [n, n, n],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let bodies = tank_bodies(n, SPACING, h);
    let nb = f.set_boundary_particles(&bodies);
    println!("2b 边界粒子：{nb} 个");
    // 剪切初速（探针同款：**在边界之后**）。
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    f
}

fn cfg_for(f: &FluidSystem, recompute_box: bool) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    let (apos, _, _, n_fluid) = f.raw_particles();
    PacketCfg {
        n: apos.len() as u32,
        n_fluid: n_fluid as u32,
        total: gdims.0 * gdims.1 * gdims.2,
        gmin: [gmin.x, gmin.y, gmin.z],
        inv: ginv,
        dims: [gdims.0, gdims.1, gdims.2],
        cap: 512,
        h,
        h2: h * h,
        k6: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)),
        w0: 315.0 / (64.0 * std::f32::consts::PI * h.powi(9)) * h.powi(6),
        ks: 45.0 / (std::f32::consts::PI * h.powi(6)),
        mass: f.particle_mass(),
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        // 探针同款：零重力。
        gravity: [0.0, 0.0, 0.0],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box,
        // **探针同款**（`build_packet`）：follow 档给 `1 << 20`（≠ total）⇒ cap_total = max(total, 1<<20)
        // **大于建包箱** ⇒ 运行时箱子可以长到 ~2×。这就是探针与本测试此前的唯一 cfg 差异。
        grid_bins_cap: if recompute_box {
            1 << 20
        } else {
            gdims.0 * gdims.1 * gdims.2
        },
    }
}

fn flatten_all(f: &FluidSystem) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let (apos, avel, apmass, _) = f.raw_particles();
    let mut pos: Vec<f32> = Vec::with_capacity(apos.len() * 3);
    let mut vel: Vec<f32> = Vec::with_capacity(avel.len() * 3);
    for k in 0..apos.len() {
        pos.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        vel.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    (pos, vel, apmass.to_vec())
}

#[test]
#[ignore = "手动诊断：--ignored 跑（需适配器，n=162 建场 ≈25 s）"]
fn follow_repro_1tick_full_chain() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过）");
        return;
    }
    let f = scene_with(N);
    let pc = cfg_for(&f, true);
    println!(
        "n = {}（流体 {} / 边界 {}）；build 箱 dims {:?}（total {}，cap_total = max(total, bins_cap)）；\
         ⌈n/64⌉ = {}（> 65535 ⇒ bbox 走二维）",
        pc.n,
        pc.n_fluid,
        pc.n - pc.n_fluid,
        pc.dims,
        pc.total,
        pc.n.div_ceil(64)
    );
    let (pos, vel, pmass) = flatten_all(&f);
    let mut pk = Packet::new(0, pc, &pos, &vel, &pmass).expect("建包失败");
    println!("--- tick 1（4 子步全链，follow）提交…");
    std::io::Write::flush(&mut std::io::stdout()).ok();
    pk.run(&pc, 1, 4, false);
    let (p, _) = pk.read_state();
    let finite = p.iter().all(|x| x.is_finite());
    println!("✅ tick 1 活着（finite = {finite}）");
}

/// **常驻判据（非 ignore；110k 档 ≈6 s）**：箱跟随（follow）下 GPU 与 CPU 的漂移必须有界。
///
/// **为什么需要它（§28.2）**：follow 档的类 1 表基址早先由主机按**建包时的 `cap_total`** 静态切片
/// 绑定，而核里的偏移按**运行时** total 算 ⇒ 探针的 follow cfg（`grid_bins_cap = 1<<20` > 建包箱）
/// 下两者**从第一个子步就不同源** ⇒ 相位核读进从未写入的区域：驱动给零 ⇒ 边界贡献**静默全空**
/// （实测 n=40 follow 的 t=1 漂移 **0.00914 m**，比 fixed 档的 2e-5 差两个数量级）；给垃圾 ⇒
/// 巨大循环 ⇒ device lost（n=162/180）。修法 = 基址在核里现算（与网格两遍同式同源）。
/// 阈值 `2e-3`：修后读数 ~1e-5 档 ⇒ **100× 余量**；修前 0.009 ⇒ **必红**。
#[test]
fn follow_box_drift_stays_bounded() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    let f = scene_with(40);
    let pc = cfg_for(&f, true);
    // **金丝雀**：必须踩到"cap_total > 建包箱"这个触发条件（否则这条判据测不到失配）。
    let cap_total = pc.total.max(pc.grid_bins_cap);
    assert!(
        cap_total > pc.total,
        "cap_total = {cap_total} 未超过建包箱 {} ⇒ 切片/核里偏移本就同源 ⇒ 判据空过",
        pc.total
    );
    let (pos, _vel, _pmass) = flatten_all(&f);
    let mut pk = Packet::new(0, pc, &pos, &_vel, &_pmass).expect("建包失败");
    pk.run(&pc, 2, 4, false);
    let (gp, _) = pk.snapshot();
    // 对照 = **CPU 同源**：同一场景独立再建一份、CPU 推 2 tick，与 GPU 末态逐粒比位置。
    // ⚠️ 常驻判据用 **n=40**（110k 粒 ≈8 s）——别学第一版直接复用 n=162 的场景（25 分钟！）。
    let mut f2 = scene_with(40);
    for _ in 0..2 {
        f2.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let (apos, _, _, n_fluid) = f2.raw_particles();
    let nf = n_fluid;
    let mut mx = 0.0f32;
    for i in 0..nf {
        let d = Vec3::new(
            gp[i * 3] - apos[i].x,
            gp[i * 3 + 1] - apos[i].y,
            gp[i * 3 + 2] - apos[i].z,
        );
        mx = mx.max((d.x * d.x + d.y * d.y + d.z * d.z).sqrt());
    }
    println!("follow 档 2 tick 后 GPU vs CPU 的 |Δpos|max = {mx:.3e} m（流体 {nf} 粒）");
    assert!(
        mx < 2.0e-3,
        "箱跟随档的漂移 {mx:.3e} m 超阈（修前 0.00914）⇒ 类 1 表基址与核里偏移不同源 \
         （边界贡献丢失/幻影段）——见 `PLAN-gpu.md` §28.2"
    );
}

/// **逐子步**版诊断：哪一子步弄丢设备，最后一条打印就是它。
#[test]
#[ignore = "手动诊断：--ignored 跑"]
fn follow_bisect_which_substep() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过）");
        return;
    }
    let f = scene_with(N);
    let pc = cfg_for(&f, true);
    let (pos, vel, pmass) = flatten_all(&f);
    let mut pk = Packet::new(0, pc, &pos, &vel, &pmass).expect("建包失败");
    let dt = 1.0 / 60.0 / 4.0;
    // 32 子步 = 8 tick：探针在 6 tick 内炸 ⇒ 覆盖得到。丢失是粘性的 ⇒ 死在哪一子步，最后一条打印就是它。
    for s in 1..=32 {
        println!("--- 子步 {s}（全链）…");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        pk.run_substep(&pc, dt, 0b111_1111, None);
        let (p, _) = pk.read_state();
        let finite = p.iter().all(|x| x.is_finite());
        println!("    poll 过了（finite = {finite}）");
    }
    println!("✅ 32 个子步都活着");
}
