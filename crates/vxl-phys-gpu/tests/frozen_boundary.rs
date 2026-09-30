//! **判据：2b 档的边界粒子必须逐位冻结**（GPU；需适配器）。
//!
//! **为什么需要它**：`integrate.wgsl` 只该积分**流体前缀** `[0, n_fluid)`（边界粒子是运动学冻结的，
//! 见 `pipeline.rs` 的注释与 CPU `FluidSystem::substep` 的 `for i in 0..nf`）。而**分派覆盖**是
//! `⌈n_fluid/64⌉` 个工作组 ⇒ 覆盖到 `⌈n_fluid/64⌉ × 64` **粒**，核里的守卫却写的是**总数** `n`
//! ⇒ 只要 `n_fluid % 64 != 0`，尾部那 `n_fluid % 64` 个**边界粒子会被一起积分**（静默、无报错）。
//! 二维展开后更狠：`split_2d` 把覆盖放大到 `65535×64×gy`（`n_fluid > 4.19M` 时）⇒**整段边界粒子**都被
//! 积分（`PLAN-gpu.md` §28 的规模断崖：n=160 干净 / n=162 起坏，阈值正是 `65535×64 = 4 194 240`）。
//! 纯流体档 `n_fluid == n` ⇒ 守卫恰好把一切挡住 ⇒ **这个缺陷只在 2b 上看得见**。
//!
//! **判据**：小场景（不需要 4.19M 粒）——只要 `n_fluid % 64 != 0`，跑几 tick 后
//! **边界段的 `pos`/`vel` 必须与初值逐位相同**。这条在修好之前**必红**（金丝雀：断言 `n_fluid % 64 != 0`，
//! 否则场景退化、判据空过）。
//!
//! ⚠️ **无适配器 ⇒ 跳过**（与其它 GPU 判据同口径：CI 不跑，跑它的是本地全量门禁）。
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

/// 晶格边长取 **13**：`13³ = 2197`、`2197 % 64 = 21` ⇒ 积分分派覆盖到 `2240` ⇒ 尾部 21 个边界粒子
/// 会被误积分（缺陷的**最小形态**，与 4.19M 那个规模断崖是同一处代码）。
const N: usize = 13;
const SPACING: f32 = 0.05;
const TICKS: usize = 3;
const SUB: usize = 2;

fn have_adapter() -> bool {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return false;
    }
    true
}

/// 小 2b 场景：晶格 + **浸没在流体里的小盒**（提供者）。
///
/// ⚠️ 为什么不用地板（第一版）：覆盖溢出段是**边界段最前面**那 ≤63 粒；地板远大于晶格 ⇒ 首批边界粒子
/// 落在角落、与流体无接触 ⇒ 就算被误积分也不动 ⇒ **判据空过**（实测过两次）。浸没盒的每一粒边界粒子
/// 都贴着流体 ⇒ 溢出段必然**受力** ⇒ 判据有齿。
fn scene() -> FluidSystem {
    let cfg = FluidConfig::default();
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.0,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    // 浸没盒：半边长 2 格 ⇒ 面粒子数 ~数十，全部在流体内部（与流体邻居同处 `h` 之内）。
    let bodies = vec![(
        0u32,
        Shape::Box {
            half: Vec3::new(2.0 * SPACING, 2.0 * SPACING, 2.0 * SPACING),
        },
        BodyPose {
            pos: Vec3::ZERO,
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    )];
    assert!(
        f.set_boundary_particles(&bodies) > 0,
        "浸没盒没造出边界粒子"
    );
    f
}

fn cfg_for(f: &FluidSystem) -> PacketCfg {
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
        // **用引擎自己的重力**：判据要的是"边界粒子有真实的成对力"（否则积分它们也不动 ⇒ 判据空过）。
        gravity: [
            f.config().gravity.x,
            f.config().gravity.y,
            f.config().gravity.z,
        ],
        b_tait: f.config().sound_speed * f.config().sound_speed * f.config().rest_density
            / f.config().gamma_tait,
        rho0: f.config().rest_density,
        gamma: f.config().gamma_tait,
        clamp_neg: f.config().tensile_instability_suppression,
        xsph_eps: f.config().xsph_viscosity,
        max_speed_frac: f.config().max_speed_frac,
        recompute_box: false,
        grid_bins_cap: gdims.0 * gdims.1 * gdims.2,
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
fn boundary_particles_are_bitwise_frozen() {
    if !have_adapter() {
        return;
    }
    let f = scene();
    let pc = cfg_for(&f);
    // **金丝雀**：场景必须真的踩到"分派覆盖溢出"（`n_fluid % 64 != 0`）——否则这条判据空过。
    assert_ne!(
        pc.n_fluid % 64,
        0,
        "n_fluid = {} 是 64 的整数倍 ⇒ 分派覆盖恰好落在 n_fluid 上 ⇒ 这条判据空过（换 N 重挑场景）",
        pc.n_fluid
    );
    assert!(pc.n_fluid < pc.n, "场景必须含边界粒子（否则不是 2b）");
    let (pos, vel, pmass) = flatten_all(&f);
    let nf = (pc.n_fluid as usize) * 3;
    let mut pk = Packet::new(0, pc, &pos, &vel, &pmass).expect("建包失败");
    pk.run(&pc, TICKS, SUB, false);
    let (gp, gv) = pk.snapshot();
    let mut dp = 0usize;
    let mut dv = 0usize;
    for k in nf..gp.len().min(pos.len()) {
        if gp[k].to_bits() != pos[k].to_bits() {
            dp += 1;
        }
        if gv[k].to_bits() != vel[k].to_bits() {
            dv += 1;
        }
    }
    // 流体段**应当**动（否则场景没在跑）——顺带当"反空跑"的金丝雀。
    let moved = (0..nf)
        .filter(|&k| gp[k].to_bits() != pos[k].to_bits())
        .count();
    // **反空跑②：分派覆盖溢出的那一段边界粒子必须**真的受力**——否则"冻结"是"力本来就是 0"的假象
    // （第一版判据就这么空过了：地板远大于晶格 ⇒ 首批边界粒子在角落里没接触）。这条用 CPU 侧的
    // 邻域关系算：`[n_fluid, n_fluid + 溢出)` 里至少有一个粒子在某个流体粒子的 `h` 邻域内。
    let over = (64 - (pc.n_fluid % 64) % 64) as usize;
    let hh = pc.h;
    let (apos, _, _, nfl) = f.raw_particles();
    let loaded = (nfl..(nfl + over).min(apos.len()))
        .filter(|&b| {
            apos[..nfl]
                .iter()
                .any(|p| (*p - apos[b]).length_squared() <= hh * hh)
        })
        .count();
    assert!(
        loaded > 0,
        "覆盖溢出段（{over} 粒）里没有一个边界粒子与流体接触 ⇒ 判据**空过**（地板要贴紧晶格）"
    );
    // **反空跑③：边界反作用必须非零**（同上，从卡上读一次）。
    let bf0 = pk.read_boundary_forces(pc.n_fluid);
    let bf_max = bf0.iter().fold(0.0f32, |m, x| m.max(x.abs()));
    assert!(
        bf_max > 0.0,
        "边界反作用恒 0 ⇒ 边界粒子本来就受力为零 ⇒ 这条判据**空过**"
    );
    println!(
        "流体段动了 {moved}/{nf} 个分量；覆盖溢出段 {over} 粒（受力 {loaded} 粒）；\
         边界反作用 max|F| = {bf_max:.3e}；边界段逐位不同：pos {dp} / vel {dv}（共 {}）",
        gp.len() - nf
    );
    assert!(
        moved > 0,
        "流体一步没动 ⇒ 场景或管线空转 ⇒ 这条判据没有分辨力"
    );
    assert_eq!(
        (dp, dv),
        (0, 0),
        "**边界粒子必须逐位冻结**（运动学冻结，见 CPU `substep` 的 `for i in 0..nf`）：\
         实得 pos {dp} / vel {dv} 个分量被改动 ⇒ 积分核把 `[n_fluid, n)` 也算了（守卫用了**总数**）"
    );
}
