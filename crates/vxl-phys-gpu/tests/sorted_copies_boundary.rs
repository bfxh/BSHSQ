//! **混合场景（流体 + 2b 边界粒子）的 run-to-run 可复现性**判据——格序副本档在耦合路径上的准入前置。
//!
//! 曾经（`PLAN-gpu.md` §24）**不是** run-to-run 可复现的（同一 `Packet`、同一设备，`restore` 回初值
//! 再跑 ⇒ 流体段 30–340 个速度分量逐位不同、次数随机）。根因（§25）：**格盒没覆盖粒子** ⇒ 格爆满
//! ⇒ `canon` 放弃规范化 ⇒ 格内原子占位序残留 ⇒ 混沌放大。修复（§25.3，换代级）：`Packet::build`
//! 的盒按**全量**粒子现算。
//!
//! 控制组 [`pure_fluid_same_device_rerun_is_bitwise_identical`]（**判红**）：纯流体同设备两遍必须逐位
//! 相同——把机制钉在"回读或计时器没坏"上；
//! 判据 [`mixed_scene_same_device_rerun_is_bitwise_identical`]（2026-09-30 从"只打印"升级）：
//! 混合场景同设备两遍必须**逐位相同** ⇒ §25.3 常驻受守。
//!
//! 格序档对拍（§26.3 开放 2b 后）由 `two_class_sorted_bitwise.rs` 承担（含档读点）；本文件专守可复现性。
//! CI 无适配器 ⇒ 与其它 GPU 探针同口径跳过（不构成 CI 门禁；跑它的是本地全量门禁的 `cargo test`）。

use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

const N: usize = 16;
const SPACING: f32 = 0.05;
/// **多子步是刻意的**：单子步的差异只有 0/1 个分量（间歇性种子），2 tick × 4 子步才被混沌放大到
/// 可靠可见（种子读数记在 `PLAN-gpu.md` §24）。
const TICKS: usize = 2;
const SUB: usize = 4;

/// 小场景：晶格 + 5 趟静置 + 剪切初速；`floor` 为真时再加一块地板（提供 2b 边界粒子）。
fn scene(floor: bool) -> FluidSystem {
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
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    if floor {
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
        let nb = f.set_boundary_particles(&bodies);
        assert!(nb > 0, "地板没造出边界粒子 ⇒ 场景退化");
    }
    f
}

/// 与探针同口径的 `PacketCfg`。
fn cfg_for(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    // ⚠️ `f.len()` 给的是 **n_fluid**（不是总数）——`Packet` 吃**全量**视图，必须走 `raw_particles`。
    let (apos, _, _, n_fluid) = f.raw_particles();
    let np = apos.len();
    PacketCfg {
        n: np as u32,
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
        gravity: [0.0, 0.0, 0.0],
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

/// **全量**粒子（含边界）的位置/速度/逐粒质量——`Packet` 吃的是全量视图。
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

/// 一格（同一个 `Packet`、同一个设备）跑两遍的比对结果：`(流体段不符数, 边界段不符数, 总分量数)`。
fn rerun_diff(floor: bool) -> (usize, usize, usize) {
    let f = scene(floor);
    let pc = cfg_for(&f);
    let (pos, vel, pmass) = flatten_all(&f);
    let mut pk = Packet::new(0, pc, &pos, &vel, &pmass).expect("建包失败");
    pk.run(&pc, TICKS, SUB, false);
    let (_, v1) = pk.snapshot();
    pk.restore(&pos, &vel);
    pk.run(&pc, TICKS, SUB, false);
    let (_, v2) = pk.snapshot();
    let nf = pc.n_fluid as usize;
    let (mut df, mut db) = (0usize, 0usize);
    for k in 0..v1.len().min(v2.len()) {
        if v1[k].to_bits() != v2[k].to_bits() {
            if k / 3 < nf {
                df += 1;
            } else {
                db += 1;
            }
        }
    }
    (df, db, v1.len())
}

fn have_adapter() -> bool {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return false;
    }
    true
}

/// **控制组**（判红）：纯流体的"同设备两遍"必须逐位相同——它把 `mixed` 那条结论钉成场景特有。
#[test]
fn pure_fluid_same_device_rerun_is_bitwise_identical() {
    if !have_adapter() {
        return;
    }
    let (df, db, tot) = rerun_diff(false);
    assert_eq!(
        (df, db),
        (0, 0),
        "纯流体在同一设备上跑两遍必须逐位相同（实得 流体段 {df} / 边界段 {db}，共 {tot} 个分量）——\
         若这里红了，说明**回读或计时器坏了**，那么混合场景那条观察也要重新解释"
    );
}

/// **报告**（不判红）：混合场景的不可复现规模——如实打印，等 `PLAN-gpu.md` §23.4。
/// **判据**（2026-09-30 从"只打印"升级）：修复前真不可复现 ⇒ 写不出来；§25.3 之后前提成立。
#[test]
fn mixed_scene_same_device_rerun_is_bitwise_identical() {
    if !have_adapter() {
        return;
    }
    let (df, db, tot) = rerun_diff(true);
    println!(
        "混合场景（流体 + 2b 边界）同设备两遍（{TICKS} tick × {SUB} 子步）：流体段 {df} / 边界段 {db} \
         个速度分量逐位不同（共 {tot}）"
    );
    // ⚠️ 别用 `n_fluid = n` 去"二分"（混杂：边界粒子从冻结变可动 ⇒ 物理变了，§24.1 负面结果）。
    assert_eq!(
        (df, db),
        (0, 0),
        "修复后的 2b 路径（盒按全量粒子现算，`PLAN-gpu.md` §25.3）同设备两遍必须**逐位相同**：\
         实得流体段 {df} / 边界段 {db}（共 {tot}）——若这里红了，先看 overflow（格爆满 ⇒ \
         `canon` 放弃规范化 ⇒ 原子占位序残留 ⇒ 混沌放大，即 §25 的根因复发）"
    );
}
