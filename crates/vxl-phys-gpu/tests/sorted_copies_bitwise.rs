//! **格序副本档的逐位判据**（`PLAN-gpu.md` §23；`Packet::new`（默认=格序档）vs `Packet::new_flat`）。
//!
//! 为什么要有这条常驻判据：格序档**默认关**，所以没有任何既有门会碰它——它那"与平铺档逐位相同"的
//! 性质此前只由**手动跑探针**（`gpu_tick_probe --sorted` 的 A/B）看着。这条测试把同一件事钉进
//! `cargo test`：本地全量门禁会跑它；CI 的 runner 没有适配器 ⇒ 与其它 GPU 探针同口径**跳过**
//! （**不构成 CI 门禁**，这是既有的口径，别把它当成 CI 覆盖）。
//!
//! 判据口径：同一初值、同一 `PacketCfg`，两条路径各推 `TICKS` 个 tick（`SUB` 子步）⇒ 状态
//! **逐位相同**。为什么要多 tick 而不是一子步：一子步只能验"接线没接反"，多 tick 才能让任何
//! "枚举序被改"的偏差在浮点上累积出来。

use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

const N: usize = 24;
const SPACING: f32 = 0.05;
const TICKS: usize = 3;
const SUB: usize = 4;

/// 小场景：晶格 + 5 趟静置 + 剪切初速（与各探针同一套，零重力 ⇒ 流体留在箱内）。
fn scene() -> FluidSystem {
    let cfg = FluidConfig::default();
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
    f
}

/// 与 `gpu_tick_probe::build_packet` 同口径的 `PacketCfg`（**纯流体** ⇒ 格序档在这条路径上合法）。
fn cfg_for(f: &FluidSystem) -> PacketCfg {
    let h = f.config().smoothing_radius;
    let (gmin, ginv, gdims) = {
        let gd = f.neighbor_grid();
        (gd.min, gd.inv, gd.dims)
    };
    let (_, _, _, n_fluid) = f.raw_particles();
    let np = f.len();
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

/// 平面化的位置/速度（与探针同口径）。
fn flatten(f: &FluidSystem) -> (Vec<f32>, Vec<f32>) {
    let np = f.len();
    let mut pos: Vec<f32> = Vec::with_capacity(np * 3);
    let mut vel: Vec<f32> = Vec::with_capacity(np * 3);
    for k in 0..np {
        let p = f.positions()[k];
        pos.extend_from_slice(&[p.x, p.y, p.z]);
        let v = f.velocities()[k];
        vel.extend_from_slice(&[v.x, v.y, v.z]);
    }
    (pos, vel)
}

/// 有几成 f32 分量逐位相同（`(相同数, 总数)`）。
fn bits_same(a: &[f32], b: &[f32]) -> (usize, usize) {
    let same = a
        .iter()
        .zip(b.iter())
        .filter(|(x, y)| x.to_bits() == y.to_bits())
        .count();
    (same, a.len())
}

#[test]
fn sorted_copies_are_bitwise_identical_to_flat() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    let f = scene();
    let pc = cfg_for(&f);
    // **金丝雀（防"假绿"）**：格序档的纯流体等价性是这条判据的语义域（2b 的对拍在
    // `two_class_sorted_bitwise`）。
    assert_eq!(
        pc.n_fluid, pc.n,
        "本判据只在纯流体下有效（格序档的守门条件；否则测试会假绿）"
    );
    let (pos, vel) = flatten(&f);
    let pmass = vec![f.particle_mass(); f.len()];
    let mk = |sorted: bool| -> Result<Packet, String> {
        if sorted {
            Packet::new(0, pc, &pos, &vel, &pmass)
        } else {
            Packet::new_flat(0, pc, &pos, &vel, &pmass)
        }
    };
    let mut flat = mk(false).expect("平铺档建包失败");
    let mut sorted = mk(true).expect("格序档建包失败");
    // 推同样多 tick 再比末态：一子步只能验"接线没接反"，多 tick 才能让"枚举序被改"累积出来。
    flat.run(&pc, TICKS, SUB, false);
    sorted.run(&pc, TICKS, SUB, false);
    let (fp, fv) = flat.snapshot();
    let (sp, sv) = sorted.snapshot();
    let (pn, pt) = bits_same(&fp, &sp);
    let (vn, vt) = bits_same(&fv, &sv);
    assert_eq!(
        (pn, vn),
        (pt, vt),
        "格序副本档与平铺档必须**逐位相同**（{TICKS} tick × {SUB} 子步）：位置 {pn}/{pt}、速度 {vn}/{vt} \
         逐位相同 ⇒ 枚举序或数值被改了（见 PLAN-gpu.md §23 的等价性论证：两档 `cell_start` 逐项相同、\
         邻域枚举序列逐条相同）"
    );
}
