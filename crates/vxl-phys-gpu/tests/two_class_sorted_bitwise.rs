//! **§26.3 判据 1/2/4**：2b 混合场景（流体 + 边界粒子）上「**格序副本档 vs 平铺档**」的**逐位**判据。
//!
//! **为什么另立一条**：`sorted_copies_bitwise.rs` 那条只在**纯流体**下有效（它的金丝雀就是
//! `assert_eq!(n_fluid, n)`）——而 §26.3 要开的是**混合场景**的格序档：两类格表分别装箱 ⇒ 副本
//! 下标 < `n_fluid` 恒是流体 ⇒ 核里的**标签**判断在副本空间重新成立。这条把那个新承诺钉住。
//!
//! **本文件三条**（`PLAN-gpu.md` §26.3 判据表）：
//! - **判据 2**（防"绿 = 空过"）：混合场景下 `new_sorted` **真建了档**（`sorted::sorted_active`）
//!   —— 这条在放开 `n_fluid == n` 那道门之前**必然是红的**；
//! - **判据 1**（主判据）：两档跑同样的 tick/子步 ⇒ `pos`/`vel` **逐位相同**；
//! - **判据 4**（守死语义）：壁面档 + 格序档同开时**仍退回平铺**（不许静默错）。
//!
//! ⚠️ **无适配器 ⇒ 跳过**（与既有 GPU 判据同口径：CI 上不跑，跑它的是本地全量门禁）。
//! ⚠️ 盲区（如实记）：`overflow == 0` 与"两张表非平凡"由 `two_class_real_scene.rs` 的
//! CPU oracle 断言（那条**只跑网格**、不跑整 tick）——本文件判的是**数值等价**，不重复判表。
mod support;

use support::scene_2b;
use vxl_phys_fluid::FluidSystem;
use vxl_phys_gpu::pipeline::sorted::sorted_active;
use vxl_phys_gpu::pipeline::{Packet, PacketCfg};

const TICKS: usize = 3;
const SUB: usize = 4;

fn have_adapter() -> bool {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return false;
    }
    true
}

/// 与其它探针同口径的 `PacketCfg`。⚠️ 粒数走 `raw_particles()`（**全量**视图，含边界）；
/// `f.len()` 给的是 `n_fluid`。
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
        // 用引擎自己的重力（这条判据是在"真物理"下比两档，不是在静止态下比）。
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

/// **全量**粒子（含边界）的位置/速度/逐粒质量。
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

/// 逐位相同分量数 / 总分量数。
fn bits_same(a: &[f32], b: &[f32]) -> (usize, usize) {
    let same = a
        .iter()
        .zip(b.iter())
        .filter(|(x, y)| x.to_bits() == y.to_bits())
        .count();
    (same, a.len())
}

/// **判据 1 + 2**：混合场景下两档逐位相同，**且档真在跑**。
#[test]
fn mixed_2b_sorted_and_flat_are_bitwise_identical() {
    if !have_adapter() {
        return;
    }
    let f = scene_2b();
    let pc = cfg_for(&f);
    // **金丝雀（防"绿 = 空过"）**：场景必须真是混合的（否则这条退化成纯流体那条）。
    assert!(
        pc.n_fluid < pc.n,
        "场景必须含边界粒子（n_fluid {} < n {}）——否则这条判据不测 2b",
        pc.n_fluid,
        pc.n
    );
    let (pos, vel, pmass) = flatten_all(&f);
    let mut flat = Packet::new_flat(0, pc, &pos, &vel, &pmass).expect("平铺档建包失败");
    let mut sorted = Packet::new(0, pc, &pos, &vel, &pmass).expect("格序档建包失败");
    // **判据 2**：档没建起来时，两边跑的是同一条路 ⇒ 判据 1 是**空过**（§26.3 触点 7 之前必红）。
    // 两头都断言：读点若是"恒真"就证明不了任何事（平铺档必须给 false）。
    assert!(
        sorted_active(&sorted) && !sorted_active(&flat),
        "格序档读点必须**分辨得出**两档：`new_sorted` 建了、`new` 没建（否则判据 1 是空过）"
    );
    flat.run(&pc, TICKS, SUB, false);
    sorted.run(&pc, TICKS, SUB, false);
    let (fp, fv) = flat.snapshot();
    let (sp, sv) = sorted.snapshot();
    let (pn, pt) = bits_same(&fp, &sp);
    let (vn, vt) = bits_same(&fv, &sv);
    println!(
        "2b（流体 {} / 边界 {}）{TICKS} tick × {SUB} 子步：位置 {pn}/{pt}、速度 {vn}/{vt} 逐位相同",
        pc.n_fluid,
        pc.n - pc.n_fluid
    );
    assert_eq!(
        (pn, vn),
        (pt, vt),
        "格序副本档与平铺档在**混合场景**下必须逐位相同：位置 {pn}/{pt}、速度 {vn}/{vt} —— \
         若红了，先看两类的表/副本是不是同一套次序（§26.3：副本按『流体块 ‖ 边界块』摆、\
         `pmass` 每子步 gather）"
    );
}

/// **判据 4（守死语义）**：壁面档与格序档**互斥** —— 同开时壁面档那条构造器**退回平铺**，
/// 不许"两档都开着但只有一档生效"这种静默错。
#[test]
fn walls_tier_still_falls_back_to_flat() {
    if !have_adapter() {
        return;
    }
    let f = scene_2b();
    let pc = cfg_for(&f);
    let (pos, vel, pmass) = flatten_all(&f);
    let (pkt, _walls) = Packet::new_with_walls(0, pc, &pos, &vel, &pmass).expect("壁面档建包失败");
    assert!(
        !sorted_active(&pkt),
        "壁面档改的是**索引序**的 `dens`，而格序档下 `dens` 是副本 ⇒ 两档同开必须退回平铺\
         （否则壁面镜像静默失效）——这条断言就是「退回」的读点"
    );
}
