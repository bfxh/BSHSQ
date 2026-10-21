//! ② 之后的各条诊断与 ③ facade 路径（子档，2026-10-16 从主档拆出——主档触阈）。
//!
//! 装的是"主档 `main` 里按序调用、各自独立成形"的那些段：②k 单子步 A/B、②m 投影锯齿、②j 分相定位、
//! ② 静置读数、②c 分箱、②g 夹断消融、②h 静置初值 k=1、②i 二分、②e 自敏感、②f 早期序列，
//! 以及 ③ facade 路径。共用件（`Sim`/`chain_sim*`/`bottom_mean_y`/`band_profile`/`sim_2b`）在
//! `diag.rs`；场景构建（`tank`/`water`/`make_cfg`/`flat`）与 ① 密度对拍留在主档。

use super::diag::{band_profile, bottom_mean_y, chain_sim, chain_sim_full, Sim};
use super::{flat, make_cfg, tank, water, SPACING};
use vxl_phys::{BodyBuildExt, FluidExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::interop::ProviderColliders;
use vxl_phys_fluid::{FluidConfig, FluidSystem};
use vxl_phys_gpu::pipeline::{GpuFluidStepper, Packet, PacketCfg};

/// 唯一动态体的 `(y, vy)`（两侧同序 ⇒ 索引相同）。
pub fn box_yv(w: &World) -> (f32, f32) {
    let mut out = (0.0f32, 0.0f32);
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            out = (w.bodies.position[i].y, w.bodies.linvel[i].y);
        }
    }
    out
}

/// **②k 单子步 A/B**（把 k=1 再切细）：两侧都设成"一个 tick = 一个子步"（dt = 1/60，**破声学 CFL 但两侧
/// 同等** ⇒ 比较仍然有效）。判读：
/// - **第一个子步就分道** ⇒ 与"tick 起点冻结表"**无关**（此时冻结表就是新鲜表）⇒ 密度/镜像相有真差；
/// - 第一个子步一致、后续才分道 ⇒ 命中 §13.10/13.11 记的已知近似①（每 tick 一张平面表）。
pub fn onestep_ab(
    adapter: usize,
    ids: &[u32],
    h: f32,
    prov: &dyn ProviderColliders,
    ghost: bool,
    project: bool,
) {
    let cfg_f = FluidConfig {
        substeps: 1,
        ..FluidConfig::default()
    };
    let mut cpu = FluidSystem::new(cfg_f, Vec3::new(-0.2, 0.99, -0.2), [8, 8, 8], SPACING);
    cpu.set_boundaries(if ghost { ids } else { &[] });
    let init = flat(&cpu);
    let cfg = make_cfg(&cpu);
    cpu.step(1.0 / 60.0, prov); // 一整个 tick = 一个子步
    let ca = cpu.positions().to_vec();
    let cd = cpu.densities().to_vec();
    let cacc = cpu.accelerations().to_vec(); // 力相输出 ⇒ 与卡上 `read_out` 同时点
    let sim = Sim {
        init: &init,
        cfg,
        substeps: 1,
        ticks: 1,
        ghost,
        project,
        ids: if ghost { ids.to_vec() } else { Vec::new() },
        h,
        adapter,
    };
    let Some(out) = chain_sim_full(&sim, prov) else {
        return;
    };
    let (gb, gd, gacc) = (out.pos, out.dens, out.acc);
    let mut mpos = 0.0f32;
    for (p, q) in ca.iter().zip(gb.iter()) {
        mpos = mpos.max((*p - *q).length());
    }
    let (mut mrho, mut cnt) = (0.0f32, 0usize);
    for (x, y) in cd.iter().zip(gd.iter()) {
        let d = (x - y).abs();
        mrho = mrho.max(d);
        if d > 0.1 {
            cnt += 1;
        }
    }
    let tag = match (ghost, project) {
        (false, _) => "无壁面档（两侧同关 ✅）",
        (true, true) => "镜像+投影（两侧同开 ✅）",
        // ⚠️ 这一档**不是**可判读的对照：CPU 侧镜像与投影**共用一个 `boundaries` 开关** ⇒ 关不掉
        // 单侧的投影 ⇒ 它测到的是"投影那一推 + 法向速度归零"的**作用量**（≈ 40 mm = `vmax·dt`），
        // 不是"镜像的差"。留着只为演示那个量级；判读用它 = 混变量的对照（比没有对照更坏）。
        (true, false) => "⚠️ 只镜像=**非对称消融**（CPU 投影仍开 ⇒ 只看作用量，**不作判读**）",
    };
    let (mut ma, mut acnt) = (0.0f32, 0usize);
    for (x, y) in cacc.iter().zip(gacc.iter()) {
        let d = (*x - *y).length();
        ma = ma.max(d);
        if d > 1.0 {
            acnt += 1; // >1 m/s²（近壁压力梯度是几百 m/s² 量级）
        }
    }
    println!(
        "  ── ②k 单子步 A/B（substeps=1；{tag}）── max|Δpos| **{mpos:.2e} m** | max|Δρ| **{mrho:.3e} kg/m³**（>0.1 的粒数 {cnt}/{}）| **max|Δacc| {ma:.3e} m/s²**（>1 的粒数 {acnt}/{}）",
        cd.len(),
        cd.len()
    );
    // 逐粒定位：最差 3 粒的**位移**（= 该链这一步的速度 × dt；`vmax·dt` 就是限速天花板）+ 密度 + 加速度。
    let dt = 1.0 / 60.0;
    let ip = |i: usize| Vec3::new(init.0[i * 3], init.0[i * 3 + 1], init.0[i * 3 + 2]);
    let mut worst: Vec<(usize, f32)> = (0..ca.len())
        .map(|i| (i, (ca[i] - gb[i]).length()))
        .collect();
    worst.sort_by(|x, y| y.1.partial_cmp(&x.1).unwrap_or(std::cmp::Ordering::Equal));
    for &(i, d) in worst.iter().take(3) {
        let (da, db) = ((ca[i] - ip(i)).length(), (gb[i] - ip(i)).length());
        println!(
            "        #{i}：差 {d:.2e} m | A 位移 {da:.2e} / B 位移 {db:.2e} m（|v| {:.2} / {:.2} m/s）| ρ A {:.3} / B {:.3} | |a| A {:.4e} / B {:.4e} m/s²",
            da / dt,
            db / dt,
            cd[i],
            gd[i],
            cacc[i].length(),
            gacc[i].length()
        );
    }
}

/// **②m 投影的"停滞-推回"锯齿**（**CPU 单侧**，无需显卡）：逐 tick 打印**最低那粒**的 `sdf`。
///
/// 机制：两侧的投影都是"**`pen > 0` 就推到 `sdf = +skin`**"（`fluid_boundary.rs:74` /
/// `wall_ghost.wgsl:161`，同式）⇒ 静置层不是停在一个点上，而是**下沉到穿透、再被整段推回静置线**
/// 的锯齿（振幅 ≈ `skin` = 15 mm）⇒ 这既是近壁层的保真问题，也是**所有壁面路径对照被"离散事件 ×
/// 机器精度"支配**的根源（②e 的 1e-8 自敏感就饱和、②f 的 3.4 mm 都是它）。
///
/// 场景里地板顶面在 `y = 1.0` ⇒ 打印的 `y − 1.0` 就是该粒的 `sdf`（判据：应在 `0`（穿透、触发推）
/// 与 `+skin = +0.015`（推回后）之间摆）。
pub fn push_sawtooth(ids: &[u32], prov: &dyn ProviderColliders, n: usize, ticks: usize) {
    let mut f = water();
    f.set_boundaries(ids);
    println!(
        "  ── ②m **投影的停滞-推回锯齿**（CPU 单侧）：最低粒的 sdf 逐 tick（地板顶面 y=1.0）──"
    );
    for k in 0..ticks {
        f.step(1.0 / 60.0, prov);
        let ps = f.positions();
        let (mut lo, mut at) = (f32::MAX, 0usize);
        for (i, p) in ps.iter().take(n).enumerate() {
            if p.y < lo {
                lo = p.y;
                at = i;
            }
        }
        if k < 14 {
            println!(
                "     tick {:>2}：最低粒 #{at} sdf = {:+.5} m",
                k + 1,
                lo - 1.0
            );
        }
    }
}

/// **②j 分相定位**（k=1，带壁面档时的逐粒密度对拍）：密度同 ⇒ 差在力/积分相；密度就不同 ⇒ 密度/镜像相。
pub fn dens_split(
    adapter: usize,
    cfg: PacketCfg,
    substeps: usize,
    h: f32,
    ids: &[u32],
    init: &(Vec<f32>, Vec<f32>, Vec<f32>),
    prov: &dyn ProviderColliders,
) {
    let mut cpu = water();
    cpu.set_boundaries(ids);
    cpu.step(1.0 / 60.0, prov);
    let cd = cpu.densities().to_vec();
    let sim = Sim {
        init,
        cfg,
        substeps,
        ticks: 1,
        ghost: true,
        project: true,
        ids: ids.to_vec(),
        h,
        adapter,
    };
    let Some(out) = chain_sim_full(&sim, prov) else {
        return;
    };
    let gd = out.dens;
    let (mut mx, mut mxs, mut cnt, mut mean) = (0.0f32, 0.0f32, 0usize, 0.0f32);
    for (x, y) in cd.iter().zip(gd.iter()) {
        let d = x - y;
        if d.abs() > mx {
            mx = d.abs();
            mxs = d;
        }
        if d.abs() > 0.1 {
            cnt += 1;
        }
        mean += d;
    }
    println!(
        "  ── ②j 分相定位（k=1 逐粒密度）── max|Δρ| **{mx:.3e} kg/m³**（带符号 {mxs:+.3e}）| |Δρ|>0.1 的粒数 {cnt}/{} | 均值 {:.3e}",
        cd.len(),
        mean / cd.len() as f32
    );
}

/// ② 静置仿真的读数（三档底层高度 + A/B 漂移）。
pub fn sim_report(a: &[Vec3], b: &[Vec3], c: &[Vec3], n: usize, ticks: usize) {
    let (ya, yb, yc) = (
        bottom_mean_y(a, n),
        bottom_mean_y(b, n),
        bottom_mean_y(c, n),
    );
    let mut mx = 0.0f32;
    for (p, q) in a.iter().zip(b.iter()) {
        mx = mx.max((*p - *q).length());
    }
    println!("  ── 静置仿真（{ticks} tick、重力开、同一初值三条链）──");
    println!("     A CPU（镜像 + 逐子步投影）  底层平均 y = {ya:.5} m");
    println!(
        "     B 卡上（鬼影 + **投影**）    底层平均 y = {yb:.5} m（与 A 差 {:.2e} m）| max|Δpos| {mx:.2e} m",
        (ya - yb).abs()
    );
    println!(
        "     C 卡上（**只鬼影**，金丝雀）底层平均 y = {yc:.5} m ⇒ {}",
        if yc < 0.0 {
            "穿地（证明投影不可省）"
        } else {
            "也站住了（异常：投影该是必需的）"
        }
    );
}

/// ②c **按"到最近壁面的水平距离"分箱**的水位对照（provider 容器）。
///
/// ⚠️ **判据警告（2026-09-24 实测）**：本表是**混沌量**——CPU 与"自己 +1e-8 扰动"（②e）给出的
/// 箱差与 CPU/GPU 的箱差**同量级**（近壁 1.58e-2 vs 3.97e-3）。**别拿它当"系统性误差"读**；
/// 要用它得先看 ②e 的自敏感读数。
pub fn band_report(a: &[Vec3], b: &[Vec3], n: usize, ticks: usize) {
    let (ba, bb) = (band_profile(a, n), band_profile(b, n));
    println!("  ── ②c 水位的**按到壁面水平距离分箱**（provider 容器，{ticks} tick）──");
    for (name, x, y) in [
        ("近壁 <1cm", ba[0], bb[0]),
        ("中 1–5cm", ba[1], bb[1]),
        ("内 >5cm ", ba[2], bb[2]),
    ] {
        println!(
            "     {name}：CPU {:.5}（{:>4} 粒）vs 卡上 {:.5}（{:>4} 粒）⇒ 差 **{:.2e} m**",
            x.0,
            x.1,
            y.0,
            y.1,
            (x.0 - y.0).abs()
        );
    }
}

/// **②g 夹断分支消融**：把 CPU/卡的 `clamp_neg`（张力不稳定抑制＝`P < 0 ⇒ 0` 这条**跳变分支**）
/// 两侧一起关掉，再看 **k=1** 的差。
///
/// ⚠️ **这不是干净的隔离（本仓纪律：混变量的对照比没有对照更坏）**——关掉夹断会**改变流场本身**
/// （张力不稳定 ⇒ 更暴烈的自由面），所以读数只能当"**不是它**"的排除用。
/// 实测：关掉后 k=1 的 `max|Δpos|` **1.50e-2 m >（开着时的）3.38e-3 m** ⇒ 夹断**不是** k=1 那条差的成因。
/// 目的（保留）：②f 显示 A/B 从 k=1 就有 3.38e-3 m 的逐粒差，而 1e-8 的扰动 1 tick 只走 1e-10 m
/// ⇒ 那个量**不是混沌** ⇒ 要找出它的成因（夹断已排除；分箱候选被 ① 削弱——① 的密度对拍是在
/// **卡上自算盒子**的前提下做到 4.88e-7 ρ0 的，若分箱不同，密度会出离散跳变）。
pub fn noclamp_early(
    adapter: usize,
    substeps: usize,
    h: f32,
    ids: &[u32],
    prov: &dyn ProviderColliders,
) {
    let cfg_f = FluidConfig {
        tensile_instability_suppression: false,
        ..FluidConfig::default()
    };
    let mut cpu = FluidSystem::new(cfg_f, Vec3::new(-0.2, 0.99, -0.2), [8, 8, 8], SPACING);
    cpu.set_boundaries(ids);
    let init = flat(&cpu);
    let mut cfg = make_cfg(&cpu);
    cfg.clamp_neg = false; // 与 CPU 同档（make_cfg 本来就跟着配置走；显式写一遍免得将来漂）
    cpu.step(1.0 / 60.0, prov);
    let a = cpu.positions().to_vec();
    let sim = Sim {
        init: &init,
        cfg,
        substeps,
        ticks: 1,
        ghost: true,
        project: true,
        ids: ids.to_vec(),
        h,
        adapter,
    };
    let Some(b) = chain_sim(&sim, prov) else {
        return;
    };
    let mut mx = 0.0f32;
    for (p, q) in a.iter().zip(b.iter()) {
        mx = mx.max((*p - *q).length());
    }
    println!(
        "  ── ②g 夹断分支消融（两侧 `clamp_neg = false`，k=1）── max|Δpos| **{mx:.2e} m**（对照 ②f 的 3.38e-3 m）"
    );
}

/// **②h 静置初值上的 k=1**（把"暴力坍落"这个自变量去掉——补记八定的两个判别里信息量高的那个）。
///
/// 做法：先让 CPU 链**静置** `SETTLE` tick，拿那时的状态**同时**当 A 的新起点（再推 1 tick）与 B 的
/// 建包初值（推 1 tick）。判读：
/// - 差仍 ~1e-3 级 ⇒ **结构性**逐 tick 差（坍落不是成因）⇒ 回去分相读码；
/// - 塌到 ~1e-9 级 ⇒ k=1 那条差归给**坍落本身**（初始那几 tick 的暴烈自由面把显微镜级差异放大到
///   mm 级——那时它与 ②e 自敏感是同一族现象，不必再当"待查的错"）。
pub fn settled_early(
    adapter: usize,
    substeps: usize,
    h: f32,
    ids: &[u32],
    prov: &dyn ProviderColliders,
) {
    const SETTLE: usize = 300;
    let mut f = water();
    f.set_boundaries(ids);
    for _ in 0..SETTLE {
        f.step(1.0 / 60.0, prov);
    }
    // **按静置后的粒子集/网格**算 cfg 与初值（新场景必须用自己那份 `cfg`——实栽过）。
    let init = flat(&f);
    let cfg = make_cfg(&f);
    let n = cfg.n_fluid as usize;
    f.step(1.0 / 60.0, prov); // A：静置态再推 1 tick
    let a = f.positions().to_vec();
    let sim = Sim {
        init: &init,
        cfg,
        substeps,
        ticks: 1,
        ghost: true,
        project: true,
        ids: ids.to_vec(),
        h,
        adapter,
    };
    let Some(b) = chain_sim(&sim, prov) else {
        return;
    };
    let mut mx = 0.0f32;
    for (p, q) in a.iter().zip(b.iter()) {
        mx = mx.max((*p - *q).length());
    }
    let (ya, yb) = (bottom_mean_y(&a, n), bottom_mean_y(&b, n));
    // **吃到跳变的粒数**（|Δpos| > 5 mm）：投影那一推 ≈ 15 mm ⇒ 这个计数就是"本 tick 有几粒的
    // `pen > 0` 分支在两链间翻了面"。它是判"离散事件主导"还是"连续相位差"的分水岭。
    let jumped = (0..a.len())
        .filter(|&i| (a[i] - b[i]).length() > 5e-3)
        .count();
    println!(
        "  ── ②h **静置初值上的 k=1**（先静置 {SETTLE} tick）── 底层均高差 {:.2e} m | max|Δpos| **{mx:.2e} m** | 吃到跳变的粒数 {jumped}/{}（对照 ②f 坍落初值：1.72e-4 / 3.38e-3）",
        (ya - yb).abs(),
        a.len()
    );
}

/// **②i 二分：把壁面档整个拿掉**（CPU 侧 `set_boundaries(&[])` 不收壁面、卡上不给 `walls`）
/// ⇒ 同一几何、同一初值下比**纯 SPH 相位**（网格 + 密度/力/积分）的 k 曲线。
///
/// 判读：这一对照里两条链的**代码路径**只剩网格与三相（壁面档完全不在场）——
/// - 差 ~1e-9 ⇒ 相位在这套几何/初值下逐位级一致 ⇒ 坍落那条 3.4 mm **只能**来自壁面档（镜像）；
/// - 差 ~1e-3 ⇒ **相位本身**在这套几何/初值下就有 mm 级差（与 2b 对照的 6.79e-6 相冲 ⇒ 要重核那个
///   2b 场景是不是真的"同一坍落"）。
///
/// 没有壁面 ⇒ 水会穿地（**两条链都穿**，可比性不受影响；这是已有的金丝雀 C 现象）。
pub fn bare_early(
    adapter: usize,
    cfg: PacketCfg,
    substeps: usize,
    h: f32,
    init: &(Vec<f32>, Vec<f32>, Vec<f32>),
    prov: &dyn ProviderColliders,
    n: usize,
) {
    println!("  ── ②i 二分：**两侧都没有壁面档**（纯 SPH 相位）──");
    for k in [1usize, 10] {
        let mut cpu = water();
        cpu.set_boundaries(&[]); // ← 与卡上 `ghost = false, project = false` 对位
        for _ in 0..k {
            cpu.step(1.0 / 60.0, prov);
        }
        let a = cpu.positions().to_vec();
        let sim = Sim {
            init,
            cfg,
            substeps,
            ticks: k,
            ghost: false,
            project: false,
            ids: Vec::new(),
            h,
            adapter,
        };
        let Some(b) = chain_sim(&sim, prov) else {
            return;
        };
        let mut mx = 0.0f32;
        for (p, q) in a.iter().zip(b.iter()) {
            mx = mx.max((*p - *q).length());
        }
        let (ya, yb) = (bottom_mean_y(&a, n), bottom_mean_y(&b, n));
        println!(
            "     k={k:>3}：底层均高差 {:.2e} m | max|Δpos| **{mx:.2e} m**",
            (ya - yb).abs()
        );
    }
}

/// **②e CPU 自敏感**（把"差多少算大"标定出来）：同一初值，只把 0 号粒速度扰动 `eps` ⇒
/// 同一 150 tick 之后看底层差与分箱差。**扫多个 `eps`**：投影是"跳变"机制（`pen > 0` 才推、
/// 一推就是 `pen + skin` ≈ 15 mm）⇒ 任何**显微镜级**差异都可能翻转这个分支 ⇒ 若小扰动也能
/// 给出 mm 级差，那 provider 路径的 2.9 mm 就不能记成"GPU 实现的系统性误差"。
pub fn cpu_self_sensitivity(ids: &[u32], prov: &dyn ProviderColliders, n: usize, ticks: usize) {
    println!("  ── ②e **CPU 自敏感**（同一初值，0 号粒 vx +eps；{ticks} tick）──");
    for eps in [1e-8f32, 1e-6, 1e-4, 1e-2] {
        let mk = || {
            let mut f = water();
            f.set_boundaries(ids);
            f
        };
        let (mut a, mut b) = (mk(), mk());
        let mut vs = b.velocities().to_vec();
        vs[0].x += eps;
        b.set_velocities(&vs);
        for _ in 0..ticks {
            a.step(1.0 / 60.0, prov);
            b.step(1.0 / 60.0, prov);
        }
        let (pa, pb) = (a.positions().to_vec(), b.positions().to_vec());
        let (ya, yb) = (bottom_mean_y(&pa, n), bottom_mean_y(&pb, n));
        let mut mx = 0.0f32;
        for (p, q) in pa.iter().zip(pb.iter()) {
            mx = mx.max((*p - *q).length());
        }
        let (ba, bb) = (band_profile(&pa, n), band_profile(&pb, n));
        println!(
            "     eps={eps:.0e}：底层平均 y 差 **{:.2e} m** | max|Δpos| {mx:.2e} m | 近壁 {:.2e} / 中 {:.2e} / 内 {:.2e}",
            (ya - yb).abs(),
            (ba[0].0 - bb[0].0).abs(),
            (ba[1].0 - bb[1].0).abs(),
            (ba[2].0 - bb[2].0).abs()
        );
    }
}

/// **②f 早期时间序列**（本仓纪律："第 1 个 tick 就大 ⇒ 接线错；随漂移一起长 ⇒ 口径 B 混沌"）：
/// A/B 各跑 k tick，看底层平均 y 差与 max|Δpos| 随 k 的走势。`base` 只当模板（其中 `ticks` 被覆盖）。
pub fn early_series(base: &Sim, prov: &dyn ProviderColliders, n: usize) {
    println!("  ── ②f **早期时间序列**（同一初值、A/B 各跑 k tick）──");
    for k in [1usize, 2, 5, 10, 25, 60, 150] {
        let mut cpu = water();
        cpu.set_boundaries(&base.ids);
        for _ in 0..k {
            cpu.step(1.0 / 60.0, prov);
        }
        let a = cpu.positions().to_vec();
        let mut sim = base.clone();
        sim.ticks = k;
        let Some(b) = chain_sim(&sim, prov) else {
            return;
        };
        let mut mx = 0.0f32;
        for (p, q) in a.iter().zip(b.iter()) {
            mx = mx.max((*p - *q).length());
        }
        let (ya, yb) = (bottom_mean_y(&a, n), bottom_mean_y(&b, n));
        println!(
            "     k={k:>3}：底层平均 y 差 {:.2e} m | max|Δpos| {mx:.2e} m",
            (ya - yb).abs()
        );
        if k == 1 {
            // k=1 的差**不可能**是混沌（1e-8 的扰动在 1 tick 里只走 1e-10 m）⇒ 逐粒定位。
            let mut worst: Vec<(usize, f32)> =
                (0..a.len()).map(|i| (i, (a[i] - b[i]).length())).collect();
            worst.sort_by(|x, y| y.1.partial_cmp(&x.1).unwrap_or(std::cmp::Ordering::Equal));
            let jumped = worst.iter().filter(|x| x.1 > 5e-3).count();
            println!("        吃到跳变的粒数（>5 mm）：{jumped}/{}", a.len());
            for &(i, d) in worst.iter().take(3) {
                println!(
                    "        #{i}：差 {d:.2e} m | A ({:.3},{:.3},{:.3}) | B ({:.3},{:.3},{:.3})",
                    a[i].x, a[i].y, a[i].z, b[i].x, b[i].y, b[i].z
                );
            }
        }
    }
}

/// ③ **facade 路径**：provider 壁面 + 卡上步进（壁面档经 `FluidStepper` 接线）。
/// 场景：体素水槽（provider 壁面）+ 铸装水块 + 浮盒（2b 边界粒子 ⇒ 反作用推它）；
/// A = 整段 CPU；B = 该流体的步进在卡上（门面每 tick 让**后端自己**收集壁面接触表——
/// 主机那份流体状态是陈的，位置必须以卡上为准）。
pub fn facade_path(adapter: usize) {
    // 跑久一点（≈6.7 s）让浮体**静下来**：要检验的预言是"浮体（不触底）的差应回到 mm 量级"
    // —— 它成立则 1.5 cm 那笔账归给"接触/缓冲双稳放大"，不成立则壁面路径另有系统性误差。
    const TICKS: usize = 400;
    let mut wa = World::new(PhysConfig::default());
    let ka = tank(&mut wa);
    wa.add_fluid_with_boundary_coupling(water(), &[ka]);
    // **浮体**（ρ=800 ⇒ 约 80% 没入、**不触底**）：把"地板接触"这一项从判据里去掉。
    wa.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 1.5, 0.0),
        Quat::IDENTITY,
        800.0,
    );
    let mut wb = World::new(PhysConfig::default());
    let kb = tank(&mut wb);
    let fi = wb.add_fluid_with_boundary_coupling(water(), &[kb]);
    wb.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 1.5, 0.0),
        Quat::IDENTITY,
        800.0,
    );
    wa.step();
    wb.step();
    let f = &wb.fluids()[fi].0;
    let (apos, avel, apmass, nf) = f.raw_particles();
    let nb = apos.len() - nf;
    let mut p = Vec::with_capacity(apos.len() * 3);
    let mut v = Vec::with_capacity(apos.len() * 3);
    for k in 0..apos.len() {
        p.extend_from_slice(&[apos[k].x, apos[k].y, apos[k].z]);
        v.extend_from_slice(&[avel[k].x, avel[k].y, avel[k].z]);
    }
    let pc = make_cfg(f);
    let substeps = f.config().substeps.max(1) as usize;
    let bounds = wb.fluids()[fi].1.clone();
    let Some((pk, walls)) = Packet::new_with_walls(adapter, pc, &p, &v, apmass).ok() else {
        println!("  ── ③ facade 路径：没起 GPU 管线（跳过）");
        return;
    };
    let st = GpuFluidStepper::new(pk, Some(walls), bounds, pc, substeps, nf as u32, nb);
    if !wb.set_fluid_stepper(fi, Box::new(st)) {
        println!("  ── ③ facade 路径：登记后端失败（跳过）");
        return;
    }
    // 末窗**均值**（不是单点）：浮体在极限环里（实测 A 侧 400 tick 时 vy 仍有 0.18）⇒ 单点差可能
    // 只是相位（见 `vxl-phys-measurement-protocol` §「决定量必须窗口均值」）。
    const WIN: usize = 40;
    let (mut sa, mut sb) = (0.0f32, 0.0f32);
    for k in 0..TICKS {
        wa.step();
        wb.step();
        if k + WIN >= TICKS {
            sa += box_yv(&wa).0;
            sb += box_yv(&wb).0;
        }
    }
    let (ya, va) = (sa / WIN as f32, box_yv(&wa).1);
    let (yb, vb) = (sb / WIN as f32, box_yv(&wb).1);
    println!(
        "  ── ③ **facade 路径**（provider 壁面 + 卡上步进，{TICKS} tick、浮体 ρ=800 ⇒ 不触底）──"
    );
    println!(
        "     浮体 y（末 {WIN} tick 均值）{ya:.5} vs {yb:.5}（差 {:.2e} m）| 瞬时 vy {va:.5} vs {vb:.5}（差 {:.2e} m/s）",
        (ya - yb).abs(),
        (va - vb).abs()
    );
}
