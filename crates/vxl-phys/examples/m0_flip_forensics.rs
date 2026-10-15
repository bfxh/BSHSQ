//! **m0 门槛档的"唤醒翻转取证"**（2026-10-13 建）**＋ 稳态抖动归因**（2026-10-15 加第二轴）：
//! `m0_gates` 的 `max_wake_flips` / 速率是当前**卡住两笔改动**的判据（P20 目标侧姿态插值、
//! P19 薄壳两面性）—— 它们都只是"微动了 CCD 的命中相位"，就把这条从 7 顶到 **10**
//! （1.000/s，闸是 `< 1.0`）。本探针把这条**变成可解释的量**：哪个体在翻转、在哪些 tick、
//! 翻转前后它长什么样，以及**翻转是被哪条判据、哪个邻居按下的**。
//!
//! ⚠️ 归因层（2026-10-15）：旧版只打 `|v|` ⇒ 把稳态读成"悬在阈下沿（0.013–0.0375 < 0.04）
//! 的接触闪烁极限环"。但睡眠判据是**与门**（`|v| < sleep_linear && |ω| < sleep_angular`
//! 才累加计时器，见 `sleep_pass_island`）⇒ **阈下沿的体本该入睡**；稳态醒着的体若 `|v|`
//! 全在阈下，只能是 `|ω| ≥ 0.05` 每子步把计时器清零。本版把 `|ω|` 与"判据分类"一起打出来，
//! 并**复算 hot 判据**做唤醒归因（线性 hot / 角速 hot / 强撞直通 / 无来源）。
//!
//! 场景与 `m0_gates::build_gate_scene` **逐字同参**（1 万静态盒地坪 + 1 千动态盒 12~40 m
//! 雨落，600 tick、默认档）。跑法：`cargo run --release -q -p vxl-phys --example m0_flip_forensics`
//!
//! ⚠️ **根因已定位到一个 30 行的最小复现（2026-10-15，同日复核后改口径）**：`min` 子命令 =
//! 两个平放的 0.8 m 立方体按静置高度叠放在瓷砖正中（零缝、零初速），**唯一变量是绕竖直轴的
//! 相对偏航**：偏航 0（或 90°，面重合）⇒ 10 tick 内入睡；偏航 0.5°–45° ⇒ **睡眠计时器恒为 0、
//! 永不入睡**（`|v|` 阈下、`|ω|` 卡在阈的 3–4 倍）。驱动项是**去穿透偏置速度**
//! （`erp·(depth−slop)` 经速度通道留在最终速度里，而睡眠判定读的正是这份速度）。
//! 消融、判别矩阵与修法候选见 `docs/EXPERIMENTS.md`「静置叠体」节。
//! 跑法：`... --example m0_flip_forensics min [偏航度] [iters] [substeps] [stab] [hold] [maxcorr] [ticks]`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 唤醒判据的两个倍数：硬编码自 `vxl-phys-solver/src/cfg.rs`（`SUBISLAND_WAKE_MULT` /
/// `WAKE_GATE_FAST_MULT`，都是 `pub(crate)`，外部取不到）。
/// ⚠️ 探针是**只读复算**：改那两个常量时必须同步这里，否则归因标签会静默说谎。
const WAKE_MULT: f32 = 2.0;
const FAST_MULT: f32 = 8.0;

/// 可选旋钮（位置参数，顺序沿用 `m1_pile`：迭代 / 子步 / shock / 无偏置末趟 / 唤醒门 K /
/// 安座趟 / tick 数）。**不传 = `PhysConfig::default()` 的值** ⇒ 无参跑的就是门槛档本身。
/// 这些旋钮都是仓里**早已实现、默认关**的机制，本探针只做消融、不新增语义。
#[derive(Clone, Copy)]
struct Knobs {
    iters: u32,
    substeps: u32,
    shock: u32,
    stab: u32,
    wake_gate_k: u32,
    hold: u32,
    ticks: u32,
}

fn knobs() -> Knobs {
    let d = PhysConfig::default();
    let mut a = std::env::args().skip(1);
    let mut nx = || a.next().and_then(|s| s.parse::<u32>().ok());
    Knobs {
        iters: nx().unwrap_or(d.velocity_iterations),
        substeps: nx().unwrap_or(d.substeps),
        shock: nx().unwrap_or(d.shock_iterations),
        stab: nx().unwrap_or(d.stabilization_iterations),
        wake_gate_k: nx().unwrap_or(d.wake_gate_k),
        hold: nx().unwrap_or(d.settled_hold_iterations),
        ticks: nx().unwrap_or(600),
    }
}

impl Knobs {
    fn config(&self) -> PhysConfig {
        PhysConfig {
            velocity_iterations: self.iters,
            substeps: self.substeps,
            shock_iterations: self.shock,
            stabilization_iterations: self.stab,
            wake_gate_k: self.wake_gate_k,
            settled_hold_iterations: self.hold,
            ..PhysConfig::default()
        }
    }

    fn label(&self) -> String {
        format!(
            "iters={} substeps={} shock={} stab={} K={} hold={} ticks={}",
            self.iters,
            self.substeps,
            self.shock,
            self.stab,
            self.wake_gate_k,
            self.hold,
            self.ticks
        )
    }

    fn scene(&self) -> (World, Vec<usize>) {
        build_gate_scene(&self.config())
    }
}

/// 与门槛档同参：高度场地坪 + 10×10 网格静置盒 + 1000 个从 12~40 m 落下的动态盒。
fn build_gate_scene(cfg: &PhysConfig) -> (World, Vec<usize>) {
    let mut w = World::new(cfg.clone());
    w.add_heightfield(vxl_phys_narrow::heightfield::HeightField::flat(
        -60.0, -60.0, 121, 121, 1.0, 0.0,
    ));
    for k in 0..10_000usize {
        let x = (k % 100) as f32 - 50.0;
        let z = (k / 100) as f32 - 50.0;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    let mut dyns = Vec::with_capacity(1_000);
    for k in 0..1_000usize {
        let x = ((k * 71) % 97) as f32 / 97.0 * 44.0 - 22.0;
        let z = ((k * 53) % 89) as f32 / 89.0 * 40.0 - 20.0;
        let y = 12.0 + ((k * 29) % 71) as f32 / 71.0 * 28.0;
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
        dyns.push(id as usize);
    }
    (w, dyns)
}

/// 数每个动态体的 sleep→wake 翻转（`ticks` 步），并各留前几次翻转的 tick。
fn flips_of(w: &mut World, dyns: &[usize], ticks: u32) -> (Vec<u32>, Vec<Vec<u64>>) {
    let mut prev: Vec<bool> = dyns.iter().map(|&i| w.bodies.awake[i]).collect();
    let mut flips = vec![0u32; dyns.len()];
    let mut when: Vec<Vec<u64>> = vec![Vec::new(); dyns.len()];
    for _ in 0..ticks {
        w.step();
        for (k, &i) in dyns.iter().enumerate() {
            let a = w.bodies.awake[i];
            if a && !prev[k] {
                flips[k] += 1;
                if when[k].len() < 8 {
                    when[k].push(w.tick);
                }
            }
            prev[k] = a;
        }
    }
    (flips, when)
}

/// 升序切片的 `p` 分位（空切片 → 0）。
fn quant(vs: &[f32], p: f32) -> f32 {
    if vs.is_empty() {
        0.0
    } else {
        vs[((vs.len() - 1) as f32 * p) as usize]
    }
}

/// **稳态抖动归因**（双轴 + 判据分类）。
///
/// 为什么必须看 `|ω|`：睡眠判据是**与门** —— `|v| < sleep_linear && |ω| < sleep_angular`
/// 才累加 `sleep_timer`（`sleep_pass_island` 逐体档），否则**每子步清零**。所以"醒着且
/// `|v|` 远低于阈"**只有一个可能的原因**：`|ω| ≥ sleep_angular`。旧版只打 `|v|`，
/// 于是把"阈下沿"读成极限环 —— 而阈下沿的体本该入睡，方向就错了。
///
/// `lin_over`/`ang_over` = 本帧末速度够得上各自阈值的体数（判据被打回的那一条）；
/// `timer_zero` = 末子步被清零的体数（恰是"这一帧判据没放过它"的体数）。
struct Jitter {
    awake: usize,
    stopped: usize,
    lin_over: usize,
    ang_over: usize,
    timer_zero: usize,
    lin: [f32; 3],
    ang: [f32; 3],
}

impl Jitter {
    fn line(&self, t: u32) -> String {
        format!(
            "  t={t:3} | {:4} | {:4} | {:4} | {:4} | {:4} | {:.4}/{:.4}/{:.4} | {:.4}/{:.4}/{:.4}",
            self.awake,
            self.stopped,
            self.lin_over,
            self.ang_over,
            self.timer_zero,
            self.lin[0],
            self.lin[1],
            self.lin[2],
            self.ang[0],
            self.ang[1],
            self.ang[2]
        )
    }
}

fn jitter_report(w: &World, dyns: &[usize], cfg: &PhysConfig) -> Jitter {
    let (mut awake, mut stopped, mut lin_over, mut ang_over, mut timer_zero) = (0, 0, 0, 0, 0);
    let (mut lin, mut ang): (Vec<f32>, Vec<f32>) = (Vec::new(), Vec::new());
    for &i in dyns {
        if !w.bodies.awake[i] {
            continue;
        }
        awake += 1;
        let (v, o) = (w.bodies.linvel[i].length(), w.bodies.angvel(i).length());
        if v < 1e-3 {
            stopped += 1;
        }
        if v >= cfg.sleep_linear {
            lin_over += 1;
        }
        if o >= cfg.sleep_angular {
            ang_over += 1;
        }
        if w.bodies.sleep_timer[i] == 0.0 {
            timer_zero += 1;
        }
        lin.push(v);
        ang.push(o);
    }
    let cmp = |a: &f32, b: &f32| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal);
    lin.sort_by(cmp);
    ang.sort_by(cmp);
    let tail = |v: &Vec<f32>| {
        [
            quant(v, 0.5),
            quant(v, 0.9),
            v.last().copied().unwrap_or(0.0),
        ]
    };
    Jitter {
        awake,
        stopped,
        lin_over,
        ang_over,
        timer_zero,
        lin: tail(&lin),
        ang: tail(&ang),
    }
}

/// 邻接体按哪条判据够得上"hot"（`sleep_pass_island` 的唤醒条件；睡眠侧速度已清零 ⇒
/// 相对速度 = 清醒侧的 `|linvel|`）。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Hot {
    Quiet,
    Ang,
    Lin,
    Fast,
}

fn hot_class(v: f32, om: f32, cfg: &PhysConfig) -> Hot {
    if v >= FAST_MULT * cfg.sleep_linear {
        Hot::Fast
    } else if v > WAKE_MULT * cfg.sleep_linear {
        Hot::Lin
    } else if om > WAKE_MULT * cfg.sleep_angular {
        Hot::Ang
    } else {
        Hot::Quiet
    }
}

/// 同流形且**醒着**的邻居：`(体号, |v|, |ω|, 判据)`。
fn hot_nbrs(w: &World, id: usize, cfg: &PhysConfig) -> Vec<(u32, f32, f32, Hot)> {
    let mut out = Vec::new();
    for m in w.manifolds() {
        let o = if m.a as usize == id {
            Some(m.b)
        } else if m.b as usize == id {
            Some(m.a)
        } else {
            None
        };
        let Some(o) = o else { continue };
        let oi = o as usize;
        if !w.bodies.awake[oi] {
            continue;
        }
        let (v, om) = (w.bodies.linvel[oi].length(), w.bodies.angvel(oi).length());
        out.push((o, v, om, hot_class(v, om, cfg)));
    }
    out
}

/// 接触度数（接触本体的流形条数，含静态邻居）。
fn degree(w: &World, id: usize) -> usize {
    w.manifolds()
        .iter()
        .filter(|m| m.a as usize == id || m.b as usize == id)
        .count()
}

/// 同流形里的**动态**邻居（去重、升序）。
fn dyn_nbrs(w: &World, id: usize) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    for m in w.manifolds() {
        let o = if m.a as usize == id {
            Some(m.b)
        } else if m.b as usize == id {
            Some(m.a)
        } else {
            None
        };
        if let Some(o) = o {
            let oi = o as usize;
            if w.bodies.is_dynamic(oi) && !out.contains(&oi) {
                out.push(oi);
            }
        }
    }
    out.sort_unstable();
    out
}

/// 唤醒归因计数（见 `wake_attribution`）。
#[derive(Default)]
struct WakeTally {
    flips: u32,
    fast: u32,
    lin: u32,
    ang: u32,
    silent: u32,
    board: Vec<u32>,
}

impl WakeTally {
    /// 角速 hot 的来源榜（体号 / 记数）降序 —— 就是"谁在吵"。
    fn top_sources(&self, n: usize) -> Vec<(u32, u32)> {
        let mut v: Vec<(u32, u32)> = self
            .board
            .iter()
            .enumerate()
            .filter(|(_, &c)| c > 0)
            .map(|(i, &c)| (i as u32, c))
            .collect();
        v.sort_by_key(|&(_, c)| std::cmp::Reverse(c));
        v.truncate(n);
        v
    }
}

/// 逐 tick 扫"睡→醒"翻转，**复算** hot 判据并归类（探针不复现门与岛的调用序，
/// 只复算"那一刻同流形醒着邻居是否够 hot"这一条）。
fn wake_attribution(w: &mut World, dyns: &[usize], ticks: u32, cfg: &PhysConfig) -> WakeTally {
    let mut prev: Vec<bool> = dyns.iter().map(|&i| w.bodies.awake[i]).collect();
    let mut t = WakeTally {
        board: vec![0; w.bodies.len()],
        ..Default::default()
    };
    for _ in 0..ticks {
        w.step();
        for (k, &i) in dyns.iter().enumerate() {
            let a = w.bodies.awake[i];
            if a && !prev[k] {
                t.flips += 1;
                let (mut f, mut l, mut g) = (false, false, false);
                for (o, _, _, kind) in hot_nbrs(w, i, cfg) {
                    match kind {
                        Hot::Fast => f = true,
                        Hot::Lin => l = true,
                        Hot::Ang => {
                            g = true;
                            t.board[o as usize] += 1;
                        }
                        Hot::Quiet => {}
                    }
                }
                if f {
                    t.fast += 1;
                }
                if l {
                    t.lin += 1;
                }
                if g {
                    t.ang += 1;
                }
                if !f && !l && !g {
                    t.silent += 1;
                }
            }
            prev[k] = a;
        }
    }
    t
}

/// 末窗极值：对给定体在**紧接着**的 `win` tick 内逐 tick 采样 `|v|`/`|ω|`。
fn last_window(w: &mut World, ids: &[usize], win: u32) -> Vec<String> {
    let n = ids.len();
    let (mut vmin, mut vmax) = (vec![f32::MAX; n], vec![0.0f32; n]);
    let (mut omin, mut omax) = (vec![f32::MAX; n], vec![0.0f32; n]);
    for _ in 0..win {
        w.step();
        for (k, &i) in ids.iter().enumerate() {
            let (v, o) = (w.bodies.linvel[i].length(), w.bodies.angvel(i).length());
            vmin[k] = vmin[k].min(v);
            vmax[k] = vmax[k].max(v);
            omin[k] = omin[k].min(o);
            omax[k] = omax[k].max(o);
        }
    }
    (0..n)
        .map(|k| {
            format!(
                "  体 {:4}  pos=({:7.3},{:7.3},{:7.3})  rot={:?}  |v| {:.4}..{:.4}  |ω| {:.4}..{:.4}  醒着={}",
                ids[k],
                w.bodies.position[ids[k]].x,
                w.bodies.position[ids[k]].y,
                w.bodies.position[ids[k]].z,
                w.bodies.rot(ids[k]),
                vmin[k],
                vmax[k],
                omin[k],
                omax[k],
                w.bodies.awake[ids[k]],
            )
        })
        .collect()
}

/// 头号体的逐次翻转快照：翻转那一刻的位置/速度/计时器 + 每个醒着邻居的判据标签。
fn head_flip_trace(w: &mut World, top: usize, ticks: u32, cfg: &PhysConfig) {
    let mut prev = w.bodies.awake[top];
    println!("== 头号体 {top} 的翻转快照：tick / y / |v| / |ω| / 计时器 / 接触 / 醒着邻居(体号,|v|,|ω|,判据)");
    for _ in 0..ticks {
        w.step();
        let a = w.bodies.awake[top];
        if a && !prev {
            println!(
                "  t={:3}  y={:8.3}  |v|={:6.3}  |ω|={:6.3}  计时器={:5.2}  接触={}  {:?}",
                w.tick,
                w.bodies.position[top].y,
                w.bodies.linvel[top].length(),
                w.bodies.angvel(top).length(),
                w.bodies.sleep_timer[top],
                degree(w, top),
                hot_nbrs(w, top, cfg)
            );
        }
        prev = a;
    }
}

/// **最小复现**（`min` 子命令，2026-10-15）：两个 0.8 m 立方体按静置高度叠放在瓷砖正中
/// （零穿透、零初速、平放），两者唯一的差别是**绕竖直轴的相对偏航**。
/// 实测（默认档）：
///   - 相对偏航 `0`（或 90°，两面重合）⇒ 10 tick 内入睡、`|ω| → 0`；
///   - 相对偏航 `0.5°…45°`（逐个跑过 0.5 / 1 / 3 / 6.1 / 45）⇒ **睡眠计时器恒为 0、永不入睡**，
///     3000 与 6000 tick 都验过；`|v| ≈ 0.01–0.02`（阈 0.04 之下）、`|ω| ≈ 0.13–0.20`（阈 0.05 的
///     3–4 倍，两体**反相**、位置 3000 tick 不动 ⇒ 是解算器留在速度里的残余，不是真在转）。
///
/// 所以旧口径的两条都是错的：**「两个轴对齐盒叠放就够」不成立**（那种盒会睡），
/// **「底体倾斜」也不是根因** —— 门槛档那对的实位姿等价于「平放 + 偏航 6.1°」（见
/// `docs/EXPERIMENTS.md` 的推导）。驱动项仍是**去穿透偏置速度**（`constraint.rs::soft_contact`
/// 的 `erp·(depth−slop)`）：偏航时底体稳态保持 6 mm 埋深（> `linear_slop` 5 mm）⇒ 偏置恒不为 0，
/// 记账 `bias×1000 ≈ 660/帧`（对齐时 0.01）；`maxcorr=0` 归零偏置 ⇒ 立刻入睡，但体会一直沉下去
/// ⇒ **只是归因，不是修法**。判别矩阵与修法候选见 `docs/EXPERIMENTS.md`「静置叠体」节。
fn minimal_scene(cfg: &PhysConfig, yaw_deg: f32, ticks: u32) {
    let mut w = World::new(cfg.clone());
    for x in 3..=10 {
        for z in 8..=15 {
            w.add_static(
                Shape::Box {
                    half: Vec3::splat(0.5),
                },
                Vec3::new(x as f32, 0.5, z as f32),
                Quat::IDENTITY,
            );
        }
    }
    // 瓷砖格心 (7, 12)：静置面 y = 1.0 ⇒ 底体心 1.4、顶体心 2.2（零缝、零沉入）。
    // 底体绕竖直轴偏航 yaw_deg、顶体不偏航 ⇒ 两体之间的**相对偏航就是 yaw_deg**（接触面错开）。
    let h = yaw_deg.to_radians() / 2.0;
    let mut ids = Vec::new();
    for (y, q) in [
        (1.4f32, Quat::new(0.0, h.sin(), 0.0, h.cos())),
        (2.2f32, Quat::IDENTITY),
    ] {
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(7.0, y, 12.0),
            q,
            1000.0,
        );
        ids.push(id as usize);
    }
    println!(
        "== 最小复现：两盒叠放@瓷砖正中 | 相对偏航={}° iters={} substeps={} stab={} hold={} maxcorr={} ticks={}",
        yaw_deg,
        cfg.velocity_iterations,
        cfg.substeps,
        cfg.stabilization_iterations,
        cfg.settled_hold_iterations,
        cfg.max_corrective_velocity,
        ticks
    );
    let mut zero = vec![0u32; ids.len()];
    let mut tau = vec![0.0f32; ids.len()];
    let mut wmax = vec![0.0f32; ids.len()];
    let (mut vsum, mut wsum) = (vec![0.0f32; ids.len()], vec![0.0f32; ids.len()]);
    let warm = ticks / 6;
    let mut acct = [0u64; 4];
    for t in 1..=ticks {
        w.step();
        let a = vxl_phys_solver::stats::solve_accounting_take();
        acct[0] += a.0;
        acct[1] += a.1;
        acct[2] += a.2;
        acct[3] += a.3;
        for (k, &i) in ids.iter().enumerate() {
            let (v, o) = (w.bodies.linvel[i].length(), w.bodies.angvel(i).length());
            if w.bodies.sleep_timer[i] == 0.0 {
                zero[k] += 1;
            }
            tau[k] = tau[k].max(w.bodies.sleep_timer[i]);
            wmax[k] = wmax[k].max(o);
            if t > warm {
                vsum[k] += v;
                wsum[k] += o;
            }
        }
        if t % 250 == 0 || t == ticks {
            let mut line = format!("  t={t:5} |");
            for &i in &ids {
                line.push_str(&format!(
                    " 体{i} y={:8.5} |v|={:7.4} |ω|={:7.4} 醒={} τ={:4.2} |",
                    w.bodies.position[i].y,
                    w.bodies.linvel[i].length(),
                    w.bodies.angvel(i).length(),
                    if w.bodies.awake[i] { 1 } else { 0 },
                    w.bodies.sleep_timer[i],
                ));
            }
            println!("{line}");
        }
    }
    let span = (ticks - warm) as f32;
    for (k, &i) in ids.iter().enumerate() {
        println!(
            "  体{i}：计时器清零 {}/{} 帧 / τ峰值 {:.3} / |ω|峰值 {:.4} / 末窗均值 |v| {:.4} |ω| {:.4} / 醒着={}",
            zero[k],
            ticks,
            tau[k],
            wmax[k],
            vsum[k] / span,
            wsum[k] / span,
            w.bodies.awake[i],
        );
    }
    println!(
        "== 解算记账（每 tick 均值）：点 {} / 分离点 {} / spec×1000 {} / bias×1000 {}",
        acct[0] as f64 / ticks as f64,
        acct[1] as f64 / ticks as f64,
        acct[2] as f64 / ticks as f64,
        acct[3] as f64 / ticks as f64
    );
}

/// `min` 子命令的位置参数（顺序固定，缺省值全取 `PhysConfig::default()`）：
/// **相对偏航(度) / 迭代 / 子步 / 无偏置末趟 / 安座趟 / 去穿透偏置上限 / tick 数**。
fn minimal_main(rest: &[String]) {
    let d = PhysConfig::default();
    let mut it = rest.iter();
    let mut n = |dv: f32| it.next().and_then(|s| s.parse::<f32>().ok()).unwrap_or(dv);
    let yaw_deg = n(6.1);
    let cfg = PhysConfig {
        velocity_iterations: n(d.velocity_iterations as f32) as u32,
        substeps: n(d.substeps as f32) as u32,
        stabilization_iterations: n(d.stabilization_iterations as f32) as u32,
        settled_hold_iterations: n(d.settled_hold_iterations as f32) as u32,
        max_corrective_velocity: n(d.max_corrective_velocity),
        ..PhysConfig::default()
    };
    let ticks = n(3000.0) as u32;
    minimal_scene(&cfg, yaw_deg, ticks);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("min") {
        minimal_main(&args[1..]);
        return;
    }
    let k = knobs();
    let cfg = k.config();
    println!("== 档：{}", k.label());
    // 第一趟：稳态抖动的**双轴**读数（含判据分类）。
    let (mut w, dyns) = k.scene();
    println!(
        "== 抖动归因：tick | 醒着 | |v|<1e-3 | |v|≥{:.2} | |ω|≥{:.2} | 计时器=0 | |v| p50/p90/max | |ω| p50/p90/max",
        cfg.sleep_linear, cfg.sleep_angular
    );
    let cad = if k.ticks <= 600 { 50 } else { k.ticks / 8 };
    for t in 1..=k.ticks {
        w.step();
        if t % cad == 0 || t == k.ticks {
            println!("{}", jitter_report(&w, &dyns, &cfg).line(t));
        }
    }

    // 第二趟：翻转榜（哪个体在翻）。
    let (mut w2, dyns2) = k.scene();
    let (flips, when) = flips_of(&mut w2, &dyns2, k.ticks);
    let mut order: Vec<usize> = (0..dyns2.len()).collect();
    order.sort_by_key(|&k| std::cmp::Reverse(flips[k]));
    println!("== 翻转榜（前 8）：体 id / 翻转数 / 前几次翻转 tick");
    for &k in order.iter().take(8) {
        if flips[k] == 0 {
            break;
        }
        println!(
            "  体 {:4}  翻转 {:2}  次 @ {:?}",
            dyns2[k], flips[k], when[k]
        );
    }
    let total: u32 = flips.iter().sum();
    let n_flip = flips.iter().filter(|&&c| c > 0).count();
    let max_flips = flips.iter().copied().max().unwrap_or(0);
    println!(
        "== 合计：{} 个动态体里有 {} 个发生过翻转，总翻转 {} 次；最大单体贴 {} 次 ⇒ {:.3} 次/秒/体（门槛档判据 < 1.0 ⇒ {}）",
        dyns2.len(),
        n_flip,
        total,
        max_flips,
        max_flips as f64 / (k.ticks as f64 / 60.0),
        if (max_flips as f64 / (k.ticks as f64 / 60.0)) < 1.0 {
            "过"
        } else {
            "**不过**"
        }
    );

    // 第三趟：唤醒归因（谁按的哪条判据）。
    let (mut w3, dyns3) = k.scene();
    let tally = wake_attribution(&mut w3, &dyns3, k.ticks, &cfg);
    println!(
        "== 唤醒归因（{} tick）：翻转 {} 次 ⇒ 强撞直通 {} / 线性 hot {} / 角速 hot {} / 无来源 {}",
        k.ticks, tally.flips, tally.fast, tally.lin, tally.ang, tally.silent
    );
    println!(
        "   角速 hot 来源榜（体 id, 记数）：{:?}",
        tally.top_sources(6)
    );

    // 第四趟：头号体 + 其动态邻居在末窗(500→600)的 |v|/|ω| 极值（证"残余抖动不衰减"）。
    let top = dyns2[order[0]];
    let (mut w4, _) = k.scene();
    let warm = k.ticks * 5 / 6;
    for _ in 0..warm {
        w4.step();
    }
    let mut ids = vec![top];
    ids.extend(dyn_nbrs(&w4, top));
    // 二阶动态邻居（头号体 → 10271 → 10659 这种"被夹住"的簇：只看一阶会漏掉另一半支撑）
    for &i in ids.clone().iter() {
        for j in dyn_nbrs(&w4, i) {
            if !ids.contains(&j) {
                ids.push(j);
            }
        }
    }
    println!("== 末窗 {warm}→{} 极值（头号体 + 当时的动态邻居）", k.ticks);
    for line in last_window(&mut w4, &ids, k.ticks - warm) {
        println!("{line}");
    }
    println!("== 末窗结束时的邻接（度 = 接触条数；含静态）");
    for &i in &ids {
        println!(
            "  体 {:4}  度={}  静态邻居={}  动态邻居={:?}",
            i,
            degree(&w4, i),
            degree(&w4, i) - dyn_nbrs(&w4, i).len(),
            dyn_nbrs(&w4, i)
        );
    }
    let (mut w5, _) = k.scene();
    head_flip_trace(&mut w5, top, k.ticks, &cfg);
}
