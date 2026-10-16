//! **排布判别探针** —— 回答一个问题：两个邻域相位（密度 + 力）的代价，有多少来自
//! **「槽位 → 空间位置」的映射**（访存局部性），有多少是每粒的固有工作量？
//!
//! 为什么需要它：`PLAN-gpu.md` §20 把 10M 档的 209.75 ms 解成「52% 每粒固定 + 48% 候选收集」，
//! 然后打算用「workgroup 按格分块 + 共享内存」去砍那 48%。**那个前提要验**：粒子数组是
//! **索引序**（`grid.wgsl` 的 `items[slot] = i` 是置换，`pos` 不按格重排）⇒ 一个 workgroup 的
//! 64 个线程在空间上是散的，**它们不共享邻域**。本探针用**零内核改动**的方式把这件事量出来。
//!
//! **做法（同位置、同工作量、只改映射）**：把同一批位置重新分配到"槽位"上——
//! - `natural`：槽位 k ← 第 k 个粒子（**今天的数据路径**，`cell_items` 是任意置换）；
//! - `sorted`：槽位 k ← 格序第 k 个粒子（**按格重排的理想档** ⇒ "把数据按格重排"的上限）；
//! - `random`：槽位 k ← 随机置换（**完全没有局部性**的下界）。
//!
//! 三档的**位置集合完全相同** ⇒ 每粒候选量与算术同量（用「候选对数」自检守住），差别只剩
//! "哪些线程在读哪些地址" ⇒ 计时差就是局部性的纯效应，且能顺带回答"探针场景的读数有没有
//! 被初始晶格序美化"（`natural` 与 `random` 的差就是这个红利）。
//!
//! ⚠️ **本探针不实现"按格重排"的生产路径**，只量它的**上限**；也不含网格重建与积分
//! （只跑两相位核，见 `probe::phases_on_adapter`）。读数用**两档 repeats 解 `t = c + R/r`**：
//! `R` 是那次同步回读（dens+acc+xsph = 28 B/粒，10M 档 ≈180 ms），它**与排布无关**，所以要解出来
//! 而不是让它污染 `c`（r=16 时会把 c 虚高 ≈11 ms）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_layout_probe -- [n] [--adapter K]
//!        [--cpu-threads K] [--rounds K]`

//!
//! **拆分（2026-10-17）**：场景与排布构造在 `scene.rs`、判据与诊断在 `diag.rs`；本文件只留编排与打印。
//! 切分只做搬移与 `pub(super)`，算式 / 注释 / 打印格式一字未改。

mod diag;
mod scene;

use diag::{diag_curve, equivalence_check, report_table, solve, Meas};
use scene::{prepare, Arrangement, Layout};

use vxl_phys_gpu::probe::{self, PhaseInputs, PhaseParams};

/// 交错轮计时：每轮把三档都过一遍（两档 repeats），档内按 `ok` 取更保守的那次。
fn measure_all(
    arrs: &mut [(Layout, Arrangement)],
    params: PhaseParams,
    np: usize,
    r1: usize,
    r2: usize,
    rounds: usize,
) -> Vec<Meas> {
    let mut best: Vec<Option<Meas>> = vec![None; arrs.len()];
    for round in 0..rounds.max(1) {
        for (i, (l, a)) in arrs.iter_mut().enumerate() {
            let (p1, setup1) = run_once(a, params, np, r1);
            let (p2, _) = run_once(a, params, np, r2);
            let (c, r, ok) = solve(r1, p1, r2, p2);
            println!(
                "  · 轮 {} {:<6}：r={r1} → {p1:.2} | r={r2} → {p2:.2} ms ⇒ c = {c:.2} ms{}，R = {r:.1} ms（setup {setup1:.0} ms）",
                round + 1,
                l.name(),
                if ok { "" } else { "（R 不可解 †）" }
            );
            let cand = Meas { c, r, p1, p2, ok };
            best[i] = Some(match best[i] {
                // 可解档之间取**更小**的 `c`；不可解档（†）保留**更大**的那次当上界
                // （保守：别把不确定读成收益）。
                Some(prev) if prev.ok == ok && (prev.c <= c) == ok => prev,
                _ => cand,
            });
        }
    }
    best.into_iter().map(|b| b.unwrap()).collect()
}

/// 跑一次 `phases_on_adapter`，返回 `(per_dispatch_ms, setup_ms)`（= `c + R/r`）。
fn run_once(arr: &mut Arrangement, params: PhaseParams, np: usize, reps: usize) -> (f64, f32) {
    let pmass = vec![params.mass; np];
    let inputs = PhaseInputs {
        pos_flat: &arr.pos_flat,
        vel_flat: &arr.vel_flat,
        pmass: &pmass,
        press: &arr.press,
        cell_start: &arr.grid.start,
        cell_items: &arr.grid.items,
    };
    let out = probe::phases_on_adapter(0, &inputs, params, np, reps);
    if let Some(e) = out.error {
        panic!("GPU 路径不可用：{e}");
    }
    if arr.out.is_empty() {
        // 拼成一条：dens[n] ‖ acc[3n] ‖ xsph[3n]
        arr.out = Vec::with_capacity(out.dens.len() * 7);
        arr.out.extend_from_slice(&out.dens);
        arr.out.extend_from_slice(&out.acc);
        arr.out.extend_from_slice(&out.xsph);
    }
    (out.per_dispatch_ms as f64, out.setup_ms)
}

fn parse_args() -> (usize, usize, usize, Vec<String>) {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let rest: Vec<String> = args.collect();
    let get = |key: &str, dflt: usize| -> usize {
        rest.iter()
            .position(|a| a == key)
            .and_then(|k| rest.get(k + 1))
            .and_then(|v| v.parse().ok())
            .unwrap_or(dflt)
    };
    (n, get("--cpu-threads", 1), get("--rounds", 2), rest)
}

fn get(rest: &[String], key: &str, dflt: usize) -> usize {
    rest.iter()
        .position(|a| a == key)
        .and_then(|k| rest.get(k + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(dflt)
}

/// `--diag 5 60 300 600`：取 `--diag` 后面的整数串当检查点。
fn diag_checkpoints(rest: &[String]) -> Option<Vec<usize>> {
    let k = rest.iter().position(|a| a == "--diag")?;
    let v: Vec<usize> = rest[k + 1..]
        .iter()
        .take_while(|a| !a.starts_with("--"))
        .filter_map(|a| a.parse().ok())
        .collect();
    Some(if v.is_empty() {
        vec![5, 60, 300, 600]
    } else {
        v
    })
}

fn main() {
    let (n, cpu_threads, rounds, rest) = parse_args();
    // `--diag-only` 纯 CPU：只跑局部性衰减曲线（不碰 GPU、不需要独占锁）。
    let diag = diag_checkpoints(&rest);
    if rest.iter().any(|a| a == "--diag-only") {
        diag_curve(
            n,
            cpu_threads,
            &diag.unwrap_or_else(|| vec![5, 60, 300, 600]),
        );
        return;
    }
    let list = probe::adapters();
    if list.is_empty() {
        println!("（本机无可用适配器 ⇒ CI 下的正常路径；本探针不构成门禁）");
        return;
    }
    for (i, a) in list.iter().enumerate() {
        println!("  [{i}] {a}");
    }

    let (f, b, dims, mut arrs) = prepare(n, cpu_threads);
    let np = f.len();

    // —— 计时：交错轮（两轮取最小），每档两档 repeats 解 `c` 与 `R` ——
    let h = f.config().smoothing_radius;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let g = f.config().gravity;
    let params = PhaseParams {
        gmin: [b.min.x, b.min.y, b.min.z],
        inv: b.inv,
        h2: h * h,
        k6,
        w0: k6 * h * h * h * h * h * h,
        mass: f.particle_mass(),
        ks: 45.0 / (std::f32::consts::PI * h.powi(6)),
        h,
        alpha_c: f.config().artificial_viscosity * f.config().sound_speed,
        _pad0: 0.0,
        gvec: [g.x, g.y, g.z],
        n_fluid: np as u32,
        nx: dims[0],
        ny: dims[1],
        nz: dims[2],
        n_total: np as u32,
    };

    // 小档 repeats：`random` 档在 10M 会慢到 ~3.4 s/子步 ⇒ r 越大越久（一发 ≈ r×c）。
    // `(8, 32)` 的固定项占比已足够小，且一发只 3–110 s。
    let (r1, r2) = (get(&rest, "--r1", 8), get(&rest, "--r2", 32));
    let best = measure_all(&mut arrs, params, np, r1, r2, rounds);
    report_table(&arrs, &best);
    equivalence_check(&arrs);
    if let Some(cp) = diag {
        println!();
        diag_curve(n, cpu_threads, &cp);
    }
    if n == 215 {
        // **标定**：n=215 正是 10M 档（215³ = 9 938 375 粒）⇒ `natural` 档应当落在真实管线
        // 相位账的同一个数上（§18.2 抬预算后：密度 93.82 + 力段 115.93 = 209.75 ms/tick ÷ 4 子步）。
        // 对上了 ⇒ 本探针复现的就是管线的访问模式，读数可直接与 tick 读数换算。
        println!(
            "  标定：真实管线 10M 档相位账 (93.82 + 115.93)/4 子步 = 52.44 ms/子步 ⇒ natural 档应 ≈ 该值"
        );
    }
    println!(
        "  ⇒ 判读：`sorted` = 「把数据按格重排」的上限（含去掉 `cell_items` 间接层）；`random` ≈ 生产上\n     流动后索引序与空间解耦的档；`natural` 与 `random` 的差 = 探针场景自带的局部性红利。"
    );
}
