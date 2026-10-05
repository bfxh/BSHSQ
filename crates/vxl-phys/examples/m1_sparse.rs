//! **M1「简单碰撞」档**（`SPEC.md` §3 的 8B/30FPS 宣称载体）—— 10 万动态体、**稀疏接触**。
//!
//! **为什么另立**（`OPEN-PROBLEMS.md` 待裁决 #3 的定案，2026-10-05）：`m1_scale` 是 10 万体**密堆**
//! （每体平均 6+ 接触、89% 成本在解算相），它只能当**上界旁证**；"30 FPS @8B 简单碰撞"要判，
//! 得用**低接触密度**的同规模场景。本档就是那个场景：动态体按 **3 m 间距**铺开（体尺寸 0.8 m
//! ⇒ 彼此**不相邻**），每个体只与地面有接触 ⇒ 接触密度 ≈ 1/体，而不是密堆的 6+/体。
//!
//! **口径（`SPEC` §5）**：性能门**只在参考硬件**判定。本档在本机**只报不判**（打印读数 +
//! 一行"旁证"标注），不得据此宣称达标/不达标。
//!
//! 运行：`cargo run --release -p vxl-phys --example m1_sparse -- [threads] [n_dynamic] [ticks] [iters]`
//! 默认：threads=8、动态 100000（317×317 铺开、间距 3 m ⇒ 约 950 m 见方）、400 tick、迭代 16。

use std::time::Instant;

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// **体间距**（米）：体半长 0.4 ⇒ 间距 3.0 时相邻体净空 2.2 m ⇒ 落到地面后**互不接触**。
const GAP: f32 = 3.0;

#[allow(clippy::too_many_lines)] // 单文件示例：参数解析 + 建场 + 计时打印在一个 main 里
fn main() {
    let mut args = std::env::args().skip(1);
    let threads: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(8);
    let n_dynamic: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    let ticks: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(400);
    let iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);

    let cfg = PhysConfig {
        velocity_iterations: iters,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    // 地面：铺满动态体覆盖面的静态瓦片（每体正下方一块，够它站住即可）。
    let side = (n_dynamic as f64).sqrt().ceil() as usize;
    for k in 0..side * side {
        let (x, z) = ((k % side) as f32, (k / side) as f32);
        w.add_static(
            Shape::Box {
                half: Vec3::new(GAP * 0.5, 0.5, GAP * 0.5),
            },
            Vec3::new(x * GAP, 0.5, z * GAP),
            Quat::IDENTITY,
        );
    }
    // 动态体：同样按 `GAP` 铺开、从 `y = 1.2` 落下（离地 1.2 体半长内 ⇒ 落地即静）。
    for k in 0..n_dynamic {
        let (x, z) = ((k % side) as f32, (k / side) as f32);
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x * GAP, 1.2, z * GAP),
            Quat::IDENTITY,
            1.0,
        );
    }

    // 计时：全窗口逐 tick；**尾窗**（后 1/4）单独报 —— 坍塌/落地瞬态与稳态分开看（#1 定案）。
    let mut ms: Vec<f64> = Vec::with_capacity(ticks as usize);
    for _ in 0..ticks {
        let t0 = Instant::now();
        w.step();
        ms.push(t0.elapsed().as_secs_f64() * 1000.0);
    }
    let tail = ms[ms.len() * 3 / 4..].to_vec();
    let (all_avg, tail_p50, tail_avg, tail_max) = (
        ms.iter().sum::<f64>() / ms.len() as f64,
        pct(&tail, 0.50),
        tail.iter().sum::<f64>() / tail.len() as f64,
        tail.iter().cloned().fold(0.0f64, f64::max),
    );
    println!(
        "M1-SPARSE  threads={threads} 动态={n_dynamic} 静态={} 间距={GAP} tick={ticks} iters={iters}",
        side * side
    );
    println!(
        "  全窗 均 {all_avg:.2} ms | 尾窗 均 {tail_avg:.2} / p50 {tail_p50:.2} / max {tail_max:.2} ms"
    );
    println!(
        "  口径：SPEC §5 —— 性能门**只在参考硬件**判定；本机读数为**旁证**，本档只报不判（30 FPS ⇒ ≤33.3 ms）"
    );
}

/// 分位（`q ∈ [0,1]`；输入会被就地排序的副本，规模量级 = tick 数）。
fn pct(v: &[f64], q: f64) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let i = ((s.len() as f64 - 1.0) * q).round() as usize;
    s.get(i).copied().unwrap_or(0.0)
}
