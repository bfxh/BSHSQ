//! **m0 门槛档的"唤醒翻转取证"**（2026-10-13）：`m0_gates` 的 `max_wake_flips` / 速率是当前
//! **卡住两笔改动**的判据（P20 目标侧姿态插值、P19 薄壳两面性）—— 它们都只是"微动了 CCD 的命中
//! 相位"，就把这条从 7 顶到 **10**（1.000/s，闸是 `< 1.0`）。本探针把这条**变成可解释的量**：
//! **哪个体在翻转、在哪些 tick、翻转前后它长什么样**（位置/速度/睡眠计时器）。
//!
//! 场景与 `m0_gates::build_gate_scene` **逐字同参**（1 万静态盒地坪 + 1 千动态盒 12~40 m 雨落，
//! 600 tick、默认档）。跑法：`cargo run --release -q -p vxl-phys --example m0_flip_forensics`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 与门槛档同参：高度场地坪 + 10×10 网格静置盒 + 1000 个从 12~40 m 落下的动态盒。
fn build_gate_scene() -> (World, Vec<usize>) {
    let mut w = World::new(PhysConfig::default());
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

/// 与 `id` 同处一个流形的邻居：`(体号, 醒着?, |v|)` —— **唤醒源候选**。
fn neighbours(w: &World, id: usize) -> Vec<(u32, bool, f32)> {
    let mut out = Vec::new();
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
            out.push((o, w.bodies.awake[oi], w.bodies.linvel[oi].length()));
        }
    }
    out
}

fn main() {
    let (mut w, dyns) = build_gate_scene();
    let (flips, when) = flips_of(&mut w, &dyns, 600);
    let mut order: Vec<usize> = (0..dyns.len()).collect();
    order.sort_by_key(|&k| std::cmp::Reverse(flips[k]));
    println!("== 翻转榜（前 8）：体 id / 翻转数 / 前几次翻转 tick");
    for &k in order.iter().take(8) {
        if flips[k] == 0 {
            break;
        }
        println!(
            "  体 {:4}  翻转 {:2}  次 @ {:?}",
            dyns[k], flips[k], when[k]
        );
    }
    let total: u32 = flips.iter().sum();
    let n_flip = flips.iter().filter(|&&c| c > 0).count();
    println!(
        "== 合计：{} 个动态体里有 {} 个发生过翻转，总翻转 {} 次；最大单体贴 {} 次（= 门槛档 max_wake_flips）",
        dyns.len(),
        n_flip,
        total,
        flips.iter().copied().max().unwrap_or(0)
    );

    // 第二趟：只盯头号体，打"翻转那一刻"的快照 + **谁在吵它**。
    let top = dyns[order[0]];
    let (mut w, _) = build_gate_scene();
    let mut prev = w.bodies.awake[top];
    println!("== 头号体 {top} 的翻转快照：tick / y / |v| / |ω| / 计时器 / 接触 / 醒着的邻居");
    for _ in 0..600 {
        w.step();
        let a = w.bodies.awake[top];
        if a && !prev {
            let nb = neighbours(&w, top);
            let awake_n = nb.iter().filter(|n| n.1).count();
            println!(
                "  t={:3}  y={:8.3}  |v|={:6.3}  |ω|={:6.3}  计时器={:5.2}  接触={} 醒着={awake_n}  邻居={:?}",
                w.tick,
                w.bodies.position[top].y,
                w.bodies.linvel[top].length(),
                w.bodies.angvel(top).length(),
                w.bodies.sleep_timer[top],
                nb.len(),
                nb.iter().take(4).collect::<Vec<_>>()
            );
        }
        prev = a;
    }
}
