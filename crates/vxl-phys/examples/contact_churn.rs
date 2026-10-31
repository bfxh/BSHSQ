//! **M1 只读探针：一 tick 内「新接触点」比例 + 速度预测覆盖率**
//! （`docs/M1-RULING.md` §3.5 的复算入口）。
//!
//! 用法：`cargo run --release -p vxl-phys --example contact_churn -- <tower|pile> <layers> [side|ticks] [ticks] [--predict]`
//!   tower 25 120            —— 25 层单柱（对齐 tower25）
//!   pile  5 20 120          —— 5 层 × 20×20（对齐 pile5）
//!   tower 25 120 --predict  —— 追加反事实预测趟（不改动力学），量"整 tick 视野能提前看到多少接触"
//!
//! `--predict` 的口径：tick 首子步在同一批候选对/同一速度上另跑一趟 `predict_dt = 整 tick`
//! 的窄相，然后看**后续子步的真实接触点**有多少已在这份预测集里。
//! 覆盖率为 100% ⇒ "检测每 tick 一次 + 速度预测"就够，不需要 swept speculative 候选对。

use std::collections::BTreeSet;

use vxl_phys::world_step::contact_probe::{self, LocStats, Point, Sample};
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

struct Args {
    mode: String,
    layers: usize,
    side: usize,
    ticks: u64,
    predict: bool,
}

#[derive(Default)]
struct Churn {
    new_pts: u64,
    all_pts: u64,
    worst: f64,
    worst_tick: u64,
}

#[derive(Default)]
struct Cover {
    need: u64,
    covered: u64,
    new_need: u64,
    new_covered: u64,
    miss_feature: u64,
    miss_narrow: u64,
    miss_broad: u64,
    pred_all: u64,
    pred_spec: u64,
    hit: u64,
}

#[derive(Default)]
struct Tick {
    pred: BTreeSet<u64>,
    pred_pairs: BTreeSet<(u32, u32)>,
    first: BTreeSet<u64>,
    pairs: BTreeSet<(u32, u32)>,
    actual: BTreeSet<u64>,
}

fn pair_of(id: u64) -> (u32, u32) {
    ((id >> 44) as u32, ((id >> 24) & 0xF_FFFF) as u32)
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let predict = argv.iter().any(|s| s == "--predict");
    let mut rest = argv.into_iter().filter(|s| s != "--predict");
    let mode = rest.next().unwrap_or_else(|| "tower".to_string());
    let layers: usize = rest.next().and_then(|s| s.parse().ok()).unwrap_or(12);
    let (side, ticks) = if mode == "pile" {
        (
            rest.next().and_then(|s| s.parse().ok()).unwrap_or(20),
            rest.next().and_then(|s| s.parse().ok()).unwrap_or(120),
        )
    } else {
        (1, rest.next().and_then(|s| s.parse().ok()).unwrap_or(120))
    };
    Args {
        mode,
        layers,
        side,
        ticks,
        predict,
    }
}

fn build_world(layers: usize, side: usize) -> World {
    // substeps 必须 >1：单子步时"tick 内新增"恒为 0
    let mut w = World::new(PhysConfig {
        substeps: 4,
        ..PhysConfig::default()
    });
    w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.1, 6.0),
        },
        Vec3::new(0.0, -0.1, 0.0),
        Quat::IDENTITY,
    );
    for l in 0..layers {
        for iz in 0..side {
            for ix in 0..side {
                let x = (ix as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                let z = (iz as f32 - (side as f32 - 1.0) * 0.5) * 0.52;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(x, 0.25 + 0.52 * l as f32, z),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

fn start_tick(s: &Sample, cover: &mut Cover) -> Tick {
    let mut t = Tick::default();
    for p in &s.predicted {
        t.pred.insert(p.id);
        t.pred_pairs.insert(pair_of(p.id));
        if p.speculative {
            cover.pred_spec += 1;
        }
    }
    t.pairs.extend(s.pairs.iter().copied());
    for p in &s.points {
        if !p.speculative {
            t.first.insert(p.id);
            t.actual.insert(p.id);
        }
    }
    t
}

/// 逐点归类（`cover` 只统计**后续子步**的实点）。
fn scan_point(p: &Point, t: &Tick, cover: &mut Cover) {
    if p.speculative {
        return;
    }
    cover.need += 1;
    let got = t.pred.contains(&p.id);
    if got {
        cover.covered += 1;
    }
    if t.first.contains(&p.id) {
        return;
    }
    cover.new_need += 1;
    if got {
        cover.new_covered += 1;
        return;
    }
    let pair = pair_of(p.id);
    // 同体对首子步已有流形 ⇒ 新点来自**特征切换**（裁剪/顶点换面），
    // 速度充气补不上（它只能把"已知特征"看远，不能预知换哪组特征）。
    if t.pred_pairs.contains(&pair) {
        cover.miss_feature += 1;
    } else if t.pairs.contains(&pair) {
        cover.miss_narrow += 1;
    } else {
        cover.miss_broad += 1;
    }
}

fn finish_tick(t: &Tick, cover: &mut Cover) {
    if t.pred.is_empty() {
        return;
    }
    cover.pred_all += t.pred.len() as u64;
    cover.hit += t.pred.intersection(&t.actual).count() as u64;
}

fn analyze(rows: &[Sample]) -> (Churn, Cover) {
    let (mut churn, mut cover) = (Churn::default(), Cover::default());
    let mut seen: BTreeSet<u64> = BTreeSet::new();
    let mut tick = Tick::default();
    let mut cur = u64::MAX;
    for s in rows {
        if s.tick != cur {
            if cur != u64::MAX {
                finish_tick(&tick, &mut cover);
            }
            tick = start_tick(s, &mut cover);
            seen = tick.first.clone();
            cur = s.tick;
            continue;
        }
        for p in &s.points {
            churn.all_pts += 1;
            if seen.insert(p.id) {
                churn.new_pts += 1;
            }
            scan_point(p, &tick, &mut cover);
        }
        let frac = churn.new_pts as f64 / churn.all_pts.max(1) as f64;
        if frac > churn.worst {
            churn.worst = frac;
            churn.worst_tick = s.tick;
        }
    }
    finish_tick(&tick, &mut cover);
    (churn, cover)
}

fn report(a: &Args, churn: &Churn, cover: &Cover, loc: &LocStats) {
    println!(
        "[churn] {} {} 层 × {}² / {} tick / substeps=4：新增点 {} / 观测点 {} = {:.2}%",
        a.mode,
        a.layers,
        a.side,
        a.ticks,
        churn.new_pts,
        churn.all_pts,
        churn.new_pts as f64 / churn.all_pts.max(1) as f64 * 100.0
    );
    println!(
        "[churn] 最差单 tick：{:.2}%（tick {}）",
        churn.worst * 100.0,
        churn.worst_tick
    );
    let loc_all = loc.same_loc + loc.new_loc + loc.new_pair;
    let pct = |n: u64| n as f64 / loc_all.max(1) as f64 * 100.0;
    println!(
        "[pos] 后续子步「换特征」点 {loc_all}：同位置换 ID {} = {:.2}% / 同对真新位置 {} = {:.2}% / 新体对 {}",
        loc.same_loc,
        pct(loc.same_loc),
        loc.new_loc,
        pct(loc.new_loc),
        loc.new_pair
    );
    if !a.predict {
        return;
    }
    let hit_pct = |n: u64, d: u64| n as f64 / d.max(1) as f64 * 100.0;
    println!(
        "[cover] 后续子步真实点 {}，其中已在整 tick 预测集里 {} = **{:.2}%**",
        cover.need,
        cover.covered,
        hit_pct(cover.covered, cover.need)
    );
    println!(
        "[cover] 其中**首子步还没有**的新点 {}，被预测集覆盖 {} = **{:.2}%**",
        cover.new_need,
        cover.new_covered,
        hit_pct(cover.new_covered, cover.new_need)
    );
    println!(
        "[cover] 新点漏掉的 {}：同对换特征 {} / 窄相视野拒收 {} / 宽相无候选对 {}",
        cover.miss_feature + cover.miss_narrow + cover.miss_broad,
        cover.miss_feature,
        cover.miss_narrow,
        cover.miss_broad
    );
    println!(
        "[cover] 预测点 {}（投机 {} = {:.1}%），命中后续真实点 {} = {:.2}%",
        cover.pred_all,
        cover.pred_spec,
        hit_pct(cover.pred_spec, cover.pred_all),
        cover.hit,
        hit_pct(cover.hit, cover.pred_all)
    );
}

/// warm 匹配读数（求解器自带的只读计数器，见 `vxl_phys_solver::stats`）。
fn report_warm() {
    let (e, f, m) = vxl_phys_solver::warm_match_stats_take();
    let t = (e + f + m).max(1) as f64;
    println!(
        "[warm] 精确 {}（{:.1}%） / 回退 {}（{:.1}%） / 未匹配 {}（{:.1}%）",
        e,
        100.0 * e as f64 / t,
        f,
        100.0 * f as f64 / t,
        m,
        100.0 * m as f64 / t
    );
    let (d, dist, j) = vxl_phys_solver::warm_match::miss_gate_take();
    println!(
        "[warm] 未匹配拒因：深度门 {} / 距离门 {} / 深度跳变门 {}（自检：和 {} == 未匹配 {}）",
        d,
        dist,
        j,
        d + dist + j,
        m
    );
}

fn main() {
    let a = parse_args();
    let mut w = build_world(a.layers, a.side);
    contact_probe::enable(true);
    contact_probe::counterfactual(a.predict);
    for _ in 0..a.ticks {
        w.step();
    }
    let rows = contact_probe::take();
    let loc = contact_probe::loc_stats();
    contact_probe::enable(false);
    let (churn, cover) = analyze(&rows);
    report(&a, &churn, &cover, &loc);
    report_warm();
}
