//! CLI：打印阈值空间审计表；`--check` 时把 `P_WAKE(After)` 当门（非 Proved ⇒ 退出码 1）。
//!
//! 用法：
//!   cargo run -p bshsq-sleep-fsm --release --            # 表 + 判决（eps = 0）
//!   cargo run -p bshsq-sleep-fsm --release -- --eps 1e-3 # 含跨平台分歧带
//!   cargo run -p bshsq-sleep-fsm --release -- --check    # 门：不 Proved 就红

use bshsq_sleep_fsm::{audit, Consts, Model, Row, Ternary, Verdict};

fn parse_eps(args: &[String]) -> f32 {
    let mut eps = 0.0f32;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if a == "--eps" {
            if let Some(v) = it.next().and_then(|s| s.parse::<f32>().ok()) {
                eps = v;
            }
        }
    }
    eps
}

fn cell(v: Ternary) -> &'static str {
    match v {
        Ternary::True => " T ",
        Ternary::False => " F ",
        Ternary::Unknown => " X ",
    }
}

fn print_sweep(rows: &[Row], wake_gate_k: u32) {
    println!("  相对速度扫描（kin=true, depth=0, ang=0, K={wake_gate_k}）");
    println!("   rel(m/s)      before  after");
    let mut seen = Vec::new();
    for r in rows {
        let s = r.s;
        if s.kin && s.depth == 0.0 && s.ang == 0.0 && s.wake_gate_k == wake_gate_k {
            if seen.contains(&s.rel) {
                continue;
            }
            seen.push(s.rel);
            println!(
                "   {:>7.4}       {}   {}",
                s.rel,
                cell(r.before),
                cell(r.after)
            );
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let eps = parse_eps(&args);
    let check = args.iter().any(|a| a == "--check");
    let c = Consts::default();
    let a = audit(&c, eps);
    println!("== 睡眠/唤醒判据三值审计（Kleene K3）==");
    println!(
        "   常数：sleep_linear={} sleep_angular={} contact_skin={} kin_eps={:e}；eps={:e}",
        c.sleep_linear, c.sleep_angular, c.contact_skin, c.kin_eps, eps
    );
    println!("   Before = 只靠 kin_deep / K==0 / 强撞绕过接触数门");
    println!("   After  = 另把『运动学支撑在动』当绕过支（与 kin_deep 同理由）");
    print_sweep(&a.rows, 8);
    println!();
    println!("   格子总数 {}，含 X 输入的 {} 格", a.rows.len(), a.x_rows);
    println!(
        "   P_WAKE：Before = {}（二值反例 {} 格）| After = {}",
        a.p_wake_before.as_str(),
        a.refuted_before,
        a.p_wake_after.as_str()
    );
    if check && a.p_wake_after != Verdict::Proved {
        println!("❌ P_WAKE(After) != Proved ⇒ 判据回归（睡着的载荷可能不被叫醒）");
        std::process::exit(1);
    }
    if let Some(w) = a.rows.iter().find(|r| {
        r.s.kin
            && r.s.depth == 0.0
            && r.s.ang == 0.0
            && r.s.wake_gate_k == 8
            && r.before == Ternary::True
    }) {
        let _ = Model::After;
        println!(
            "   Before 的『能醒』悬崖起点：rel = {:.4} m/s（= 8×sleep_linear）",
            w.s.rel
        );
    }
}
