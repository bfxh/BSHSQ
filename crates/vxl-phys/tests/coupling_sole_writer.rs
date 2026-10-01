//! **"耦合量唯一落点"断言**（`PLAN-COUPLING.md` §3.6：统一施加器是唯一允许写跨域量的地方）。
//!
//! 机制：`include_str!` 扫描四个**编排文件**（`world_step.rs` / `world_soft.rs` /
//! `world_step/aero.rs` / `world_step/fluid_stepper.rs`），找对
//! `force/torque/linvel/angvel/position` 累加器的**直接赋值**（`.xxx[…] +=` 或 `.xxx[…] = `）
//! ——必须为空：这些写入只允许出现在 `world_step/coupling.rs` 的施加器里
//! （引擎自用的求解器/关节/CCD 在别的文件，不属本扫描面）。
//! 扫描器自带两条金丝雀（合成"直接写"必命中 / 纯读不命中），防门空转与误伤。
//!
//! 零行为：本判据只读源码文本。修前实测命中 1 处（介质 2a 的就地写，见提交记录）⇒ 收口后为 0。

/// 命中列表：`(行号, 行文本)`。只认**赋值**（`+=` / `= `；`==` 不算）。
fn direct_writes(src: &str) -> Vec<(usize, String)> {
    const FIELDS: [&str; 5] = ["force", "torque", "linvel", "angvel", "position"];
    let mut hits = Vec::new();
    for (ln, line) in src.lines().enumerate() {
        for f in FIELDS {
            let pat = format!(".{f}[");
            let mut from = 0;
            while let Some(i) = line[from..].find(&pat) {
                let start = from + i + pat.len();
                // 找配对的 ']'（本仓这些字段下标里不会再嵌括号）。
                let Some(close) = line[start..].find(']') else {
                    break;
                };
                let after = line[start + close + 1..].trim_start();
                let assign =
                    after.starts_with("+=") || (after.starts_with('=') && !after.starts_with("=="));
                if assign {
                    hits.push((ln + 1, line.to_string()));
                }
                from = start + close + 1;
            }
        }
    }
    hits
}

#[test]
fn coupling_writes_live_only_in_the_applier() {
    // ── 金丝雀（门在扫吗）──
    assert_eq!(
        direct_writes("x.linvel[3] += v;").len(),
        1,
        "金丝雀①：合成'直接写'应命中"
    );
    assert_eq!(
        direct_writes("let v = x.linvel[3];").len(),
        0,
        "金丝雀②：纯读不该命中"
    );

    let mut all: Vec<(&str, usize, String)> = Vec::new();
    for (name, src) in [
        ("world_step.rs", include_str!("../src/world_step.rs")),
        ("world_soft.rs", include_str!("../src/world_soft.rs")),
        (
            "world_step/aero.rs",
            include_str!("../src/world_step/aero.rs"),
        ),
        (
            "world_step/fluid_stepper.rs",
            include_str!("../src/world_step/fluid_stepper.rs"),
        ),
    ] {
        for (ln, line) in direct_writes(src) {
            all.push((name, ln, line));
        }
    }
    assert!(
        all.is_empty(),
        "跨域量的写入必须只出现在 `world_step/coupling.rs` 的施加器里；实得：{all:#?}"
    );
}
