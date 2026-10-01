//! **域轮次表一致性断言**（`PLAN-COUPLING.md` §3.6 / §5 C3 片 1）：跨域触点必须登记，
//! 「新增域忘登记即红」。
//!
//! 机制：本文件持有**声明式轮次表**（`ROUNDS`）与**引擎内部豁免表**（`ENGINE_PASSES`）；
//! `include_str!` 扫描 `world_step.rs` / `world_soft.rs` 两个**编排文件**里所有
//! `self.<name>_pass(` 形状的调用点（步进/子步/域轮次的调用点都在这两个文件 ⇒ 新域的
//! 调用点必然被扫到），两向断言：
//! - 扫描集 − 豁免集 == 登记表（**新域不登记 ⇒ 红**）；
//! - 登记表 ∪ 豁免集 ⊆ 扫描集（**改名/删域后表留尸 ⇒ 红**）。
//!
//! **扫描器是注释盲的**（不做词法剥离）：源码注释里写出 `self.X_pass(` 字面量也会命中
//! ——文档里提及请改写形式。扫描器自带金丝雀（合成源必命中），防「门空转」；注入实测
//! （2026-10-01）：在 `world_step.rs` 末尾加一行注释 `self.probe_stray_pass();`
//! ⇒ 本判据红（`probe_stray_pass` 未登记）⇒ 随即撤回（证据即本条）。
//!
//! 零行为：本片不新增运行时面、不改任何既有读数（C3 的 `apply_round` 收口 = 片 2）。

use std::collections::BTreeSet;

/// **声明式轮次表**：跨域触点（名 = 编排文件里被调用的 `*_pass` 函数）。
/// 每项对应 `PLAN-COUPLING.md` §3.6 的一行：域 / 产出通道 / 段位 / 滞后拍数。
/// **动行为的分片不许绕过本表**（`apply_round` 收口也要先在此登记）。
const ROUNDS: &[&str] = &[
    "fluid_pass",          // 流体推进 + 2b 边界重建（tick 末；产出下一拍的反作用）
    "fluid_reaction_pass", // 2b 反作用施加（子步 D；Force；受体滞后 1 拍）
    "rope_pass",           // 绳索（tick 末；Impulse Δv + Position Δx + Torque）
    "cloth_pass",          // 布片（tick 末；Impulse Δv + Position Δx）
    "medium_pass",         // 介质 2a：流体作介质（子步 A/B；Force）
    "splat_medium_pass",   // 介质①：喷溅场作介质（子步 B；Force）
    "aero_pass",           // 面元气动（子步 C；Force + Torque）
];

/// **引擎内部豁免**：同为 `*_pass` 形状但不是跨域作用（轮次驱动器本身 / 窄相档 / CCD /
/// 卡上步进）。§3.6 的「引擎自用的求解器/关节/CCD 除外」在名单上的落地。
const ENGINE_PASSES: &[&str] = &[
    "domain_pass",
    "narrow_tier_pass",
    "ccd_pass",
    "fluid_stepper_pass",
];

/// 扫描 `self.<name>(` 且 `name` 以 `_pass` 结尾的调用点（去重集合）。
fn scanned_passes(src: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for seg in src.split("self.").skip(1) {
        let name: String = seg
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        if name.ends_with("_pass") {
            out.insert(name);
        }
    }
    out
}

#[test]
fn coupling_rounds_are_declared_and_not_stale() {
    // ── 金丝雀（门在扫吗）：扫描器对合成源必命中。──
    let canary = scanned_passes("let _ = self.wind_pass();");
    assert_eq!(
        canary.len(),
        1,
        "扫描器金丝雀：合成源应命中 1 个（实得 {canary:?}）"
    );
    assert!(
        canary.contains("wind_pass"),
        "扫描器金丝雀：应命中 wind_pass（实得 {canary:?}）"
    );

    let mut found = scanned_passes(include_str!("../src/world_step.rs"));
    found.extend(scanned_passes(include_str!("../src/world_soft.rs")));
    let declared: BTreeSet<String> = ROUNDS.iter().map(|s| (*s).to_string()).collect();
    let engine: BTreeSet<String> = ENGINE_PASSES.iter().map(|s| (*s).to_string()).collect();
    let coupling: BTreeSet<String> = found.difference(&engine).cloned().collect();
    println!("扫描集 {found:?}\n登记表 {declared:?}\n豁免集 {engine:?}");

    // ① 活域 == 登记表：**新域不登记即红**；表留尸（删域/改名不同步）也红（差集两向同一条）。
    assert_eq!(
        coupling, declared,
        "跨域触点必须与轮次表一一对应（左 = 源码扫描，右 = ROUNDS）——\n\
         新域把调用点加进 world_step.rs / world_soft.rs 后要同步登记本表"
    );
    // ② 豁免表不得留尸（引擎内部 pass 改名/删除后也要同步，否则豁免面越滚越宽）。
    for n in ENGINE_PASSES {
        assert!(
            found.contains(*n),
            "豁免表里的 `{n}` 在源码里已找不到调用点——表陈旧（改名/删除后没同步）"
        );
    }
}
