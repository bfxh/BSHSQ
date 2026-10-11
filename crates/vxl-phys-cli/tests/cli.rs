//! **CLI 的判据**：它是对外工具链的第一道脸——子命令要能跑、读数要能复现、坏输入要响亮拒绝。

use vxl_phys_cli::run;

fn args(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

/// 跑一条命令并取输出；失败（Err）折成空串 ⇒ 断言自然变红（本仓新文件禁 `unwrap/expect/panic`）。
fn ok(v: &[&str]) -> String {
    run(&args(v)).unwrap_or_default()
}

/// 跑一条命令并取错误文本；成功（Ok）折成空串。
fn err(v: &[&str]) -> String {
    run(&args(v)).err().unwrap_or_default()
}

#[test]
fn demo_runs_and_reports_a_finite_world() {
    let out = ok(&["demo", "drop", "120"]);
    assert!(out.contains("体 4 具"), "摘要应报体数：{out}");
    assert!(!out.contains("NaN 1"), "正常场景不该有 NaN：{out}");
}

#[test]
fn hash_is_reproducible_across_runs() {
    let (a, b) = (ok(&["hash", "stack", "60"]), ok(&["hash", "stack", "60"]));
    assert!(!a.is_empty(), "hash 应当成功");
    assert_eq!(a, b, "同一场景两次跑必须给同一个 state_hash");
}

#[test]
fn cue_stream_is_jsonl_and_deterministic() {
    // `voxel` 是唯一 provider 地面场景（`impacts` 只对 provider 接触记录 ⇒ 盒地面是空流）。
    let (a, b) = (ok(&["cue", "voxel", "180"]), ok(&["cue", "voxel", "180"]));
    assert_eq!(a, b, "cue 流必须逐字一致（感官域的确定性契约）");
    let first = a.lines().next().unwrap_or("");
    assert!(
        first.starts_with('{') && first.contains("\"gain\""),
        "首行不是 cue：{first}"
    );
}

#[test]
fn unknown_scene_and_subcommand_are_rejected_loudly() {
    let e = err(&["demo", "nope", "10"]);
    assert!(e.contains("未知场景"), "{e}");
    let e = err(&["frobnicate"]);
    assert!(e.contains("未知子命令") && e.contains("用法"), "{e}");
}

/// `acoustic`：声学路径流（JSONL）—— 逐字一致，且**真的含穿孔与透射泄漏两条新路径**。
#[test]
fn acoustic_stream_is_jsonl_and_deterministic() {
    let (a, b) = (
        ok(&["acoustic", "voxel", "180"]),
        ok(&["acoustic", "voxel", "180"]),
    );
    assert_eq!(a, b, "声学路径流必须逐字一致（声学域的确定性契约）");
    let first = a.lines().next().unwrap_or("");
    assert!(
        first.starts_with('{') && first.contains("\"kind\""),
        "首行不是声学路径：{first}"
    );
    assert!(
        a.contains("\"kind\":\"portal\""),
        "流里必须有穿门洞的路径（A2）：{a}"
    );
    assert!(
        a.contains("\"kind\":\"transmitted\""),
        "流里必须有墙的透射泄漏（A2）：{a}"
    );
    assert!(
        a.contains("\"kind\":\"diffracted\""),
        "流里必须有从门框绕过来的路径（A4）：{a}"
    );
    // A3 频带：每行都要有逐带能量（格式换代一次，见 crate 头注）
    let lines = a.lines().count();
    let with_bands = a.lines().filter(|l| l.contains("\"bands\":[")).count();
    assert!(
        lines > 0 && with_bands == lines,
        "每行都要有逐带能量 bands（A3 频带）：{lines} 行里只有 {with_bands} 行有"
    );
}

#[test]
fn help_is_self_describing() {
    let out = ok(&["help"]);
    for need in ["demo", "hash", "cue", "acoustic", "ticks"] {
        assert!(out.contains(need), "用法缺 {need}：{out}");
    }
}

/// `--json`：给脚本/CI 吃的机器可读输出（键名是本工具对外承诺的一部分）。
#[test]
fn json_flag_gives_machine_readable_output() {
    let h = ok(&["hash", "stack", "60", "--json"]);
    assert!(
        h.starts_with('{') && h.contains("\"state_hash\":\"0x"),
        "hash --json 形状不对：{h}"
    );
    let d = ok(&["demo", "drop", "60", "--json"]);
    for key in [
        "\"scene\":\"drop\"",
        "\"bodies\":4",
        "\"health\":",
        "\"poses\":[[",
    ] {
        assert!(d.contains(key), "demo --json 缺 {key}：{d}");
    }
}

/// `check`：集成方可以直接把它当 CI 门（一致 ⇒ PASS；不一致 ⇒ Err ⇒ CLI 退出码 2）。
#[test]
fn check_is_a_ci_gate() {
    let out = ok(&["check", "drop", "120"]);
    assert!(out.contains("PASS"), "一致场景应当 PASS：{out}");
    assert!(
        out.contains("2 个观测点"),
        "120 tick / 60 一采 ⇒ 2 个观测点：{out}"
    );
    let j = ok(&["check", "voxel", "60", "--json"]);
    assert!(
        j.contains("\"pass\":true") && j.contains("\"samples\":1"),
        "{j}"
    );
}

/// `record` / `replay`：录基线再回放 ⇒ PASS；**把基线改坏 ⇒ 必须 FAIL**（证明这门不是橡皮图章）。
#[test]
fn record_then_replay_round_trips_and_catches_tampering() {
    let p = std::env::temp_dir()
        .join("vxl_phys_cli_replay_baseline.jsonl")
        .to_string_lossy()
        .to_string();
    let rec = ok(&["record", "drop", "180", &p]);
    assert!(
        rec.contains("已写入") && rec.contains("3 个观测点"),
        "{rec}"
    );
    let got = ok(&["replay", "drop", "180", &p]);
    assert!(got.contains("PASS"), "原样回放必须 PASS：{got}");
    // 篡改基线（往第一行的哈希里插一个字符）⇒ 回放必须 FAIL
    let tampered = std::fs::read_to_string(&p)
        .unwrap_or_default()
        .replacen("0x", "0x0", 1);
    std::fs::write(&p, tampered).ok();
    let bad = err(&["replay", "drop", "180", &p]);
    assert!(
        bad.contains("FAIL") && bad.contains("不一致"),
        "篡改必须被抓到：{bad}"
    );
    std::fs::remove_file(&p).ok();
}

/// `record` 不带文件 ⇒ 直接打流（给管道用）；`replay` 缺基线 ⇒ 响亮拒绝。
#[test]
fn record_to_stdout_and_replay_without_baseline() {
    let out = ok(&["record", "stack", "60"]);
    assert!(
        out.starts_with('{') && out.contains("\"state_hash\":\"0x"),
        "{out}"
    );
    let e = err(&["replay", "stack", "60"]);
    assert!(e.contains("基线文件"), "{e}");
}
