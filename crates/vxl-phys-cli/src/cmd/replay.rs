//! `replay` 子命令：与基线逐行对拍（纯搬移自 `lib.rs`）。

use crate::record::record_text;

/// `replay`：基线文件是**必需**的（缺了就是用法错误，不该静默什么都不比）。
pub(super) fn replay_cmd(name: &str, ticks: usize, path: Option<&str>) -> Result<String, String> {
    match path {
        Some(p) => replay(name, ticks, p),
        None => Err("replay 需要基线文件：`replay <场景> [ticks] <基线.jsonl>`".to_string()),
    }
}

/// 重跑场景并与基线文件**逐字**比对（行数不同也算不一致 ⇒ 报首个差异观测点）。
fn replay(name: &str, ticks: usize, path: &str) -> Result<String, String> {
    let want = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => return Err(format!("读不了 {path}：{e}")),
    };
    let got = record_text(name, ticks)?;
    for (i, (w, g)) in want.lines().zip(got.lines()).enumerate() {
        if w != g {
            return Err(format!(
                "FAIL：第 {} 个观测点起不一致\n  基线 {w}\n  本次 {g}\n（跨机器/跨版本复现失败：先确认档位与版本）",
                i + 1
            ));
        }
    }
    let (wl, gl) = (want.lines().count(), got.lines().count());
    if wl != gl {
        return Err(format!(
            "FAIL：观测点行数不同（基线 {wl} vs 本次 {gl}）——tick 数或档位不一样？"
        ));
    }
    Ok(format!(
        "PASS：{name} / {ticks} tick / {gl} 个观测点与 {path} 逐字一致\n"
    ))
}
