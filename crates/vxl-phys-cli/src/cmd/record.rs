//! `record` 子命令：录观测点序列（JSONL；纯搬移自 `lib.rs`）。

use crate::scene::scene;
use vxl_phys::DiagExt;

/// `record`：给了文件就写盘并回报行数，没给就打流（两条分支抽出来是为了压 `run` 的圈复杂度）。
pub(super) fn record_cmd(name: &str, ticks: usize, path: Option<&str>) -> Result<String, String> {
    let text = record_text(name, ticks)?;
    let Some(p) = path else {
        return Ok(text);
    };
    match std::fs::write(p, &text) {
        Ok(()) => Ok(format!(
            "已写入 {p}（{} 个观测点 + 1 行头部）
",
            text.lines().filter(|l| l.contains("state_hash")).count()
        )),
        Err(e) => Err(format!("写不了 {p}：{e}")),
    }
}

/// 观测点周期（ticks/行）——`record` 与 `replay` 共用的**格式常量**（改它 = 换格式）。
const PERIOD: u64 = 60;

/// 逐 tick 推进，每 [`PERIOD`] tick 写一行 `{"tick":..,"state_hash":"0x.."}`（JSONL）。
pub(super) fn record_text(name: &str, ticks: usize) -> Result<String, String> {
    let mut w = scene(name)?;
    // **头部行**（`schema` 前缀）：把"这条基线是什么场景/多少 tick/什么周期"写进文件本身。
    // 没有它时，跨版本对拍只能看到"哈希不同"，分不清是引擎变了还是场景/档位配错了。
    // **构建标识**也进头部：跨机器/跨构建对拍失败时，能一眼分辨"换了构建"还是"物理变了"。
    // `vxl-phys` 是工作区包（脚本/门会改它），这里只取 crate 自报告的版本 + 目标三元组。
    let mut s = format!(
        "{{\"schema\":1,\"scene\":\"{}\",\"ticks\":{},\"period\":{},\"engine\":\"{}\",\"target\":\"{}-{}\",\"profile\":\"{}\"}}
",
        name,
        ticks,
        PERIOD,
        env!("CARGO_PKG_VERSION"),
        std::env::consts::ARCH,
        std::env::consts::OS,
        if cfg!(debug_assertions) { "debug" } else { "release" }
    );
    for t in 0..ticks as u64 {
        w.step();
        if (t + 1) % PERIOD == 0 {
            s.push_str(&format!(
                "{{\"tick\":{},\"state_hash\":\"0x{:032x}\"}}\n",
                t + 1,
                w.state_hash()
            ));
        }
    }
    Ok(s)
}
