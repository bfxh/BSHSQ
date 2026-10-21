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
        Ok(()) => Ok(format!("已写入 {p}（{} 个观测点）\n", text.lines().count())),
        Err(e) => Err(format!("写不了 {p}：{e}")),
    }
}

/// 观测点周期（ticks/行）——`record` 与 `replay` 共用的**格式常量**（改它 = 换格式）。
const PERIOD: u64 = 60;

/// 逐 tick 推进，每 [`PERIOD`] tick 写一行 `{"tick":..,"state_hash":"0x.."}`（JSONL）。
pub(super) fn record_text(name: &str, ticks: usize) -> Result<String, String> {
    let mut w = scene(name)?;
    let mut s = String::new();
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
