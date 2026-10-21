//! `hash` 子命令：末态 `state_hash`（人读 + `--json`；纯搬移自 `lib.rs`）。

use crate::scene::run_scene;
use vxl_phys::DiagExt;

/// `hash`：末态 `state_hash`（人读带提示，`--json` 给机器）。
pub(super) fn hash_cmd(name: &str, ticks: usize, json: bool) -> Result<String, String> {
    let (mut w, _) = run_scene(name, ticks)?;
    let h = w.state_hash();
    if json {
        return Ok(json_hash(name, ticks, h));
    }
    Ok(format!(
        "{h}\n（同一个场景两次跑必须给出同一个值——集成方的复现自检）\n"
    ))
}

/// `hash --json`。
fn json_hash(name: &str, ticks: usize, h: u128) -> String {
    format!("{{\"scene\":\"{name}\",\"ticks\":{ticks},\"state_hash\":\"0x{h:032x}\"}}\n")
}
