//! `cue` 子命令：感官 cue 流（JSONL；纯搬移自 `lib.rs`）。

use crate::scene::scene;
use vxl_phys_sense::{jsonl, Cue, SenseConfig, SenseExt};

/// `cue`：感官 cue 流（JSONL，一行一条；`cue` 的机器可读形式就是它本身）。
pub(super) fn cue_cmd(name: &str, ticks: usize) -> Result<String, String> {
    let mut w = scene(name)?;
    let cfg = SenseConfig::default();
    let mut cues: Vec<Cue> = Vec::new();
    for _ in 0..ticks {
        w.step();
        w.sense_cues(&cfg, &mut cues);
    }
    Ok(jsonl(&cues))
}
