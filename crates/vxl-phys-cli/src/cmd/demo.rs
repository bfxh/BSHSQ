//! `demo` 子命令：末态摘要（人读文本 + `--json` 机器可读；纯搬移自 `lib.rs`）。

use crate::scene::run_scene;
use vxl_phys::{Vec3, World};

/// `demo --json` 的机器可读末态（**键名是本工具对外的承诺**：脚本/CI 会按它取值）。
/// `demo`：跑场景后按 `json` 打机器可读或人读摘要（抽出来是为了压 `run` 的圈复杂度）。
pub(super) fn demo_cmd(name: &str, ticks: usize, json: bool) -> Result<String, String> {
    let (w, poses) = run_scene(name, ticks)?;
    if json {
        return Ok(json_demo(name, ticks, &w, &poses));
    }
    let mut s = format!("场景 {name} / {ticks} tick：体 {} 具\n", w.bodies.len());
    for (i, p) in poses.iter().enumerate().take(6) {
        s.push_str(&format!("  体{i}: ({:.4}, {:.4}, {:.4})\n", p.x, p.y, p.z));
    }
    let rep = w.health();
    s.push_str(&format!(
        "  健康：清醒 {} / NaN {} / 接触点 {}\n",
        rep.awake_bodies, rep.nan_bodies, rep.contacts
    ));
    Ok(s)
}

fn json_demo(name: &str, ticks: usize, w: &World, poses: &[Vec3]) -> String {
    let mut s = format!(
        "{{\"scene\":\"{name}\",\"ticks\":{ticks},\"bodies\":{},\"poses\":[",
        w.bodies.len()
    );
    for (i, p) in poses.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!("[{:.6},{:.6},{:.6}]", p.x, p.y, p.z));
    }
    let rep = w.health();
    s.push_str(&format!(
        "],\"health\":{{\"awake\":{},\"nan\":{},\"contacts\":{}}}}}\n",
        rep.awake_bodies, rep.nan_bodies, rep.contacts
    ));
    s
}
