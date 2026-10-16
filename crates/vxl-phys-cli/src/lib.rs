//! vxl-phys-cli 的**可测内核**：`run(args) -> Result<String, String>`（不自己打印，便于判据与
//! 被别的工具再包装）。`src/main.rs` 只做"取 argv → 调 run → 打印/退出码"。
//!
//! 子命令（全部只用 `vxl-phys` 的公共 API —— 这份代码本身就是给集成方的样板）：
//!
//! - `demo <drop|stack|cloth> [ticks]`：跑一个最小场景，打印末态摘要；
//! - `hash <drop|stack> [ticks]`：打印末态 `state_hash`（**两次跑必须同值** ⇒ 集成方的复现自检）；
//! - `cue <drop> [ticks]`：打印感官 cue 流（JSONL，一行一条，见 `vxl-phys-sense`）。
//! - `acoustic <voxel> [ticks]`：打印声学路径流（JSONL，一行一条，见 `vxl-phys-acoustics`）——
//!   **声学几何是声明的"房间"**（地板 + 带开口的墙），与碰撞体无关。
//! - `audio <voxel> [ticks] <出.wav> [干信号.wav]`：把同一批声学路径渲染成**能听的 WAV**
//!   （A6 离线音频渲染：逐带分频 + 按 `delay` 直叠；给了干信号就用它，否则用确定性的一记闷响）。
//!
//! 依赖面刻意压在"一个门面 + 两个域 crate"上：要让别人敢接，先得证明外部视角够用。

#![forbid(unsafe_code)]

// ⚠️ 本仓 `glob-gate` 对新文件的 `use ...::*` 是硬判 ⇒ 这里**逐项具名导入**（调用方自己那边
// 用 `use vxl_phys::prelude::*;` 才是我们要给他们的便利，见 `docs/QUICKSTART.md`）。
// 子命令按职责分文件（`src/cmd/` 下不放 `mod.rs`）⇒ 每个 `mod` 自带 `#[path]`，各文件同样逐项具名导入。
#[path = "cmd/acoustic.rs"]
mod acoustic;
#[path = "cmd/audio.rs"]
mod audio;
#[path = "cmd/check.rs"]
mod check;
#[path = "cmd/cue.rs"]
mod cue;
#[path = "cmd/demo.rs"]
mod demo;
#[path = "cmd/hash.rs"]
mod hash;
#[path = "cmd/record.rs"]
mod record;
#[path = "cmd/replay.rs"]
mod replay;
#[path = "cmd/scene.rs"]
mod scene;

/// 命令行入口（见模块头注）。
pub fn run(args: &[String]) -> Result<String, String> {
    // `--json` 可放任意位置（脚本/CI 吃机器可读输出）；位置参数按"不以 `-` 开头"取。
    let json = args.iter().any(|a| a == "--json");
    // `--realtime` 只对 `acoustic` 有意义：换成实时档（确定性的工作量上界，见 `vxl-phys-acoustics`）。
    let realtime = args.iter().any(|a| a == "--realtime");
    // `--reverb` 只对 `audio` 有意义：叠上 A7 的统计混响尾巴（Sabine 逐带 RT60，确定性）。
    let reverb = args.iter().any(|a| a == "--reverb");
    // `--order2`：`acoustic` / `audio` 都把反射阶数开到 2（A8 二阶镜像展开；候选 ≈ 面数² ⇒ 慢）。
    let order2 = args.iter().any(|a| a == "--order2");
    // `--inside` 只对 `audio` 有意义：听者挪到**声源那间屋**里（默认在墙外侧，只能听到门洞/绕射）。
    let inside = args.iter().any(|a| a == "--inside");
    let pos: Vec<&str> = args
        .iter()
        .map(String::as_str)
        .filter(|a| !a.starts_with('-'))
        .collect();
    let sub = pos.first().copied().unwrap_or("help");
    let name = pos.get(1).copied().unwrap_or("drop");
    let ticks: usize = pos.get(2).and_then(|s| s.parse().ok()).unwrap_or(240);
    // `record`/`replay` 的基线文件走第 4 个位置参数（不引 `--out/--in` 这类带值开关，参数解析保持零依赖）。
    let path = pos.get(3).copied();
    match sub {
        "demo" => demo::demo_cmd(name, ticks, json),
        "hash" => hash::hash_cmd(name, ticks, json),
        // **集成方的 CI 自检**：同场景跑两遍、每 60 tick 采一个观测点，逐点比对 `state_hash`。
        "check" => check::check(name, ticks, json),
        // **录观测点序列**（JSONL，一行一个 tick 的哈希）⇒ 给 `replay` 当基线。
        "record" => record::record_cmd(name, ticks, path),
        // **回放对拍**：重跑场景，与基线逐行比对（跨机器/跨版本的复现检查）。
        "replay" => replay::replay_cmd(name, ticks, path),
        "cue" => cue::cue_cmd(name, ticks),
        // **第二条输出侧域**：声学路径流（走声明的"房间"几何，与碰撞体无关）。
        "acoustic" => acoustic::acoustic_cmd(name, ticks, realtime, order2),
        // **A6**：把同一批路径渲染成能听的 WAV（第 3 个位置参数 = 输出路径，第 4 个 = 干信号）。
        "audio" => audio::audio_cmd(
            name,
            ticks,
            path,
            pos.get(4).copied(),
            audio::AudioFlags {
                reverb,
                order2,
                inside,
            },
        ),
        "help" => Ok(USAGE.to_string()),
        other => Err(format!("未知子命令 `{other}`\n{USAGE}")),
    }
}

/// 用法文本（`--help` 与错误信息共用一份）。
pub const USAGE: &str = "\
vxl-phys —— vxl_phys 的命令行入口
用法：
  vxl-phys demo  <drop|stack|cloth|voxel> [ticks]  示例场景末态摘要（默认 drop / 240 tick）
  vxl-phys hash  <drop|stack> [ticks]              末态 state_hash（可复现自检）
  vxl-phys check <drop|stack|cloth|voxel> [ticks]  跑两遍、逐观测点比对哈希（CI 自检；不一致退出码 2）
  vxl-phys cue   <voxel> [ticks]                   感官 cue 流（JSONL；只有 voxel 是 provider 地面）
  vxl-phys acoustic <voxel> [ticks] [--realtime]   声学路径流（JSONL；声学几何是声明的房间，与碰撞体无关）
  vxl-phys audio    <voxel> [ticks] <出.wav> [干信号.wav]  声学路径渲染成 WAV（A6；默认激励 = 确定性的闷响）
  vxl-phys record <scene> [ticks] [基线.jsonl]      录观测点序列（JSONL；省略文件则打到 stdout）
  vxl-phys replay <scene> [ticks] <基线.jsonl>      与基线逐行对拍（跨机器/跨版本复现检查）
  vxl-phys help                                    本说明
任意子命令都可加 `--json`：输出机器可读 JSON（cue 本来就是 JSONL）。
`acoustic` 可加 `--realtime`：走实时档（候选与产出都收紧，确定性、不看时钟）。
`audio` 的时间轴按物理步长排（每 tick 一声事件）；干信号给 WAV 就用它（PCM16/float32、多声道降混）。
`audio` 可加 `--reverb`：叠上 A7 的统计混响尾巴（Sabine 逐带 RT60，确定性；几何不闭合就按声明的包络算）。
coustic / udio 可加 --order2：开 A8 的二阶镜像展开（候选 ≈ 面数²，慢）。udio 可加 --inside：听者挪进屋里（默认在墙外）。
";
