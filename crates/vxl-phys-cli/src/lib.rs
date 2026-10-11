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
//!
//! 依赖面刻意压在"一个门面 + 两个域 crate"上：要让别人敢接，先得证明外部视角够用。

#![forbid(unsafe_code)]

// ⚠️ 本仓 `glob-gate` 对新文件的 `use ...::*` 是硬判 ⇒ 这里**逐项具名导入**（调用方自己那边
// 用 `use vxl_phys::prelude::*;` 才是我们要给他们的便利，见 `docs/QUICKSTART.md`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl as aco_jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticsExt, Edge,
    LayeredMedium, Listener, Medium, Portal, SoundPath, BANDS,
};
use vxl_phys_replay::Recorder;
use vxl_phys_sense::{jsonl, Cue, SenseConfig, SenseExt};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 场景名 → 世界（每个场景自带地面与若干体，全确定性布局）。
fn scene(name: &str) -> Result<World, String> {
    let mut w = World::new(PhysConfig::default());
    match name {
        // 三个盒子错开高度落下（最省事的"能看见物理"场景）。
        "drop" => {
            w.add_static(
                Shape::Box {
                    half: Vec3::new(4.0, 0.5, 4.0),
                },
                Vec3::new(0.0, -0.5, 0.0),
                Quat::IDENTITY,
            );
            for k in 0..3 {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(0.4 * k as f32 - 0.4, 1.0 + 0.6 * k as f32, 0.0),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
        // 5 层塔（堆叠稳定性与睡眠的最小可观测场景）。
        "stack" => {
            w.add_static(
                Shape::Box {
                    half: Vec3::new(4.0, 0.5, 4.0),
                },
                Vec3::new(0.0, -0.5, 0.0),
                Quat::IDENTITY,
            );
            for k in 0..5 {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(0.0, 0.25 + 0.5 * k as f32, 0.0),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
        // 一张 2×2 格的布（两角钉住）：软体域的入口长这样。
        "cloth" => {
            let mut pts = Vec::new();
            for iz in 0..=2u32 {
                for ix in 0..=2u32 {
                    pts.push(Vec3::new(0.25 * ix as f32, 2.0, 0.25 * iz as f32));
                }
            }
            let mut tris = Vec::new();
            for iz in 0..2u32 {
                for ix in 0..2u32 {
                    let a = iz * 3 + ix;
                    let (c, d) = (a + 1, a + 3);
                    tris.push([a, d, c]);
                    tris.push([c, d, d + 1]);
                }
            }
            let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
            sheet.set_pinned(0, true);
            sheet.set_pinned(2, true);
            w.add_cloth(sheet);
        }
        // 体素地面 + 三个盒子：**感官 cue 流需要 provider 接触**（`World::impacts` 只对
        // `Shape::Provider` 记录冲击）——拿盒地面跑 `cue` 会得到空流，这是该域最常踩的坑。
        "voxel" => {
            let mut vol = vxl_phys_terrain::voxel::VoxelVolume::new(
                Vec3::new(-4.0, -0.5, -4.0),
                0.25,
                32,
                3,
                32,
            );
            vol.fill_box(Vec3::new(-4.0, -0.5, -4.0), Vec3::new(4.0, 0.25, 4.0));
            w.add_voxel(vol);
            for k in 0..3 {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(0.4 * k as f32 - 0.4, 1.5 + 0.6 * k as f32, 0.0),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
        other => {
            return Err(format!(
                "未知场景 `{other}`（可选：drop / stack / cloth / voxel）"
            ))
        }
    }
    Ok(w)
}

/// 跑 `ticks` 步并回读末态摘要。
fn run_scene(name: &str, ticks: usize) -> Result<(World, Vec<Vec3>), String> {
    let mut w = scene(name)?;
    for _ in 0..ticks {
        w.step();
    }
    let poses: Vec<Vec3> = (0..w.bodies.len()).map(|i| w.bodies.position[i]).collect();
    Ok((w, poses))
}

/// 命令行入口（见模块头注）。
pub fn run(args: &[String]) -> Result<String, String> {
    // `--json` 可放任意位置（脚本/CI 吃机器可读输出）；位置参数按"不以 `-` 开头"取。
    let json = args.iter().any(|a| a == "--json");
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
        "demo" => demo_cmd(name, ticks, json),
        "hash" => hash_cmd(name, ticks, json),
        // **集成方的 CI 自检**：同场景跑两遍、每 60 tick 采一个观测点，逐点比对 `state_hash`。
        "check" => check(name, ticks, json),
        // **录观测点序列**（JSONL，一行一个 tick 的哈希）⇒ 给 `replay` 当基线。
        "record" => record_cmd(name, ticks, path),
        // **回放对拍**：重跑场景，与基线逐行比对（跨机器/跨版本的复现检查）。
        "replay" => replay_cmd(name, ticks, path),
        "cue" => cue_cmd(name, ticks),
        // **第二条输出侧域**：声学路径流（走声明的"房间"几何，与碰撞体无关）。
        "acoustic" => acoustic_cmd(name, ticks),
        "help" => Ok(USAGE.to_string()),
        other => Err(format!("未知子命令 `{other}`\n{USAGE}")),
    }
}

/// `demo --json` 的机器可读末态（**键名是本工具对外的承诺**：脚本/CI 会按它取值）。
/// `demo`：跑场景后按 `json` 打机器可读或人读摘要（抽出来是为了压 `run` 的圈复杂度）。
fn demo_cmd(name: &str, ticks: usize, json: bool) -> Result<String, String> {
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

/// `hash`：末态 `state_hash`（人读带提示，`--json` 给机器）。
fn hash_cmd(name: &str, ticks: usize, json: bool) -> Result<String, String> {
    let (mut w, _) = run_scene(name, ticks)?;
    let h = w.state_hash();
    if json {
        return Ok(json_hash(name, ticks, h));
    }
    Ok(format!(
        "{h}\n（同一个场景两次跑必须给出同一个值——集成方的复现自检）\n"
    ))
}

/// `cue`：感官 cue 流（JSONL，一行一条；`cue` 的机器可读形式就是它本身）。
fn cue_cmd(name: &str, ticks: usize) -> Result<String, String> {
    let mut w = scene(name)?;
    let cfg = SenseConfig::default();
    let mut cues: Vec<Cue> = Vec::new();
    for _ in 0..ticks {
        w.step();
        w.sense_cues(&cfg, &mut cues);
    }
    Ok(jsonl(&cues))
}

/// 声学几何的**声明式房间**（`acoustic` 子命令用）：顶点 / 三角 / 吸声 / 透射 / 开口。
struct Room {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    portals: Vec<Portal>,
    /// **衍射棱**（A4）：门框靠外那一侧的竖棱 —— 声音会从门框绕到墙后（"拐角后仍可闻"）。
    edges: Vec<Edge>,
}

impl Room {
    fn geom<'a>(&'a self, medium: Option<&'a dyn AcousticMedium>) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            portals: &self.portals,
            edges: &self.edges,
            medium,
        }
    }
}

/// **介质体积场**（A3）：地板附近一层"雾 / 湿热层"（声速略低、**高频吸收明显更强**），
/// 上面是常温空气 —— 分频带之后，"听起来闷"就是一个可算的量。
///
/// 分层介质给**解析积分**（按层边界切段）⇒ 时延与衰减都不是采样近似。要换密度/温度剖面，
/// 实现 `AcousticMedium` 交进来即可（也可用固定步长回退档）。
fn medium() -> LayeredMedium {
    let haze = Medium {
        speed_of_sound: 330.0,
        attenuation: [0.02, 0.06, 0.15],
    };
    LayeredMedium::split(1.0, haze, Medium::AIR)
}

/// 一间"声学房间"：地板（y = 0.25，与体素地面同高）+ 墙（z = 1.5，透射 0.02）+ **门洞开口**（全开）。
///
/// 它**不是**碰撞几何（`scene()` 里那些盒/体素才是）—— 这正是"声学几何 ≠ 碰撞几何"的样板：
/// 关上门（`open: 0.0`）就只剩墙的透射泄漏，开门则房间外听得见。
fn room() -> Room {
    let verts = vec![
        Vec3::new(-4.0, 0.25, -4.0),
        Vec3::new(4.0, 0.25, -4.0),
        Vec3::new(4.0, 0.25, 4.0),
        Vec3::new(-4.0, 0.25, 4.0),
        Vec3::new(-4.0, 0.25, 1.5),
        Vec3::new(4.0, 0.25, 1.5),
        Vec3::new(4.0, 3.0, 1.5),
        Vec3::new(-4.0, 3.0, 1.5),
    ];
    Room {
        verts,
        tris: vec![[0u32, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
        // 地板吸声一般；墙**逐带**吸声（越高频越吸 ⇒ 门缝里漏出来的声音自然"发闷"）。
        absorb: vec![
            [0.2f32; BANDS],
            [0.2; BANDS],
            [0.6, 0.9, 0.95],
            [0.6, 0.9, 0.95],
        ],
        // 墙的透射也是逐带的：高频更难透（现实里隔声量随频率上升）。
        trans: vec![
            [0.0f32; BANDS],
            [0.0; BANDS],
            [0.03, 0.02, 0.008],
            [0.03, 0.02, 0.008],
        ],
        portals: vec![Portal {
            quad: [
                Vec3::new(-1.0, 0.25, 1.5),
                Vec3::new(1.0, 0.25, 1.5),
                Vec3::new(1.0, 2.5, 1.5),
                Vec3::new(-1.0, 2.5, 1.5),
            ],
            open: 1.0,
        }],
        edges: vec![Edge {
            a: Vec3::new(1.0, 0.25, 1.5),
            b: Vec3::new(1.0, 2.5, 1.5),
            coeff: [0.5, 0.35, 0.2],
        }],
    }
}

/// `acoustic`：声学路径流（JSONL，一行一条；机器可读形式就是它本身）。
///
/// 听者站在墙**外侧**（z = 3.0）⇒ 听到的是"隔壁房间"的冲击：穿门洞的 `portal` 路径 +
/// 墙的透射泄漏 `transmitted` + 反射。逐 tick 归约（每 tick 用干净缓冲 ⇒ 预算按 tick 算）。
fn acoustic_cmd(name: &str, ticks: usize) -> Result<String, String> {
    let mut w = scene(name)?;
    let room = room();
    let field = medium();
    let geo = room.geom(Some(&field));
    let cfg = AcousticConfig::default();
    let listener = Listener {
        pos: Vec3::new(0.0, 1.0, 3.0),
        radius: 0.5,
    };
    let mut all: Vec<SoundPath> = Vec::new();
    for _ in 0..ticks {
        w.step();
        let mut tick: Vec<SoundPath> = Vec::new();
        w.acoustic_cues(&geo, &listener, &cfg, &mut tick);
        all.extend(tick);
    }
    Ok(aco_jsonl(&all))
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

/// `hash --json`。
fn json_hash(name: &str, ticks: usize, h: u128) -> String {
    format!("{{\"scene\":\"{name}\",\"ticks\":{ticks},\"state_hash\":\"0x{h:032x}\"}}\n")
}

/// `record`：给了文件就写盘并回报行数，没给就打流（两条分支抽出来是为了压 `run` 的圈复杂度）。
fn record_cmd(name: &str, ticks: usize, path: Option<&str>) -> Result<String, String> {
    let text = record_text(name, ticks)?;
    let Some(p) = path else {
        return Ok(text);
    };
    match std::fs::write(p, &text) {
        Ok(()) => Ok(format!("已写入 {p}（{} 个观测点）\n", text.lines().count())),
        Err(e) => Err(format!("写不了 {p}：{e}")),
    }
}

/// `replay`：基线文件是**必需**的（缺了就是用法错误，不该静默什么都不比）。
fn replay_cmd(name: &str, ticks: usize, path: Option<&str>) -> Result<String, String> {
    match path {
        Some(p) => replay(name, ticks, p),
        None => Err("replay 需要基线文件：`replay <场景> [ticks] <基线.jsonl>`".to_string()),
    }
}

/// 观测点周期（ticks/行）——`record` 与 `replay` 共用的**格式常量**（改它 = 换格式）。
const PERIOD: u64 = 60;

/// 逐 tick 推进，每 [`PERIOD`] tick 写一行 `{"tick":..,"state_hash":"0x.."}`（JSONL）。
fn record_text(name: &str, ticks: usize) -> Result<String, String> {
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

/// 跑两遍同一场景，逐观测点比对 `state_hash`（`Recorder` 是公共 API 里最小的"观测点"容器）。
/// 不一致 ⇒ `Err`（CLI 退出码 2 ⇒ 可直接当 CI 门用）。
fn check(name: &str, ticks: usize, json: bool) -> Result<String, String> {
    let period = 60u64;
    let sample = |name: &str| -> Result<(Recorder, u128), String> {
        let mut w = scene(name)?;
        let mut rec = Recorder::new(period);
        for t in 0..ticks as u64 {
            w.step();
            if (t + 1) % period == 0 {
                rec.observe(t + 1, w.state_hash());
            }
        }
        Ok((rec, w.state_hash()))
    };
    let (a, ha) = sample(name)?;
    let (b, hb) = sample(name)?;
    let samples = (ticks as u64 / period) as usize;
    let pass = a.matches(&b) && ha == hb;
    if json {
        return Ok(format!(
            "{{\"scene\":\"{name}\",\"ticks\":{ticks},\"samples\":{samples},\"pass\":{pass},\
             \"state_hash\":\"0x{ha:032x}\"}}\n"
        ));
    }
    if !pass {
        return Err(format!(
            "FAIL：{name} / {ticks} tick 两次跑不一致（观测点 {samples} 个，末态 0x{ha:032x} vs 0x{hb:032x}）"
        ));
    }
    Ok(format!(
        "PASS：{name} / {ticks} tick / {samples} 个观测点逐位一致（0x{ha:032x}）\n"
    ))
}

/// 用法文本（`--help` 与错误信息共用一份）。
pub const USAGE: &str = "\
vxl-phys —— vxl_phys 的命令行入口
用法：
  vxl-phys demo  <drop|stack|cloth|voxel> [ticks]  示例场景末态摘要（默认 drop / 240 tick）
  vxl-phys hash  <drop|stack> [ticks]              末态 state_hash（可复现自检）
  vxl-phys check <drop|stack|cloth|voxel> [ticks]  跑两遍、逐观测点比对哈希（CI 自检；不一致退出码 2）
  vxl-phys cue   <voxel> [ticks]                   感官 cue 流（JSONL；只有 voxel 是 provider 地面）
  vxl-phys acoustic <voxel> [ticks]                声学路径流（JSONL；声学几何是声明的房间，与碰撞体无关）
  vxl-phys record <scene> [ticks] [基线.jsonl]      录观测点序列（JSONL；省略文件则打到 stdout）
  vxl-phys replay <scene> [ticks] <基线.jsonl>      与基线逐行对拍（跨机器/跨版本复现检查）
  vxl-phys help                                    本说明
任意子命令都可加 `--json`：输出机器可读 JSON（cue 本来就是 JSONL）。
";
