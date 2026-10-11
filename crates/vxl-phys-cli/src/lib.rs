//! vxl-phys-cli 的**可测内核**：`run(args) -> Result<String, String>`（不自己打印，便于判据与
//! 被别的工具再包装）。`src/main.rs` 只做"取 argv → 调 run → 打印/退出码"。
//!
//! 子命令（全部只用 `vxl-phys` 的公共 API —— 这份代码本身就是给集成方的样板）：
//!
//! - `demo <drop|stack|cloth> [ticks]`：跑一个最小场景，打印末态摘要；
//! - `hash <drop|stack> [ticks]`：打印末态 `state_hash`（**两次跑必须同值** ⇒ 集成方的复现自检）；
//! - `cue <drop> [ticks]`：打印感官 cue 流（JSONL，一行一条，见 `vxl-phys-sense`）。
//!
//! 依赖面刻意压在"一个门面 + 两个域 crate"上：要让别人敢接，先得证明外部视角够用。

#![forbid(unsafe_code)]

// ⚠️ 本仓 `glob-gate` 对新文件的 `use ...::*` 是硬判 ⇒ 这里**逐项具名导入**（调用方自己那边
// 用 `use vxl_phys::prelude::*;` 才是我们要给他们的便利，见 `docs/QUICKSTART.md`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
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
    let sub = args.first().map(String::as_str).unwrap_or("help");
    let rest = &args[args.len().min(1)..];
    let name = rest.first().map(String::as_str).unwrap_or("drop");
    let ticks: usize = rest.get(1).and_then(|s| s.parse().ok()).unwrap_or(240);
    match sub {
        "demo" => {
            let (w, poses) = run_scene(name, ticks)?;
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
        "hash" => {
            let (mut w, _) = run_scene(name, ticks)?;
            Ok(format!(
                "{}\n（同一个场景两次跑必须给出同一个值——集成方的复现自检）\n",
                w.state_hash()
            ))
        }
        "cue" => {
            let mut w = scene(name)?;
            let cfg = SenseConfig::default();
            let mut cues: Vec<Cue> = Vec::new();
            for _ in 0..ticks {
                w.step();
                w.sense_cues(&cfg, &mut cues);
            }
            Ok(jsonl(&cues))
        }
        "help" | "--help" | "-h" => Ok(USAGE.to_string()),
        other => Err(format!("未知子命令 `{other}`\n{USAGE}")),
    }
}

/// 用法文本（`--help` 与错误信息共用一份）。
pub const USAGE: &str = "\
vxl-phys —— vxl_phys 的命令行入口
用法：
  vxl-phys demo <drop|stack|cloth> [ticks]   跑示例场景，打印末态摘要（默认 drop / 240 tick）
  vxl-phys hash <drop|stack> [ticks]         打印末态 state_hash（可复现自检）
  vxl-phys cue  <voxel> [ticks]              打印感官 cue 流（JSONL；**只有 voxel 是 provider 地面**）
  vxl-phys help                              本说明
";
