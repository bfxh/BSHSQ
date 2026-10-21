//! 场景工厂：场景名 → 世界；跑 tick 并回读末态（纯搬移自 `lib.rs`，无语义改动）。

use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, SoftExt, Vec3, World};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 场景名 → 世界（每个场景自带地面与若干体，全确定性布局）。
pub(super) fn scene(name: &str) -> Result<World, String> {
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
pub(super) fn run_scene(name: &str, ticks: usize) -> Result<(World, Vec<Vec3>), String> {
    let mut w = scene(name)?;
    for _ in 0..ticks {
        w.step();
    }
    let poses: Vec<Vec3> = (0..w.bodies.len()).map(|i| w.bodies.position[i]).collect();
    Ok((w, poses))
}
