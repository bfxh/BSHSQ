//! tests：从 `lib.rs` 拆出的单元测试（纯搬移 + 去一层缩进）。
//!
//! 2026-10-05 拆：本文件原 967 行（全仓仅剩的两个 >800 行文件之一）⇒ 27 个用例按域分到
//! `tests/` 四个子模块（`providers` / `splat` / `mesh_carve` / `solver_misc`），两个共用夹具留本文件。

use super::*;

/// 立方体点云（外壳测试用；顶点序固定 ⇒ 确定性）。
fn cube_hull_points(half: f32) -> Vec<Vec3> {
    let mut pts = Vec::new();
    for &x in &[-half, half] {
        for &y in &[-half, half] {
            for &z in &[-half, half] {
                pts.push(Vec3::new(x, y, z));
            }
        }
    }
    pts
}

fn ground_world() -> World {
    let mut w = World::new(PhysConfig::default());
    let hf = HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0);
    w.add_heightfield(hf);
    w
}

mod mesh_carve;
mod providers;
mod solver_misc;
mod splat;
mod splat_dynamics;
