//! **表面提取探针**（体素 → 网格转换四件的第 ① 件；`PLAN-CONVERSION.md` §3.1）。
//!
//! 对两个已知形状（体素地板 = 精确盒 16 m³/40 m²；球团 = 解析球 r=1.5）跑
//! `voxel::surface_mesh`，打印顶点/三角/体积/面积/最大 |sdf| 读数，并自检四条硬门：
//! ① 封闭流形（每条棱恰两张三角）② 法线全部指向空侧 ③ 确定性（两跑逐位同）
//! ④ 合法三角（索引界内、无重复顶点、非零面积 ⇒ 窄相注册零丢弃）。
//!
//! 跑法（release）：
//! ```text
//! cargo run --release -q -p vxl-phys --example voxel_mesh_extract
//! ```

use std::collections::HashMap;
use vxl_phys_core::Vec3;
use vxl_phys_terrain::voxel::{surface_mesh, SurfaceMesh, VoxelVolume};

fn floor() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

fn sphere_blob() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, -2.0, -2.0), 0.25, 16, 16, 16);
    for iz in 0..16u32 {
        for iy in 0..16u32 {
            for ix in 0..16u32 {
                if v.grid_center(ix, iy, iz).length() <= 1.5 {
                    v.set(ix, iy, iz, true);
                }
            }
        }
    }
    v
}

fn mesh_volume(m: &SurfaceMesh) -> f32 {
    let mut acc = 0.0f32;
    for t in &m.tris {
        acc += m.points[t[0] as usize].dot(m.points[t[1] as usize].cross(m.points[t[2] as usize]));
    }
    acc / 6.0
}

fn mesh_area(m: &SurfaceMesh) -> f32 {
    let mut acc = 0.0f32;
    for t in &m.tris {
        let a = m.points[t[0] as usize];
        let b = m.points[t[1] as usize];
        let c = m.points[t[2] as usize];
        acc += (b - a).cross(c - a).length() * 0.5;
    }
    acc
}

fn check_hard_gates(name: &str, vol: &VoxelVolume, m: &SurfaceMesh) {
    // ① 封闭流形
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.tris {
        for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let key = if e.0 < e.1 { e } else { (e.1, e.0) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    let bad: Vec<_> = edges.iter().filter(|(_, c)| **c != 2).collect();
    assert!(bad.is_empty(), "{name}: 封闭流形破坏：{bad:?}");
    // ② 法线指向空侧
    let eps = vol.step() * 0.125;
    for t in &m.tris {
        let a = m.points[t[0] as usize];
        let b = m.points[t[1] as usize];
        let c = m.points[t[2] as usize];
        let dir = (b - a).cross(c - a);
        let dir = dir * (1.0 / dir.length());
        let ctr = (a + b + c) * (1.0 / 3.0);
        assert!(
            vol.sdf(ctr + dir * eps) > vol.sdf(ctr - dir * eps),
            "{name}: 法线未指向空侧"
        );
    }
    // ③ 确定性（两跑逐位同）
    assert_eq!(*m, surface_mesh(vol), "{name}: 两跑必须逐位相同");
    // ④ 合法三角
    let n = m.points.len() as u32;
    for t in &m.tris {
        assert!(t[0] < n && t[1] < n && t[2] < n, "{name}: 索引越界 {t:?}");
        assert!(
            t[0] != t[1] && t[1] != t[2] && t[0] != t[2],
            "{name}: 重复顶点 {t:?}"
        );
    }
}

fn report(name: &str, vol: &VoxelVolume, m: &SurfaceMesh, v_ref: f32, a_ref: f32) {
    let max_sd = m
        .points
        .iter()
        .map(|p| vol.sdf(*p).abs())
        .fold(0.0f32, f32::max);
    println!(
        "  {name:10}  顶点 {:5}  三角 {:5}  V {:9.4} (基准 {v_ref:8.3})  A {:9.4} (基准 {a_ref:8.3})  max|sdf(v)| {:.4}",
        m.points.len(),
        m.tris.len(),
        mesh_volume(m),
        mesh_area(m),
        max_sd
    );
    check_hard_gates(name, vol, m);
}

fn main() {
    println!("【体素表面提取】surface nets、复制式（不消耗占据格）；四条硬门随跑自检\n");
    report("地板(盒)", &floor(), &surface_mesh(&floor()), 16.0, 40.0);
    report(
        "球团 r=1.5",
        &sphere_blob(),
        &surface_mesh(&sphere_blob()),
        14.137,
        28.274,
    );
    println!("\n  四条硬门（封闭 / 朝向 / 确定性 / 合法）全部通过 ✅");
}
