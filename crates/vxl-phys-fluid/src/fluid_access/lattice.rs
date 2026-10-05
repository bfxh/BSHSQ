//! **晶格初始化原语**（`fluid_access.rs` 的子模块）：位置生成 + 核质量标定求和。
//!
//! 从 `new` 抽出（那里是 `fluid_access.rs` 的最长函数；god 门"合法交换"要求最长函数**严格下降**）。
//! 单开文件是为了让宿主文件**净缩** —— 两个原语的函数体（27 行）比 `#[path] mod` 两行声明长。
use vxl_phys_core::Vec3;

/// 晶格点 `origin + (i + ½)·spacing`（三轴 `dims` 个，`i/j/k` 顺序 = 索引序 ⇒ 确定性）。
pub(super) fn lattice_positions(origin: Vec3, dims: [usize; 3], spacing: f32) -> Vec<Vec3> {
    let mut pos = Vec::with_capacity(dims[0] * dims[1] * dims[2]);
    for i in 0..dims[0] {
        for j in 0..dims[1] {
            for k in 0..dims[2] {
                pos.push(Vec3::new(
                    origin.x + (i as f32 + 0.5) * spacing,
                    origin.y + (j as f32 + 0.5) * spacing,
                    origin.z + (k as f32 + 0.5) * spacing,
                ));
            }
        }
    }
    pos
}

/// **晶格核质量标定**：间距 `spacing` 的晶格按 poly6 截断求和 `Σ W`（含自身项）。
pub(super) fn lattice_w_sum(h: f32, h2: f32, k6: f32, spacing: f32) -> f32 {
    let mut wsum = 0.0f32;
    let side = (h / spacing.max(1e-6)).ceil() as i32;
    for i in -side..=side {
        for j in -side..=side {
            for k in -side..=side {
                let d = Vec3::new(i as f32 * spacing, j as f32 * spacing, k as f32 * spacing);
                let r2 = d.length_squared();
                if r2 <= h2 {
                    let t = h2 - r2;
                    wsum += k6 * t * t * t;
                }
            }
        }
    }
    wsum
}
