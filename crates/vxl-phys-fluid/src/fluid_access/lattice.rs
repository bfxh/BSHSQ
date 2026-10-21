//! **晶格初始化原语**（`fluid_access.rs` 的子模块）：位置生成。
//!
//! 从 `new` 抽出（那里是 `fluid_access.rs` 的最长函数；god 门"合法交换"要求最长函数**严格下降**）。
//! 单开文件是为了让宿主文件**净缩** —— 原语的函数体比 `#[path] mod` 两行声明长。
//!
//! **质量标定那一半（`Σ_lattice W`）搬走了**：它唯一的消费者是**核标定**
//! （`mass = ρ0/ΣW`）⇒ 与 `KernelParams::calibrate` 住在一起（`system/domains.rs`），
//! 依赖方向与名字都对上了（这里只剩"晶格点在哪"）。
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
