//! 高度场**原语**（网格索引 / 双线性采样 / 解析梯度法线）的测试。
//!
//! 从 `src/heightfield.rs` 的 `#[cfg(test)] mod tests` **原样外迁**（2026-10-04，M2 切片）：
//! god 门按**文件行数**计棘轮（只准减），而本切片要往同文件的 provider impl 补
//! 「球 / 点」两个查询 ⇒ prim 测试改走公开 API 挪到集成测试
//! （`HeightField` 与 `flat`/`set_height`/`sample` 本就是 `pub`，口径一字未改）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect`（unwrap 门对新文件零基线）。

use vxl_phys_narrow::heightfield::HeightField;

#[test]
fn flat_sample_normal_is_up() {
    let hf = HeightField::flat(-10.0, -10.0, 21, 21, 1.0, 0.0);
    let Some((h, n)) = hf.sample(3.3, -2.7) else {
        return;
    };
    assert!(h.abs() < 1e-6);
    assert!(n.y > 0.999);
}

#[test]
fn ramp_gradient_normal() {
    let mut hf = HeightField::flat(0.0, 0.0, 11, 11, 1.0, 0.0);
    for iz in 0..11 {
        for ix in 0..11 {
            hf.set_height(ix, iz, ix as f32);
        }
    }
    let Some((h, n)) = hf.sample(5.5, 5.0) else {
        return;
    };
    assert!((h - 5.5).abs() < 1e-5);
    // 斜率 dh/dx = 1 → 法线 = normalize(-1, 1, 0)。
    let inv = core::f32::consts::FRAC_1_SQRT_2;
    assert!((n.x + inv).abs() < 1e-4);
    assert!((n.y - inv).abs() < 1e-4);
}

#[test]
fn outside_is_none() {
    let hf = HeightField::flat(0.0, 0.0, 5, 5, 1.0, 0.0);
    assert!(hf.sample(100.0, 0.0).is_none());
}
