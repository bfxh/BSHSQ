//! **喷溅核质量的积分对拍**（P11 定案，2026-10-08）。
//!
//! 口径的来源不是"选一个公式"，而是**场的定义**：介质密度场 `σ(p) = opacity·exp(−½α)` 的
//! 积分质量 `∫ρ_medium·σ dV`。本判据用**数值积分**（中点法，同一份 `MediumField::sample`）
//! 对拍解析闭式 [`Splat::mass`]。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::MediumField as _;
use vxl_phys_core::Vec3;
use vxl_phys_splat::{GaussianSplatField, Splat};

#[test]
fn splat_mass_equals_the_field_integral() {
    let mut f = GaussianSplatField::new(0.5);
    f.push(Splat::isotropic(Vec3::ZERO, 0.25, 0.8));
    f.medium_density = 2.0;
    let analytic = f.splats()[0].mass(f.medium_density);
    assert!(analytic > 0.0, "闭合质量必须为正：{analytic}");

    // 中点法积分：盒 [-1,1]^3（覆盖 ±4σ），每轴 144 段 ⇒ 相对误差量级 ~1e-3
    let n = 144usize;
    let h = 2.0 / n as f32;
    let mut acc = 0.0f32;
    for ix in 0..n {
        let x = -1.0 + (ix as f32 + 0.5) * h;
        for iy in 0..n {
            let y = -1.0 + (iy as f32 + 0.5) * h;
            for iz in 0..n {
                let z = -1.0 + (iz as f32 + 0.5) * h;
                acc += f.sample(Vec3::new(x, y, z)).density;
            }
        }
    }
    let numeric = acc * h * h * h;
    let rel = ((numeric - analytic) / analytic).abs();
    assert!(
        rel < 5e-3,
        "积分质量应等于解析质量：rel={rel:e}（积分 {numeric} vs 解析 {analytic}）"
    );
}
