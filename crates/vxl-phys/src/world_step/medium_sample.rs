//! world_step/medium_sample：**体的介质采样台**（`world_step.rs` 段② 与 `viscous.rs` 共用同一把尺）。
//!
//! 为什么单列一个文件：`medium.rs`（段①喷溅场）受 god 门的**文件行数棘轮**（只准减）
//! ⇒ 按 `aero.rs` / `medium.rs` / `viscous.rs` 的先例另起文件、调用点只加 1 行。
//!
//! 采样点 = **体心 + 4 个水平表面点**（单点采样在水线处是阶跃 ⇒ 盒子会抖；四点平均把水线过渡抹平），
//! 返回四点均值 `(浸没分数, 密度, 介质流速, 动力黏度 Pa·s)`。
use crate::Vec3;

/// 体心 `c`、包围半径 `r` 处的介质四量均值（读-only：不改流场、不改哈希）。
pub(crate) fn sample_around(
    sys: &vxl_phys_fluid::FluidSystem,
    c: Vec3,
    r: f32,
) -> (f32, f32, Vec3, f32) {
    use vxl_phys_core::interop::MediumField as _;
    let pts = [
        c,
        Vec3::new(c.x + r, c.y, c.z),
        Vec3::new(c.x - r, c.y, c.z),
        Vec3::new(c.x, c.y, c.z + r),
        Vec3::new(c.x, c.y, c.z - r),
    ];
    let (mut occ, mut dens, mut visc) = (0.0f32, 0.0f32, 0.0f32);
    let mut vmed = Vec3::ZERO;
    for p in pts {
        let s = sys.sample(p);
        occ += s.occupied;
        dens += s.density;
        visc += s.viscosity;
        vmed += s.velocity;
    }
    let inv = 1.0 / pts.len() as f32;
    (occ * inv, dens * inv, vmed * inv, visc * inv)
}
