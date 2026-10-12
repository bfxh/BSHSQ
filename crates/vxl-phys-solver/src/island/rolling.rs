//! **滚动阻力行** —— `island` 的子模块（`PhysConfig::roll_resistance`，米 = 等效力臂）。
//!
//! 为什么单独一个模块：`island.rs` 受 god 门文件行数棘轮管，而"接触面上绕滚动方向的一维角约束"
//! 是与"建约束/迭代"相对独立的关注点（同 `solver_impl/sleep.rs`、`constraint/friction.rs` 的拆法）。

use crate::{ContactConstraint, Mat3, Vec3};

/// 给一条约束加"滚动阻力行"：绕**当前滚动方向**（`ω_rel` 的切向分量）的一维角约束，
/// 上限 `roll_res · Σpn`（与 Rapier 的 twist 行同构，只是轴取滚动方向而不是法向）。
///
/// **不做跨帧暖启动**：滚动阻力是阻尼项、跨帧累计价值低，而暖启动要动"每点 + 每流形"的暖数据
/// 布局（记忆与换代成本都不划算）⇒ 累加器 `roll_imp` 寿命 = 一个子步。
pub(crate) fn solve_rolling(
    c: &mut ContactConstraint,
    av: &mut [Vec3],
    local_of: &[u32],
    iw: &[Mat3],
) {
    let (ka, kb) = (local_of[c.a as usize], local_of[c.b as usize]);
    if ka == u32::MAX && kb == u32::MAX {
        return;
    }
    let pn: f32 = c.points[..c.npts as usize].iter().map(|p| p.pn).sum();
    if pn <= 0.0 {
        return;
    }
    let wa = if ka == u32::MAX {
        Vec3::ZERO
    } else {
        av[ka as usize]
    };
    let wb = if kb == u32::MAX {
        Vec3::ZERO
    } else {
        av[kb as usize]
    };
    let rel = wa - wb;
    let roll = rel - c.normal * rel.dot(c.normal);
    let rate = roll.length();
    if rate < 1e-6 {
        return; // 没在滚（含纯滑动/静置）
    }
    let axis = roll * (1.0 / rate);
    let mut k = 0.0f32;
    if ka != u32::MAX {
        k += axis.dot(iw[ka as usize].mul_vec3(axis));
    }
    if kb != u32::MAX {
        k += axis.dot(iw[kb as usize].mul_vec3(axis));
    }
    if k <= 1e-12 {
        return; // 两侧都不可绕该轴转（全静态/惯量奇异）
    }
    let limit = c.roll_res * pn;
    let imp = (c.roll_imp - rate / k).clamp(-limit, limit);
    let dl = imp - c.roll_imp;
    c.roll_imp = imp;
    if dl == 0.0 {
        return;
    }
    if ka != u32::MAX {
        let q = ka as usize;
        av[q] += iw[q].mul_vec3(axis) * dl;
    }
    if kb != u32::MAX {
        let q = kb as usize;
        av[q] -= iw[q].mul_vec3(axis) * dl;
    }
}
