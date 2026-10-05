//! **布 × 介质（湿布）**：`cloth.rs` 的**子模块**（`#[path]` 声明在那里；`soft/lib.rs` 零余量）。
//!
//! **口径**：逐三角面用**门面预先采好的**介质样本（`cloth.medium`，下标 = 三角序，采样点 = 面心），
//! 施加 **Bridson 线化阻力** `F = ½·ρ_s·Cd·A·u·|u|`（`u = v_medium − v_face`、`A` = 三角面积）
//! ——与 `cloth_aero` 同式，`Cd` 固定 1.0（同 `world_step/medium.rs` 的 `DRAG_CD`）。
//!
//! **为什么力在这里、而不是"门面按时钟给速度加冲量"**（2026-10-10 试刀负结果）：
//! XPBD 是**位置式** —— `predict` 里 `prev = pos; pos += vel·h`，`write_back` 再用
//! `(pos − prev)/h` **重算 `vel`** ⇒ 步前/步后的速度级注入都会被吞。本函数由 `predict`
//! 在 `prev = pos` **之前**调用（与 `apply_aero` 同段位），加的是 `vel`，随后才预测位置。
//!
//! **为什么是自由函数**：本仓 god 门的**类型账**按类型累计方法数（`ClothSheet` 已登记债务
//! "只准减"，方法 36）⇒ 新能力不加方法位。
//!
//! **Jacobi 纪律**（同 `cloth_aero`）：先按调用时的顶点速度把各面的力全算出来、累加进 `dv`，
//! 最后**一次性**施加。⚠️ 已知欠账：`dv` 每子步建一次（只有"布 + 介质同时存在"才走到这里；
//! 并入分组 scratch 属后续片）。
//! **双向**（2026-10-05）：本模块只给 `face_drag`（单一公式来源），反作用 `−F·dt` 的沉积在门面。
use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

/// **单面介质阻力**（N；`k` = 三角序）：`F = ½·ρ·Cd·A·u·|u|`、`u = v_medium − v_face`；
/// `ρ ≤ 0` 或退化面（`2A ≤ 1e-12`）⇒ [`Vec3::ZERO`]。反作用 `−F·h` 由 `inject` 就地累加
/// 进 `cloth.medium_reaction`（**同一份公式**，不给门面开二次实现的口子）。
fn face_drag(cloth: &ClothSheet, k: usize) -> Vec3 {
    let s = cloth.medium[k];
    if s.density <= 0.0 {
        return Vec3::ZERO; // 真空/无介质 ⇒ 零力
    }
    let t = cloth.tris[k];
    let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
    let (p0, p1, p2) = (cloth.pos[i0], cloth.pos[i1], cloth.pos[i2]);
    let two_a = (p1 - p0).cross(p2 - p0).length();
    if two_a <= 1e-12 {
        return Vec3::ZERO;
    }
    let v_face = (cloth.vel[i0] + cloth.vel[i1] + cloth.vel[i2]) * (1.0 / 3.0);
    let u = s.velocity - v_face;
    u * (0.5 * s.density * CD * (two_a * 0.5) * u.length())
}

/// 把"这一子步的介质阻力"注入 `cloth.vel`（`Δv_i = (F/3)·inv_mass_i·h`；长度不符首行短路），
/// 并把**反作用冲量** `−F·h` 累加进 `cloth.medium_reaction`（门面 tick 末读走并清零）。
pub(crate) fn inject(cloth: &mut ClothSheet, h: f32) {
    if cloth.medium.len() != cloth.tris.len() || h <= 0.0 {
        return;
    }
    let mut dv = vec![Vec3::ZERO; cloth.pos.len()];
    for k in 0..cloth.tris.len() {
        let f = face_drag(cloth, k); // 真空/退化面 = ZERO ⇒ 下方的加/减都逐位不变
        let [i0, i1, i2] = cloth.tris[k].map(|x| x as usize);
        cloth.medium_reaction[k] -= f * h;
        dv[i0] += f * (1.0 / 3.0) * (cloth.inv_mass[i0] * h);
        dv[i1] += f * (1.0 / 3.0) * (cloth.inv_mass[i1] * h);
        dv[i2] += f * (1.0 / 3.0) * (cloth.inv_mass[i2] * h);
    }
    cloth.vel.iter_mut().zip(&dv).for_each(|(v, d)| *v += *d);
}

/// 面元阻力系数（与 `world_step/medium.rs` 的 `DRAG_CD` 同值；专用 setter 属后续片）。
const CD: f32 = 1.0;
