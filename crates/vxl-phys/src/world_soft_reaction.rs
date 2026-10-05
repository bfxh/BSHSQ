//! **软体域的反作用自由函数**（`world_soft.rs` 的 `#[path]` 子模块）：绳索角反作用 +
//! **布 → 介质**的反作用段。都是 `&mut World` 的旁观者逻辑 —— 挂 `impl World` 会让
//! god 门的**方法数债务**变胖（`World` 76 方法，只准减），故走自由函数（同
//! `world_step` 的 `splat_body_drag` / `integrate_velocity_pass` 先例）。
//!
//! **为什么单开文件**：`world_soft.rs` 受**文件尺寸棘轮**（只准减）；本片新增的说明文字
//! 量大，留在原文件会把"函数降 13 行"的净账冲掉。显式导入（glob-gate：新文件零通配）。
use crate::world_step::coupling as cpl;
use crate::{BodySet, Vec3, World};
use vxl_phys_soft::Rope;

/// **绳索的角反作用回填**（计划 2c-3，**默认开** since 2026-10-01；从 `rope_pass` 原样搬出）。
///
/// 关闭的理由不是"力矩算错了"，而是**接触模型看不见转动**：`body_disp` 只跟踪平移、
/// 摩擦的滑移用 `b.linvel` 而非 `linvel + ω×r`、`crossed_face` 用的是**冻结的** `rot`
/// ⇒ 回填角动量等于注入模型看不见的运动（实测 1 kg 薄盒：`hit = 1` 时 `|ω|` 一 tick 就到
/// 5~8 rad/s ⇒ 接触立刻丢 ⇒ 盒子被甩下去，门面 1800 tick y = −2420）。**翻默认的依据**
/// （换代级，与 C2 同批，`PLAN-COUPLING.md` §5 P1）：§8.4.31/§8.4.32 实测打开后中心场景仍托住，
/// 且 `angular_reaction_holds`（带转动的自扮引擎）与 `rope_scene` 偏置判据守着符号与量级；
/// 残留局限 = 上面的"模型看不见转动"三条（转动感知代理，见 `PLAN-COUPLING.md` §9）。
///
/// ⚠️ **口径**（`crates/vxl-phys/tests/angular_impulse_contract.rs` 钉住，2026-09-28 §8.4.28）：
/// `torque` 是**每子步消费并清零**的累加器，而本处注入发生在**所有子步之后** ⇒ 只被**一个**
/// 子步消费 ⇒ 交付"整 tick 的角冲量 `r.torque`"必须 `÷ dt_sub`（= `× substeps / dt`）；
/// 写成 `÷ dt` 只会交付 `1/substeps`（默认 2 ⇒ **差 2×**，那条判据的金丝雀里实测比值
/// 正好 `0.500000`）。
pub(super) fn apply_rope_angular_reactions(
    bodies: &mut BodySet,
    rope: &Rope,
    substeps: f32,
    dt: f32,
) {
    for r in &rope.reactions {
        // 经受体门 + tick 末注入契约（`PLAN-COUPLING.md` §2 A3 / §3.5：原先无 awake 检查）。
        cpl::add_tick_torque(bodies, r.body, r.torque, substeps, dt);
    }
}

/// **布 × 液的反作用段（布 → 流体，2026-10-05）**：把每张布逐面**已累加好的**反作用冲量沉积回
/// 流场（`MediumField::deposit`，点式；分摊与守恒证明见 `vxl_phys_fluid::fluid_medium`）。
///
/// **冲量不在这里算**：`predict` 每个子步把 `−F(s)·h` 累加进 `cloth.medium_reaction`（与受力
/// **同一份公式**）⇒ 这里读到的是**整 tick 的精确冲量**，不是"按 tick 末速度重算"的近似。
/// 口径仍与 2a 同款（**一 tick 滞后**）；**为什么默认生效**：全仓"布 + 介质"场景只有
/// `wet_cloth_gap.rs` 一个，三条冻结哈希都不含布 ⇒ 默认档逐位不变；多流体取第一个。
pub(super) fn cloth_medium_reaction(w: &mut World) {
    if w.soft.cloths.is_empty() || w.fluids.is_empty() {
        return;
    }
    let pending = cloth_impulses(w);
    // 冲量是**逐 tick 累加器**：读完立刻原地清零（**不改长度** ⇒ 不触发重分配）。
    for cloth in &mut w.soft.cloths {
        for j in cloth.medium_reaction.iter_mut() {
            *j = Vec3::ZERO;
        }
    }
    if pending.is_empty() {
        return;
    }
    use vxl_phys_core::interop::MediumField as _;
    for (p, j) in pending {
        w.fluids[0].0.deposit(p, j, 0.0, 0.0);
    }
}

/// 逐面收**已累加好的**反作用冲量（`cloth.medium_reaction`）与面心；未采样（本 tick 无介质）跳过。
fn cloth_impulses(w: &World) -> Vec<(Vec3, Vec3)> {
    let mut pending: Vec<(Vec3, Vec3)> = Vec::new();
    for cloth in &w.soft.cloths {
        if cloth.medium.len() != cloth.tris.len() || cloth.medium_reaction.len() != cloth.tris.len()
        {
            continue;
        }
        for k in 0..cloth.tris.len() {
            let j = cloth.medium_reaction[k];
            if j == Vec3::ZERO {
                continue;
            }
            let t = cloth.tris[k];
            let c =
                (cloth.pos[t[0] as usize] + cloth.pos[t[1] as usize] + cloth.pos[t[2] as usize])
                    * (1.0 / 3.0);
            pending.push((c, j));
        }
    }
    pending
}
