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

/// **布 × 液的反作用段（布 → 流体，2026-10-05）**：把每张布逐面受的介质阻力**等大反向**
/// 沉积回流场（`MediumField::deposit`，点式；分摊与守恒证明见 `vxl_phys_fluid::fluid_medium`）。
///
/// **口径**：与 2a 既有约定同款 —— **一 tick 滞后、值取最新状态**（流体本 tick 已推进，沉积
/// 喂的是**下一 tick**）。逐面力由 `vxl_phys_soft` 的 `cloth_medium::face_drag` 给（与 `predict`
/// 里那份**同一式**）⇒ 本段不重写任何公式。**无布或无流体 ⇒ 首行短路**（其余场景零成本）。
///
/// **为什么默认生效（不是开关）**：牛顿第三定律不是可选项；且全仓"布 + 介质同时存在"的场景
/// 只有 `tests/wet_cloth_gap.rs` 一个，三条冻结哈希（`determinism` / `m0_gates` / `m1_islands`）
/// 都不含布 ⇒ 默认档逐位不变。多流体时取第一个（与采样侧 `fill_cloth_medium` 同口径）。
pub(super) fn cloth_medium_reaction(w: &mut World) {
    if w.soft.cloths.is_empty() || w.fluids.is_empty() {
        return;
    }
    let dt = w.config.dt;
    let mut pending: Vec<(Vec3, Vec3)> = Vec::new();
    for cloth in &w.soft.cloths {
        if cloth.medium.len() != cloth.tris.len() {
            continue; // 本 tick 没采到介质（无流体/未填）⇒ 零作用
        }
        for k in 0..cloth.tris.len() {
            let f = vxl_phys_soft::cloth::cloth_medium::face_drag(cloth, k);
            if f == Vec3::ZERO {
                continue;
            }
            let t = cloth.tris[k];
            let c =
                (cloth.pos[t[0] as usize] + cloth.pos[t[1] as usize] + cloth.pos[t[2] as usize])
                    * (1.0 / 3.0);
            pending.push((c, f * (-dt)));
        }
    }
    if pending.is_empty() {
        return;
    }
    use vxl_phys_core::interop::MediumField as _;
    let fluid = &mut w.fluids[0].0;
    for (p, j) in pending {
        fluid.deposit(p, j, 0.0, 0.0);
    }
}
