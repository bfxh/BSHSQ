//! **2b 反作用冲量大账**（两族共用一条判据）。
//!
//! 为什么另起一条：WCSPH 档的守卫（`boundary_reaction_balance.rs`）读的是
//! `Σ_i m·a_i + Σ_j bforce_j ≈ 0`——那**依赖"力写在 `acc` 里"**。PBF 档的力不是加速度而是
//! **位移修正**（`acc` 里只有重力）⇒ 同一条断言在 PBF 档会退化成空断言。改成**冲量式**：
//!
//! 零重力 + `NoProviders`（无壁面 provider）⇒ 除成对力外没有别的力 ⇒ 一个 tick 内
//! ```text
//! Σ_i m_i·(v_i(末) − v_i(初)) + Σ_tick Σ_体 F̄_tick · dt_tick ≈ 0
//! ```
//! `F̄_tick = boundary_reactions().1` 是 **tick 平均力**（C2 口径：逐子步累加、`step` 末尾 ÷
//! 子步数）⇒ 冲量 = `F̄ · dt_tick`；流体侧用 `raw_particles()` 的 `n_fluid` 前缀（`velocities()`
//! 含边界粒子，不能整段求和）。
//!
//! 顺带钉**符号**：界面斥力把流体推离盒子 ⇒ 盒子受的反作用与流体动量变化**反向**
//! （`dot(Σ F̄dt, Σ m Δv) < 0`）——反作用符号错在真耦合里极难一眼看出。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, DensityIterations, FluidConfig, FluidFamily, FluidSystem};

struct Ledger {
    fluid_impulse: Vec3,
    reaction_impulse: Vec3,
    scale: f32,
}

/// 场景与 `boundary_reaction_balance.rs` **逐参数相同**（4³ 流体块 @ y∈[0.2,0.35]，一个
/// 半长 0.15 的盒 @ y=0.55 ⇒ 底面离块顶一个晶格间距，形成真实成对力），只换 `family`
/// 与 `xsph_eps`。
///
/// ⚠️ `xsph_eps` 是**这条账本的关键轴**：XSPH 的成对权重对流体粒子对是**对称的**（动量守恒），
/// 但**边界粒子永不回填**（运动学冻结）⇒ 流体-边界那一半的动量被 `(v_j − v_i)` 提走、
/// 却不记进 `bforce` ⇒ **XSPH 是反作用账的漏项**。WCSPH 的老守卫读 `acc`（力相位），
/// 而 XSPH 是**积分相位**加的 ⇒ 老守卫看不见它（那正是它俩读数差这么大的原因）。
/// 所以账本分两档：`eps = 0`（纯成对力账，判据）+ `eps = 默认`（漏项的量）。
fn ledger(family: FluidFamily, xsph_eps: f32) -> Ledger {
    let dt_tick = 1.0 / 60.0;
    let cfg = FluidConfig {
        family,
        density_iterations: DensityIterations::Four,
        xsph_viscosity: xsph_eps,
        gravity: Vec3::ZERO,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [4, 4, 4], 0.05);
    let body = (
        0u32,
        Shape::Box {
            half: Vec3::splat(0.15),
        },
        BodyPose {
            pos: Vec3::new(0.0, 0.55, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
        },
    );
    let nb = f.set_boundary_particles(&[body]);
    assert!(nb > 0, "应当造出边界粒子");
    // 给流体一个**朝向盒子**的初速度 ⇒ 真压缩：盒子的边界质量落进核内 ⇒ `C > 0` ⇒ 真排斥。
    //
    // ⚠️ 不能再用"表面截断的假吸引"当驱动力：`lambda_pass` 的 `λ ≤ 0` 一边倒约束（自由表面
    // 口径，见 `pbf_incompressibility_probe.rs` 头注）已经把 `C < 0 ⇒ λ > 0` 的吸引拿掉了。
    // 旧版这条用例之所以量到反作用，恰恰是因为流体被**吸**向盒子——那是伪力，不是耦合。
    let nf_init = f.len();
    f.set_velocities(&vec![Vec3::new(0.0, 0.5, 0.0); nf_init]);
    let (_, v0, mass, nf) = f.raw_particles();
    let v0: Vec<Vec3> = v0[..nf].to_vec();
    let mass: Vec<f32> = mass[..nf].to_vec();
    let mut reaction_impulse = Vec3::ZERO;
    let mut scale = 0.0f32;
    for _ in 0..3 {
        f.step(dt_tick, &NoProviders);
        let mut fr = Vec3::ZERO;
        for &(_, force, _) in f.boundary_reactions() {
            fr += force;
        }
        reaction_impulse += fr * dt_tick;
        scale += fr.length() * dt_tick;
    }
    let mut fluid_impulse = Vec3::ZERO;
    for (i, v1) in f.velocities()[..nf].iter().enumerate() {
        let dv = *v1 - v0[i];
        fluid_impulse += dv * mass[i];
        scale += dv.length() * mass[i];
    }
    Ledger {
        fluid_impulse,
        reaction_impulse,
        scale,
    }
}

#[test]
fn pbf_and_wcsph_reaction_impulse_balances_fluid_momentum() {
    // 先量**漏项**（XSPH 默认档），再量**纯成对力账**（该断言的判据）。
    for (name, family) in [("wcsph", FluidFamily::CpuSph), ("pbf", FluidFamily::CpuPbf)] {
        let d = FluidConfig::default().xsph_viscosity;
        let leak = ledger(family, d);
        let rel_leak = if leak.scale > 0.0 {
            (leak.fluid_impulse + leak.reaction_impulse).length() / leak.scale
        } else {
            0.0
        };
        eprintln!(
            "{name:6} XSPH 漏项（ε={d}）：|Σ m·Δv| = {:.3e} | |Σ F̄dt| = {:.3e} | 相对残差 {rel_leak:.2e}",
            leak.fluid_impulse.length(),
            leak.reaction_impulse.length()
        );

        let l = ledger(family, 0.0);
        let sum = l.fluid_impulse + l.reaction_impulse;
        let rel = if l.scale > 0.0 {
            sum.length() / l.scale
        } else {
            0.0
        };
        eprintln!(
            "{name:6} 纯成对力账（ε=0）：|Σ m·Δv| = {:.3e} | |Σ F̄dt| = {:.3e} | |残差| = {:.3e} | 相对 {rel:.2e}",
            l.fluid_impulse.length(),
            l.reaction_impulse.length(),
            sum.length()
        );
        // 非平凡：本用例必须真的产生了反作用冲量与流体动量变化，否则下面是空断言。
        assert!(
            l.scale > 0.0 && l.reaction_impulse.length() > 0.0 && l.fluid_impulse.length() > 0.0,
            "{name} 用例非平凡性不足：fluid {:?} / reaction {:?}",
            l.fluid_impulse,
            l.reaction_impulse
        );
        // 成对力等大反向 ⇒ 只剩浮点累加序的残差（同 `boundary_reaction_balance` 口径）。
        assert!(
            rel < 1e-2,
            "{name} 冲量账不平：|Σ m·Δv + Σ F̄dt| / scale = {rel}（残差 {sum:?}）"
        );
        // 符号：反作用与流体动量变化反向。
        assert!(
            l.fluid_impulse.dot(l.reaction_impulse) < 0.0,
            "{name} 反作用符号错（应与流体动量变化反向）：mv {:?} vs F̄dt {:?}",
            l.fluid_impulse,
            l.reaction_impulse
        );
        // XSPH 漏项必须**存在**且**同向**：它与成对力走同一套邻域权重，边界粒子却永不回填
        // `bforce` ⇒ 加上它只会把账本残差推大（与场景里流体升/降无关，故比"动量多少"稳）。
        assert!(
            rel_leak > rel * 100.0,
            "{name} XSPH 漏项应显著推大残差：ε 档 {rel_leak} vs ε=0 档 {rel}"
        );
    }
}
