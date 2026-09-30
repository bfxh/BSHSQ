//! **耦合契约：统一受体门 + 审计**（`PLAN-COUPLING.md` §3.2/§5 的 **C0 + C1** 落地）。
//!
//! 这一片**不动物理**：门只把"写进 `force/torque`、随后被积分器静默丢弃"换成"提前不写"——
//! 两种写法在物理上都是"睡眠/静态体不生效"（丢弃点见 `vxl-phys-integrate/src/lib.rs:26-30`）；
//! 审计是**纯查询**（不持状态）。
//!
//! **为什么审计不持状态**：`World` 的成员位顶在 god 门棘轮上（`world_struct.rs` 基线
//! `max_type_members = 23/24`，且该文件 `max_fn_lines = 0` ⇒ 没有"函数变短"可交换）⇒ 加字段即红。
//! 于是账一律**现算**：输入是既有状态（体集 + 反作用表），输出是一个可断言的数字。
//!
//! ## 契约两件（本文件的全部对外语义）
//!
//! - [`is_receptor`] / [`add_force`]：**受体门的唯一定义处**。`PLAN-COUPLING.md` §2 B1 记录了原先
//!   三种写法（无门 / 只看 `is_dynamic` / `+awake`）散在十条写入点上；C1 起，门面侧的写入
//!   统一走这里。
//! - [`dropped_force_writes`] / [`fluid_reaction_ledger`]：**两条账**。
//!   前者量"静默丢弃的暴露面"（睡眠/静态体上仍挂着非零 `force/torque`——它们会被积分器置零丢掉）；
//!   后者量"2b 反作用交给了不动的体多少力"（流体那边**照样付了**，这是 §2 A1 的账）。
//!
//! ## 边界（如实）
//!
//! - `vxl-phys-field` 的力场写入**没有**过门（它在另一个 crate，且 `ForceField::apply` 的语义是
//!   "对全体动体累加"）⇒ 那部分暴露面由 [`dropped_force_writes`] 记账，**不**在本片门化。
//! - 软体两腿（`world_soft.rs` 的 `apply_two_leg_reactions`）的门在**代理快照侧**
//!   （`inv_mass = 0` = "睡眠体对软体域呈现为静态"），语义不同、本片**不改**（只登记）。
//! - 引擎自用的接触/关节/CCD 通道不过这道门（它们是求解器内部语义）。

use super::*;

/// **统一受体门**（契约的唯一定义处）：只有"动态 + 清醒 + 质量为正"的体收跨域作用。
///
/// 三条缺一不可：`is_dynamic` 挡静态体；`awake` 挡睡眠体（睡眠体的作用会被积分器丢弃，
/// 早挡一步只是把它从"写后丢"变成"提前不写"）；`inv_mass > 0` 兜底零质量体。
#[inline]
pub(crate) fn is_receptor(bodies: &BodySet, i: usize) -> bool {
    i < bodies.len() && bodies.is_dynamic(i) && bodies.awake[i] && bodies.inv_mass[i] > 0.0
}

/// **经门写力/力矩**（力通道的唯一收口）：门不过 ⇒ 一个字都不写。
///
/// 传 `Vec3::ZERO` 的那个分量是恒等操作（`+= 0.0` 在 f32 下逐位不变）⇒ 只写力的调用点
/// （2a 介质、喷溅阻力）与力+力矩的调用点（2b 反作用、气动）共用这一个入口。
#[inline]
pub(crate) fn add_force(bodies: &mut BodySet, i: usize, force: Vec3, torque: Vec3) {
    if is_receptor(bodies, i) {
        bodies.force[i] += force;
        bodies.torque[i] += torque;
    }
}

/// **静默丢弃的暴露面**（纯查询）：睡眠/静态体上仍挂着非零 `force/torque` 的
/// `(体数, Σ|force| 的向量和, Σ|torque| 的向量和)`。
///
/// 这些量在下一次 `Integrator::integrate_velocities` 里被**置零丢弃**（不消费、不清醒）——
/// 判据把它钉成"可命名、可断言"的一条，而不是一句"已知会丢"。
pub(crate) fn dropped_force_writes(bodies: &BodySet) -> (usize, Vec3, Vec3) {
    let (mut n, mut f, mut t) = (0usize, Vec3::ZERO, Vec3::ZERO);
    for i in 0..bodies.len() {
        if is_receptor(bodies, i) {
            continue;
        }
        if bodies.force[i] != Vec3::ZERO || bodies.torque[i] != Vec3::ZERO {
            n += 1;
            f += bodies.force[i];
            t += bodies.torque[i];
        }
    }
    (n, f, t)
}

/// **2b 反作用的分账**（力与力矩同口径；单位 = 力 / 力矩，未乘 dt）。
///
/// `handed = applied + dropped` 逐项成立（按门做划分），所以这条账本身就是判据。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct ReactionLedger {
    /// 流体交给刚体的**总力**（全部体的反作用之和，含被门挡下的）。
    pub handed: Vec3,
    /// 真正施加到体上的部分。
    pub applied: Vec3,
    /// 交给"不动的体"（睡眠/静态）的部分——被门挡下，**流体那边照样付了**。
    pub dropped: Vec3,
    /// 同一分账的力矩分量。
    pub handed_tau: Vec3,
    pub applied_tau: Vec3,
    pub dropped_tau: Vec3,
    /// 交出反作用的体数 / 其中被挡下的体数（体数按段表条目计，不去重前即"每体一条"）。
    pub handed_bodies: usize,
    pub dropped_bodies: usize,
}

/// 一次划分：给一族反作用表算账（纯函数，可独立断言）。
pub(crate) fn ledger_of(bodies: &BodySet, reactions: &[(u32, Vec3, Vec3)]) -> ReactionLedger {
    let mut led = ReactionLedger::default();
    for &(body, f, tau) in reactions {
        let i = body as usize;
        led.handed += f;
        led.handed_tau += tau;
        led.handed_bodies += 1;
        if is_receptor(bodies, i) {
            led.applied += f;
            led.applied_tau += tau;
        } else {
            led.dropped += f;
            led.dropped_tau += tau;
            led.dropped_bodies += 1;
        }
    }
    led
}

/// **全部 2b 流体的分账**（现算；卡上步进档经 [`super::fluid_stepper::reactions_of`] 同口径读）。
pub(crate) fn fluid_reaction_ledger(world: &World) -> ReactionLedger {
    let mut led = ReactionLedger::default();
    for fi in 0..world.fluids.len() {
        if !world.fluid_boundary.is_two_b(fi) {
            continue;
        }
        let part = ledger_of(
            &world.bodies,
            super::fluid_stepper::reactions_of(&world.fluids[fi]),
        );
        led.handed += part.handed;
        led.applied += part.applied;
        led.dropped += part.dropped;
        led.handed_tau += part.handed_tau;
        led.applied_tau += part.applied_tau;
        led.dropped_tau += part.dropped_tau;
        led.handed_bodies += part.handed_bodies;
        led.dropped_bodies += part.dropped_bodies;
    }
    led
}

/// **tick 末力矩注入**（角反作用那一支的唯一收口）。
///
/// 口径按 `crates/vxl-phys/tests/angular_impulse_contract.rs` 的契约：本注入在**全部子步之后**
/// ⇒ 只被**一个**子步消费 ⇒ 交付"整 tick 角冲量 `tau_tick`"必须 `× substeps / dt`（= `÷ dt_sub`）。
/// ⚠️ 含受体门：原先这条腿在调用点**没有任何 awake/动态检查**（`PLAN-COUPLING.md` §2 A3）。
#[inline]
pub(crate) fn add_tick_torque(
    bodies: &mut BodySet,
    body: u32,
    tau_tick: Vec3,
    substeps: f32,
    dt: f32,
) {
    let b = body as usize;
    if is_receptor(bodies, b) {
        bodies.torque[b] += tau_tick * (substeps / dt);
    }
}

/// **把耦合的账填进 `HealthReport`**（`World::health()` 的调用点；真消费者 ⇒ 审计不是死代码）。
///
/// 两个字段的语义见 `HealthReport` 的字段注释；它们**不**进 `is_clean`（静态墙"吃掉"反作用是
/// 正常物理，不是不干净）。
pub(crate) fn fill_health(rep: &mut HealthReport, world: &World) {
    let led = fluid_reaction_ledger(world);
    let (n, f, _) = dropped_force_writes(&world.bodies);
    rep.coupling_dropped_bodies = led.dropped_bodies as u32 + n as u32;
    rep.coupling_dropped_force = led.dropped.length() + f.length();
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::mass::mass_props;

    /// 手工体集：一个动态体（清醒）、一个动态体（睡眠）、一个静态体。
    fn bodies3() -> (BodySet, [usize; 3]) {
        let mut b = BodySet::new();
        let half = Vec3::new(0.1, 0.1, 0.1);
        let shape = Shape::Box { half };
        let id_dyn = b.push_dynamic(shape, Vec3::ZERO, Quat::IDENTITY, 1000.0) as usize;
        let id_sleep =
            b.push_dynamic(shape, Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
        let id_static = b.push_static(shape, Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY) as usize;
        b.awake[id_sleep] = false;
        assert!(mass_props(&shape, 1000.0).inv_mass > 0.0);
        (b, [id_dyn, id_sleep, id_static])
    }

    /// ① 门的真值表（C1 的语义定义）。
    #[test]
    fn receptor_gate_truth_table() {
        let (b, [dyn_id, sleep_id, static_id]) = bodies3();
        assert!(is_receptor(&b, dyn_id), "动态 + 清醒 + 质量正 ⇒ 收");
        assert!(!is_receptor(&b, sleep_id), "睡眠 ⇒ 不收");
        assert!(!is_receptor(&b, static_id), "静态 ⇒ 不收");
        assert!(!is_receptor(&b, 999), "越界 ⇒ 不收");
    }

    /// ② 经门写：门内写、门外一个字都不写；且丢弃的那部分**可见**（`dropped_force_writes`）。
    #[test]
    fn gate_writes_only_for_receptors_and_the_drop_is_visible() {
        let (mut b, [dyn_id, sleep_id, static_id]) = bodies3();
        let f = Vec3::new(3.0, -4.0, 5.0);
        add_force(&mut b, dyn_id, f, Vec3::ZERO);
        add_force(&mut b, sleep_id, f, Vec3::ZERO);
        add_force(&mut b, static_id, f, Vec3::ZERO);
        assert_eq!(b.force[dyn_id], f, "门内：照写");
        assert_eq!(b.force[sleep_id], Vec3::ZERO, "睡眠：不写");
        assert_eq!(b.force[static_id], Vec3::ZERO, "静态：不写");
        // 丢弃的账：手工把力放回睡眠体（模拟"没过门的写入"）⇒ 账必须看见它。
        b.force[sleep_id] = f;
        let (n, sum, tau) = dropped_force_writes(&b);
        assert_eq!((n, sum, tau), (1, f, Vec3::ZERO), "睡眠体上挂着的力被记账");
        // 积分器随后把它丢掉且不唤醒（现状语义：睡眠体的作用不生效）。
        Integrator::integrate_velocities(&mut b, Vec3::ZERO, 1.0 / 60.0, 1e6, 1e6);
        assert_eq!(
            b.linvel[sleep_id],
            Vec3::ZERO,
            "睡眠体被动过速度 ⇒ 口径破了"
        );
        assert_eq!(b.force[sleep_id], Vec3::ZERO, "积分器消费后清零");
        assert_eq!(b.force[dyn_id], Vec3::ZERO, "清醒体被消费（加速度进速度）");
        assert!(b.linvel[dyn_id].length() > 0.0, "清醒体真的动了");
    }

    /// ③ 分账：`handed == applied + dropped`，且划分成员与门一致（含越界体号的兜底）。
    #[test]
    fn ledger_partitions_applied_and_dropped() {
        let (b, [dyn_id, sleep_id, static_id]) = bodies3();
        let mk = |x: f32| Vec3::new(x, 0.0, 0.0);
        // ⚠️ 全部取**可精确表示的二进制值**（0.25/0.5/1/2）：`0.1+0.2+0.4+0.8` 在 f32 下 ≠ 1.5
        // （累加序的舍入）⇒ 用十进制小数会让这条"账必须平"的判据红在浮点上而不是账上。
        let reacts = vec![
            (dyn_id as u32, mk(1.0), mk(0.25)),
            (sleep_id as u32, mk(2.0), mk(0.5)),
            (static_id as u32, mk(4.0), mk(1.0)),
            (999u32, mk(8.0), mk(2.0)), // 越界体号（不应出现，但账要有定义）
        ];
        let led = ledger_of(&b, &reacts);
        assert_eq!(led.handed, mk(15.0));
        assert_eq!(led.applied, mk(1.0));
        assert_eq!(led.dropped, mk(14.0));
        assert_eq!(led.handed, led.applied + led.dropped, "账必须平");
        assert_eq!(led.handed_tau, led.applied_tau + led.dropped_tau);
        assert_eq!((led.handed_bodies, led.dropped_bodies), (4, 3));
        assert_eq!(led.dropped_tau, mk(3.5));
    }

    /// ④ **端到端**：真 2b 场景（静态地板 + 水块）⇒ 流体确实把力交给了"不动的体"，
    /// 且这笔账非零（金丝雀：`handed` 必须非零，否则判据空过）。
    ///
    /// 物理事实（不是缺陷）：静态体是墙，水对墙的作用被墙的地基吃掉 ⇒ `applied = 0`。
    /// 本判据钉的是**账的存在**：这笔力不再"没人知道"，且门是唯一的分界。
    #[test]
    fn static_floor_liquid_reaction_is_accounted() {
        let mut w = World::new(PhysConfig::default());
        // 静态地板（解析盒 ⇒ 2b 会为它生成边界粒子；体素 provider **不会**）。
        let floor = w.bodies.push_static(
            Shape::Box {
                half: Vec3::new(0.6, 0.05, 0.6),
            },
            Vec3::new(0.0, 0.0, 0.0),
            Quat::IDENTITY,
        ) as usize;
        // 水块（8³@0.05 = 0.4 m 立方）铸装在地板上方。
        let sys = vxl_phys_fluid::FluidSystem::new(
            vxl_phys_fluid::FluidConfig::default(),
            Vec3::new(-0.2, 0.06, -0.2),
            [8, 8, 8],
            0.05,
        );
        w.add_fluid_with_boundary_coupling(sys, &[]);
        for _ in 0..8 {
            w.step();
        }
        let led = fluid_reaction_ledger(&w);
        // **金丝雀**：反作用必须真的是非零的（否则这条判据测的是"零 == 零"）。
        assert!(
            led.handed.length() > 0.0,
            "2b 反作用为零 ⇒ 场景没建立起接触 ⇒ 判据空过（handed = {:?}）",
            led.handed
        );
        assert!(led.dropped_bodies >= 1, "静态地板应当被记为'不动的体'");
        assert_eq!(led.applied, Vec3::ZERO, "场景里没有动态体 ⇒ 一分钱也没进体");
        assert_eq!(led.handed, led.dropped, "全部丢失（有账）");
        // 地板没被推动（静态体不动）。
        assert_eq!(w.bodies.position[floor], Vec3::ZERO);
    }

    /// ⑤ **睡眠体**（不是静态）：同样被门挡下、同样有账；且它不会被反作用唤醒。
    #[test]
    fn sleeping_floor_liquid_reaction_is_accounted() {
        let mut w = World::new(PhysConfig::default());
        let floor = w.bodies.push_dynamic(
            Shape::Box {
                half: Vec3::new(0.6, 0.05, 0.6),
            },
            Vec3::new(0.0, 0.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        let sys = vxl_phys_fluid::FluidSystem::new(
            vxl_phys_fluid::FluidConfig::default(),
            Vec3::new(-0.2, 0.06, -0.2),
            [8, 8, 8],
            0.05,
        );
        w.add_fluid_with_boundary_coupling(sys, &[]);
        for _ in 0..4 {
            w.step();
        }
        let before = w.bodies.position[floor];
        let v_before = w.bodies.linvel[floor];
        w.bodies.awake[floor] = false; // 手工入睡（判据要的是"睡眠体"这一档）
        w.step();
        let led = fluid_reaction_ledger(&w);
        assert!(led.handed.length() > 0.0, "反作用为零 ⇒ 判据空过");
        assert!(led.dropped_bodies >= 1, "睡眠地板应当被记为'不动的体'");
        assert_eq!(w.bodies.position[floor], before, "睡眠体不该被推走");
        assert_eq!(w.bodies.linvel[floor], v_before, "睡眠体不该被提速");
        assert!(
            !w.bodies.awake[floor],
            "反作用不该唤醒它（现状语义；唤醒策略另立）"
        );
    }
}
