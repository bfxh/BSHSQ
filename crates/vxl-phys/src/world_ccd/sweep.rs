//! 选择性 CCD 的**单体裁扫掠**（从 `world_ccd.rs` 抽出；F-15 两处修复落在这里）。
//!
//! **审计 F-15 的三条**（`config.rs` 默认档 / `ccd.rs` 采样上限 / 目标集只含静态与沉睡）：
//!
//! 1. **默认选择性**（2026-10-13 起，`SPEC §4.12` 的应然落地）：`ccd_speed_threshold`
//!    默认 **10 m/s**（与 §4.3 的 `contact_skin = 0.02` **成对**：0.02/0.01/0.005/0.002 ↔
//!    10/5/2/1）。本模块只在**速度阈值 × 尺寸比**双判据同时满足时才跑（`World::ccd_pass`
//!    首行短路）。显式 `INFINITY` = **关档**：那时**没有穿透保证**（有无取决于采样相位，
//!    实测两端都有）——判据 `tests/ccd_tunneling.rs` 把"默认挡住 / 关档无保证"两条都钉住。
//!    ⚠️ 默认档变更是**零换代**的：determinism / m0_gates 的冻结哈希与 5+3 个金样都没变
//!    （双判据下这些场景里没有体同时满足）。
//! 2. **采样上限自洽**：段长必须 ≤ 0.5×最小半长（`ccd.rs` 的设计意图）。预算
//!    （`ccd_max_steps`）不够覆盖本 tick 位移时，旧实现**默默**把段数截到上限 ⇒ 仍会穿透
//!    （审计实测：3000 m/s 开 CCD 照样穿）。现在改为**限速到预算覆盖得了的速度**——宁可
//!    这一 tick 走慢一点，也不静默穿透。
//! 3. **目标集含运动体**：旧实现 `retain(!is_dynamic || !awake)` 把**运动目标整条排除**
//!    ⇒ 移动墙必穿（审计实测）。现在把动态目标也纳入，并按**相对运动**采样与判定
//!    （目标位置随采样时刻推进、闭合判据用法向**相对**速度）。响应仍是 M1 口径的非弹性
//!    停（只改动体的法向速度）——**双向冲量留给 M2**，这里如实标出，不假装它是完备解。

use vxl_phys_core::{Aabb, Vec3};
use vxl_phys_narrow::NarrowPhase;

use crate::ccd;
use crate::World;

/// **限速到预算覆盖得了的速度**（审计 F-15 ②）：段长 ≤ 0.5×最小半长 ⇒
/// `v_max = ccd_max_steps × 0.5·ext / dt`。默认档下这条基本不触发（100 m/s 的位移在
/// 64 段 × 0.5·ext 之内），只有把 `max_linear_velocity` 放开到几千 m/s 才会咬住。
fn clamp_one(world: &mut World, i: usize, dt: f32) {
    let ext = ccd::min_half_extent(&world.bodies.shape[i]);
    if !ext.is_finite() || ext <= 0.0 || !dt.is_finite() || dt <= 0.0 {
        return;
    }
    let budget = world.config.ccd_max_steps.max(1) as f32;
    let v_max = budget * (ext * 0.5) / dt;
    let sp = world.bodies.linvel[i].length();
    if sp > v_max && sp > 0.0 {
        world.bodies.linvel[i] *= v_max / sp;
    }
}

/// **位置积分之前**收口：把"开 CCD 但预算覆盖不了"的高速体限速。
///
/// ⚠️ 必须在**位置积分之前**（`substep` 里 `integrate_positions` 之前）调用：CCD 的回扫窗口是
/// `p1 − v·dt`，只有"限速后的 v"就是"本子步积分用的 v"时，那个窗口才等于真实走过的路径。
/// 放在积分之后（本函数从 `ccd_pass` 里拆出来之前就是这么写的）会让第一个子步整段不受检——
/// 实测 3000 m/s 的球第一子步就跳过 25 m，回扫窗口只剩 1.6 m，直接穿过薄墙。
pub(crate) fn clamp_flagged(world: &mut World, dt: f32) {
    if !world.config.ccd_speed_threshold.is_finite() {
        return;
    }
    for i in 0..world.bodies.len() {
        if ccd::needs_ccd(&world.bodies, i, &world.config, dt) {
            clamp_one(world, i, dt);
        }
    }
}

/// 一个候选目标在采样时刻的位置：`q(t) = q1 − v_j·(dt − t)`（静态/沉睡 ⇒ `v_j = 0`）。
struct Target {
    id: usize,
    q1: Vec3,
    vel: Vec3,
}

/// 一次扫掠的不变量（`p0 = p1 − v·dt`、段数/段长），打包起来免得每个助手都摊一长串参数
/// （`args-gate` 的 soft 阈值实测把 7 个形参也算违规）。
struct Scan {
    p0: Vec3,
    p1: Vec3,
    v: Vec3,
    dt: f32,
    sub: f32,
    steps: u32,
}

/// 扫掠 AABB（`p0..p1` 膨胀 `skin`）。
fn swept_aabb(p0: Vec3, p1: Vec3, skin: f32) -> Aabb {
    Aabb {
        min: p0.min(p1) - Vec3::splat(skin),
        max: p0.max(p1) + Vec3::splat(skin),
    }
}

/// 扫掠 AABB → `(窄相配对, 目标表)`。**目标表含运动体**（审计 F-15 ③）：旧实现把动态体整条
/// 排除 ⇒ 移动墙必穿；这里把每个候选的"tick 末位置 + 速度"记下来，采样时按时间推进。
fn gather(world: &mut World, i: usize, swept: Aabb) -> (Vec<(u32, u32)>, Vec<Target>) {
    let mut candidates: Vec<u32> = Vec::new();
    world.broad.query_aabb(&swept, &mut candidates);
    candidates.retain(|&j| j as usize != i);

    let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(candidates.len());
    let mut targets: Vec<Target> = Vec::with_capacity(candidates.len());
    for &j in &candidates {
        let ju = j as usize;
        let (a, b) = if ju < i { (j, i as u32) } else { (i as u32, j) };
        pairs.push((a, b));
        targets.push(Target {
            id: ju,
            q1: world.bodies.position[ju],
            // 目标的速度：**运动学体也算**（移动的墙/平台必须在相对运动里带上自己的速度，
            // 否则"移动目标必穿"这条又会回来 —— 见 `ROUTE.md` 的 CCD 三处修复）。
            vel: if world.bodies.is_dynamic_or_kinematic(ju) {
                world.bodies.linvel[ju]
            } else {
                Vec3::ZERO
            },
        });
    }
    (pairs, targets)
}

/// 沿段推进采样，返回首个"正在接近"的命中 `(采样时刻, 指向动体的法线, 对方体号)`。
fn scan(
    world: &mut World,
    i: usize,
    sc: &Scan,
    pairs: &[(u32, u32)],
    targets: &[Target],
) -> Option<(f32, Vec3, usize)> {
    for s in 1..=sc.steps {
        let t = sc.sub * s as f32;
        world.bodies.position[i] = sc.p0 + sc.v * t;
        // 运动目标随采样时刻推进（静态/沉睡目标 `vel = 0` ⇒ 位置不变）。
        for tg in targets {
            if tg.vel != Vec3::ZERO {
                world.bodies.position[tg.id] = tg.q1 - tg.vel * (sc.dt - t);
            }
        }
        // CCD 逐采样已在时间维上走路径 ⇒ 不再叠加视野预测（否则会继承主窄相设的 predict_dt）。
        world.narrow.set_predict_dt(0.0);
        world.narrow.collide(
            &world.bodies,
            pairs,
            world.terrain.slice(),
            &world.providers,
            &mut world.ccd_manifolds,
            world.jobs.as_ref(),
        );
        if let Some(m) = world.ccd_manifolds.first() {
            let other = if m.a == i as u32 { m.b } else { m.a } as usize;
            // 法线 a→b；换算成「推离表面、指向动体」的方向。
            let n_into_body = if m.a == i as u32 { -m.normal } else { m.normal };
            // **只有"正在接近表面"的采样才算命中**：贴地滑行/静置的体每个采样都天生有接触，
            // 按"有接触即命中"会把它钳回起点、原地锁死（实测：弹体滑到墙前 0.1 m 停住）。
            // 判据用**相对速度**（审计 F-15 ③）：目标迎面开来时，静止的动体也在关闭。
            let v_rel = sc.v - world.bodies.linvel[other];
            if n_into_body.dot(v_rel) < 0.0 {
                return Some((t, n_into_body, other));
            }
        }
    }
    None
}

/// 目标位置复原（扫掠借出去的位移必须还回来——它们不是这一 tick 的最终状态）。
fn restore_targets(world: &mut World, targets: &[Target]) {
    for tg in targets {
        world.bodies.position[tg.id] = tg.q1;
    }
}

/// 命中后的收口：钳到上一安全采样点 + **按质量分摊的非弹性相对停**。
///
/// 为什么不能只改动体（旧写法）：2 kg 的球撞 1600 kg 的墙时，两体各跑一遍 sweep，各自把
/// "相对法向速度"清零 ⇒ 结果像"球把墙拖走"（实测两体都以球的速度前进）。沿法线做标准一维
/// 非弹性冲量后：轻球被重墙挡下、墙几乎不动；静态目标 `inv_mass = 0` ⇒ 退化成原写法
/// （与 M1 逐位一致）。双向**转动**耦合仍留给 M2，这里如实标出。
fn resolve(world: &mut World, i: usize, sc: &Scan, hit: Option<(f32, Vec3, usize)>) {
    let Some((t_hit, n_into_body, other)) = hit else {
        world.bodies.position[i] = sc.p1;
        return;
    };
    let t_safe = (t_hit - sc.sub).max(0.0);
    world.bodies.position[i] = sc.p0 + sc.v * t_safe;
    let v_rel = sc.v - world.bodies.linvel[other];
    let vn = v_rel.dot(n_into_body);
    if vn >= 0.0 {
        return;
    }
    let (ia, ib) = (world.bodies.inv_mass[i], world.bodies.inv_mass[other]);
    let denom = ia + ib;
    if denom > 0.0 {
        let jn = -vn / denom;
        world.bodies.linvel[i] += n_into_body * (jn * ia);
        world.bodies.linvel[other] -= n_into_body * (jn * ib);
    }
}

/// 单个体 `i` 本 tick 的保守推进扫掠（见模块头注）。
pub(crate) fn sweep(world: &mut World, i: usize, dt: f32) {
    let v = world.bodies.linvel[i];
    let sc = Scan {
        p1: world.bodies.position[i],
        p0: world.bodies.position[i] - v * dt,
        v,
        dt,
        sub: dt / ccd::step_count(&world.bodies, i, &world.config, dt) as f32,
        steps: ccd::step_count(&world.bodies, i, &world.config, dt),
    };
    let swept = swept_aabb(sc.p0, sc.p1, world.config.contact_skin);
    let (pairs, targets) = gather(world, i, swept);
    let hit = scan(world, i, &sc, &pairs, &targets);
    restore_targets(world, &targets);
    resolve(world, i, &sc, hit);
}
