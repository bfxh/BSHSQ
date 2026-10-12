//! 选择性 CCD 的**单体裁扫掠**（从 `world_ccd.rs` 抽出；F-15 两处修复落在这里）。
//!
//! **审计 F-15 的三条**（`config.rs` 默认关 / `ccd.rs` 采样上限 / 目标集只含静态与沉睡）：
//!
//! 1. **默认选择性**（2026-10-13 起，§4.12 的应然落地）：`ccd_speed_threshold` 默认 **10 m/s**
//!    （与 §4.3 的 `contact_skin = 0.02` **成对**：0.02/0.01/0.005/0.002 ↔ 10/5/2/1）。
//!    本模块只在**速度阈值 × 尺寸比**双判据同时满足时才跑（`World::ccd_pass`）。
//!    显式 `INFINITY` = **关档**：那时**没有穿透保证**（有无取决于采样相位，实测两端都有，
//!    `tests/ccd_tunneling.rs` ① 与 ①′ 分别钉住）。
//!    ⚠️ **历史（P17，同日修）**：首次落它时被自己的真实性基准挡下 —— R6（10 节球链）的关节分离
//!    `sep_p99` 0.00966 → **0.12382**（13×）、>5cm 违规率 0 → **1.97%**，机制是**扫掠钳位不看
//!    关节约束**；修法 = CCD 之后按**关节连通分量**做位置级投影（`vxl-phys-solver/src/joints/project.rs`）
//!    ⇒ 修后开档回到 `0.00984 / 0.000`（R6 逐字基线按 ADR-0004 换代）。
//! 2. **采样上限自洽**：段长必须 ≤ 0.5×最小半长（`ccd.rs` 的设计意图）。预算
//!    （`ccd_max_steps`）不够覆盖本 tick 位移时，旧实现**默默**把段数截到上限 ⇒ 仍会穿透
//!    （审计实测：3000 m/s 开 CCD 照样穿）。现在改为**限速到预算覆盖得了的速度**——宁可
//!    这一 tick 走慢一点，也不静默穿透。
//! 3. **目标集含运动体**：旧实现 `retain(!is_dynamic || !awake)` 把**运动目标整条排除**
//!    ⇒ 移动墙必穿（审计实测）。现在把动态目标也纳入，并按**相对运动**采样与判定
//!    （目标位置随采样时刻推进、闭合判据用法向**相对**速度）。响应仍是 M1 口径的非弹性
//!    停（只改动体的法向速度）——**双向冲量留给 M2**，这里如实标出，不假装它是完备解。
//! 4. **旋转扫掠**（2026-10-13）：判据只看线速度时，**细长体高速自旋**（尖端 `ω×r` 可达
//!    数十 m/s，质心却接近静止）既不会被标记、采样也只插位置不插姿态 ⇒ 尖端直接扫过薄墙。
//!    现在：标记口径 = 最大点速（`ccd.rs`）、采样同时插**姿态**、闭合判据用**接触点速度**
//!    `v + ω×r`（`BodySet::velocity_at`）、命中后**位置与姿态一起**钳回上一安全采样点。
//!    `ω = 0` 时这四条全部退化为旧式（**逐位不变**）。
//!    实测（`tests/ccd_rotational_sweep.rs`，4×4 cm 小柱、`substeps = 1`）：峰值埋深 0.0078 → **0.0000**、
//!    杆心漂移 46.9 m → **0.0000**（关档时位置校正会把自旋杆弹飞）。
//!    ⚠️ 已知限制（本片**有意不做**，见 `OPEN-PROBLEMS.md` P18）：① 目标侧只带**平移**速度
//!    ——自旋目标的姿态/角速度不进采样；② **转动响应不做** ⇒ 自旋体在障碍前"空转顶住"
//!    （姿态被钳住、`ω` 不被吸收），离散管线随后按住它。代价实测：`m0_gates` 门槛档
//!    末态活跃 13 → 19、深穿透 tick 80 → 97，但最大深穿透 0.659 → **0.409**（换代账见 RECIPES 换代链）。
//! 5. **薄壳 TOI**（2026-10-13 同日第二片）：① 扫掠 AABB 从"**质心路径** ± 皮肤"改成
//!    **体在两端各自 AABB 之并** —— 旧写法下大体积体撞薄壁时，接触发生在质心还差半个体长的地方、
//!    不在质心路径上 ⇒ 配对进不来 ⇒ **径直穿过**（实测 60 m/s 的 1 m 盒穿过 5 mm 立板）；
//!    ② 命中后不再一律退到"上一安全采样点"（停位精度 = 一整段采样长 ⇒ **悬停**），改为**二分
//!    细化贴合时刻** ⇒ 停位收进皮肤带内、离散接触接手（判据 `tests/ccd_thin_shell.rs`）。
//!    ⚠️ CCD 提前接管会吃掉主窄相的"解算前接近速度" ⇒ 命中那一瞬要把 CCD 自测的闭合速率按
//!    provider 记进冲击账（`record_ccd_impact`），否则破坏管线永不触发（实测 R9 108 → 0 块）。

use vxl_phys_core::{Aabb, Quat, Shape, Vec3};
use vxl_phys_narrow::NarrowPhase;

use crate::ccd;
use crate::World;

/// **限速到预算覆盖得了的速度**（审计 F-15 ②）：段长 ≤ 0.5×最小半长 ⇒
/// `v_max = ccd_max_steps × 0.5·ext / dt`。默认档下这条基本不触发（100 m/s 的位移在
/// 64 段 × 0.5·ext 之内），只有把 `max_linear_velocity` 放开到几千 m/s 才会咬住。
///
/// **旋转扫掠（2026-10-13）**：预算覆盖的是**点速** `|v| + |ω|·r_bound`（尖端才是会穿的那个点）
/// ⇒ 超预算时按**同一个比例**把线速度与角速度一起压（本 tick 的轨迹形状不变、只是走慢一点）。
/// 只压线速度是**无益**的：自旋本身没被覆盖，尖端照样穿（实测过一版：自旋体线速度被清零、
/// 绕桩照切）。也**不能只压角速度**：那会平白吃掉平动那部分的覆盖。
/// `ω = 0` ⇒ `|v| + 0` 与旧式同条件、同比例（**逐位不变**）。
fn clamp_one(world: &mut World, i: usize, dt: f32) {
    let ext = ccd::min_half_extent(&world.bodies.shape[i]);
    if !ext.is_finite() || ext <= 0.0 || !dt.is_finite() || dt <= 0.0 {
        return;
    }
    let budget = world.config.ccd_max_steps.max(1) as f32;
    let omega = world.bodies.angvel(i);
    let r = ccd::bounding_radius(&world.bodies.shape[i]);
    let spin = if r.is_finite() {
        omega.length() * r
    } else {
        0.0
    };
    let point = world.bodies.linvel[i].length() + spin;
    let v_max = budget * (ext * 0.5) / dt;
    if point > v_max && point > 0.0 {
        let s = v_max / point;
        world.bodies.linvel[i] *= s;
        if spin > 0.0 {
            world.bodies.set_angvel_raw(i, omega * s);
        }
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
///
/// **旋转扫掠（2026-10-13）**：`omega`/`q1` 让采样把**姿态**也按时间插值
/// （`q(t) = q1 ⊖ ω·(dt − t)`）——旧实现只插位置、姿态用 tick 末那一份 ⇒ 自旋细杆的
/// 尖端扫过的位置**根本没被测到**（见 `ccd.rs` 的模块头注）。
struct Scan {
    p0: Vec3,
    p1: Vec3,
    v: Vec3,
    /// 体自身的角速度（姿态插值用）。
    omega: Vec3,
    /// tick 末姿态（采样后必须原样放回）。
    q1: Quat,
    dt: f32,
    sub: f32,
    steps: u32,
}

/// 扫掠 AABB（`p0..p1` 膨胀 `skin` **+ 转动摆幅** `|ω|·r_bound·dt`）。
/// `ω = 0` ⇒ 加宽量 0（既有场景逐位不变）。
fn swept_aabb(world: &World, i: usize, sc: &Scan, skin: f32, swing: f32) -> Aabb {
    let shape = &world.bodies.shape[i];
    let m = skin + swing;
    let a0 = vxl_phys_broad::shape_aabb(
        shape,
        sc.p0,
        q_at(sc, 0.0),
        m,
        &world.hf_bounds,
        &world.provider_bounds,
    );
    let a1 = vxl_phys_broad::shape_aabb(
        shape,
        sc.p1,
        q_at(sc, sc.dt),
        m,
        &world.hf_bounds,
        &world.provider_bounds,
    );
    vxl_phys_core::union_aabb(&a0, &a1)
}

/// 采样时刻 `t` 的姿态（纯函数；`ω = 0` ⇒ 原样 `q1`；`t ≥ dt` ⇒ `q1`，不经归一化）。
fn q_at(sc: &Scan, t: f32) -> Quat {
    if sc.omega == Vec3::ZERO || t >= sc.dt {
        sc.q1
    } else {
        sc.q1.integrate_angular(-sc.omega, sc.dt - t)
    }
}

/// 转动摆幅：本子步内**尖端**最多能扫过多远（`|ω|·r_bound·dt`；不可用 ⇒ 0）。
fn swing_of(shape: &Shape, omega: Vec3, dt: f32) -> f32 {
    let r = ccd::bounding_radius(shape);
    if !r.is_finite() || omega == Vec3::ZERO {
        return 0.0;
    }
    omega.length() * r * dt
}

/// 采样时刻 `t` 的姿态写回：`q(t) = q1 ⊖ ω·(dt − t)`（从 tick 末姿态往回退）。
/// `ω = 0` ⇒ **一个字节都不写**（既有场景逐位不变）；`t ≥ dt` ⇒ 原样写回 `q1`
/// （不经 `integrate_angular`/`normalize`，免得二次归一化扰动末位）。
fn put_rot(world: &mut World, i: usize, sc: &Scan, t: f32) {
    if sc.omega == Vec3::ZERO {
        return;
    }
    let q = q_at(sc, t);
    world.bodies.set_rot(i, q);
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

/// 一次命中：采样时刻、指向动体的法线、对方体号、接触点、**法向闭合速率**（`> 0` = 压向表面）。
#[derive(Clone, Copy)]
struct Hit {
    t: f32,
    n: Vec3,
    other: usize,
    point: Vec3,
    close: f32,
}

/// TOI 细化的二分迭代数（`sub / 2ⁿ` 的时刻分辨率；每趟一次**单对**窄相）。
const REFINE_ITERS: u32 = 8;

fn scan(
    world: &mut World,
    i: usize,
    sc: &Scan,
    pairs: &[(u32, u32)],
    targets: &[Target],
) -> Option<Hit> {
    for s in 1..=sc.steps {
        let t = sc.sub * s as f32;
        place_body(world, i, sc, t);
        place_targets(world, sc, targets, t);
        if let Some(h) = eval_hit(world, i, pairs, t) {
            return Some(h);
        }
    }
    None
}

/// 把动体放到时刻 `t` 的姿态。**姿态也按时间插值**（旋转扫掠）：旧实现只插位置 ⇒
/// 自旋尖端扫过的位置没被测到。
fn place_body(world: &mut World, i: usize, sc: &Scan, t: f32) {
    world.bodies.position[i] = sc.p0 + sc.v * t;
    put_rot(world, i, sc, t);
}

/// **CCD 命中也要进破坏管线的冲击账**（`World::record_impacts` 的同款记录）。
///
/// 为什么必须补（2026-10-13 实测翻案）：`record_impacts` 读的是**主窄相**的流形 + **解算前**的
/// 速度；扫掠 AABB 修成"体 AABB 之并"之后，CCD 会在**体表面刚进皮肤带**时就把体钉住并零化法向
/// 速度 ⇒ 下一子步主窄相看到的接近速度只剩 ≈0 ⇒ **破坏永不触发**（实测 R9：12 m/s 弹体原本
/// 挖 108 块 → **0 块**）✗。所以命中那一刻把 **CCD 自己测到的法向闭合速率**记进同一张账
/// （`approach` 取接触点速度的口径、含 `ω×r`），破坏管线照旧消费。
/// 只对 **provider 目标**记账 —— 与 `record_impacts` 的口径一致（地形/高度场不进破坏管线）。
fn record_ccd_impact(world: &mut World, i: usize, h: &Hit) {
    let Shape::Provider(prov) = world.bodies.shape[h.other] else {
        return;
    };
    if !world.bodies.is_dynamic(i) {
        return;
    }
    crate::world_body::merge_impact(
        &mut world.impacts,
        crate::ImpactRecord {
            provider: prov,
            body: i as u32,
            point: h.point,
            approach: h.close,
            velocity: world.bodies.linvel[i],
        },
    );
}

/// 运动目标推进到时刻 `t`（静态/沉睡目标 `vel = 0` ⇒ 不动）。
fn place_targets(world: &mut World, sc: &Scan, targets: &[Target], t: f32) {
    for tg in targets {
        if tg.vel != Vec3::ZERO {
            world.bodies.position[tg.id] = tg.q1 - tg.vel * (sc.dt - t);
        }
    }
}

/// **TOI 细化**：命中样本 `h` 与其前一个样本之间二分"首个有闭合接触的时刻"。
///
/// 为什么需要它（2026-10-13 实测）：旧写法一律退到**上一安全采样点** ⇒ 停位精度只有一整段
/// 采样长（`0.5×最小半长`）——1 m 盒撞 5 mm 立板时**停在墙前 0.25 m 且法向速度已清零** =
/// 悬停；间距远大于皮肤带，离散管线接不上手（体既不落位也不入睡）。二分把停位收到皮肤带内
/// （`|v|·sub/2⁸`），下一子步的窄相就在带内、离散求解器接管 ⇒ 落位 + 入睡。
///
/// 谓词与命中判据**同一把尺**（`eval_hit`）⇒ 不会细到"谓词不成立"的那一侧；`lo` 恒取
/// `t_hit − sub`（已实测无命中）⇒ 细化**不可能**比旧写法更深入。每趟只跑**命中那一对**窄相。
fn refine_touch(world: &mut World, i: usize, sc: &Scan, targets: &[Target], h: &Hit) -> Hit {
    let pair = if h.other < i {
        (h.other as u32, i as u32)
    } else {
        (i as u32, h.other as u32)
    };
    let one = [pair];
    let (mut lo, mut hi) = ((h.t - sc.sub).max(0.0), h.t);
    for _ in 0..REFINE_ITERS {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        place_body(world, i, sc, mid);
        place_targets(world, sc, targets, mid);
        if eval_hit(world, i, &one, mid).is_some() {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    // 在**细化后的贴合时刻**重取一次命中：接触点/闭合速率都取贴合那一瞬的值 ⇒ 与子步数无关
    // （判据 `destruction_substep_invariance` 会量这一条：取命中样本的点会随子步数漂）。
    place_body(world, i, sc, hi);
    place_targets(world, sc, targets, hi);
    eval_hit(world, i, &one, hi).unwrap_or(*h)
}

/// 当前姿态下**首个"正在接近"**的接触（窄相一趟）：返回命中的法线/对方体号/接触点/闭合速率。
///
/// **只有"正在接近表面"的采样才算命中**：贴地滑行/静置的体每个采样都天生有接触，按"有接触即
/// 命中"会把它钳回起点、原地锁死（实测：弹体滑到墙前 0.1 m 停住）。判据用**接触点速度**
/// `v + ω×r`（旋转扫掠 2026-10-13）：纯自旋体的质心速度可以是 0，但接触点正压向表面；只看
/// 质心速度 ⇒ 自旋体永远"不闭合"、扫掠形同虚设。`ω = 0` ⇒ 退化成 `sc.v`（既有场景逐位不变）。
/// 目标侧只带**平移**速度（自旋目标的姿态/角速度不进采样，见模块头注）。
fn eval_hit(world: &mut World, i: usize, pairs: &[(u32, u32)], t: f32) -> Option<Hit> {
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
    let m = world.ccd_manifolds.first()?;
    let other = if m.a == i as u32 { m.b } else { m.a } as usize;
    // 法线 a→b；换算成「推离表面、指向动体」的方向。
    let n_into_body = if m.a == i as u32 { -m.normal } else { m.normal };
    let point = m
        .points
        .first()
        .map_or(world.bodies.position[i], |p| p.point);
    let v_pt = world
        .bodies
        .velocity_at(i, point - world.bodies.position[i]);
    let v_rel = v_pt - world.bodies.linvel[other];
    let vn = n_into_body.dot(v_rel);
    (vn < 0.0).then_some(Hit {
        t,
        n: n_into_body,
        other,
        point,
        close: -vn,
    })
}

/// 目标位置复原（扫掠借出去的位移必须还回来——它们不是这一 tick 的最终状态）。
fn restore_targets(world: &mut World, targets: &[Target]) {
    for tg in targets {
        world.bodies.position[tg.id] = tg.q1;
    }
}

/// 命中后的收口：钳到**贴合时刻（TOI）** + **按质量分摊的非弹性相对停**。
///
/// 为什么不能只改动体（旧写法）：2 kg 的球撞 1600 kg 的墙时，两体各跑一遍 sweep，各自把
/// "相对法向速度"清零 ⇒ 结果像"球把墙拖走"（实测两体都以球的速度前进）。沿法线做标准一维
/// 非弹性冲量后：轻球被重墙挡下、墙几乎不动；静态目标 `inv_mass = 0` ⇒ 退化成原写法
/// （与 M1 逐位一致）。双向**转动**耦合仍留给 M2，这里如实标出。
///
/// **TOI 细化（2026-10-13，薄壳片）**：旧写法一律退到**上一安全采样点**（`t_hit − sub`）⇒
/// 停位精度只有**一整段采样长**（0.5×最小半长；1 m 盒撞 5 mm 立板时停在墙前 0.25 m、法向
/// 速度已清零 ⇒ **悬停**，离散管线又因为间距远大于皮肤带而接不上手、既不落位也不入睡）。
/// 现在命中后在 `[t_hit − sub, t_hit]` 上**二分**"首个有闭合接触的时刻"（`refine_touch`）⇒
/// 停位收进**皮肤带内**（`|v|·sub/2⁸` 的余量），下一子步离散接触接手 ⇒ 落位 + 入睡。
fn resolve(world: &mut World, i: usize, sc: &Scan, targets: &[Target], hit: Option<Hit>) {
    let Some(h) = hit else {
        // 没命中：回到 tick 末（位置 + 姿态；`ω = 0` ⇒ 姿态一个字节都不写）。
        place_body(world, i, sc, sc.dt);
        return;
    };
    // 贴合时刻（TOI 细化后的命中：接触点/闭合速率都取贴合那一瞬）。
    let touch = refine_touch(world, i, sc, targets, &h);
    let (other, n_into_body) = (touch.other, touch.n);
    place_body(world, i, sc, touch.t);
    // 目标必须放回 tick 末（细化期间借用过它们的位移，它们不是这一 tick 的最终状态）。
    place_targets(world, sc, targets, sc.dt);
    // **冲击账**（必须在零化速度**之前**记）：见 `record_ccd_impact`。
    record_ccd_impact(world, i, &touch);
    // 平动响应：**M1 口径未变**（质心相对速度的非弹性停，按质量分摊）。
    let v_rel = sc.v - world.bodies.linvel[other];
    let vn = v_rel.dot(n_into_body);
    let (ia, ib) = (world.bodies.inv_mass[i], world.bodies.inv_mass[other]);
    let denom = ia + ib;
    if vn < 0.0 && denom > 0.0 {
        let jn = -vn / denom;
        world.bodies.linvel[i] += n_into_body * (jn * ia);
        world.bodies.linvel[other] -= n_into_body * (jn * ib);
    }
    // ⚠️ **转动响应有意不做**（本片只到"被检出 + 采样到 + 姿态钳回安全采样点"）：两种写法都试过，
    // 都被实测反例挡下 ——
    // ① 只按**角向**有效质量收口会过冲：2 m 细杆 ω=40 顶薄墙时 ω 被收到 **−7**（标准点冲量给
    //    ≈20），差一个 `(1/m + k)/k` 的因子，等于凭空反弹；
    // ② 用**完整**点冲量（线性 + 角向）则给细杆的**自旋轴**注入巨大扭转：细杆绕自身长轴的逆惯量
    //    ~1200 ⇒ 4 cm 小柱的擦碰把 |ω| 顶到 **504 rad/s**（对拍：不做响应 38.4、关档 40）。
    // ⇒ 结果如实标出：自旋体在障碍前**姿态被钳住、ω 不被吸收**（"空转顶住"），离散管线随后
    // 按住它（判据 `tests/ccd_rotational_sweep.rs` 把这三种读数都钉住）。转动响应留给 M2 的
    // 转动耦合（TGS-Soft 接触），不在这里硬塞一个会过头或会扭爆的近似。
}

/// 单个体 `i` 本 tick 的保守推进扫掠（见模块头注）。
pub(crate) fn sweep(world: &mut World, i: usize, dt: f32) {
    let v = world.bodies.linvel[i];
    let omega = world.bodies.angvel(i);
    let steps = ccd::step_count(&world.bodies, i, &world.config, dt);
    let sc = Scan {
        p1: world.bodies.position[i],
        p0: world.bodies.position[i] - v * dt,
        v,
        omega,
        q1: world.bodies.rot(i),
        dt,
        sub: dt / steps as f32,
        steps,
    };
    // 扫掠 AABB 还要含**转动摆幅**（否则自旋体的尖端扫过的空间连宽相都过不了）。
    let swing = swing_of(&world.bodies.shape[i], omega, dt);
    let swept = swept_aabb(world, i, &sc, world.config.contact_skin, swing);
    let (pairs, targets) = gather(world, i, swept);
    let hit = scan(world, i, &sc, &pairs, &targets);
    restore_targets(world, &targets);
    resolve(world, i, &sc, &targets, hit);
}
