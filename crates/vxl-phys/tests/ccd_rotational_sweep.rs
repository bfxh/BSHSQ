//! **旋转扫掠判据**（2026-10-13）：CCD 的动体侧判据原来只看**线速度** ⇒ 细长体高速自旋时
//! 尖端速度 `ω×r` 远超质心速度（2 m 细杆 ω=40 ⇒ 尖端 40 m/s、质心**静止**），
//! 体**根本不会被标记**；采样也只插位置不插姿态 ⇒ 刀锋扫过的角位根本没被测到。
//! （`tests/ccd_tunneling.rs` 的五条全是平移场景，覆盖不到这条。）
//!
//! **场景**：2 m 细杆（`half = (1, 0.02, 0.02)`）无重力、质心静止，绕 **y 轴**自旋 40 rad/s；
//! 半径 0.7 m 处立一根 4×4 cm 的**小柱**（`half = (0.02, 1.0, 0.02)`）。
//! 刀锋切向扫过小柱 ⇒ 小柱的角窗口只有 `±asin(0.04/0.7) = ±3.27°`，而**每 tick 转过 38.2°**
//! ⇒ 离散采样几乎必然**跨过**它（这就是"转动隧道"的入口）。
//! `substeps = 1`（**有意**）：一个 tick 就是一个积分步 ⇒ 亚步内埋深在 tick 末端**可见**；
//! 默认的 `substeps = 2` 会让第二个子步把埋深修掉大半，把本判据要量的那个量藏起来。
//!
//! **量测**：沿杆轴 41 点 × 截面 4 角 = 164 个**杆材料采样点**，取"落在小柱 AABB 内的点到最近
//! 面的距离"的最大值 = 峰值埋深（杆心漂移不影响它）；另记 `min|φ|`（杆线朝向角与小柱角位的
//! 最小夹角，**只在杆心未漂移时有意义**）。
//!
//! | 用例 | 关档实测 | 开档实测 | 钉住的事 |
//! |---|---|---|---|
//! | ① 判据 | — | `max_point_speed` ≥ 10 ⇒ 标记 | 质心静止的自旋体必须被 CCD 看见 |
//! | ② 埋深 | **0.0078 m** | **0.0000 m** | 开档时刀锋**一个采样点都不进**小柱材料 |
//! | ③ 角位 | 心飞 **46.9 m**、`min\|φ\|=0.08°` | 心 **0.0000 m**、`min\|φ\|=5.08°` | 开档时刀锋停在柱面（3.27°）之外 |
//!
//! ⚠️ **本片只到"被检出 + 采样到 + 姿态钳回安全采样点"，转动响应有意不做**：见
//! `world_ccd/sweep.rs` 的 `resolve`（残余点冲量两种写法都被实测反例挡下：只算角向有效质量
//! 会过冲成 −7 rad/s；完整点冲量把细杆的自旋轴扭到 504 rad/s）。后果如实钉在这里：
//! **开档时 ω 不被吸收** —— 杆在柱面外"空转顶住"（末态 `|ω| = 38.4`，关档 13.7 是被离散
//! 管线撞散后的读数）。转动耦合留给 M2 的 TGS-Soft 接触。

use vxl_phys::{ccd, PhysConfig, Quat, Shape, Vec3, World};

/// 小柱中心（半径 0.7 m 处，刀锋切向扫过它）。
const POST_C: Vec3 = Vec3::new(0.7, 0.0, 0.0);
/// 小柱半尺寸（4×4 cm 截面）。
const POST_HALF: Vec3 = Vec3::new(0.02, 1.0, 0.02);
/// 柱面角位（杆线朝角下，刀锋正好贴到柱面）：`asin((0.02 + 0.02) / 0.7)`。
const POST_GRAZE_DEG: f32 = 3.274;
/// 杆自旋角速度（绕 y）。
const OMEGA: f32 = 40.0;
/// 杆半长 / 半粗。
const ROD_HALF: Vec3 = Vec3::new(1.0, 0.02, 0.02);

/// 场景：小柱 + 细杆（初始沿 z 轴 ⇒ 离柱角位 90°，开局不重叠）。无重力、无初速线速度。
fn scene(ccd_on: bool) -> (World, usize) {
    let cfg = PhysConfig {
        ccd_speed_threshold: if ccd_on { 10.0 } else { f32::INFINITY },
        gravity: Vec3::ZERO,
        // 亚步内埋深要能在 tick 末端看见（见文件头注）。
        substeps: 1,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(Shape::Box { half: POST_HALF }, POST_C, Quat::IDENTITY);
    let rod = w.add_dynamic(
        Shape::Box { half: ROD_HALF },
        Vec3::ZERO,
        Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2),
        1000.0,
    ) as usize;
    w.bodies.set_angvel(rod, Vec3::new(0.0, OMEGA, 0.0));
    (w, rod)
}

/// 杆材料采样点里**最深的埋入小柱**的深度（点全在外面 ⇒ 0）。
fn burial(w: &World, rod: usize) -> f32 {
    let (p, q) = (w.bodies.position[rod], w.bodies.rot(rod));
    let ax = q.rotate_vec3(Vec3::X);
    let mut deepest = 0.0f32;
    for k in 0..=40 {
        let s = -1.0 + 2.0 * (k as f32) / 40.0;
        for t in [-ROD_HALF.y, ROD_HALF.y] {
            for u in [-ROD_HALF.z, ROD_HALF.z] {
                let d = (p + ax * s + q.rotate_vec3(Vec3::new(0.0, t, u))) - POST_C;
                if d.x.abs() <= POST_HALF.x && d.y.abs() <= POST_HALF.y && d.z.abs() <= POST_HALF.z
                {
                    deepest = deepest.max(
                        (POST_HALF.x - d.x.abs())
                            .min(POST_HALF.y - d.y.abs())
                            .min(POST_HALF.z - d.z.abs()),
                    );
                }
            }
        }
    }
    deepest
}

/// 杆的**线**朝向角（mod 180，取 [-90, 90)）：`φ = 0` 就是刀锋扫到小柱角位。
fn phi_deg(w: &World, rod: usize) -> f32 {
    let ax = w.bodies.rot(rod).rotate_vec3(Vec3::X);
    let mut d = ax.z.atan2(ax.x).to_degrees();
    while d >= 90.0 {
        d -= 180.0;
    }
    while d < -90.0 {
        d += 180.0;
    }
    d
}

/// 一次跑完的读数：`(峰值埋深, min|φ|, 杆心最大漂移)`。
fn run(ccd_on: bool, ticks: u32) -> (f32, f32, f32) {
    let (mut w, rod) = scene(ccd_on);
    let (mut deep, mut min_phi, mut drift) = (
        burial(&w, rod),
        phi_deg(&w, rod).abs(),
        w.bodies.position[rod].length(),
    );
    for _ in 0..ticks {
        w.step();
        deep = deep.max(burial(&w, rod));
        min_phi = min_phi.min(phi_deg(&w, rod).abs());
        drift = drift.max(w.bodies.position[rod].length());
    }
    (deep, min_phi, drift)
}

/// **③ 判据**：质心静止的自旋细杆必须被 CCD 标记（旧口径只看 `|v|` ⇒ 恒不标记）。
#[test]
fn spinning_rod_is_flagged_by_point_speed() {
    let (w, rod) = scene(true);
    assert_eq!(
        w.bodies.linvel[rod],
        Vec3::ZERO,
        "本判据要的就是「质心静止」——线速度不为 0 就测不到旧口径的盲区"
    );
    let v_pt = ccd::max_point_speed(&w.bodies, rod);
    assert!(
        v_pt >= 10.0,
        "最大点速要进 CCD 阈值（10 m/s）；实际 {v_pt}（ω×r ≈ 40）"
    );
    assert!(
        ccd::needs_ccd(&w.bodies, rod, &w.config, w.config.dt),
        "自旋细杆必须被标记要扫掠——否则尖端穿墙无从拦起"
    );
}

/// **① 开档**：旋转扫掠把自旋的尖端钳在墙前（角点最大 x 不许越过墙的远面）。
#[test]
fn rotational_sweep_keeps_the_blade_out_of_the_post() {
    let (deep, min_phi, drift) = run(true, 240);
    assert_eq!(
        deep, 0.0,
        "开档时刀锋**一个采样点都不该进**小柱材料（实测埋深 {deep}）"
    );
    assert!(
        min_phi > POST_GRAZE_DEG,
        "刀锋应被停在柱面（{POST_GRAZE_DEG:.2}°）之外：min|φ| = {min_phi:.3}°"
    );
    assert!(
        drift < 1e-3,
        "本场景是纯自旋（质心静止）⇒ 杆心不许漂（实测 {drift}）"
    );
}

/// **② 关档对照**：同场景、同相位。关档 = 旧行为（旧判据根本不标记自旋体 ⇒ 逐位等同）。
/// 离散采样跨过 6.5° 的角窗口 ⇒ 刀锋**咬进**小柱材料，并被位置校正**弹飞**（杆心漂 46.9 m）。
#[test]
fn ccd_off_blade_bites_into_the_post() {
    let (deep, _min_phi, drift) = run(false, 240);
    assert!(
        deep > 0.005,
        "关档时刀锋应咬进小柱（实测埋深 {deep}，小柱半宽 0.02）——若红了先查采样相位是否被改动"
    );
    assert!(
        drift > 1.0,
        "关档时位置校正会把杆弹飞（实测杆心漂 {drift} m）；这条同时说明「咬入」不是无害的小重叠"
    );
}
