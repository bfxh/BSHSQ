//! **CCD 的能量账**（2026-10-13，`SURVEY-RIGID-HARD-CASES §7.2⑤` 那条"CCD 不该吃掉超过解析界的
//! 动能"至今没判据）。CCD 的响应口径是 **M1 的非弹性相对停**（只清沿接触法向的相对速度）⇒
//! 解析界是**两侧**的：
//!
//! - **不许注入**：`KE_after ≤ KE_before`（法向冲量只能减少动能）；
//! - **不许过吃**：`KE_after ≥ KE_before − ½·m_eff·v_n²`（切线分量与另一体的动能不该被拿走）。
//!
//! 三条判据：
//! | 用例 | 钉住的事 |
//! |---|---|
//! | ① 正撞（1 m 盒 1000 kg、100 m/s 撞静态墙） | 撞后动能**正好落在解析界**（KE 5.0e6 → 0）；关档对照不注入 |
//! | ② 斜撞 45° | **切线动能必须原样留着**（KE 10.0e6 → **5.0e6** = 只剩切向，不许当正撞吃干净） |
//! | ③ 记录档：自旋细杆顶墙 | P18 的已知缺口（转动冲量缺席 ⇒ KE 853.7 → 932.5，**×1.092**，只判上界） |

use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

/// 动能（平动 + 转动）。
fn ke(w: &World, i: usize) -> f32 {
    let m = 1.0 / w.bodies.inv_mass[i];
    let v = w.bodies.linvel[i].length();
    let omega = w.bodies.angvel(i);
    let ixx = 1.0 / w.bodies.local_inv_inertia[i].x.max(1e-12);
    let iyy = 1.0 / w.bodies.local_inv_inertia[i].y.max(1e-12);
    let izz = 1.0 / w.bodies.local_inv_inertia[i].z.max(1e-12);
    let spin = omega.x * omega.x * ixx + omega.y * omega.y * iyy + omega.z * omega.z * izz;
    0.5 * m * v * v + 0.5 * spin
}

/// 一条正撞/斜撞：1 m 立方、密度 1000（m = 1000 kg）撞静态薄墙。返回 `(撞前 KE, 撞后 KE, v_n, m_eff)`。
fn impact(ccd_on: bool, speed: Vec3, ticks: u32) -> (f32, f32, f32, f32) {
    let cfg = PhysConfig {
        ccd_speed_threshold: if ccd_on { 10.0 } else { f32::INFINITY },
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    // 墙要够大：45° 斜撞时体沿 y 也在走，墙沿 y/z 半长 50 m ⇒ 一定撞得上（否则量的是"没撞"）。
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.25, 50.0, 50.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(-6.0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = speed;
    let ke0 = ke(&w, b);
    let n = Vec3::new(-1.0, 0.0, 0.0);
    let v_n = speed.dot(n);
    let m = 1.0 / w.bodies.inv_mass[b];
    for _ in 0..ticks {
        w.step();
    }
    // 取撞后动能的最大值（撞前一刻到停住之间；墙体静止 ⇒ 只有这个体的动能）。
    (ke0, ke(&w, b), v_n, m)
}

/// **① 正撞**：两侧界都成立（不许注入、不许比解析界吃得更多）。
#[test]
fn head_on_impact_respects_both_bounds() {
    let (ke0, ke1, v_n, m) = impact(true, Vec3::new(100.0, 0.0, 0.0), 240);
    let bound = 0.5 * m * v_n * v_n;
    println!("① 正撞：KE0 {ke0} → KE1 {ke1}（解析界 {bound}）");
    assert!(ke1 <= ke0 + 1e-3, "CCD 不许注入动能：KE {ke0} → {ke1}");
    assert!(
        ke1 >= ke0 - bound - 1e-3,
        "CCD 不许吃过解析界：掉了 {}，界是 {bound}",
        ke0 - ke1
    );
    assert!(
        ke1 <= ke0 - bound + 1e-3,
        "正撞应**正好**停在解析界（非弹性停 = 法向动能全清）：撞后 {ke1}，界 {bound}"
    );
    // 关档对照：离散管线（同样是非弹性停）也不该注入。
    let (ke0o, ke1o, _, _) = impact(false, Vec3::new(100.0, 0.0, 0.0), 240);
    assert!(
        ke1o <= ke0o + 1e-3,
        "关档（离散管线）也不许注入：{ke0o} → {ke1o}"
    );
}

/// **② 斜撞 45°**：切线分量必须活着（不许把斜撞当正撞吃干净）。
#[test]
fn oblique_impact_keeps_tangential_energy() {
    let (ke0, ke1, v_n, m) = impact(true, Vec3::new(100.0, -100.0, 0.0), 240);
    let bound = 0.5 * m * v_n * v_n;
    let tangential = ke0 - bound;
    println!("② 斜撞：KE0 {ke0} → KE1 {ke1}（法向 {bound}、切向 {tangential}）");
    assert!(ke1 <= ke0 + 1e-3, "不许注入：{ke0} → {ke1}");
    assert!(
        ke1 >= tangential - 1e-3,
        "切向动能必须留着：撞后 {ke1} 应 ≥ 切向 {tangential}"
    );
    assert!(
        ke1 <= tangential + 1e-3,
        "斜撞应**正好**只剩切向（法向清干净、切向不动）：撞后 {ke1}，切向 {tangential}"
    );
}

/// **③ 记录档：自旋细杆顶墙**（2 m 细杆 ω=40 rad/s、质心静止）——P18 的已知缺口：
/// 转动响应缺席 ⇒ 这里**只判"不许把动能顶到离谱"**（上界），真实读数打在日志里。
#[test]
fn spinning_rod_hits_wall_energy_upper_bound() {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_angular_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(0.05, 2.0, 2.0),
        },
        Vec3::new(0.9, 0.0, 0.0),
        Quat::IDENTITY,
    );
    let rod = w.add_dynamic(
        Shape::Box {
            half: Vec3::new(1.0, 0.02, 0.02),
        },
        Vec3::ZERO,
        Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2),
        1000.0,
    ) as usize;
    w.bodies.set_angvel(rod, Vec3::new(0.0, 40.0, 0.0));
    let ke0 = ke(&w, rod);
    let mut peak = ke0;
    for _ in 0..400 {
        w.step();
        peak = peak.max(ke(&w, rod));
    }
    let ratio = peak / ke0.max(1e-9);
    println!("③ 自旋细杆顶墙：KE0 {ke0:.3} → 峰值 {peak:.3}（×{ratio:.3}，P18 已知缺口）");
    assert!(
        ratio < 3.0,
        "自旋体撞墙不该把动能顶到 3 倍以上（实测 ×{ratio:.3}）——再高说明接触口径崩了"
    );
}
