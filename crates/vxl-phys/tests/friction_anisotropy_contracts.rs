//! **各向异性摩擦的方向应用 + 传送带**（K5，`SPEC §4.4` 的另一半）。
//!
//! 口径：`FrictionModel::Anisotropic { mu1, mu2, angle, belt }` —— 接触面内把 μ1 轴**投影**
//! 成切向基 `t1`，摩擦锥从"圆"变成**椭圆**（沿 `t1` 的 μ1、沿 `t2` 的 μ2）；`belt` 是该方向的
//! **切向目标速度**（传送带/跑步机面）。组合律与各向同性同源：逐轴 `√(μ_axis·μ_other)`。
//!
//! | 判据 | 钉住的行为 |
//! |---|---|
//! | A1 方向比 | 推力相同时，沿 μ1 / 沿 μ2 的滑移距离比 ≈ `(1−μ1c)/(1−μ2c)`（解析） |
//! | A2 传送带 | 静止地表 + `belt` ⇒ 载荷被拖到带速；`belt = 0` ⇒ 不动（金丝雀） |
//! | A3 各向同性等效 | `μ1 == μ2 且 belt == 0` 与 `Coulomb{μ}` **逐位相同**（零退化） |
//! | A4 体局部轴 | 地表转 90° ⇒ 主轴跟着转（"难滑"方向从 x 换到 z） |
//!
//! ⚠️ 场景用 `substeps = 1`：本文件每次 `step()` 施加一个**整 tick 的力**（力累加器每子步清零，
//! 多子步下"每 tick 施一次力"只有首子步受力、读数不再是解析式）。

use vxl_phys::{FrictionModel, Material, PhysConfig, Quat, Shape, Vec3, World};

/// 各向异性地表（静止大板，顶面 `y = 0`）+ 其上的 1 m 盒；返回 `(世界, 盒, 板)`。
/// `mu` 是盒自己的各向同性摩擦（组合律 `√(μ_axis·μ_box)` 的可控那一项）。
fn scene(aniso: FrictionModel, ground_rot: Quat) -> (World, usize, usize) {
    let cfg = PhysConfig {
        substeps: 1,
        velocity_iterations: 8,
        sleep_time: f32::INFINITY,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let ground = w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.5, 6.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        ground_rot,
    );
    let gmat = w.add_material(Material::new(aniso, 0.0));
    w.bodies.set_material(ground as usize, gmat);
    let bmat = w.add_material(Material::new(FrictionModel::Coulomb { mu: 1.0 }, 0.0));
    let cube = w.add_dynamic(
        Shape::Box {
            // 扁盒：`F = m·g` 的水平推力下**不翻倒**（翻倒阈值 ∝ 宽/高），
            // 这样 A1 的读数才是纯滑动的解析式（立方形在 `F = m·g` 恰好压线）。
            half: Vec3::new(0.5, 0.1, 0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        500.0,
    );
    w.bodies.set_material(cube as usize, bmat);
    (w, cube as usize, ground as usize)
}

/// **量一个方向的摩擦极限** `μ_eff`：给初速让盒纯滑动，摩擦按 `μg` 减速 ⇒ `μ = Δv/(g·t)`。
///
/// ⚠️ 夹具纪律：每 tick **清角速度并复位姿态**（"全钉住"同款手法）—— 摩擦冲量在体底面、
/// 惯性在质心，不锁的话冲量会被分进角动量（实测扁盒 μ=0.95 时读数掉到 0.49 ≈ 一半），
/// 那是夹具的俯仰，不是摩擦锥。本判据只测**线性**摩擦极限。
fn mu_eff(aniso: FrictionModel, ground_rot: Quat, dir: Vec3) -> f32 {
    let (mut w, cube, _g) = scene(aniso, ground_rot);
    for _ in 0..30 {
        w.step(); // 坐稳
    }
    let v0 = 4.0f32;
    w.bodies.linvel[cube] = dir * v0;
    let ticks = 12usize;
    for _ in 0..ticks {
        w.step();
        w.bodies.set_angvel_raw(cube, Vec3::ZERO);
        w.bodies.set_rot(cube, Quat::IDENTITY);
    }
    let v1 = w.bodies.linvel[cube].dot(dir);
    let t = ticks as f32 * w.config.dt;
    (v0 - v1) / (9.81 * t)
}

/// **A1 方向比**：直接量**两轴各自的摩擦极限**（比值就是判据本身）。
///
/// 手法：给一个初速 `v0` 让盒**纯滑动**（无外力 ⇒ 没有翻倒/驱动力的混淆），
/// 摩擦把它按 `μg` 减速 ⇒ `μ_eff = (v0 − v1)/(g·t)`。
/// 解析：组合律逐轴 `√(μ_axis·μ_box)`，盒的 `μ_box = 1.0` ⇒ 期望 `√0.9 = 0.9487` / `√0.1 = 0.3162`。
#[test]
fn a1_friction_limit_follows_principal_axes() {
    let aniso = FrictionModel::Anisotropic {
        mu1: 0.9,
        mu2: 0.1,
        angle: 0.0,
        belt: 0.0,
    };
    let mu_along_1 = mu_eff(aniso, Quat::IDENTITY, Vec3::X);
    let mu_along_2 = mu_eff(aniso, Quat::IDENTITY, Vec3::Z);
    let (want1, want2) = (0.9f32.sqrt(), 0.1f32.sqrt());
    println!(
        "[A1 方向比] 沿 μ1 轴 μ_eff = {mu_along_1:.4}（解析 {want1:.4}）｜沿 μ2 轴 μ_eff = {mu_along_2:.4}（解析 {want2:.4}）"
    );
    assert!(
        (mu_along_1 - want1).abs() < 0.08,
        "沿 μ1 轴的摩擦极限应 ≈ {want1}，实得 {mu_along_1}"
    );
    assert!(
        (mu_along_2 - want2).abs() < 0.08,
        "沿 μ2 轴的摩擦极限应 ≈ {want2}，实得 {mu_along_2}"
    );
}

/// **A2 传送带**：静止地表的带面以 0.6 m/s 沿局部 x 走（μ1 = 0.7、μ2 = 0.5）。
/// 正确行为：载荷被拖到 ≈ 0.6 m/s；垂直方向不动；**`belt = 0` 时一动不动**（金丝雀）。
#[test]
fn a2_static_belt_surface_drags_the_load() {
    let mk = |belt: f32| FrictionModel::Anisotropic {
        mu1: 0.7,
        mu2: 0.5,
        angle: 0.0,
        belt,
    };
    let run = |belt: f32| {
        let (mut w, cube, _g) = scene(mk(belt), Quat::IDENTITY);
        let p0 = w.bodies.position[cube];
        for _ in 0..60 {
            w.step();
        }
        (w.bodies.linvel[cube], w.bodies.position[cube] - p0)
    };
    let (v, d) = run(0.6);
    let (v_off, d_off) = run(0.0);
    println!(
        "[A2 传送带] 带速 0.6 ⇒ v = ({:.4}, {:.4})、位移 ({:.4}, {:.4})｜belt = 0 ⇒ v = ({:.4}, {:.4})",
        v.x, v.z, d.x, d.z, v_off.x, v_off.z
    );
    assert!(
        (v.x - 0.6).abs() < 0.05,
        "载荷应被拖到带速 0.6 m/s，实得 {}",
        v.x
    );
    assert!(d.z.abs() < 0.005, "垂直带向不该动（Δz = {}）", d.z);
    assert!(
        // 阈值放到求解器残差量级：belt = 0 时只剩静置接触的数值噪声（实测 v ≈ 1.2e-7 m/s）。
        v_off.x.abs() < 1e-5 && d_off.x.abs() < 1e-4,
        "belt = 0 ⇒ 零拖动（金丝雀，只剩残差）：v = {} / Δx = {}",
        v_off.x,
        d_off.x
    );
}

/// **A3 各向同性等效 = 逐位不变**：`μ1 == μ2 == 0.6 且 belt == 0` 必须与 `Coulomb{0.6}`
/// 走**同一条代码路径**（`friction_frame` 返回 `None`）⇒ 末态**逐位相同**。
#[test]
fn a3_isotropic_equivalent_is_bit_identical() {
    let iso = FrictionModel::Anisotropic {
        mu1: 0.6,
        mu2: 0.6,
        angle: 0.0,
        belt: 0.0,
    };
    let run = |fm: FrictionModel| {
        let (mut w, cube, _g) = scene(fm, Quat::IDENTITY);
        let m = 1.0 / w.bodies.inv_mass[cube];
        for _ in 0..30 {
            w.step();
        }
        for _ in 0..30 {
            w.bodies.force[cube] = Vec3::X * (m * 9.81);
            w.step();
        }
        (
            w.bodies.position[cube],
            w.bodies.linvel[cube],
            w.bodies.angvel(cube),
        )
    };
    let (pa, va, wa) = run(iso);
    let (pb, vb, wb) = run(FrictionModel::Coulomb { mu: 0.6 });
    let bits = |a: f32, b: f32| a.to_bits() == b.to_bits();
    println!("[A3 等效金丝雀] 各向异性(0.6,0.6) 与 Coulomb(0.6)：p {pa:?} vs {pb:?}");
    assert!(
        bits(pa.x, pb.x) && bits(pa.y, pb.y) && bits(pa.z, pb.z),
        "位置应逐位相同：{pa:?} vs {pb:?}"
    );
    assert!(
        bits(va.x, vb.x) && bits(va.y, vb.y) && bits(va.z, vb.z),
        "速度应逐位相同：{va:?} vs {vb:?}"
    );
    assert!(
        bits(wa.x, wb.x) && bits(wa.y, wb.y) && bits(wa.z, wb.z),
        "角速度应逐位相同：{wa:?} vs {wb:?}"
    );
}

/// **A4 主轴是体局部的**：地表绕 +y 转 90° ⇒ 局部 x 映射到世界 −z ⇒ "难滑"方向从 x 换到 z。
#[test]
fn a4_principal_axes_are_body_local() {
    let aniso = FrictionModel::Anisotropic {
        mu1: 0.9,
        mu2: 0.1,
        angle: 0.0,
        belt: 0.0,
    };
    let rot = Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_2);
    let mu_x = mu_eff(aniso, rot, Vec3::X);
    let mu_z = mu_eff(aniso, rot, Vec3::Z);
    println!(
        "[A4 体局部轴] 转 90° 后：沿世界 x 的 μ_eff = {mu_x:.4}（应为 μ2 ≈ 0.316）｜沿世界 z = {mu_z:.4}（应为 μ1 ≈ 0.949）"
    );
    assert!(
        (mu_x - 0.1f32.sqrt()).abs() < 0.08 && (mu_z - 0.9f32.sqrt()).abs() < 0.08,
        "转 90° 后主轴应换到世界 z：x 得 {mu_x}、z 得 {mu_z}"
    );
}
