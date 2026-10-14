//! **自由旋转守恒判据**（`SURVEY-RIGID-HARD-CASES §1.5` 的 ①②④ 条落地；2026-10-14）。
//!
//! 缺口（`§1.2` 的"三处有意的近似"之首）：速度积分只做 `ω += I⁻¹_world·τ·dt`，**没有陀螺项**
//! `ω × (Iω)` ⇒ 无接触自由旋转的**非球体**里 `ω` 在世界系恒定、体绕**固定轴**转，而
//! `I_world = R·I_local·Rᵀ` 随姿态变 ⇒ **`L = I_world·ω` 的矢量不守恒**。
//!
//! 修法：`crates/vxl-phys-integrate/src/gyro.rs`（陀螺项 + **隐式中点** + 三条早退：各向同性 /
//! 退化惯量 / **自由进动率快过 `dt` 能分辨的尺度**）。读数（`examples/free_rotation_probe.rs`，
//! 1200 步 / 20 s、无重力无接触）：
//!
//! | 用例 | 修前 `max |ΔL|/|L|` | 修后 | 解析对拍 |
//! |---|---|---|---|
//! | 不对称盒 2×1×0.5（非主轴自旋 ω = 6） | 0.622 | **0.0232**（KE 漂移 5e-6） | —（极迹摆动：`ω₃` 摆 ±4.46） |
//! | 对称陀螺（方盒 1×1×0.5，ω = (1,0,6)） | 0.123 | **0.0031** | 进动率 **3.600** vs `Ω = ω₃(I₃−I₁)/I₁` = **3.6** ✓ |
//! | 细杆（1×0.02×0.02）自旋**沿主轴** | 0（本来就守恒） | 0 | 陀螺项恒 0（`ω ∥ Iω`） |
//! | 细杆自旋**偏离主轴**（`ω·dt·(I_⊥/I_∥−1) ≈ 4·10²`） | 0.199 | 0.199（**已知缺口**：解不动档） | — |
//!
//! ⚠️ **动能读数抓不到这个缺口**（两档 KE 漂移都 ~1e-6：`ω` 恒定 ⇒ KE 也恒定）⇒ 判据必须落在
//! **L 矢量**（或对称陀螺的进动率）上 —— 首版探针只量了 `|L|` 的模长，读数全 0（踩过）。
//!
//! **换代账（ADR-0004）**：`determinism` `0x655cdbf4… → 0x8c5c8fc9…`（场景里有可分辨的不对称
//! 旋转体）；`m0_gates` 门槛/压力两档与 `m3_collapse` **逐位未变**（那些场景的体是立方体/各向同性
//! ⇒ 走"各向同性早退"），健康读数也一字未动。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::Mat3;

/// `(L 世界系, ω 局部系)`：本仓局部惯量是**对角**（`Vec3` 存 `I⁻¹`）⇒ 正向惯量 = 逐分量倒数。
fn momentum(w: &World, i: usize) -> (Vec3, Vec3) {
    let inv = w.bodies.local_inv_inertia[i];
    let r = Mat3::from_quat(w.bodies.rot(i));
    let w_l = r.transpose_mul_vec3(w.bodies.angvel(i));
    let l = Vec3::new(w_l.x / inv.x, w_l.y / inv.y, w_l.z / inv.z);
    (r.mul_vec3(l), w_l)
}

/// 自由旋转一档的读数：`(L 矢量最大相对漂移, 动能最大相对漂移, 横向进动率 rad/s)`。
fn free_spin(shape: Shape, w0: Vec3, n: usize) -> (f32, f32, f32) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let i = w.add_dynamic(shape, Vec3::ZERO, Quat::IDENTITY, 1000.0) as usize;
    w.bodies.linvel.set_ang(i, w0);
    let (l0, _) = momentum(&w, i);
    let ke0 = 0.5 * w.bodies.angvel(i).dot(l0);
    let (mut dl, mut dke, mut phase) = (0.0f32, 0.0f32, 0.0f32);
    let (mut px, mut py) = (1.0f32, 0.0f32);
    for _ in 0..n {
        w.step();
        let (l, w_l) = momentum(&w, i);
        dl = dl.max((l - l0).length() / l0.length());
        let ke = 0.5 * w.bodies.angvel(i).dot(l);
        dke = dke.max((ke - ke0).abs() / ke0);
        let (nx, ny) = (w_l.x, w_l.y);
        if nx * nx + ny * ny > 1e-12 {
            phase += (px * ny - py * nx).atan2(px * nx + py * ny);
            (px, py) = (nx, ny);
        }
    }
    (dl, dke, phase / (n as f32 / 60.0))
}

/// **① 自由旋转守恒（不对称体）**：`L` 矢量与动能都必须在 1200 步内守住。
/// 修前 `max |ΔL|/|L|` = **0.622** ⇒ 这条判据当时会红（实修后 0.0232）。
#[test]
fn asymmetric_free_spin_conserves_angular_momentum() {
    let w0 = Vec3::new(1.0, 2.0, 0.3).normalize() * 6.0;
    let (dl, dke, _) = free_spin(
        Shape::Box {
            half: Vec3::new(1.0, 0.5, 0.25),
        },
        w0,
        1200,
    );
    println!("① 不对称盒自由旋转：max |ΔL|/|L| = {dl:.5}、KE = {dke:.7}");
    assert!(dl < 0.05, "L 矢量必须守恒（修前 0.622）：实测 {dl}");
    assert!(
        dke < 0.01,
        "动能不许漂（隐式中点应当无能量注入）：实测 {dke}"
    );
}

/// **② 对称陀螺解析对拍**：`I₁ = I₂ ≠ I₃` 的欧拉陀螺有闭式进动率 `Ω = ω₃·(I₃ − I₁)/I₁`。
/// 方盒 `half = (0.5,0.5,0.25)`：`I₃/I₁ = 1.6` ⇒ `Ω = 6·0.6 = 3.6 rad/s`；修前实测 **0.000**
/// （`ω` 在世界系恒定 ⇒ 体坐标系里一动不动），修后 **3.600**。
#[test]
fn symmetric_top_precession_rate_matches_analytic() {
    let (dl, dke, rate) = free_spin(
        Shape::Box {
            half: Vec3::new(0.5, 0.5, 0.25),
        },
        Vec3::new(1.0, 0.0, 6.0),
        1200,
    );
    println!("② 对称陀螺：进动率 {rate:+.4} rad/s（解析 +3.6）、max |ΔL|/|L| = {dl:.5}");
    assert!(
        (rate - 3.6).abs() < 0.1,
        "进动率必须对上解析值 3.6 rad/s（修前是 0.000）：实测 {rate}"
    );
    assert!(dl < 0.01, "L 矢量必须守恒：实测 {dl}");
    assert!(dke < 0.01, "动能不许漂：实测 {dke}");
}

/// **④ 金丝雀：各向同性体逐位不变** —— 球/立方体的 `ω × (Iω) ≡ 0` ⇒ 走早退，与旧式**逐位相同**。
/// 这条同时解释了换代面为什么很小（`m0_gates` 两档与 `m3_collapse` 一字未动：它们的体是立方体）。
#[test]
fn isotropic_body_keeps_the_old_bitwise_path() {
    let dt = 1.0 / 60.0;
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5), // 立方体 ⇒ 各向同性
        },
        Vec3::ZERO,
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let w0 = Vec3::new(1.0, -2.0, 0.5);
    let tau = Vec3::new(3.0, 1.0, -2.0);
    w.bodies.linvel.set_ang(i, w0);
    w.bodies.torque[i] = tau;
    // 旧式的解析一步：ω + I⁻¹·τ·dt（此处 **必须逐位相等** —— 早退路径就是这个表达式）
    let expected = w0 + w.bodies.apply_world_inv_inertia(i, tau) * dt;
    let mut bodies = w.bodies.clone();
    vxl_phys_integrate::Integrator::integrate_velocities(&mut bodies, Vec3::ZERO, dt, 1e6, 1e6);
    assert_eq!(
        bodies.angvel(i),
        expected,
        "各向同性体必须走旧式（逐位相等）"
    );
    let _ = w;
}

/// **已知缺口（金丝雀）**：**极各向异性 + 自旋偏离主轴**时自由进动率远超 `1/dt` 能分辨的尺度
/// （细杆 `ω·dt·(I_⊥/I_∥ − 1) ≈ 4·10²`）⇒ 实现按"可分辨性"**退回旧式**，`L` 矢量照旧不守恒
/// （实测 0.199）。⚠️ **这不是"格式没选对"**：显式格式在该处发散（实测动能 +323%）、隐式中点的
/// 固定点迭代也不收缩 ⇒ 要真解它得**亚步积分旋转**或换辛格式（登记为下一片）。
/// 修好那天这条会红 ⇒ 翻面成"同一个 `dl < 0.05`"。
#[test]
fn tilted_thin_rod_is_the_known_stiffness_gap() {
    let (dl, dke, _) = free_spin(
        Shape::Box {
            half: Vec3::new(0.5, 0.01, 0.01),
        },
        Vec3::new(4.0, 40.0, 0.0),
        1200,
    );
    println!("④ 细杆（偏离主轴）：max |ΔL|/|L| = {dl:.5}、KE = {dke:.7}");
    assert!(
        dl > 0.15,
        "已知缺口被修好了（好事）：细杆的 L 漂移降到 {dl} ⇒ 把这条翻面成 `dl < 0.05`"
    );
    assert!(dke < 1e-4, "退回旧式仍是刚性自旋 ⇒ 动能守恒：实测 {dke}");
}
