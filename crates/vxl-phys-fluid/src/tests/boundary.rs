//! 边界粒子：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::{still_pose, FloorY, Tank};
use crate::{FluidConfig, FluidSystem};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Shape, Vec3};

/// #5 边界：粒子落到地板（替身半空间）上不穿透、不弹飞。
#[test]
fn particles_rest_on_boundary_without_penetration() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.1, 1.2, -0.1), [5, 5, 5], 0.05);
    f.set_boundaries(&[0]);
    for _ in 0..300 {
        f.step(1.0 / 60.0, &FloorY);
    }
    for (i, p) in f.positions().iter().enumerate() {
        assert!(p.y > -0.01, "粒子 {i} 穿透地板：y = {}", p.y);
        assert!(p.y < 0.6, "粒子 {i} 弹飞：y = {}", p.y);
    }
}

/// #11 **近壁密度补偿**（2b 的核心主张）：贴壁层密度靠边界粒子补到 ρ0 量级。
/// 对照 = 同水块**无地板**（自由面 ⇒ 贴壁层只剩 ~0.7ρ0）。两场景都只跑 1 步
/// （密度只依赖位置，位置在 1 步内几乎不动 ⇒ 对照干净、测试快）。
#[test]
fn boundary_particles_restore_near_floor_density() {
    let cfg = FluidConfig::default();
    let mk = || FluidSystem::new(cfg.clone(), Vec3::new(-0.15, 0.0, -0.15), [7, 7, 7], 0.05);
    let mean_lo = |f: &FluidSystem| {
        let (mut s, mut n) = (0.0f32, 0usize);
        for (p, d) in f.positions().iter().zip(f.densities().iter()) {
            if p.y < 0.06 {
                s += d;
                n += 1;
            }
        }
        assert!(n > 0, "贴壁层应有粒子");
        s / n as f32
    };
    // 对照：无地板。
    let mut free = mk();
    free.step(1.0 / 60.0, &NoProviders);
    let rho_free = mean_lo(&free);
    // 实验：地板 = 一层 Box 体的边界粒子（顶面 y = 0，托住水块底面）。
    let mut on = mk();
    let floor = (
        0u32,
        Shape::Box {
            half: Vec3::new(0.3, 0.05, 0.3),
        },
        still_pose(Vec3::new(0.0, -0.05, 0.0)),
    );
    let n = on.set_boundary_particles(std::slice::from_ref(&floor));
    assert!(n > 0, "地板应生成边界粒子");
    on.step(1.0 / 60.0, &NoProviders);
    let rho_on = mean_lo(&on);
    let rho0 = on.config().rest_density;
    assert!(
        rho_free < 0.85 * rho0,
        "自由面贴底密度应偏低：{rho_free:.0}"
    );
    // 门槛按**实测**给（2026-09-22，Akinci 自洽体积标定后：+123 kg/m³，0.72→0.84ρ0）。
    // 旧口径（固定 `V_b = s³`）是 +80：自洽标定把补偿**做强**了，且不再对稀疏小体过量注入。
    assert!(
        rho_on > rho_free + 0.10 * rho0,
        "边界粒子应显著补回核质量：{rho_on:.0} vs 自由面 {rho_free:.0}"
    );
    assert!(
        rho_on < 1.3 * rho0,
        "补偿不得过量（会造虚假压力）：{rho_on:.0}"
    );
}

/// #12 **压力承住流体**（不穿透）：只靠边界粒子地板，平台范围内的粒子不得漏下去。
/// ⚠️ 平台**边缘外**的粒子会（正确地）摊出去——那不是穿透。判据只看"站得住"：
/// 芯内（|xz| ≤ 0.25）最低点不得越过第二层边界粒子（−0.075）以下。
#[test]
fn boundary_particles_hold_fluid_column() {
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.3, 0.0, -0.3),
        [12, 12, 10],
        0.05,
    );
    let floor = (
        0u32,
        Shape::Box {
            half: Vec3::new(0.5, 0.05, 0.5),
        },
        still_pose(Vec3::new(0.0, -0.05, 0.0)),
    );
    let n = f.set_boundary_particles(std::slice::from_ref(&floor));
    assert!(n > 0);
    for _ in 0..60 {
        let _ = f.set_boundary_particles(std::slice::from_ref(&floor));
        f.step(1.0 / 60.0, &NoProviders);
    }
    let mut core_min = f32::INFINITY;
    for p in f.positions() {
        assert!(
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
            "NaN/Inf"
        );
        if p.x.abs() <= 0.25 && p.z.abs() <= 0.25 {
            core_min = core_min.min(p.y);
        }
    }
    assert!(
        core_min > -0.06,
        "芯内粒子漏过边界粒子地板：min_y = {core_min:.4}"
    );
}

/// #13 **反作用 = 浮力**（端到端量纲闸）：槽里全潜盒应受 ≈ `ρ·V·g` 的上浮力。
/// 槽用既有 `Tank` 提供者（**真实场景里围水是提供者通道的活**，边界粒子只管与体的
/// 动量交换——这条把两者分工钉死）。
///
/// ⚠️ **口径 = 窗口均值**（2026-09-22 用 `tests/boundary_accuracy_probe.rs` 两轴实测后定）：
/// - **单帧端点值不可用**：反作用是"两个大数之差"，同一场景端点可读 −6.11×ρVg 而窗口均值
///   是另一个号；另一格端点 1.00 却在窗口内以 ±4.3×ρVg 摆。端点会让门随机红/绿。
/// - 本档（盒半长 0.06 = 1.2h、`Tank`、7³ 水）窗口均值实测 **1.3–1.7×ρVg**；
///   **侧向均值 ≈ 0.01–0.1×ρVg**（先记录在案的 0.6–2.1× 是**端点瞬态**，不是稳态偏差——
///   这条是对我自己先前读数的更正）。
/// - 机理：贴壁核质量（离散补偿）经 Tait q⁷ 放大 ⇒ 近壁压强量级远高于静水；
///   与**既有提供者方案的已接受偏差同族**（`PLAN-0.3.md` §4.3 底压 ≈1.65× 静水）。
/// - **误差不是"体尺寸/h"的干净函数**：探针实测半长 0.04/0.06/0.09/0.12/0.15（h=0.1）
///   给出 3.30/1.38/1.63/**−6.13**/1.21 ⇒ 存在**离散相位共振**（体面栅格与流体晶格不可公约）。
///   ⇒ 2b 是**定性档**（水被推动、轻物浮起、方向对），**不是定量浮力模型**；
///   升级路径见 `OPEN-PROBLEMS.md` P7。断言带按此**诚实地**给。
#[test]
fn submerged_box_gets_buoyant_reaction() {
    let cfg = FluidConfig {
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.15, 0.05, -0.15), [7, 7, 7], 0.05);
    f.set_boundaries(&[0]);
    let half = 0.06f32;
    let body = (
        7u32,
        Shape::Box {
            half: Vec3::splat(half),
        },
        still_pose(Vec3::new(0.0, 0.12, 0.0)),
    );
    // **窗口均值**（2026-09-22 改；探针 `tests/boundary_accuracy_probe.rs` 实测得出）：
    // 反作用是"两个大数之差"，**单帧端点值可整号翻转**（实测同一场景 h=0.1 端点 −6.11×ρVg、
    // 而窗口均值 −6.13 是**系统性**的；另一格端点 1.00 而窗口内波动 ±4.3）⇒ 端点读数会让门
    // 随机红/绿。这里按仓库既有纪律（`vxl-phys-measurement-protocol` §5 窗口均值）取
    // **静置 180 + 窗口 60 tick 的均值**。
    for _ in 0..180 {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Tank);
    }
    let win = 60usize;
    let (mut sy, mut sx, mut sz) = (0.0f32, 0.0f32, 0.0f32);
    let (mut tsum, mut tmax) = (Vec3::ZERO, 0.0f32);
    for _ in 0..win {
        let _ = f.set_boundary_particles(std::slice::from_ref(&body));
        f.step(1.0 / 60.0, &Tank);
        if let Some(r) = f.boundary_reactions().iter().find(|r| r.0 == 7) {
            sy += r.1.y;
            sx += r.1.x;
            sz += r.1.z;
            tsum += r.2;
            tmax = tmax.max(r.2.length());
        }
    }
    let inv = 1.0 / win as f32;
    let force = Vec3::new(sx * inv, sy * inv, sz * inv);
    let tau = tsum * inv;
    let expect = f.config().rest_density * (2.0 * half).powi(3) * 9.81;
    println!(
        "2b 浮力（窗口 {win} tick 均值）{:+.2} N / ρVg = {expect:.2} N（比值 {:.2}）；\
         侧向 ({:+.2}, {:+.2})；|τ| 均值 {:.3} / 峰 {:.3}",
        force.y,
        force.y / expect,
        force.x,
        force.z,
        tau.length(),
        tmax
    );
    assert!(
        force.y > 0.5 * expect && force.y < 2.0 * expect,
        "浮力均值应 ≈ ρVg = {expect:.2} N（本档实测 1.3–1.7×），实测 {:+.2}（f = {force:?}）",
        force.y
    );
    // 侧向：**窗口均值下本来就近消**（探针实测 ≈ 0.01–0.1×ρVg；先前的 0.6–2.1× 是端点瞬态）
    // ⇒ 这条恢复成"近消"判据，只留宽带回旋余量。
    assert!(
        force.x.abs() < 0.5 * expect && force.z.abs() < 0.5 * expect,
        "对称场景侧向**均值**应近消（本档实测 ≤0.1×ρVg）：{force:?}"
    );
    assert!(
        tau.length() < 0.5 * expect * half,
        "对称场景力矩均值应近消：{tau:?}"
    );
}
