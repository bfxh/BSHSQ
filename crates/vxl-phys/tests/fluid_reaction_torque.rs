//! **2b 反作用力矩的窗口统计 + 分辨率扫描**（仪表，只打印不断言）。
//!
//! 为什么要换测量对象（`OPEN-PROBLEMS` P7）：自由漂浮体在小盆里**永远在晃**（`float_quiet_probe`
//! 实测：静置 900/3000 tick 后 45°/球仍有 \|v\| 0.05–0.12）⇒ **"稳态自旋"根本读不出来**。
//! 而 **静态体 + 流体**那一格无投放、无动力学 ⇒ 读数是干净的（挖空后 \|τ\| ≈ 0.0026 N·m）。
//!
//! **本仪表要回答的那一个科学问题**：那个力矩是**离散噪声**还是**与网格无关的系统性偏置**？
//!   判据 = **细化 h 之后 \|τ\| 是否趋零**：
//!   · 趋零 ⇒ 离散噪声（量级随 h 下降）⇒ 不必当引擎缺陷追；
//!   · 不降 ⇒ 存在系统性力矩 ⇒ 值得单独立项。
//!
//! 跑法：`cargo test --release -p vxl-phys --test fluid_reaction_torque -- --ignored --nocapture`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 0.5 m 盆腔：底两层实体 + 一圈围堰（与 `float_quiet_probe::tank` 同形，独立复制以免耦合）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// 一档分辨率：粒子间距 `s`、核半径 `h = 2s`（默认口径）、`n³` 个粒子（水块物理尺寸固定 0.4 m）。
///
/// ⚠️ **子步必须随 `1/h` 缩放**：声学 CFL 是 `c·dt_sub/h ≤ ~0.5`，默认档（h=0.1、substeps=4）
/// 实测 0.42。若只把 h 减半而不动子步 ⇒ CFL 0.83 **超限**（第一版就是这么错的：细档读出的
/// \|τ\| 比粗档大 50× —— 那是数值失稳，不是物理）。⇒ `substeps = 4·(0.1/h)`。
fn water(s: f32, n: usize) -> vxl_phys_fluid::FluidSystem {
    let h = 2.0 * s;
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            smoothing_radius: h,
            substeps: (4.0 * 0.1 / h).round().max(1.0) as u32,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [n, n, n],
        s,
    )
}

/// 一档：静态盒（半长 0.06 @ y=1.20）+ 流体。返回 `(τ 窗口均值, |τ| 窗口均值, |τ| p95, 采样数)`。
fn torque_stats(s: f32, n: usize, settle: usize, win: usize) -> (Vec3, f32, f32, usize) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let mut sys = water(s, n);
    // 铸装挖空：范围 = **体 AABB + 核半径 h**（收紧反而更糟，见 `carve.rs` 文档）。
    vxl_phys_fluid::carve_sphere(&mut sys, Vec3::new(0.0, 1.20, 0.0), 0.06 + 2.0 * s);
    let fid = w.add_fluid_with_boundary_coupling(sys, &[v]);
    let body = w.add_static(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.20, 0.0),
        Quat::IDENTITY,
    );
    for _ in 0..settle {
        w.step();
    }
    // **窗口前自检**（同 `float_quiet_probe` 的教训）：把"流体自己静下来了吗"读出来——
    // 反作用是"两个大数之差"，腔内还在晃时读到的 τ 只是**铸装空腔的余波**，不是稳态偏置。
    let vmax = w.fluids()[fid]
        .0
        .velocities()
        .iter()
        .fold(0.0f32, |m, v| m.max(v.length()));
    let vsum: f32 = w.fluids()[fid]
        .0
        .velocities()
        .iter()
        .map(|v| v.length())
        .sum();
    let vmean = vsum / w.fluids()[fid].0.velocities().len().max(1) as f32;
    let settled = vmax < 0.01;
    let mut sum = Vec3::ZERO;
    let mut mags: Vec<f32> = Vec::new();
    for _ in 0..win {
        w.step();
        if let Some(r) = w.fluids()[fid]
            .0
            .boundary_reactions()
            .iter()
            .find(|r| r.0 == body)
        {
            sum += r.2;
            mags.push(r.2.length());
        }
    }
    let ns = mags.len();
    if ns == 0 {
        return (Vec3::ZERO, 0.0, 0.0, 0);
    }
    let mean = sum * (1.0 / ns as f32);
    mags.sort_by(f32::total_cmp);
    let a_mean = mags.iter().sum::<f32>() / ns as f32;
    let p95 = mags[(ns * 95 / 100).min(ns - 1)];
    println!(
        "    ↳ 窗口起点：流体 |v|max {vmax:.5} ｜ |v|均 {vmean:.5} ⇒ {}",
        if settled {
            "已静止（读数有效）"
        } else {
            "**尚未静止 ⇒ 本次读数含铸装余波，勿当稳态**"
        }
    );
    (mean, a_mean, p95, ns)
}

#[test]
#[ignore = "仪表（只打印）：2b 反作用力矩的窗口统计 + 分辨率扫描"]
fn reaction_torque_stats_and_resolution_sweep() {
    let (settle, win) = (480usize, 480usize);
    // 物理刻度：盒半长 0.06、密度 300 ⇒ 排水体积 = 0.3·(0.12)³；F_b = ρ0·V·g。
    let fb = 1000.0f32 * 0.3 * 0.12f32.powi(3) * 9.81;
    println!("2b 反作用力矩窗口统计（静态盒 半长 0.06 @ y=1.20；静置 {settle} + 窗口 {win} tick）");
    println!("  参照浮力 F_b = {fb:.3} N（300 kg/m³、0.12 m 盒、水）⇒ 等效力臂 = |τ|/F_b");
    println!(
        "{:>8} {:>7} {:>7} {:>28} {:>11} {:>11} {:>14}",
        "间距s", "核半径h", "粒子数", "τ 窗口均值 (x,y,z) N·m", "|τ| 均", "|τ| p95", "等效力臂 p95"
    );
    for (s, n) in [(0.05f32, 8usize), (0.025, 16)] {
        let (mean, a_mean, p95, ns) = torque_stats(s, n, settle, win);
        println!(
            "{:>8.3} {:>7.3} {:>7} ({:+.5},{:+.5},{:+.5}) {:>11.6} {:>11.6} {:>11.3} mm  (n={ns})",
            s,
            2.0 * s,
            n * n * n,
            mean.x,
            mean.y,
            mean.z,
            a_mean,
            p95,
            p95 / fb * 1000.0
        );
    }
    println!(
        "  判读：细化后 |τ| **趋零** ⇒ 离散噪声；**不降** ⇒ 与网格无关的系统性力矩（值得立项）。"
    );
}
