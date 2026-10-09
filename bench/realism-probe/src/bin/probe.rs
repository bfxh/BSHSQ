//! BSHSQ 真实性补测探针（外部只读调用公共 API，不改引擎代码）。
//! 输出一行一个 JSON 对象，供 CI 解析。
//!
//! 七个探针（R1–R7）覆盖：能量漂移 / 弹性碰撞守恒 / 静置接触力连续性 / 穿透深度分布 /
//! 质量比极限 / 关节约束违反 / 流体体积守恒。**全部量机器无关**（跨 OS / CPU / 编译器
//! 逐字一致，2026-10-09 在 Windows + Linux 沙箱双环境复现）⇒ 可以直接做 CI 判据。
//!
//! 参考基线：`bench/baselines/realism.jsonl`；门脚本：`scripts/gate_bench.sh`。
use vxl_phys::{Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::Mat3;

const G: f32 = 9.81;

/// 已排序切片的分位（调用方先 `sort_by(f32::total_cmp)`；非 NaN 数据下与 `partial_cmp` 同序）。
fn pct_sorted(v: &[f32], p: f64) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    let k = ((v.len() - 1) as f64 * p / 100.0).round() as usize;
    v[k]
}

fn body_energy(w: &World, i: usize) -> (f64, f64, f64) {
    let b = &w.bodies;
    let m = 1.0 / b.inv_mass[i] as f64;
    let v = b.linvel[i];
    let ke_t = 0.5 * m * (v.dot(v) as f64);
    let (p, q) = b.pose(i);
    let wl = Mat3::from_quat(q).transpose_mul_vec3(b.angvel(i));
    let ii = b.local_inv_inertia[i];
    let inv = |x: f32| if x > 0.0 { 1.0 / x as f64 } else { 0.0 };
    let ke_r = 0.5
        * (inv(ii.x) * (wl.x as f64).powi(2)
            + inv(ii.y) * (wl.y as f64).powi(2)
            + inv(ii.z) * (wl.z as f64).powi(2));
    let pe = m * G as f64 * p.y as f64;
    (ke_t, ke_r, pe)
}

/// R1 自由飞行能量漂移：无接触、无阻尼；E = KE_t + KE_r + PE。
fn free_flight(ticks: usize) {
    let mut w = World::new(PhysConfig::default());
    let id = w.add_dynamic(
        Shape::Box {
            half: Vec3::new(0.4, 0.2, 0.1),
        },
        Vec3::new(0.0, 200.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_linvel(id, Vec3::new(3.0, 0.0, 0.0));
    w.bodies.set_angvel(id, Vec3::new(0.3, 2.0, 0.1));
    let (a, b0, c) = body_energy(&w, id);
    let e0 = a + b0 + c;
    let (mut max_rel, mut max_rot_rel) = (0.0f64, 0.0f64);
    let mut ke_scale = 0.0f64;
    for _ in 0..ticks {
        w.step();
        let (kt, kr, pe) = body_energy(&w, id);
        ke_scale = ke_scale.max(kt + kr);
        max_rel = max_rel.max(((kt + kr + pe) - e0).abs());
        max_rot_rel = max_rot_rel.max((kr - b0).abs() / b0);
    }
    println!(
        "{{\"probe\":\"R1_free_flight_energy\",\"ticks\":{ticks},\"dt_s\":0.016667,\"E0_J\":{e0:.3},\
         \"max_abs_dE_J\":{max_rel:.4},\"max_dE_over_E0_pct\":{:.5},\"max_dE_over_KEmax_pct\":{:.5},\
         \"rot_KE_max_rel_drift_pct\":{:.5}}}",
        100.0 * max_rel / e0.abs(),
        100.0 * max_rel / ke_scale,
        100.0 * max_rot_rel
    );
}

/// R2 弹性落体（e=1）：10 次反弹后机械能保留率。
fn elastic_bounce(ticks: usize) {
    let cfg = PhysConfig {
        restitution: 1.0,
        restitution_threshold: 0.01,
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.0 },
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let id = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 2.25, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let (_, _, pe0) = body_energy(&w, id);
    let m = 1.0 / w.bodies.inv_mass[id] as f64;
    let e0 = pe0 - m * G as f64 * 0.25; // 以落地质心高度为零点
    let mut peaks = Vec::new();
    let mut prev_vy = 0.0f32;
    let mut nan = false;
    for _ in 0..ticks {
        w.step();
        let vy = w.bodies.linvel[id].y;
        if !vy.is_finite() {
            nan = true;
            break;
        }
        if prev_vy > 0.0 && vy <= 0.0 {
            let (kt, kr, pe) = body_energy(&w, id);
            peaks.push((kt + kr + pe - m * G as f64 * 0.25) / e0);
        }
        prev_vy = vy;
    }
    let s: Vec<String> = peaks.iter().take(10).map(|r| format!("{:.4}", r)).collect();
    println!("{{\"probe\":\"R2_elastic_bounce_e1\",\"ticks\":{ticks},\"drop_height_m\":2.0,\"bounces\":{},\"energy_retained_ratio_per_apex\":[{}],\"nan\":{nan}}}", peaks.len(), s.join(","));
}

/// R3 静置接触力连续性：单盒静置，关睡眠；F_n(t) = m·Δv_y/dt + m·g（动量平衡，精确）。
fn contact_force_jitter(warm: usize, meas: usize) {
    let cfg = PhysConfig {
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    };
    let dt = cfg.dt as f64;
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let id = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let m = 1.0 / w.bodies.inv_mass[id] as f64;
    for _ in 0..warm {
        w.step();
    }
    let mut prev = w.bodies.linvel[id].y as f64;
    let mut f = Vec::with_capacity(meas);
    let mut ymin = f32::MAX;
    let mut ymax = f32::MIN;
    for _ in 0..meas {
        w.step();
        let vy = w.bodies.linvel[id].y as f64;
        f.push(m * (vy - prev) / dt + m * G as f64);
        prev = vy;
        let y = w.bodies.position[id].y;
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    let mg = m * G as f64;
    let mean = f.iter().sum::<f64>() / f.len() as f64;
    let var = f.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / f.len() as f64;
    let dmax = f
        .windows(2)
        .map(|p| (p[1] - p[0]).abs())
        .fold(0.0, f64::max);
    println!(
        "{{\"probe\":\"R3_resting_contact_force\",\"warmup_ticks\":{warm},\"measured_ticks\":{meas},\
         \"mg_N\":{mg:.2},\"mean_Fn_over_mg\":{:.6},\"var_Fn_N2\":{var:.6e},\
         \"std_Fn_over_mg_pct\":{:.6},\"max_tick_to_tick_dFn_over_mg_pct\":{:.6},\
         \"rest_height_range_m\":{:.3e}}}",
        mean / mg,
        100.0 * var.sqrt() / mg,
        100.0 * dmax / mg,
        (ymax - ymin)
    );
}

/// R4 穿透深度分布：N 盒雨落到静态瓦片地面（同 m1_scale 布局），统计全部接触点 depth>0 的分布。
fn penetration(n_dyn: usize, ticks: usize) {
    let mut w = World::new(PhysConfig::default());
    let d_side = (n_dyn as f64).sqrt() as usize + 1;
    let s_side = d_side + 2;
    for k in 0..s_side * s_side {
        let x = (k % s_side) as f32 - s_side as f32 * 0.5;
        let z = (k / s_side) as f32 - s_side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.5, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..n_dyn {
        let x = (k % d_side) as f32 - d_side as f32 * 0.5;
        let z = (k / d_side) as f32 - d_side as f32 * 0.5;
        let y = 3.0 + ((k * 29) % 71) as f32 / 71.0 * 8.0;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    let mut all = Vec::new();
    let mut tail = Vec::new();
    let skin4 = 4.0 * 0.02f32;
    let mut deep_samples = 0usize;
    for t in 0..ticks {
        w.step();
        for mf in w.manifolds() {
            for cp in &mf.points {
                if cp.depth > 0.0 {
                    all.push(cp.depth);
                    if cp.depth > skin4 {
                        deep_samples += 1;
                    }
                    if t + 1 > ticks - 60 {
                        tail.push(cp.depth);
                    }
                }
            }
        }
    }
    let n = all.len();
    all.sort_by(|a, b| a.total_cmp(b));
    let (p50, p99, mx) = (
        pct_sorted(&all, 50.0),
        pct_sorted(&all, 99.0),
        pct_sorted(&all, 100.0),
    );
    let tn = tail.len();
    tail.sort_by(|a, b| a.total_cmp(b));
    let tp99 = pct_sorted(&tail, 99.0);
    println!(
        "{{\"probe\":\"R4_penetration_distribution\",\"n_dynamic\":{n_dyn},\"n_static\":{},\"ticks\":{ticks},\
         \"contact_samples\":{n},\"depth_p50_m\":{p50:.5},\"depth_p99_m\":{p99:.5},\"depth_max_m\":{mx:.5},\
         \"deep_gt_4skin_samples\":{deep_samples},\"deep_rate_pct\":{:.4},\"tail60_samples\":{tn},\
         \"tail60_depth_p99_m\":{tp99:.5}}}",
        s_side * s_side,
        100.0 * deep_samples as f64 / n.max(1) as f64
    );
}

/// R5 质量比极限：两盒叠放（下盒 ρ=1000，上盒 ρ=1000·r），600 tick。
fn mass_ratio(ticks: usize) {
    let mut first_fail: Option<f64> = None;
    let mut rows = Vec::new();
    let env = |k: &str, d: u32| {
        std::env::var(k)
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(d)
    };
    for &r in &[
        1.0f32,
        2.0,
        5.0,
        10.0,
        20.0,
        30.0,
        50.0,
        70.0,
        100.0,
        1_000.0,
        10_000.0,
        100_000.0,
        1_000_000.0,
    ] {
        let mut w = World::new(PhysConfig {
            substeps: env("MR_SUBSTEPS", 2),
            velocity_iterations: env("MR_ITERS", 3),
            ..PhysConfig::default()
        });
        w.add_static(
            Shape::Box {
                half: Vec3::new(5.0, 0.5, 5.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        let lo = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.5, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize;
        let hi = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 1.5, 0.0),
            Quat::IDENTITY,
            1000.0 * r,
        ) as usize;
        let mut max_depth = 0.0f32;
        let mut nan = false;
        for _ in 0..ticks {
            w.step();
            for mf in w.manifolds() {
                for cp in &mf.points {
                    max_depth = max_depth.max(cp.depth);
                }
            }
            if !w.bodies.position[hi].y.is_finite() {
                nan = true;
                break;
            }
        }
        let y_lo = w.bodies.position[lo].y;
        let y_hi = w.bodies.position[hi].y;
        let err = ((y_lo - 0.5).abs()).max((y_hi - 1.5).abs());
        let fail = nan || err > 0.08 || max_depth > 0.08;
        if fail && first_fail.is_none() {
            first_fail = Some(r as f64);
        }
        rows.push(format!("{{\"ratio\":{r},\"y_lo\":{y_lo:.4},\"y_hi\":{y_hi:.4},\"max_depth_m\":{max_depth:.4},\"final_pos_err_m\":{err:.4},\"nan\":{nan},\"fail\":{fail}}}"));
    }
    println!("{{\"probe\":\"R5_mass_ratio_stack2\",\"ticks\":{ticks},\"fail_rule\":\"nan|pos_err>0.08m|depth>4*skin(0.08m)\",\"rows\":[{}],\"first_fail_ratio\":{}}}",
        rows.join(","), first_fail.map(|x| x.to_string()).unwrap_or("null".into()));
}

/// R6 关节约束违反：10 节球铰链（一端挂静态体），600 tick 摆动。
fn joint_chain(links: usize, ticks: usize) {
    let mut w = World::new(PhysConfig {
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    });
    let anchor = w.add_static(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 20.0, 0.0),
        Quat::IDENTITY,
    );
    let mut prev = anchor;
    let mut joints = Vec::new();
    for k in 0..links {
        // 水平摆放 ⇒ 释放后大幅摆动
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::new(0.25, 0.05, 0.05),
            },
            Vec3::new(0.25 + 0.5 * k as f32, 20.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
        let aa = if k == 0 {
            Vec3::ZERO
        } else {
            Vec3::new(0.25, 0.0, 0.0)
        };
        w.add_joint(Joint::new(
            JointKind::Spherical,
            prev,
            id,
            aa,
            Vec3::new(-0.25, 0.0, 0.0),
        ));
        joints.push((prev as usize, id as usize, aa, Vec3::new(-0.25, 0.0, 0.0)));
        prev = id;
    }
    let mut seps = Vec::new();
    for _ in 0..ticks {
        w.step();
        for &(a, b, la, lb) in &joints {
            let (pa, qa) = w.bodies.pose(a);
            let (pb, qb) = w.bodies.pose(b);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(la);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(lb);
            seps.push((wb - wa).length());
        }
    }
    let n = seps.len();
    let v1 = seps.iter().filter(|&&s| s > 0.01).count();
    let v5 = seps.iter().filter(|&&s| s > 0.05).count();
    seps.sort_by(|a, b| a.total_cmp(b));
    println!(
        "{{\"probe\":\"R6_spherical_chain_violation\",\"links\":{links},\"ticks\":{ticks},\"samples\":{n},\
         \"sep_p50_m\":{:.5},\"sep_p99_m\":{:.5},\"sep_max_m\":{:.5},\
         \"violation_rate_gt_1cm_pct\":{:.3},\"violation_rate_gt_5cm_pct\":{:.3}}}",
        pct_sorted(&seps, 50.0),
        pct_sorted(&seps, 99.0),
        pct_sorted(&seps, 100.0),
        100.0 * v1 as f64 / n as f64,
        100.0 * v5 as f64 / n as f64
    );
}

/// R7 流体体积守恒（WCSPH，溃坝场景，与 m5_dam_break 同几何）：V = Σ m/ρ_i 相对首 tick 的变化。
fn fluid_volume(ticks: usize) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 0 || ix == 4 || iz == 0 || iz == 4 {
                vol.set(ix, 2, iz, true);
            }
        }
    }
    let vid = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.73, 1.05, -0.125),
        [6, 6, 14],
        0.05,
    );
    w.add_fluid(sys, &[vid]);
    let vol_of = |w: &World| -> (f64, usize, f64) {
        let Some((s, ..)) = w.fluids().first() else {
            return (0.0, 0, 0.0);
        };
        let rho0 = s.config().rest_density as f64;
        let d = s.densities();
        let v: f64 = d.iter().map(|&r| 1.0 / r.max(1e-6) as f64).sum();
        let mean = d.iter().map(|&r| r as f64).sum::<f64>() / d.len().max(1) as f64;
        (v, d.len(), mean / rho0)
    };
    w.step();
    let (v1, n1, _) = vol_of(&w);
    let mut max_dev = 0.0f64;
    let mut last = (0.0, 0, 0.0);
    for _ in 1..ticks {
        w.step();
        let (v, n, r) = vol_of(&w);
        max_dev = max_dev.max((v / v1 - 1.0).abs());
        last = (v, n, r);
    }
    println!(
        "{{\"probe\":\"R7_fluid_volume_wcsph\",\"ticks\":{ticks},\"particles_t1\":{n1},\
         \"particles_end\":{},\"volume_end_rel_err_pct\":{:.3},\"volume_max_abs_rel_err_pct\":{:.3},\
         \"mean_rho_over_rho0_end\":{:.4}}}",
        last.1,
        100.0 * (last.0 / v1 - 1.0),
        100.0 * max_dev,
        last.2
    );
}

/// 邻居计数的"满格"参考：spacing 晶格在半径 h 内的邻居数（不含自身）。
/// 体内粒子 = 邻居数 ≥ 满格数；其余 = 自由面/近壁。
fn full_support_count(spacing: f32, h: f32) -> usize {
    let r = (h / spacing).floor() as i32;
    let mut c = 0;
    for i in -r..=r {
        for j in -r..=r {
            for k in -r..=r {
                let d2 = (i * i + j * j + k * k) as f32 * spacing * spacing;
                if i == 0 && j == 0 && k == 0 {
                    continue;
                }
                if d2 <= h * h {
                    c += 1;
                }
            }
        }
    }
    c
}

/// 粒子所在格坐标（与 `UniformGrid::bin_of` 同公式，钳边）。
fn bin_of(ng: &vxl_phys_fluid::NeighborGrid<'_>, p: Vec3) -> (u32, u32, u32) {
    let f = |o: f32, v: f32, n: u32| -> u32 {
        (((v - o) * ng.inv).floor().max(0.0) as u32).min(n - 1)
    };
    (
        f(ng.min.x, p.x, ng.dims.0),
        f(ng.min.y, p.y, ng.dims.1),
        f(ng.min.z, p.z, ng.dims.2),
    )
}

/// 流体粒子 `i` 在 27 邻域内（r ≤ h）的流体-流体邻居数。
fn count_fluid_neighbors(
    ng: &vxl_phys_fluid::NeighborGrid<'_>,
    pos: &[Vec3],
    n: usize,
    h2: f32,
    i: usize,
) -> usize {
    let pi = pos[i];
    let (bx, by, bz) = bin_of(ng, pi);
    let mut c = 0usize;
    for dz in -1i32..=1 {
        let z = bz as i32 + dz;
        if z < 0 || z >= ng.dims.2 as i32 {
            continue;
        }
        for dy in -1i32..=1 {
            let y = by as i32 + dy;
            if y < 0 || y >= ng.dims.1 as i32 {
                continue;
            }
            for dx in -1i32..=1 {
                let x = bx as i32 + dx;
                if x < 0 || x >= ng.dims.0 as i32 {
                    continue;
                }
                let idx = ((x as usize * ng.dims.1 as usize) + y as usize) * ng.dims.2 as usize
                    + z as usize;
                let lo = ng.start[idx] as usize;
                let hi = ng.start[idx + 1] as usize;
                for &j in &ng.items[lo..hi] {
                    let j = j as usize;
                    if j == i || j >= n {
                        continue;
                    }
                    if (pi - pos[j]).length_squared() <= h2 {
                        c += 1;
                    }
                }
            }
        }
    }
    c
}

/// 按"邻居数 ≥ `full`"把粒子分成内部/自由面，返回
/// `(n_total, n_interior, ρ̄/ρ₀_all, ρ̄/ρ₀_interior, ρ̄/ρ₀_surface)`。
fn split_density_stats(
    ng: &vxl_phys_fluid::NeighborGrid<'_>,
    pos: &[Vec3],
    dens: &[f32],
    h2: f32,
    full: usize,
    rho0: f32,
) -> (usize, usize, f64, f64, f64) {
    let n = dens.len();
    let mut s_all = 0.0f64;
    let mut s_int = 0.0f64;
    let mut s_surf = 0.0f64;
    let (mut n_int, mut s_int_n, mut s_surf_n) = (0usize, 0u64, 0u64);
    for i in 0..n {
        s_all += dens[i] as f64;
        if count_fluid_neighbors(ng, pos, n, h2, i) >= full {
            n_int += 1;
            s_int += dens[i] as f64;
            s_int_n += 1;
        } else {
            s_surf += dens[i] as f64;
            s_surf_n += 1;
        }
    }
    let per = |s: f64, m: u64| if m > 0 { s / m as f64 / rho0 as f64 } else { 0.0 };
    (
        n,
        n_int,
        s_all / n.max(1) as f64 / rho0 as f64,
        per(s_int, s_int_n),
        per(s_surf, s_surf_n),
    )
}

/// R7i —— R7 的口径分离版：把"自由面核截断"与"内部真误差"分开报。
///
/// 背景：R7 溃坝 `|ΔV/V|max = 68.7%`、`ρ̄/ρ₀_end = 0.7999` 是**合数**——
/// 自由面粒子的支持域被截断、SPH 密度天然偏低，于是"体积"被系统性高估。
/// 修不修 68.7% 之前必须先**分离口径**：本探针分别报内部/自由面两档密度均值与占比。
///
/// 判据：
/// - 内部粒子 = 流体-流体邻居数 ≥ `full_support_count`（晶格满格参考，**0.05/0.1 ⇒ 32**）。
///   静置深水柱实测 ρ/ρ₀ ≈ 0.99–1.01（核归一化正确），溃坝薄片全程 maxN ≤ 29 ⇒ 无内部。
/// - 同时仍报"全粒子" ρ̄/ρ₀ 与粒子数守恒，与 R7 对照。
///
/// 输出字段全部机器无关（邻居/密度是 SPH 标定的确定函数，与 CPU/OS/编译器无关）。
fn fluid_volume_split(ticks: usize) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 0 || ix == 4 || iz == 0 || iz == 4 {
                vol.set(ix, 2, iz, true);
            }
        }
    }
    let vid = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.73, 1.05, -0.125),
        [6, 6, 14],
        0.05,
    );
    w.add_fluid(sys, &[vid]);
    let spacing = 0.05f32;
    let h = 0.1f32;
    let h2 = h * h;
    let full = full_support_count(spacing, h);
    let mut snapshot = (0usize, 0usize, 0.0f64, 0.0f64, 0.0f64); // (n_total, n_interior, mean_all, mean_int, mean_surf)
    w.step();
    let _ = ticks;
    for tick in 0..ticks {
        w.step();
        let Some((s, ..)) = w.fluids().first() else { return };
        let rho0 = s.config().rest_density;
        let d = s.densities();
        let pos = s.positions();
        let ng = s.neighbor_grid();
        let (n, n_int, mean_all, mean_int, mean_surf) =
            split_density_stats(&ng, pos, d, h2, full, rho0);
        if tick == ticks - 1 {
            snapshot = (n, n_int, mean_all, mean_int, mean_surf);
        }
    }
    let (n_total, n_int, mean_all, mean_int, mean_surf) = snapshot;
    println!(
        "{{\"probe\":\"R7i_fluid_volume_split\",\"ticks\":{ticks},\"particles_total\":{n_total},\
         \"interior_count\":{n_int},\"interior_frac_pct\":{:.3},\
         \"mean_rho_over_rho0_all\":{:.4},\"mean_rho_over_rho0_interior\":{:.4},\
         \"mean_rho_over_rho0_surface\":{:.4},\"full_support_ref\":{full}}}",
        100.0 * n_int as f64 / n_total.max(1) as f64,
        mean_all,
        mean_int,
        mean_surf,
    );
}

fn main() {
    free_flight(300);
    elastic_bounce(900);
    contact_force_jitter(120, 600);
    penetration(1000, 600);
    mass_ratio(600);
    joint_chain(10, 600);
    fluid_volume(400);
    fluid_volume_split(400);
}
