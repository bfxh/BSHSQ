//! **点循环子块微基准**（手动跑：`cargo test --release -p vxl-phys-solver --lib -- --ignored`）。
//!
//! 为什么不在热路径插钟：单点一段 ~40 ns，低于本机 `Instant` 分辨率（~100 ns，见
//! `M1-RULING §3.12`）。这里用 1e6 次循环把每次调用摊到可测的毫秒级，得到**分支/算术**成本
//! （不含真实场景的随机访存），用来判断"哪块值得向量化"，而不是给线上耗时做绝对值。
use std::time::Instant;

use super::{contact_masses, match_warm_point, resolve_anchor, soft_contact, tangential_target};
use crate::types::SolverParams;
use vxl_phys_core::{BodySet, PhysConfig, Quat, Shape, Vec3};
use vxl_phys_narrow::ContactPoint;

const N: u32 = 1_000_000;

fn bench<T>(name: &str, f: impl Fn() -> T) -> f64 {
    // 预热
    for _ in 0..1000 {
        std::hint::black_box(f());
    }
    let t0 = Instant::now();
    for _ in 0..N {
        std::hint::black_box(f());
    }
    let ns = t0.elapsed().as_nanos() as f64 / N as f64;
    println!("{name:28} {ns:8.1} ns/call");
    ns
}

#[test]
#[ignore = "手动微基准（--ignored 才跑）"]
fn point_blocks() {
    let mut bodies = BodySet::new();
    for k in 0..64 {
        bodies.push_dynamic(
            Shape::Box {
                half: Vec3::splat(0.25),
            },
            Vec3::new(k as f32, 0.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
    }
    let (a, b) = (0usize, 1usize);
    let n = Vec3::new(0.0, 1.0, 0.0);
    let t1 = Vec3::new(1.0, 0.0, 0.0);
    let t2 = Vec3::new(0.0, 0.0, 1.0);
    let ra = Vec3::new(0.1, 0.2, 0.3);
    let rb = Vec3::new(-0.1, -0.2, -0.3);
    let sp = SolverParams::from_config(&PhysConfig::default(), 1.0 / 240.0);
    let rot = vxl_phys_core::Mat3::from_quat(Quat::IDENTITY);
    let cp = ContactPoint {
        point: Vec3::new(0.05, 0.02, 0.03),
        depth: -0.001,
        feature: 7,
    };

    println!("---- 单次调用（1e6 次平均）----");
    let masses = bench("contact_masses + cross", || {
        (
            contact_masses(&bodies, a, b, ra, rb, n, t1, t2),
            super::contact_mass_cross(
                bodies.inv_mass[a],
                bodies.inv_mass[b],
                ra,
                rb,
                t1,
                t2,
                &bodies,
                a,
                b,
            ),
        )
    });
    let vel = bench("velocity_at x2", || {
        bodies.velocity_at(a, ra) - bodies.velocity_at(b, rb)
    });
    let anchor_miss = bench("resolve_anchor(none)", || {
        resolve_anchor(&cp, None, Vec3::ZERO, Vec3::ZERO, &rot, &rot, n)
    });
    let soft = bench("soft_contact", || {
        soft_contact(-0.001, 0.1, 0.0, 1.0, &sp, false)
    });
    let tan = bench("tangential_target", || {
        tangential_target(None, 0.0, 0.4, Vec3::ZERO, &sp, (t1, t2))
    });
    let wm = vxl_phys_solver_test_warm();
    let warm = bench("match_warm_point(hit)", || {
        match_warm_point(&cp, &manifold_probe(), Some(&wm), &[cp.point; 4], 4, 0.04)
    });

    let q = bodies.rot(a);
    let quat = bench("Mat3::from_quat", || vxl_phys_core::Mat3::from_quat(q));
    let awi = bench("apply_world_inv_inertia", || {
        bodies.apply_world_inv_inertia(a, ra.cross(n))
    });
    // 实验：三方向一起算（SoA + 3 趟循环，等 LLVM 自动向量化）；每算子的运算序不变。
    let wide = bench("contact_masses 3-wide", || {
        let dirs = [n, t1, t2];
        let im = [bodies.inv_mass[a], bodies.inv_mass[b]];
        let mut k = [im[0] + im[1]; 3];
        for (lane, d) in dirs.iter().enumerate() {
            if im[0] > 0.0 {
                let rn = ra.cross(*d);
                let w = bodies.apply_world_inv_inertia(a, rn);
                k[lane] += w.cross(ra).dot(*d);
            }
            if im[1] > 0.0 {
                let rn = rb.cross(*d);
                let w = bodies.apply_world_inv_inertia(b, rn);
                k[lane] += w.cross(rb).dot(*d);
            }
        }
        (
            [1.0 / k[0], 1.0 / k[1], 1.0 / k[2]],
            super::contact_mass_cross(
                bodies.inv_mass[a],
                bodies.inv_mass[b],
                ra,
                rb,
                t1,
                t2,
                &bodies,
                a,
                b,
            ),
        )
    });
    println!("---- 合计（每点一次）----");
    println!(
        "sum = {:.1} ns/点（不含 warm 匹配的键查找、结构体写入、循环开销）",
        masses + vel + anchor_miss + soft + tan + warm
    );
    println!(
        "三方向批量 = {wide:.1} ns（原 {masses:.1}，比 {:.2}×）",
        masses / wide.max(0.1)
    );
    println!(
        "其中 quat→Mat3 = {quat:.1} ns × [contact_masses 内约 6–8 次] ≈ {:.0} ns；apply_world_inv_inertia = {awi:.1} ns",
        quat * 7.0
    );
}

/// 造一个"命中"的 warm 流形（4 点，特征号与 `cp` 对上）。
fn vxl_phys_solver_test_warm() -> crate::warm_data::WarmManifold {
    let mut wm = crate::warm_data::WarmManifold::EMPTY;
    for (i, p) in wm.points.iter_mut().enumerate() {
        p.feature = if i == 0 { 7 } else { 100 + i as u32 };
        p.la = Vec3::new(0.01, 0.0, 0.0);
        p.lb = Vec3::new(-0.01, 0.0, 0.0);
        p.depth0 = -0.001;
    }
    wm.n = 4;
    wm
}

fn manifold_probe() -> vxl_phys_narrow::Manifold {
    vxl_phys_narrow::Manifold {
        a: 0,
        b: 1,
        normal: Vec3::new(0.0, 1.0, 0.0),
        points: vxl_phys_narrow::ContactPoints::from_slice(&[ContactPoint {
            point: Vec3::ZERO,
            depth: -0.001,
            feature: 7,
        }]),
    }
}
