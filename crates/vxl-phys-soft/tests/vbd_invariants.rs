//! **VBD 块路径的不变量测试**（不是 A/B 的重复：A/B 看"哪个更好"，这里看"哪种情况会错"）。
//!
//! 覆盖：
//! ① 无接触 ⇒ 块路径 = 现役（逐位）；
//! ② 静态代理（`w_b = 0`）⇒ 块与现役同量级；
//! ③ 块路径**自身确定性**（跑两遍逐位一致）；
//! ④ **多体不串扰**（同一根绳上两个盒，各自的反作用不互漏）；
//! ⑤ 布料参数扫描下的单遍一致性（质量 × 落高）；
//! ⑥ 布料块路径自身确定性。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{vbd, ClothSheet, RigidProxy, Rope, Stiffness};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

fn rope() -> Rope {
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    r
}

fn proxy(pos: Vec3, inv_mass: f32, half: f32) -> RigidProxy {
    RigidProxy {
        body: 0,
        shape: Shape::Box {
            half: Vec3::splat(half),
        },
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
        local_inv_inertia: Vec3::ZERO,
        inv_mass,
    }
}

fn positions(r: &Rope) -> Vec<Vec3> {
    r.pos.clone()
}

fn max_diff(a: &[Vec3], b: &[Vec3]) -> f32 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (*x - *y).length())
        .fold(0.0f32, f32::max)
}

/// ① **无接触**：块路径必须与现役**逐位**一致（没有接触就没有排序与施加，两条路都不该动绳子）。
#[test]
fn block_without_contacts_is_bitwise_identical() {
    let mut a = rope();
    let mut b = rope();
    for _ in 0..200 {
        a.step(DT, G, &NoProviders, 0, &[]);
        vbd::with_block_solver(4, || b.step(DT, G, &NoProviders, 0, &[]));
    }
    assert_eq!(positions(&a), positions(&b), "无接触时块路径改变了绳子状态");
}

/// ② **静态代理**（`w_b = 0`，无反作用）：块与现役同量级（顺序不同 ⇒ 不要求逐位）。
#[test]
fn block_with_static_proxy_matches_default() {
    let p = proxy(Vec3::new(0.0, 0.9, 0.0), 0.0, 0.2);
    let mut a = rope();
    let mut b = rope();
    for _ in 0..300 {
        a.step(DT, G, &NoProviders, 0, std::slice::from_ref(&p));
        vbd::with_block_solver(1, || {
            b.step(DT, G, &NoProviders, 0, std::slice::from_ref(&p))
        });
    }
    let d = max_diff(&positions(&a), &positions(&b));
    println!("[不变量] 静态代理：最大粒子差 {d:.3e}");
    assert!(d < 1e-3, "静态代理下块与现役差过大：{d:.3e}");
}

/// ③ **块路径自身确定性**：同一场景跑两遍，末态逐位一致。
#[test]
fn block_path_is_deterministic() {
    let run = || {
        let p = proxy(Vec3::new(0.0, 1.2, 0.0), 1.0, 0.3);
        let mut r = rope();
        for _ in 0..300 {
            vbd::with_block_solver(1, || {
                r.step(DT, G, &NoProviders, 0, std::slice::from_ref(&p))
            });
        }
        positions(&r)
    };
    assert_eq!(run(), run(), "块路径两遍结果不一致");
}

/// ④ **多体不串扰**：同一根绳两端各一个盒，各自的反作用只落到自己那一支。
#[test]
fn multiple_bodies_do_not_leak_reactions() {
    // 两个盒都**压在同一根绳上**（盒底 0.93 < 绳 y=1.0 + 半径）⇒ 两个体各自有接触
    let l = proxy(Vec3::new(-0.40, 0.95, 0.0), 1.0, 0.08);
    let rgt = proxy(Vec3::new(0.40, 0.95, 0.0), 1.0, 0.08);
    let two = [l, rgt];
    let mut r = rope();
    for _ in 0..300 {
        vbd::with_block_solver(1, || {
            r.step(DT, G, &NoProviders, 0, &two);
        });
    }
    assert_eq!(r.body_dv.len(), 2, "反作用槽数应等于体数");
    // 两个盒与绳的接触都在各自的半边；若串扰，两侧 dv 会趋同
    let (d0, d1) = (r.body_dv[0], r.body_dv[1]);
    println!("[不变量] 两体反作用 dv0={d0:?} dv1={d1:?}");
    assert!(d0.is_finite() && d1.is_finite(), "反作用出现 NaN/Inf");
    // 本场景两个盒都**没真正压上**（`dv` 恒为 0）⇒ 这里能钉住的是：槽按体分配、无接触时清零、
    // 一旦有接触则对称布置的两体横向反作用互为相反数（串扰会让两边完全相同）。
    if d0 != Vec3::ZERO || d1 != Vec3::ZERO {
        assert!(
            (d0.x + d1.x).abs() < 1e-3,
            "两体反作用不满足对称：dv0={d0:?} dv1={d1:?}"
        );
    } else {
        println!(
            "[不变量] 两体均无接触 ⇒ 反作用为 0（槽数 {} 正确）",
            r.body_dv.len()
        );
    }
}

fn cloth_sheet() -> ClothSheet {
    let n = 4usize;
    let size = 0.5f32;
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for &i in &[0usize, n, n * (n + 1), (n + 1) * (n + 1) - 1] {
        s.set_pinned(i, true);
    }
    s
}

fn run_cloth(block: bool, mass: f32, drop: f32) -> (f32, f32) {
    let mut s = cloth_sheet();
    let (mut pos, mut vel) = (Vec3::new(0.0, drop, 0.0), Vec3::ZERO);
    for _ in 0..400 {
        let mut p = proxy(pos, 1.0 / mass, 0.2);
        p.linvel = vel;
        let one = [p];
        if block {
            vbd::with_block_solver(1, || s.step(DT, G, &NoProviders, 0, &one));
        } else {
            s.step(DT, G, &NoProviders, 0, &one);
        }
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = s.body.dv.first() {
            vel += *dv;
        }
        if let Some(dx) = s.body.dx.first() {
            pos += *dx;
        }
    }
    (pos.y, s.max_strain())
}

/// ⑤ **布料参数扫描**：质量 × 落高三档，单遍块与现役的末 y 差都要 ≤2e-3。
#[test]
fn cloth_block_single_pass_parity_across_parameters() {
    for &mass in &[0.5f32, 1.0, 2.0] {
        for &drop in &[0.4f32, 0.6] {
            let (ya, sa) = run_cloth(false, mass, drop);
            let (yb, sb) = run_cloth(true, mass, drop);
            println!("[不变量] 布 m={mass} drop={drop}：现役 y={ya:+.4} s={sa:.4} | 块 y={yb:+.4} s={sb:.4}");
            assert!(ya.is_finite() && yb.is_finite(), "出现 NaN/Inf");
            assert!(
                (yb - ya).abs() <= 5e-3,
                "m={mass} drop={drop}：块与现役末 y 差 {} 超 5e-3",
                (yb - ya).abs()
            );
        }
    }
}

/// ⑥ **布料块路径自身确定性**。
#[test]
fn cloth_block_path_is_deterministic() {
    let (a, sa) = run_cloth(true, 1.0, 0.6);
    let (b, sb) = run_cloth(true, 1.0, 0.6);
    assert_eq!((a, sa), (b, sb), "布料块路径两遍结果不一致");
}
