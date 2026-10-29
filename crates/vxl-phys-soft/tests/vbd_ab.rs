//! **M1 A/B：块求解 vs 现役（几何排序 + 就地 GS）**——同一判据场景（`rigid_coupling_gap.rs` 的
//! 33 节点绳 + 0.6 宽盒 + 1800 tick 自扮引擎），只切"体接触怎么施加"。
//!
//! 判据：
//! ① 两条路都得**托住**（末 y > 0.5、末次接触仍在绳高度）——块路径不能把物理做坏；
//! ② 末态差（Δy/Δv/Δx）要小——两条路径的定点应当接近；
//! ③ 报 ms/子步（现役含几何排序；块路径不含）。

use std::time::Instant;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{vbd, RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

struct R {
    y: f32,
    v: f32,
    x: f32,
    xmax: f32,
    contact_end: bool,
    ms_per_step: f64,
}

fn run(iters: u32) -> R {
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &[]);
    }
    let shape = Shape::Box {
        half: Vec3::new(0.3, 0.05, 0.3),
    };
    let (mut pos, mut vel) = (Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    let (mut xmax, mut contact_end) = (0.0f32, false);
    let t0 = Instant::now();
    for t in 0..1800 {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: Quat::IDENTITY,
            linvel: vel,
            angvel: Vec3::ZERO,
            local_inv_inertia: Vec3::ZERO,
            inv_mass: 1.0,
        };
        let one = [proxy];
        if iters > 0 {
            vbd::with_block_solver(iters, || {
                r.step(DT, G, &NoProviders, 0, &one);
            });
        } else {
            r.step(DT, G, &NoProviders, 0, &one);
        }
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = r.body_dv.first() {
            vel += *dv;
        }
        if let Some(dx) = r.body_dx.first() {
            pos += *dx;
        }
        xmax = xmax.max(pos.x.abs());
        let hits = r
            .entry_faces()
            .iter()
            .filter(|(b, _)| *b != u32::MAX)
            .count();
        if t == 1799 {
            contact_end = hits > 0;
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1e3 / 1800.0;
    R {
        y: pos.y,
        v: vel.y,
        x: pos.x,
        xmax,
        contact_end,
        ms_per_step: ms,
    }
}

#[test]
fn block_solver_ab_same_scene() {
    let a = run(0);
    let b = run(1);
    let c = run(4);
    let d = run(8);
    println!(
        "[AB] 现役(off)：末 y={:+.4} v={:+.3} x={:+.4} |x|max={:.4} 末接触={} | {:.1} µs/步",
        a.y,
        a.v,
        a.x,
        a.xmax,
        a.contact_end,
        a.ms_per_step * 1e3
    );
    for (tag, r) in [("块1", &b), ("块4", &c), ("块8", &d)] {
        println!(
            "[AB] {tag}  ：末 y={:+.4} v={:+.3} x={:+.4} |x|max={:.4} 末接触={} | {:.1} µs/步",
            r.y,
            r.v,
            r.x,
            r.xmax,
            r.contact_end,
            r.ms_per_step * 1e3
        );
    }
    println!(
        "[AB] 收敛 |Δy(4→8)|={:.4} |Δx(4→8)|={:.4}",
        (d.y - c.y).abs(),
        (d.x - c.x).abs()
    );
    println!(
        "[AB] 差：Δy={:+.4} Δv={:+.3} Δx={:+.4} | 时间比 {:.2}×",
        b.y - a.y,
        b.v - a.v,
        b.x - a.x,
        b.ms_per_step / a.ms_per_step
    );
    // ① 两条路都得托住
    assert!(
        a.y > 0.5 && a.contact_end,
        "现役路径没托住（y={}），场景本身不适用",
        a.y
    );
    assert!(
        b.y > 0.5 && b.contact_end,
        "块路径没托住（y={}、末接触={}）",
        b.y,
        b.contact_end
    );
    // ② 定点接近（两条路径解的是同一组约束；差别只来自 GS 路径）
    assert!((b.y - a.y).abs() < 0.15, "末 y 差过大：{} vs {}", b.y, a.y);
    assert!((b.x - a.x).abs() < 0.15, "末 x 差过大：{} vs {}", b.x, a.x);
}
