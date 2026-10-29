//! **M3.1 布料专用判据**：块求解（不排序 + Jacobi）vs 现役（几何排序 + 就地 GS），
//! 同一"四角钉住的布兜 + 动态盒"场景。
//!
//! 布料这边不看横向漂移（那是绳子的判据），看四条**布料自己的量**：
//! ① 盒**被兜住**（末 y 不塌到布下面）；② 布的**最大应变**有界；③ 盒的**横向漂移**；
//! ④ 步耗时。另加"4 遍 vs 8 遍"的收敛判据（残余穿透口径生效 ⇒ 应几乎相同）。

use std::time::Instant;

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{vbd, ClothSheet, RigidProxy, Stiffness};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

struct R {
    box_y: f32,
    strain: f32,
    x_max: f32,
    cloth_lo: f32,
    ms: f64,
    finite: bool,
}

/// 四角钉住的布（`n×n` 格、跨度 ±0.5，初始 y = 0）+ 从 y = 0.6 落下的 0.4 宽动态盒。
fn run(iters: u32) -> R {
    let n = 6usize;
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
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    // 四角钉住
    for &i in &[0usize, n, n * (n + 1), (n + 1) * (n + 1) - 1] {
        sheet.set_pinned(i, true);
    }
    let shape = Shape::Box {
        half: Vec3::new(0.2, 0.05, 0.2),
    };
    let mut pos = Vec3::new(0.0, 0.6, 0.0);
    let mut vel = Vec3::ZERO;
    let (mut x_max, mut finite) = (0.0f32, true);
    let t0 = Instant::now();
    for _ in 0..600 {
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
        let bodies = [proxy];
        if iters > 0 {
            vbd::with_block_solver(iters, || {
                sheet.step(DT, G, &NoProviders, 0, &bodies);
            });
        } else {
            sheet.step(DT, G, &NoProviders, 0, &bodies);
        }
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = sheet.body.dv.first() {
            vel += *dv;
        }
        if let Some(dx) = sheet.body.dx.first() {
            pos += *dx;
        }
        x_max = x_max.max(pos.x.abs());
        finite &= pos.is_finite() && vel.is_finite();
    }
    let ms = t0.elapsed().as_secs_f64() * 1e3 / 600.0;
    let cloth_lo = sheet.pos.iter().fold(f32::MAX, |a, p| a.min(p.y));
    R {
        box_y: pos.y,
        strain: sheet.max_strain(),
        x_max,
        cloth_lo,
        ms,
        finite,
    }
}

#[test]
fn cloth_block_vs_scalar_ab() {
    let a = run(0);
    let b = run(1);
    let c = run(4);
    let d = run(8);
    for (tag, r) in [("现役", &a), ("块1", &b), ("块4", &c), ("块8", &d)] {
        println!(
            "[布AB] {tag}：盒 y={:+.4} 应变={:.4} |x|max={:.4} 布最低 y={:+.4} | {:.1} µs/步",
            r.box_y,
            r.strain,
            r.x_max,
            r.cloth_lo,
            r.ms * 1e3
        );
    }
    println!(
        "[布AB] 收敛 |Δy(4→8)|={:.4} |Δ应变(4→8)|={:.5}",
        (d.box_y - c.box_y).abs(),
        (d.strain - c.strain).abs()
    );
    // ① 都被兜住（盒不塌到布下面），且都有限
    for (tag, r) in [("现役", &a), ("块1", &b), ("块4", &c), ("块8", &d)] {
        assert!(r.finite, "{tag} 出现 NaN/Inf");
        assert!(r.box_y > -0.05, "{tag} 盒子穿过布兜（y={}）", r.box_y);
    }
    // ② 应变有界（布料不炸）
    for (tag, r) in [("现役", &a), ("块1", &b), ("块4", &c), ("块8", &d)] {
        assert!(r.strain < 1.0, "{tag} 应变爆炸（{}）", r.strain);
    }
    // ③ 单遍块与现役的末态**几乎相同**（同帧反作用后实测差 1e-4 量级；旧版是 3.5e-3）
    assert!(
        (b.box_y - a.box_y).abs() < 0.01,
        "块 1 遍与现役末 y 差过大：{} vs {}",
        b.box_y,
        a.box_y
    );
    // ④ **已知边界**：布料侧块求解仍被钳到一遍（体反作用走 dv/disp，下一帧才进位置 ⇒
    //    "本遍残余"没有良定义；实测 1/4/8 遍末 y = +0.0038/+0.0108/+0.0842，多遍会**上漂**）。
    //    所以这里只钉"4 遍与 8 遍与 1 遍完全一致"——即钳位生效、不会悄悄多推。
    assert!(
        (d.box_y - b.box_y).abs() < 1e-6 && (c.box_y - b.box_y).abs() < 1e-6,
        "布料侧未被钳到一遍：1={} 4={} 8={}",
        b.box_y,
        c.box_y,
        d.box_y
    );
}
