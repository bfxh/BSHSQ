//! **scratch 复现**（不提交）：自由布片落到「冻结但 `inv_mass > 0`」的盒顶（`μ = 0`，300 tick）。
//!
//! 目的：独立复现"动态代理支正常腿托不住轻粒子"的现象，并扫描 `w_p/w_b`。
//! 夹具口径：盒**冻结**（位置/速度固定），但 `inv_mass > 0` ⇒ 两体约束走动态路径；
//! `dv/dx` 只累计不消费（"只吃反作用"的对偶：模型算反作用、夹具不动）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{ClothSheet, RigidProxy, Stiffness};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// 平铺网格（n×n 格、跨度 ±size、y = 0 平面；与 cloth_body.rs 同款）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
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
    (pts, tris)
}

/// 布片落盒顶：返回（y 起点、300 tick 后 mean_y、min_y、是否穿过盒顶 y = 0）。
fn run(
    inv_mass_b: f32,
    ratio_label: &str,
    half_y: f32,
    apply_reaction: bool,
) -> (f32, f32, f32) {
    let (pts, tris) = plate(4, 0.25); // 0.5 m 见方、间距 0.125
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.substeps = 8;
    s.contact.friction = 0.0;
    for p in &mut s.pos {
        p.y += 0.30; // 盒顶之上 0.30 m
    }
    let y0 = s.pos.iter().map(|p| p.y).sum::<f32>() / s.pos.len() as f32;
    let w_p = s.inv_mass[0];
    let shape = Shape::Box {
        half: Vec3::new(0.2, half_y, 0.2),
    };
    let mut box_pos = Vec3::new(0.0, -half_y, 0.0); // 顶面 y = 0
    let mut box_vel = Vec3::ZERO;
    for _ in 0..300 {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos: box_pos,
            rot: Quat::IDENTITY,
            linvel: box_vel,
            angvel: Vec3::ZERO,
            local_inv_inertia: Vec3::ZERO,
            inv_mass: inv_mass_b,
        };
        s.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        if apply_reaction {
            // 真实自由体：消费两条腿（盒在布下自由下落 ⇒ "托住"本就不该期待；
            // 判据看的是**几何是否保持接触**，不是是否停住）
            box_vel += G * DT;
            box_pos += box_vel * DT;
            if let Some(dv) = s.body.dv.first() {
                box_vel += *dv;
            }
            if let Some(dx) = s.body.dx.first() {
                box_pos += *dx;
            }
        }
    }
    let mean_y = s.pos.iter().map(|p| p.y).sum::<f32>() / s.pos.len() as f32;
    let min_y = s.pos.iter().map(|p| p.y).fold(f32::MAX, f32::min);
    let gap = mean_y - box_pos.y - half_y; // 布片均值相对盒顶面的净距离（>0 = 贴在面上侧）
    let wb = if inv_mass_b > 0.0 {
        format!("{:.3}", w_p / inv_mass_b)
    } else {
        "static".into()
    };
    println!(
        "  {ratio_label:>12} hy={half_y:<4} react={:<5}  w_p/w_b={wb:>8}  mean_y {y0:.4} → {mean_y:.4}  min_y={min_y:.4}  gap={gap:+.4}  {}",
        if apply_reaction { "yes" } else { "frozen" },
        if min_y < -2.0 * half_y { "❌ 穿出底部" } else { "✅ 未穿出" }
    );
    (y0, mean_y, min_y)
}

#[test]
fn scratch_free_cloth_on_frozen_dynamic_proxy() {
    // 布片粒子质量：ρ·面积·厚 ≈ 1000·0.015625·0.01 = 0.156 kg ⇒ w_p ≈ 6.4
    println!("自由布片 → 盒顶（μ=0，300 tick，8 子步，起点 y=0.30）");
    let _ = run(0.0, "static", 0.05, false);
    let w_p = 1.0 / (1000.0 * 0.015625 * 0.01);
    // ① 冻结夹具 + 薄盒（复现用户表）：不同质量比
    for (ratio, label) in [(0.16f32, "0.16"), (0.8, "0.8"), (2.0, "2"), (8.0, "8")] {
        let _ = run(w_p / ratio, label, 0.05, false);
    }
    // ② 冻结夹具 + **厚盒**（半高 0.5 ⇒ 逆穿透不会跨中面）：验证"翻面阈值"解释
    let _ = run(w_p / 0.16, "0.16", 0.5, false);
    // ③ **真实自由体**（消费两腿）+ 薄盒：接触应保持（几何不过穿）
    let _ = run(w_p / 0.16, "0.16", 0.05, true);
    let _ = run(w_p / 2.0, "2", 0.05, true);
}

/// **单粒子降维**（去布片连通性）：一颗粒子自由落 → 冻结盒顶（w_b>0），逐 tick 打
/// 位置/速度/体的 dv，量出"每子步被移除多少接近速度"。
#[test]
fn scratch_single_particle_removal_rate() {
    let mut s = ClothSheet::new(
        vec![Vec3::new(0.0, 0.35, 0.0)],
        vec![],
        1000.0,
        0.01,
        Stiffness::Hard,
    );
    s.substeps = 8;
    s.contact.friction = 0.0;
    let w_p = s.inv_mass[0];
    println!("单粒子：w_p={w_p:.3}，盒顶 y=0（半高 0.05），冻结 w_b=w_p/0.16={:.3}", w_p / 0.16);
    let shape = Shape::Box {
        half: Vec3::new(0.2, 0.05, 0.2),
    };
    for t in 0..40 {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos: Vec3::new(0.0, -0.05, 0.0),
            rot: Quat::IDENTITY,
            linvel: Vec3::ZERO,
            angvel: Vec3::ZERO,
            local_inv_inertia: Vec3::ZERO,
            inv_mass: w_p / 0.16,
        };
        s.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        let y = s.pos[0].y;
        let v = s.vel[0].y;
        let dv = s.body.dv.first().map(|d| d.y).unwrap_or(0.0);
        let dx = s.body.dx.first().map(|d| d.y).unwrap_or(0.0);
        println!("t={t:>2} y={y:+.5} v={v:+.5} body_dv_y={dv:+.5} body_dx_y={dx:+.5}");
    }
}
