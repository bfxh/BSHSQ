//! **判据：布片升力**（`AeroConfig.lift_slope` 的落点 —— 骨架里立了很久、本片才有的消费方）。
//!
//! **模型**（`vxl_phys_aero::face_force_with_lift`）：`F = 阻力 + 升力`，升力 =
//! `½·ρ·A·|u|²·lift_slope·sinα·cosα·d̂`（`sinα = n·û` 带符号；实现取未归一化形式，
//! **幅值自带 `cosα`** ⇒ 迎角 0°/90° 两端平滑归零 —— 没有 `cosα` 会在 ⊥风 附近跳变，
//! 判据④实测翻车过，留档在 `face_force_with_lift` 的注里）。
//!
//! **判据**（动量口径，与 `cloth_aero.rs` ① 同一族）：
//! ① **解析对拍**：自由平铺布片（法线 `Y`）、风在 `x–y` 面内以迎角 `φ = 30°` 吹来 ⇒ 单子步
//!    总动量 = `½ρ·A·W²·[Cd·û + lift_slope·sinφ·cosφ·d̂]·h`（闭式，`d̂ = (−sinφ, cosφ, 0)`）；
//! ② **符号反对称**（双面口径在布片上的表现）：风从另一侧吹（`φ → −φ`）⇒ 升力分量**反号**、
//!    阻力分量**不变**；
//! ③ **关升力 = 纯阻力**：`lift_slope = 0` 与"只有 [`face_force`]"的旧行为逐位相同
//!    ⇒ 升力开关是精确的。
use vxl_phys_aero::AeroConfig;
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::cloth_aero::ClothAero;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const SIZE: f32 = 0.5;
const W: f32 = 10.0;
/// 迎角（风与**面内**方向的夹角；`φ = 30°` ⇒ 迎角 60°？——不：这里 `sinα = n·û = sinφ`，
/// 迎角即 `φ` 的正弦口径，闭式里只出现 `sinφ/cosφ` ⇒ 记号自洽即可）。
const PHI: f32 = core::f32::consts::FRAC_PI_6; // 30°

/// 平铺网格（`2×2` 格 ⇒ 总面积 `1 m²`、法线 `Y`）。
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

/// 自由平铺布片（不钉、单子步）+ 风向 `(cosφ, sinφ, 0)·W`、`lift_slope` 可设。
fn sheet_at(phi: f32, lift_slope: f32) -> ClothSheet {
    let (pts, tris) = plate(2, SIZE);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.substeps = 1;
    s.aero = ClothAero {
        enabled: true,
        cfg: AeroConfig {
            wind: [W * phi.cos(), W * phi.sin(), 0.0],
            lift_slope,
            ..AeroConfig::default()
        },
        ..ClothAero::default()
    };
    s
}

/// 单子步后的总动量 `Σ m·v`（t = 0 时 `u = wind` 精确 ⇒ 与质量分布无关）。
fn momentum(s: &mut ClothSheet) -> Vec3 {
    s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let mut p = Vec3::ZERO;
    for i in 0..s.pos.len() {
        p += s.vel[i] * s.mass[i];
    }
    p
}

/// 闭式：`½ρ·A·W²·[Cd·û + ls·sinφ·cosφ·d̂]·h`（`d̂ = (−sinφ, cosφ, 0)`）。
fn analytic(phi: f32, lift_slope: f32) -> Vec3 {
    let cfg = AeroConfig::default();
    let a = (2.0 * SIZE) * (2.0 * SIZE);
    let k = 0.5 * cfg.air_density * a * W * W * DT;
    let drag = Vec3::new(phi.cos(), phi.sin(), 0.0) * cfg.drag_coefficient;
    let lift = Vec3::new(-phi.sin(), phi.cos(), 0.0) * (lift_slope * phi.sin() * phi.cos());
    (drag + lift) * k
}

/// ① **解析对拍**：动量的两个非零分量都对该上闭式（同族判据 ① 的口径，相对差 ≤ 1e-5）。
#[test]
fn lift_momentum_matches_the_closed_form() {
    let mut s = sheet_at(PHI, AeroConfig::default().lift_slope);
    let p = momentum(&mut s);
    let want = analytic(PHI, AeroConfig::default().lift_slope);
    let ex = (p.x - want.x).abs() / want.x.abs().max(1e-9);
    let ey = (p.y - want.y).abs() / want.y.abs().max(1e-9);
    println!(
        "[判据①升力解析] Σ m·v = ({:+.5}, {:+.5}, {:+.5})；闭式 = ({:+.5}, {:+.5}, {:+.5})\
         （相对差 x {:.1e} / y {:.1e}）",
        p.x, p.y, p.z, want.x, want.y, want.z, ex, ey
    );
    assert!(p.z == 0.0, "面外动量该严格为 0（实测 {:+.8}）", p.z);
    assert!(ex < 1e-5 && ey < 1e-5, "升力+阻力的动量该对上闭式");
}

/// ② **符号反对称**（双面口径）：`φ → −φ` ⇒ 升力 `y` 分量反号、阻力 `x` 分量不变。
#[test]
fn lift_is_antisymmetric_across_the_sheet() {
    let mut sp = sheet_at(PHI, 5.0);
    let pp = momentum(&mut sp);
    let mut sm = sheet_at(-PHI, 5.0);
    let pm = momentum(&mut sm);
    println!(
        "[判据②反对称] φ=+30°：({:+.5}, {:+.5})；φ=−30°：({:+.5}, {:+.5})",
        pp.x, pp.y, pm.x, pm.y
    );
    assert!(
        (pm.y + pp.y).abs() < pp.y.abs() * 1e-4,
        "升力分量该**反号**（+φ 给 {:+.5}，−φ 给 {:+.5}）",
        pp.y,
        pm.y
    );
    assert!(
        (pm.x - pp.x).abs() < pp.x.abs() * 1e-4,
        "阻力分量该**不变**（+φ 给 {:+.5}，−φ 给 {:+.5}）",
        pp.x,
        pm.x
    );
}

/// ③ **关升力 = 纯阻力**：`lift_slope = 0` 与"没接升力那一版"（纯 [`vxl_phys_aero::face_force`]）
/// 逐位相同 ⇒ 升力开关是精确的（同样保护"判据④收敛"那类无升力工况）。
#[test]
fn zero_lift_slope_is_pure_drag() {
    let mut s = sheet_at(PHI, 0.0);
    let p = momentum(&mut s);
    // 纯阻力闭式（`cloth_aero.rs` 判据②同族，但风在 `x–y` 面内 ⇒ 动量沿 `û`）
    let cfg = AeroConfig::default();
    let a = (2.0 * SIZE) * (2.0 * SIZE);
    let want = Vec3::new(PHI.cos(), PHI.sin(), 0.0)
        * (0.5 * cfg.air_density * cfg.drag_coefficient * a * W * W * DT);
    println!(
        "[判据③关升力] Σ m·v = ({:+.6}, {:+.6})；纯阻力闭式 = ({:+.6}, {:+.6})",
        p.x, p.y, want.x, want.y
    );
    assert!(
        (p.x - want.x).abs() < want.x.abs() * 1e-5 && (p.y - want.y).abs() < want.y.abs() * 1e-5,
        "`lift_slope = 0` 该退化为**纯阻力**（实测 ({:+.5}, {:+.5})，闭式 ({:+.5}, {:+.5})）",
        p.x,
        p.y,
        want.x,
        want.y
    );
}
