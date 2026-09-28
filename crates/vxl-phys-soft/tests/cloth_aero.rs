//! **气动消费判据**（切片 T4 收尾）：`ClothSheet` 消费 `vxl_phys_aero::face_force`
//! （Bridson 线化面元力 `F = ½·ρ·Cd·A·u·|u|`，`u = v_wind − v_face_center`）。
//!
//! **判据（全部机器无关）**：
//! ① **解析对拍（动量口径）**：自由平铺布片（1.0 m²、**不钉**、无重力）+ 法向风 ⇒ 单子步后
//!    **总动量** `Σ m_i v_i = ½·ρ·Cd·|w|²·A·h`。用**动量**而不是"某个点的速度"，是因为
//!    **解析式与质量分布无关**（`Σ F/3·h` 与顶点质量无关）⇒ 判据不依赖"质量怎么摊"。
//! ② **面内风也在推（全相对速度口径）**：片在 `x–z` 平面（法向 `Y`）、风沿 `+Z` ⇒ 力沿 `+Z`、
//!    `Y` 方向**逐位零**。⚠️ 这条**有分辨力**：若口径只取"法向分量"（很多文献的写法），
//!    面内风会给出**零力**，与 `F ∥ u` 差得很远。
//! ③ **零风金丝雀**：`wind = 0` ⇒ 与"气动关"**逐位相同**（`face_force(0) = 0` 逐位成立）。
//! ④ **收敛朝向风速**（长窗读数）：自由片在风里加速、速度**有界且单调**逼近风速
//!    （`u = w − v` ⇒ 力随 `v` 衰减）。
use vxl_phys_aero::AeroConfig;
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::cloth_aero::ClothAero;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const N: usize = 2;
const SIZE: f32 = 0.5;
const RHO: f32 = 1000.0;
const THICK: f32 = 0.01;
const WIND: f32 = 10.0;

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 `y = 0` 平面）。
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

/// 自由平铺布片：**不钉**、**单子步**（⇒ 一 tick 恰好一次气动施加 ⇒ 解析对拍干净）。
fn free_sheet(wind: [f32; 3], enabled: bool) -> ClothSheet {
    let (pts, tris) = plate(N, SIZE);
    let mut s = ClothSheet::new(pts, tris, RHO, THICK, Stiffness::Hard);
    s.substeps = 1;
    s.aero = ClothAero {
        enabled,
        cfg: AeroConfig {
            wind,
            ..AeroConfig::default()
        },
    };
    s
}

/// **总动量** `Σ m_i v_i`（与质量分布无关的那条量；`m_i` = 薄壳均分质量）。
fn momentum(s: &ClothSheet) -> Vec3 {
    let mut p = Vec3::ZERO;
    for i in 0..s.pos.len() {
        p += s.vel[i] * s.mass[i];
    }
    p
}

/// 解析式 `½·ρ·Cd·|w|²·A·h`（`A` = 整片面积 = `(2·size)²`）。
fn analytic(wind: f32) -> f32 {
    let cfg = AeroConfig::default();
    0.5 * cfg.air_density * cfg.drag_coefficient * (2.0 * SIZE).powi(2) * wind * wind * DT
}

fn bits_eq(p: Vec3, q: Vec3) -> bool {
    p.x.to_bits() == q.x.to_bits()
        && p.y.to_bits() == q.y.to_bits()
        && p.z.to_bits() == q.z.to_bits()
}

/// ① **解析对拍（动量口径）**：法向风（沿 `+Y`，片在 `x–z` 平面 ⇒ 面元 ⊥ 风）⇒
/// 平板阻力解析式 `½ρv²A·Cd` 的**动量**必须逐项对上，且横向动量**严格为 0**。
#[test]
fn wind_momentum_matches_the_analytic_drag() {
    let mut s = free_sheet([0.0, WIND, 0.0], true);
    s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let p = momentum(&s);
    let want = analytic(WIND);
    println!(
        "[判据①解析对拍] 单子步后 Σ m·v = ({:+.8}, {:+.8}, {:+.8})；解析 = {:+.8}（相对差 {:.2e}）",
        p.x,
        p.y,
        p.z,
        want,
        (p.y - want).abs() / want
    );
    assert!(
        (p.y - want).abs() < want * 1e-5,
        "法向风的动量该 = ½ρCd|w|²A·h = {want:.8}（实测 {:.8}，相对差 {:.2e}）——\
         红了说明面积/密度/Cd/步长的口径错了",
        p.y,
        (p.y - want).abs() / want
    );
    assert!(
        p.x == 0.0 && p.z == 0.0,
        "法向风的横向动量该**严格为 0**（实测 ({:+.8}, {:+.8})）",
        p.x,
        p.z
    );
}

/// ② **面内风也在推（全相对速度口径）**：风沿 `+Z`（**与片平行**）⇒ 力仍沿 `+Z`、`Y` 严格 0。
#[test]
fn in_plane_wind_pushes_along_the_wind_not_the_normal() {
    let mut s = free_sheet([0.0, 0.0, WIND], true);
    s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let p = momentum(&s);
    let want = analytic(WIND);
    println!(
        "[判据②面内风] Σ m·v = ({:+.8}, {:+.8}, {:+.8})；解析 = {:+.8}（法向 Y 该严格 0）",
        p.x, p.y, p.z, want
    );
    assert!(
        (p.z - want).abs() < want * 1e-5,
        "面内风该沿风向推（`F ∥ u`，Bridson 线的**全相对速度**口径）：该 {want:.8}，实测 {:.8}\
         ——若这里 ≈ 0，说明口径被改成了「只取法向分量」",
        p.z
    );
    assert!(p.y == 0.0, "面内风不该产生法向动量（实测 y={:+.8}）", p.y);
}

/// ③ **零风金丝雀**：`wind = 0` **且无运动**（无重力）⇒ 与"气动关闭"**逐位相同**。
///
/// ⚠️ **"零风 ≠ 无气动力"**（本判据首版就栽在这条上，留档）：相对风是 `u = w − v` ——
/// **一旦有运动，`u = −v ≠ 0`**，这正是**气动阻力**的本体（下落中的片会自己吃到阻力）。
/// 所以金丝雀必须把"无运动"一起钉住（重力 = 0），否则测到的不是"口径为 0"，而是"阻力在起作用"。
#[test]
fn zero_wind_at_rest_is_bitwise_identical_to_aero_off() {
    let still = Vec3::ZERO; // 无重力 ⇒ 无运动 ⇒ `u ≡ 0`
    let mut on = free_sheet([0.0; 3], true);
    let mut off = free_sheet([0.0; 3], false);
    for _ in 0..120 {
        on.step(DT, still, &NoProviders, 0, &[]);
        off.step(DT, still, &NoProviders, 0, &[]);
    }
    let pos_same = on.pos.iter().zip(&off.pos).all(|(a, b)| bits_eq(*a, *b));
    let vel_same = on.vel.iter().zip(&off.vel).all(|(a, b)| bits_eq(*a, *b));
    println!(
        "[判据③零风金丝雀] 120 tick 逐位比较：pos {} / vel {}（中心 y = {:+.6}）",
        if pos_same {
            "相同 ✅"
        } else {
            "**不同 ❌**"
        },
        if vel_same {
            "相同 ✅"
        } else {
            "**不同 ❌**"
        },
        on.pos[4].y
    );
    assert!(
        pos_same && vel_same,
        "零风该与气动关闭**逐位相同**（pos_same={pos_same} / vel_same={vel_same}）——\
         红了说明 `face_force(0)` 不再恰好是 0，或 `enabled` 短路漏了"
    );
}

/// ④ **长窗读数**：自由片在风里加速；速度**有界**（< 风速）且**单调增**（`u = w − v` 衰减）。
#[test]
fn sheet_accelerates_toward_the_wind_speed() {
    let mut s = free_sheet([0.0, WIND, 0.0], true);
    let m: f32 = s.mass.iter().sum();
    let vel = |s: &ClothSheet| momentum(s).y / m;
    let mut prev = 0.0f32;
    let mut monotone = true;
    for _ in 0..600 {
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        let v = vel(&s);
        if v < prev {
            monotone = false;
        }
        prev = v;
    }
    println!(
        "[判据④收敛] 600 tick（10 s）末平均速度 = {prev:+.4} m/s（风速 {WIND}）| 单调增 = {monotone}"
    );
    assert!(s.pos.iter().all(|p| p.is_finite()), "出现非有限值");
    assert!(
        prev > 0.0 && prev < WIND,
        "速度该为正且**有界于风速**（实测 {prev:+.4}）——`u = w − v` ⇒ 力随 v 衰减，不该越过风速"
    );
    assert!(monotone, "速度该**单调增**（阻力式只朝风速逼近，不该回头）");
}
