//! **判据 ②：自碰撞不许自穿**（切片 T3，`PLAN-triangle-first-class.md` 判据 ②）。
//!
//! **场景 = 把布片"折成两半"**（1.0 m 见方、8×8 格、壳厚 0.01 m）：
//! - **折线** = `x = 0`（沿 `z`），**折痕那一列钉住**（夹住折痕 ⇒ 无刚体漂移、结果可复现）；
//! - **驱动** = 两半绕折线**反向**的角速度场 `v = s·ω·ẑ × r`（`s = sign(x)`）⇒ 两半**相向合拢**；
//! - **无重力**（本片只测自碰撞，不叠载荷）。
//!
//! **判据（全部机器无关）**：
//! ① **不穿模（主判据，也是金丝雀的判据）**：稳态窗内**跨半区、非网格邻居**的粒子最小间距
//!    `min_sep > 0`，且 ≥ 碰撞距离的一半（留"单遍 Gauss-Seidel"的余量）。实测：**ON = `d_c`
//!    逐位对上、OFF = 0.35·`d_c`** ⇒ 分辨力就在这一条上。
//! ② **不越界（补充不变量，**不是**金丝雀）**：两半的**楔角** `θ_A − θ_B > 0`（`θ` = 绕折线的
//!    极角）—— 反号就是两半对穿（∧ 变 ∨）。⚠️ 本条**测不出**本场景的失效模式：驱动按**瞬时**
//!    `sign(x)` 取，粒子一越过折线就自刹车（见 `drive` 的注），所以两端楔角都 > 0；
//!    留着它是为了兜住"整体被推穿"这类粗失效（那种情况下 ① 也一定红）。
//! ③ **解析对拍**：折合停在**楔角 ≈ `d_c / r_tip`**（`r_tip = 0.5` ⇒ **0.02 rad**）——
//!    实测 **0.0245 rad**（同量级）；这条把"碰撞距离是不是真的壳厚"钉住，防"半径设错也能过"。
//! ④ **金丝雀**：`cfg.enabled = false` ⇒ 判据 ①（与 ②③）**必须红**；没有它，"不穿模"完全
//!    可能是被约束/驱动凑出来的。
//!
//! **为什么关弯曲**（`bend_compliance = ∞`）：弯曲约束是**二环对距离约束**，它在折痕处**天然抵抗
//! 折叠**（平铺时二环对相距 0.125，折 180° 时该距离 → 0）⇒ 开着它测的是"弯曲 vs 自碰撞"的混合
//! 效应，不是自碰撞本身。主判据关弯曲（**一次只测一个变量**）；开着弯曲的读数另作对照打印。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
/// 格数（`9×9 = 81` 粒、间距 0.125 m）与总跨度（`x, z ∈ ±0.5`）。
const N: usize = 8;
/// **驱动角速度**（rad/s）：240 tick = 4 s ⇒ 4 rad ≈ 229°（不接触时足够折过头，用于压出穿模）。
const OMEGA: f32 = 1.0;
/// 稳态窗（后 60 tick）与总时长。
const WIN: usize = 60;
const TICKS: usize = 240;

/// 平铺网格（行主序：`z` 外层、`x` 内层）。
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

/// 折痕那一列的粒子号（`x == 0`）。
fn crease_row(n: usize) -> Vec<usize> {
    let w = n + 1;
    (0..=n).map(|iz| iz * w + (n / 2)).collect()
}

/// 平铺布片；`bending` = 是否保留弯曲约束（见文末判据注）。
fn sheet(bending: bool) -> ClothSheet {
    let (pts, tris) = plate(N, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    if !bending {
        s.bend_compliance = f32::INFINITY; // 关弯曲（非有限 ⇒ 显式短路）
    }
    s.damping = 0.999;
    for i in crease_row(N) {
        s.set_pinned(i, true); // 夹住折痕：无刚体漂移 + 折线固定
    }
    s
}

/// **驱动**：两半绕折线（`x = 0`、沿 `z`）**反向**的角速度场 ⇒ 相向合拢（每 tick 重设 ⇒ 定速驱动）。
///
/// ⚠️ **半区按"瞬时" `sign(x)` 取，是刻意的**（试过按初始半区定死，**反而更差**，留档）：
/// 定死 ⇒ 两半会**转过**楔角 0；而在楔角 0 附近两半的粒子近**同心**、推出方向退化
/// （`len < 1e-9` 被跳过）⇒ 单遍位置投影拦不住 ⇒ **ON 与 OFF 都会穿过去**（实测两端末态楔角
/// 都到 −1.8 rad、且最小间距反而是 0.21 —— 判据失去分辨力）。按瞬时符号取 ⇒ 粒子一越过折线
/// 驱动**自刹车** ⇒ 折合停在"压到穿不动"的位置：**ON 停在 `d_c`、OFF 停在压穿 35%**。
fn drive(s: &mut ClothSheet, omega: f32) {
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        let p = s.pos[i];
        if p.x.abs() < 1e-6 {
            continue; // 折线上的粒子（已钉住）
        }
        let sgn = if p.x > 0.0 { 1.0 } else { -1.0 };
        let r = Vec3::new(p.x, p.y, 0.0); // 相对折线
        s.vel[i] = Vec3::new(-r.y, r.x, 0.0) * (sgn * omega);
    }
}

/// **楔角**：`(A 半区均值, B 半区均值)`，`θ = atan2(y, x)`（绕折线）。
/// A = `x < 0`（初始 `θ ≈ π`）、B = `x > 0`（初始 `θ ≈ 0`）；合拢时两者相向趋近 `π/2`。
fn wedge(s: &ClothSheet) -> (f32, f32) {
    let (mut na, mut nb) = (0.0f32, 0.0f32);
    let (mut sa, mut sb) = (0.0f32, 0.0f32);
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        let p = s.pos[i];
        if p.x < -1e-6 {
            na += 1.0;
            sa += p.y.atan2(p.x);
        } else if p.x > 1e-6 {
            nb += 1.0;
            sb += p.y.atan2(p.x);
        }
    }
    (sa / na.max(1.0), sb / nb.max(1.0))
}

/// **跨半区、非网格邻居**的最小间距（`is_forbidden` = 与自碰撞同一份排除集 ⇒ **同口径**）。
fn min_sep(s: &ClothSheet) -> f32 {
    let mut best = f32::MAX;
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        for j in 0..s.pos.len() {
            if i == j || s.inv_mass[j] == 0.0 {
                continue;
            }
            // 只取跨半区对（同一半区内部的最近对由网格自身决定，不是"两层"的量）
            let cross = (s.pos[i].x < 0.0) != (s.pos[j].x < 0.0);
            if !cross || s.self_contacts.is_forbidden(i as u32, j as u32) {
                continue;
            }
            best = best.min((s.pos[j] - s.pos[i]).length());
        }
    }
    best
}

/// 一次折合跑完的读数（判据量取**稳态窗**）。
struct Read {
    /// 稳态窗内最小间距的**最小值**（判据 ① 取它）。
    sep_min: f32,
    /// 稳态窗内楔角 `θ_A − θ_B` 的**最小值**（判据 ② 取它）。
    wedge_min: f32,
    /// 末态楔角与末态最小间距（打印用）。
    wedge_end: f32,
    sep_end: f32,
    /// 稳态窗内单子步解算对数的最小/最大（判"每步都在传"还是"偶发"）。
    pairs_lo: u32,
    pairs_hi: u32,
    /// 壳厚 ⇒ 碰撞距离 `2·particle_radius`（判据阈值的参照）。
    d_c: f32,
    finite: bool,
}

/// 折一次：`enabled = false` ⇒ **金丝雀**。
fn run(enabled: bool, bending: bool) -> Read {
    let mut s = sheet(bending);
    s.self_contacts.cfg.enabled = enabled;
    let d_c = 2.0 * s.self_contacts.cfg.particle_radius;
    let mut r = Read {
        sep_min: f32::MAX,
        wedge_min: f32::MAX,
        wedge_end: 0.0,
        sep_end: 0.0,
        pairs_lo: u32::MAX,
        pairs_hi: 0,
        d_c,
        finite: true,
    };
    for t in 0..TICKS {
        drive(&mut s, OMEGA);
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        if t >= TICKS - WIN {
            let (a, b) = wedge(&s);
            r.sep_min = r.sep_min.min(min_sep(&s));
            r.wedge_min = r.wedge_min.min(a - b);
            let p = s.self_contacts.pairs;
            r.pairs_lo = r.pairs_lo.min(p);
            r.pairs_hi = r.pairs_hi.max(p);
        }
    }
    let (a, b) = wedge(&s);
    r.wedge_end = a - b;
    r.sep_end = min_sep(&s);
    r.finite = s.pos.iter().all(|p| p.is_finite());
    r
}

/// **判据本体**：不穿模（`sep_min` 不小于碰撞距离的一半）+ 不越界（楔角不反号）。
fn held(r: &Read) -> bool {
    r.finite && r.sep_min > 0.0 && r.sep_min >= 0.5 * r.d_c && r.wedge_min > 0.0
}

fn report(tag: &str, r: &Read) {
    println!(
        "  {tag:<26} 碰撞距离 d_c={:.4} | 稳态：最小间距 {:.5}（阈 {:.5}）| 楔角 min {:.4} rad\
         （末 {:.4}）| 解算对数 ∈ [{}, {}] ⇒ {}",
        r.d_c,
        r.sep_min,
        0.5 * r.d_c,
        r.wedge_min,
        r.wedge_end,
        r.pairs_lo,
        r.pairs_hi,
        if held(r) {
            "不穿模 ✅"
        } else {
            "**穿模 ❌**"
        }
    );
}

#[test]
fn folded_cloth_does_not_self_penetrate() {
    println!(
        "[判据② 自碰撞·折两半] 8×8 格 / 壳厚 0.01 / 折痕钉住 / 角速度驱动 ω={OMEGA} rad/s / 关弯曲"
    );
    let r = run(true, false);
    report("自碰撞 ON", &r);
    assert!(r.finite, "折合过程出现非有限值（NaN/inf）");
    assert!(
        r.sep_min > 0.0,
        "跨半区非邻居粒子的最小间距该 **> 0**（实测 {:.6}）——红了说明两层已互穿",
        r.sep_min
    );
    assert!(
        r.sep_min >= 0.5 * r.d_c,
        "最小间距 {:.5} 该 ≥ 碰撞距离的一半 {:.5}（单遍 Gauss-Seidel 留一半余量）——\
         红了说明自碰撞没顶住连续压合",
        r.sep_min,
        0.5 * r.d_c
    );
    assert!(
        r.wedge_min > 0.0,
        "楔角（θ_A − θ_B）该 **> 0**（实测最小 {:.5} rad）——反号就是两半**已对穿**（∧ 变 ∨）",
        r.wedge_min
    );
    // **判据③ 解析对拍**：折合停在楔角 ≈ `d_c / r_tip`（`r_tip = 0.5 m` ⇒ 0.02 rad）。
    // 这条防的是"把 particle_radius 设错也能过"——比如半径取 1e-6 时 ① 与 ② 都仍然会过。
    let wedge_want = r.d_c / 0.5;
    assert!(
        r.wedge_min > 0.5 * wedge_want && r.wedge_min < 3.0 * wedge_want,
        "折合停角 {:.5} rad 该与解析 `d_c/r_tip = {:.5}` 同量级（0.5×–3×）——\
         差了就是碰撞距离的口径不对（`particle_radius` 不是壳厚一半）",
        r.wedge_min,
        wedge_want
    );
    // **判据④ 金丝雀的证据腿**：自碰撞必须**每 tick 都在传**（不是偶发触发才勉强过阈值）。
    assert!(
        r.pairs_lo > 0,
        "稳态窗内每个子步都该有解算对（实测最少 {} 对）——0 说明自碰撞根本没在跑，\
         ① 的读数就不是它的功劳",
        r.pairs_lo
    );
}

/// **金丝雀**：关掉自碰撞 ⇒ 同样两枚判据**必须红**（否则判据没有分辨力）。
#[test]
fn canary_without_self_collision_the_fold_passes_through() {
    println!("[金丝雀③] 同一场景、`cfg.enabled = false`：");
    let r = run(false, false);
    report("自碰撞 OFF（金丝雀）", &r);
    assert!(
        !held(&r),
        "金丝雀本该穿模（实测最小间距 {:.6}、楔角 {:.5}）——仍然「不穿模」 ⇒ 判据没有分辨力",
        r.sep_min,
        r.wedge_min
    );
}

/// **对照读数（不判红）**：保留弯曲约束（默认档）时的折合 —— 弯曲弹簧会抵抗折痕，
/// 本条的用途是"看两个机制叠加时会怎样"（`bend_compliance = ∞` 的显式短路也顺带被走过）。
#[test]
fn fold_with_bending_enabled_reference_reading() {
    println!("[对照] 保留弯曲约束（`bend_compliance = Soft`）：");
    let r = run(true, true);
    report("自碰撞 ON + 弯曲 ON", &r);
    assert!(r.finite, "折合过程出现非有限值（NaN/inf）");
}
