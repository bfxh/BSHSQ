//! **A9a：Fresnel 积分 + 刀刃（半平面）绕射** —— A4 的 `D(θ) = (1 + cosθ)/2` 是**工程近似**
//! （只谈「拐角后仍可闻」的**单调**性）；这里是**真系数**：Fresnel 积分 `C/S` + **逐频带**波长依赖，
//! 落地 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9 第 2 条（`Fresnel 积分`）。
//!
//! ## 口径一：绝对频率**仍由宿主显式声明**（crate 头注那条不破）
//! [`BANDS`](crate::BANDS) 头注明写「本层**不给绝对频率** —— Hz 分组由宿主定」。本模块不推翻它：
//! [`AcousticConfig::band_hz`](crate::AcousticConfig::band_hz) 默认 `[0.0; BANDS]` = **无绝对频率**
//! ⇒ 逐带退回 A4 旧口径（**逐位不变**）；要真 Fresnel 就**显式声明频率**（survey §6：实时档要显式声明）。
//!
//! ## 口径二：频域只承诺**高频可闻**（survey §7 已声明不承诺低频）
//! 几何声学在低频失效（波长与几何尺度可比时不给干涉/驻波）。`band_hz[b]` 取到几百 Hz 以下时
//! 结果**只保证工程可信**，不做波动求解。
//!
//! ## 口径三：`ν` 由**多余路程**定（3D、含斜掠、量纲干净）
//! `ν = 2·√(Δ/λ)`，`Δ = |声源−棱点| + |听者−棱点| − |声源−听者|` = 折线超出直连的**多余路程**
//! （`Δ ≥ 0`，正是 A4 已经在算的那条 Fermat 折线）；`λ = c / band_hz[b]`。
//! 与经典**刀口**口径 `ν = h·√(2(s+s')/(λ·s·s'))` 等价（`Δ = h²(s+s')/(2·s·s')` 时两者相同，
//! 判据 `tests/a9_utd.rs` 用手算几何对拍）。这样 `ν` 直接吃 3D 几何，斜掠（`β₀ ≠ π/2`）自带。
//!
//! ## 求值口径（**全域 f32**，SPEC §5「严格 f32」；两段式）
//! `C/S` 的幂级数 `Σ (−1)^k (π/2)^{2k} x^{4k+1}/((4k+1)(2k)!)` 对**任意** `x` 都收敛，但项在
//! `k ≈ πx²/4` 附近涨到 ~`e^{πx²/2}` 再相消 ⇒ `f32` 到 `x ≳ 2` 就只剩两三位、`x ≳ 4` 直接溢出。
//! 故分两段（**都在 f32 算**，实测最坏误差 ~`1.4e-5`）：
//! · `x < 2.5` ⇒ **幂级数**（项数 = `⌊πx²/2⌋ + 64`，无早退、逐位可复现）；
//! · `x ≥ 2.5` ⇒ **渐近展开**（分部积分得到的 `½−C`、`½−S` 的辅助级数），**取到最小项为止**
//!   （渐近级数发散 ⇒ 不能让项反弹）。
//! `2.5` 是实测两段误差交叉点（`2.5` 处 series `1.4e-5` / asym `6.9e-6`；往两边任一分支都更差）。
//! 判据：`tests/a9_utd.rs`（表值 / 导数据恒等式 / 与**独立 Simpson 积分**逐点对拍）。
//!
//! ## 边界（写清，不是漏）
//! · **A9a 只做刀刃（半平面，`n = 2`）**：墙沿 / 屏障顶这类「薄边」就是它。**A9b** 补上**一般
//!   楔角 `n`**（直角墙角之类）：给 `Edge` 引 `wedge: Option<[`Wedge`]>`，按 KP 式算楔角系数，
//!   再以**刀刃修正因子** `fac = min(|D_n|²/|D_2|², [`WEDGE_FACTOR_MAX`])` 乘到 A9a 上 ——
//!   `n = 2` ⇒ `fac = 1`（**逐位回到 A9a**）；`n → 1`（平壁）⇒ `fac → 0`。非破坏（`None` = 刀口）。
//! · **阴影边界值变了**：旧口径在掠射（刚被挡）时 `D → 1`；真 Fresnel 在阴影边界是 `|E/E₀| = 1/2`
//!   （**恰好 −6.02 dB**，经典的「半场」）。这是**口径修正**、不是回归。

use core::f32::consts::{FRAC_PI_4, PI};
use vxl_phys::Vec3;

/// 幂级数项数上限（`x < 2.5` 时用不到这么多；防病态入参把循环拉长）。
const MAX_TERMS: u32 = 4096;
/// 渐近级数项数上限（**取到最小项为止**，正常 `x ≥ 2.5` 只需 ~6 项）。
const MAX_ASYM: u32 = 64;
/// 幂级数 / 渐近的交接点（见模块头注「求值口径」；实测定为 `2.5`）。
const CROSSOVER: f32 = 2.5;

/// Fresnel 积分 `C(x) = ∫₀ˣ cos(πt²/2)dt`、`S(x) = ∫₀ˣ sin(πt²/2)dt`（`x` 任意实号）。
///
/// 奇对称：`C(−x) = −C(x)`、`S(−x) = −S(x)`（判据钉住）。两段式求值见模块头注。
pub fn fresnel_cs(x: f32) -> (f32, f32) {
    if x < 0.0 {
        let (c, s) = fresnel_cs(-x);
        (-c, -s)
    } else {
        fresnel_cs_nonneg(x)
    }
}

/// `x ≥ 0`：`x < CROSSOVER` 走幂级数，否则走渐近。
fn fresnel_cs_nonneg(x: f32) -> (f32, f32) {
    if x == 0.0 {
        (0.0, 0.0)
    } else if x < CROSSOVER {
        series(x)
    } else {
        asymptotic(x)
    }
}

/// 幂级数：`C = Σ (−1)^k (π/2)^{2k} x^{4k+1} / ((4k+1)(2k)!)`，`S` 同构。
fn series(x: f32) -> (f32, f32) {
    let p = 0.5 * PI;
    let x2 = x * x;
    let fac = p * p * x2 * x2; // (π/2)² x⁴
    let terms = ((p * x2) as u32).saturating_add(64).min(MAX_TERMS);
    // base_c(k) = (π/2)^{2k} x^{4k+1} / (2k)!     ⇒ C 的第 k 项 = base_c/(4k+1)
    // base_s(k) = (π/2)^{2k+1} x^{4k+3} / (2k+1)! ⇒ S 的第 k 项 = base_s/(4k+3)
    let mut base_c = x;
    let mut base_s = p * x * x2;
    let (mut c, mut s) = (0.0f32, 0.0f32);
    for k in 0..terms {
        let kf = k as f32;
        c += base_c / (4.0 * kf + 1.0);
        s += base_s / (4.0 * kf + 3.0);
        base_c *= -fac / ((2.0 * kf + 1.0) * (2.0 * kf + 2.0));
        base_s *= -fac / ((2.0 * kf + 2.0) * (2.0 * kf + 3.0));
    }
    (c, s)
}

/// 渐近展开（`x ≥ CROSSOVER`）：由 `½ − C = ∫ₓ^∞ cos` 分部积分得
/// `½ − C = −s₂·A + c₂·B`、`½ − S = c₂·A + s₂·B`，其中 `s₂ = sin(πx²/2)`、`c₂ = cos(πx²/2)`、
/// `A = (1/πx)·Σ (−1)^m (4m−1)!! (πx²)^{−2m}`、`B = (1/πx)·Σ (−1)^m (4m+1)!! (πx²)^{−2m−1}`。
///
/// 实现注意：`(4m−1)!!` 与 `(πx²)^{−2m}` 单独算会**溢出 / 下溢 `f32`**（两者乘积才是有界的项）。
/// 故按**项比**递推（`tA₀ = 1`、`tB₀ = t`）：
/// `tAₘ = tAₘ₋₁·(−(4m−1)(4m−3)·t²)`、`tBₘ = tBₘ₋₁·(−(4m−1)(4m+1)·t²)`，`t = (πx²)^{−1}`。
/// 渐近级数**发散** ⇒ 取到最小项即止（`|项|` 反弹就停）。
fn asymptotic(x: f32) -> (f32, f32) {
    let u = 1.0 / (PI * x);
    let t = 1.0 / (PI * x * x);
    let t2 = t * t;
    let a2 = 0.5 * PI * x * x;
    let (s2, c2) = (a2.sin(), a2.cos());
    let (mut a_sum, mut b_sum) = (0.0f32, 0.0f32);
    let (mut ta, mut tb) = (1.0f32, t); // m = 0
    let mut prev = f32::INFINITY;
    let mut m = 0u32;
    loop {
        let mag = ta.abs().max(tb.abs());
        if mag > prev || m >= MAX_ASYM {
            break;
        }
        prev = mag;
        a_sum += ta;
        b_sum += tb;
        let mm = (m + 1) as f32;
        ta *= -(4.0 * mm - 1.0) * (4.0 * mm - 3.0) * t2;
        tb *= -(4.0 * mm - 1.0) * (4.0 * mm + 1.0) * t2;
        m += 1;
    }
    let (a, b) = (u * a_sum, u * b_sum);
    (0.5 - (-s2 * a + c2 * b), 0.5 - (c2 * a + s2 * b))
}

/// Fresnel 参数 `ν = 2·√(Δ/λ)`（`Δ` = **多余路程**、`λ` = 波长）；`Δ ≤ 0` 或 `λ ≤ 0` ⇒ `0`。
pub fn fresnel_nu(excess_path: f32, wavelength: f32) -> f32 {
    if wavelength <= 0.0 {
        return 0.0;
    }
    2.0 * (excess_path.max(0.0) / wavelength).sqrt()
}

/// 波长 `λ = c / hz`（`hz ≤ 0` ⇒ `0` = 「没有频率」哨兵，调用侧据此退回旧口径）。
pub fn wavelength(speed_of_sound: f32, hz: f32) -> f32 {
    if hz <= 0.0 {
        0.0
    } else {
        speed_of_sound.max(1e-3) / hz
    }
}

/// 刀刃（半平面）绕射的**能量比** `|E_d/E₀|²`（相对自由场；值域 `(0, 1]`）。
///
/// 经典 Fresnel–Kirchhoff 闭式：
/// `E_d/E₀ = ((1+j)/2)·[(½ − C(ν)) − j(½ − S(ν))]` ⇒ 能量比 `= ½·[(½ − C(ν))² + (½ − S(ν))²]`。
///
/// 端点（判据钉死）：`ν = 0`（阴影边界）⇒ **`1/4`（恰好 −6.02 dB）**；`ν → −∞`（深处通视）⇒ `1`；
/// `ν → +∞`（深处阴影）⇒ `0`（衰减 ~ `1/(2π²ν²)`）。
pub fn knife_gain(nu: f32) -> f32 {
    let (c, s) = fresnel_cs(nu);
    let (a, b) = (0.5 - c, 0.5 - s);
    0.5 * (a * a + b * b)
}

// ---------------------------------------------------------------------------
// A9b：一般楔角 UTD 系数（Kouyoumjian–Pathak）+ 刀刃修正因子
// ---------------------------------------------------------------------------

/// **A9b：一般楔角**的几何（挂在 [`Edge::wedge`](crate::Edge::wedge) 上）。
///
/// 绕棱在 **⊥ 棱**平面里量角：`face0` 是**场区一侧**的一条棱面方向（量角基准，角度 `0`）；
/// 另一条棱面在 `b_axis = edge_dir × a_axis` 正向转过 `n·π`（`a_axis` = `face0` 在 ⊥ 棱平面内的
/// 单位投影，`n = 2 − Ω/π`）。**场区** = `[0, n·π]`，**楔体（材料）** = 余下 `[n·π, 2π]`，
/// 内角 `Ω = (2−n)·π`。声源 / 听者应落在场区。
///
/// `Ω = 0`（无材料）⇒ `n = 2` = **刀口** ⇒ 修正因子恒 `1`（**逐位回到 A9a**）；
/// `Ω = π`（平壁）⇒ `n = 1` ⇒ 系数 `→ 0`（无绕射）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wedge {
    /// 楔体**内角** `Ω ∈ [0, π]`（弧度；越界按端点钳）。
    pub interior_angle: f32,
    /// **面 0 方向**（自棱点指向场区一侧；不必 ⊥ 棱，用时会投影到 ⊥ 棱平面）。
    pub face0: Vec3,
}

/// [`WedgeFrame::factor`] 的**钳位上界**（A9b 工程口径）。
///
/// 系数比 `|D_n|²/|D_2|²` 在**参考刀刃系数趋零**处会发散（实测中位数 ~`1`、长尾到 `1e6`）；
/// 本层按**分母地板 + 钳位**处理：`fac = min(|D_n|²/|D_2|², WEDGE_FACTOR_MAX)`（等价于把
/// `|D_2|²` 抬到 `|D_n|²/WEDGE_FACTOR_MAX`）。这是**工程上界**、不是物理推导。
pub const WEDGE_FACTOR_MAX: f32 = 4.0;

/// 楔角几何的 **逐带无关**框架（`φ`、`φ'`、`n`、`L`）；`build` 退化时给 `None`（调用侧退回刀口）。
pub struct WedgeFrame {
    phi: f32,
    phip: f32,
    n: f32,
    l: f32,
}

impl WedgeFrame {
    /// 从棱点 `apex`、棱方向 `edge_dir`（单位）、声源 / 听者构造；退化时 `None`（调用侧退回刀口）。
    pub fn build(w: &Wedge, apex: Vec3, edge_dir: Vec3, src: Vec3, dst: Vec3) -> Option<Self> {
        let ax = (w.face0 - edge_dir * w.face0.dot(edge_dir)).normalize();
        if ax.length_squared() < 0.5 {
            return None;
        }
        let bx = edge_dir.cross(ax);
        let (vi, vr) = (src - apex, dst - apex);
        if vi.length_squared() < 1e-6 || vr.length_squared() < 1e-6 {
            return None;
        }
        let ui = vi - edge_dir * vi.dot(edge_dir);
        let ur = vr - edge_dir * vr.dot(edge_dir);
        if ui.length_squared() < 1e-6 || ur.length_squared() < 1e-6 {
            return None;
        }
        let (d1, d2) = (vi.length(), vr.length());
        let omega = w.interior_angle.clamp(0.0, PI);
        Some(Self {
            phi: angle_in_plane(ur, ax, bx),
            phip: angle_in_plane(ui, ax, bx),
            n: 2.0 - omega / PI,
            l: d1 * d2 / (d1 + d2),
        })
    }

    /// 该带的**楔角修正因子** `fac ∈ [0, WEDGE_FACTOR_MAX]`（`n = 2` ⇒ 恒 `1.0`）。
    pub fn factor(&self, k: f32) -> f32 {
        if self.n >= 2.0 {
            return 1.0;
        }
        if self.n <= 1.0 {
            return 0.0;
        }
        let (mn, qn) = mag_scaled(utd_soft(self.n, self.phi, self.phip, k, self.l));
        let (m2, q2) = mag_scaled(utd_soft(2.0, self.phi, self.phip, k, self.l));
        if m2 <= 0.0 {
            return if mn > 0.0 { WEDGE_FACTOR_MAX } else { 0.0 };
        }
        let t = (mn / m2) * (qn / q2); // |D_n| / |D_2|（用缩放幅值算，避免溢出）
        let fac = (t * t).min(WEDGE_FACTOR_MAX);
        if fac.is_nan() {
            WEDGE_FACTOR_MAX
        } else {
            fac
        }
    }
}

/// 命中场区平面内的**极角**（`v` 相对基准 `ax`、正向 `bx`），落在 `[0, 2π)`。
fn angle_in_plane(v: Vec3, ax: Vec3, bx: Vec3) -> f32 {
    let a = v.dot(ax);
    let b = v.dot(bx);
    let t = b.atan2(a);
    if t < 0.0 {
        t + 2.0 * PI
    } else {
        t
    }
}

/// `(m, q)`：`m = max(|re|, |im|)`、`|D| = m·q`（`q ∈ [1, √2]`）—— 幅值的**缩放**表示，乘/除不溢出。
fn mag_scaled((re, im): (f32, f32)) -> (f32, f32) {
    let m = re.abs().max(im.abs());
    if m <= 0.0 {
        (0.0, 0.0)
    } else {
        let (nr, ni) = (re / m, im / m);
        (m, (nr * nr + ni * ni).sqrt())
    }
}

/// KP **软边界**（声压释放）绕射系数 `D`（实部 / 虚部）；全 `f32`。
///
/// `D = −e^{−jπ/4}/(2n√(2πk))·[T(β⁻) − T(β⁺)]`，
/// `T(β) = cot((π+β)/(2n))·F(kL·a⁺(β)) + cot((π−β)/(2n))·F(kL·a⁻(β))`，
/// `β⁻ = φ − φ'`、`β⁺ = φ + φ'`；`a±`、`F` 见 [`apm`]、[`transition`]。
fn utd_soft(n: f32, phi: f32, phip: f32, k: f32, l: f32) -> (f32, f32) {
    let pre = 1.0 / (2.0 * n * (2.0 * PI * k).max(1e-12).sqrt());
    let (pr, pim) = (-FRAC_PI_4.cos() * pre, FRAC_PI_4.sin() * pre);
    let (ar, ai) = term(phi - phip, n, k, l);
    let (br, bi) = term(phi + phip, n, k, l);
    let (tr, ti) = (ar - br, ai - bi);
    (pr * tr - pim * ti, pr * ti + pim * tr)
}

/// `T(β) = cot((π+β)/(2n))·F(kL·a⁺) + cot((π−β)/(2n))·F(kL·a⁻)`（复式两项和）。
fn term(beta: f32, n: f32, k: f32, l: f32) -> (f32, f32) {
    let (ar, ai) = transition(k * l * apm(beta, n, true));
    let (br, bi) = transition(k * l * apm(beta, n, false));
    let c1 = cot((PI + beta) / (2.0 * n));
    let c2 = cot((PI - beta) / (2.0 * n));
    (c1 * ar + c2 * br, c1 * ai + c2 * bi)
}

/// 过渡函数 `F(χ) = 2j√χ·e^{jχ}·∫_{√χ}^∞ e^{−jt²}dt`（`χ ≤ 0 ⇒ 0`），用 `C/S` 闭环求值。
fn transition(chi: f32) -> (f32, f32) {
    if chi <= 0.0 {
        return (0.0, 0.0);
    }
    let u = (2.0 * chi / PI).sqrt();
    let (c, sn) = fresnel_cs(u);
    let q = (0.5 * PI).sqrt();
    let (ire, iim) = (q * (0.5 - c), -q * (0.5 - sn));
    let (ex, ey) = (chi.cos(), chi.sin());
    let r = chi.sqrt();
    let re = 2.0 * r * (ex * ire - ey * iim);
    let im = 2.0 * r * (ex * iim + ey * ire);
    (-im, re) // ×j
}

/// `a±(β) = 2·cos²((2πn·N∓ − β)/2)`，`N∓ = round((β ∓ π)/(2πn))`（`plus` ⇒ 取 `(β+π)` 那支）。
fn apm(beta: f32, n: f32, plus: bool) -> f32 {
    let num = if plus { beta + PI } else { beta - PI };
    let nn = (num / (2.0 * n * PI)).round();
    let t = (2.0 * n * PI * nn - beta) / 2.0;
    2.0 * t * t
}

/// `cot`；`sin → 0` 时给**有限大数**（`±1e12`）而非 `±∞`，避免与过渡函数的 `0` 相乘得 `NaN`。
fn cot(x: f32) -> f32 {
    let s = x.sin();
    if s.abs() < 1e-12 {
        if x.cos() >= 0.0 {
            1e12
        } else {
            -1e12
        }
    } else {
        x.cos() / s
    }
}
