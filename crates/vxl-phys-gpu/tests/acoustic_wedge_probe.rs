//! **A9a/A9b 上卡探针（GPU；需适配器）**：绕射核（M3）从**旧口径** `D(θ) = (1+cosθ)/2` 升到
//! **真刀刃 Fresnel 能量比**（A9a）**× 一般楔角修正因子**（A9b）之后，卡上的读数必须落在
//! **独立参照**上；**且不给波长时旧口径逐位不变**（既有 `acoustic_diffract_probe` 也守这条）。
//!
//! ⚠️ **参照为什么在测试里自写、而不是 dev-depend `vxl-phys-acoustics`**：`dag-gate` 的
//! member 顺序即权威拓扑序，`vxl-phys-gpu`（17）**不得**依赖 `vxl-phys-acoustics`（20）——
//! **后端不依赖领域 crate**，这条就是要守住的方向。所以参照另写一份 **`f64` + C/S 走数值积分**
//! 的实现：它与卡上（`f32`、级数/渐近两分支）**不同语言、不同精度、不同求积**，能抓住 WGSL 移植
//! 的翻译错误（分量/布局/符号/分支），而这正是上卡的风险所在。
//!
//! 场景沿用 `acoustic_diffract_probe.rs` 的屏障：`z = 1`、`x ∈ [-1,1]`、`y ∈ [0,1]`，顶棱
//! `(-1,1,1)→(1,1,1)`；`src = (-0.5,0.5,-1)`、`dst = (0.5,0.5,3)`（分居两侧、被屏障挡死）。
//! 驻点 = 棱中点 `(0,1,1)` ⇒ `d₁ = d₂ = √4.5`、直达 `√17` ⇒ 多余路程 `Δ = 2√4.5 − √17`。
//!
//! ⚠️ **一条测试里顺序跑多段**：`cargo test` 默认并行跑测试函数，每条都会自己建 wgpu 设备，
//! 两个设备并存会互相卡死（见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。分段也是为了让每个
//! 函数留在 god 门的 120 行以内。
//! ⚠️ 无适配器 ⇒ 跳过（打印 `[SKIP]`）；CI 口径 `cargo test --workspace --exclude vxl-phys-gpu`
//! ⇒ 这条只在有卡的机器上跑。

use std::f32::consts::PI;

use vxl_phys_core::Vec3;
use vxl_phys_gpu::acoustic::{
    diffracted_on_adapter, diffracted_wedge_on_adapter, DiffractOut, SceneIn, WedgeIn,
};

const SPEED: f32 = 343.0;
/// 频带（Hz）⇒ `λ = 343 / 1000 = 0.343 m`（卡上按 `max(speed,1e-3)/hz` 算，与 CPU 同式）。
const HZ: f32 = 1000.0;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 屏障：`z = 1`、`x ∈ [-1,1]`、`y ∈ [0,1]`（两块三角；顶边就是那条衍射棱）。
fn barrier() -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
        ],
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ],
    ]
}

/// 顶棱（屏障的顶边）。
fn top_edge() -> Vec<[Vec3; 2]> {
    vec![[Vec3::new(-1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0)]]
}

/// 棱方向（单位）。
fn edge_dir(top: &[[Vec3; 2]]) -> Vec3 {
    (top[0][1] - top[0][0]).normalize()
}

/// 棱上的驻点（对称 ⇒ 棱中点；手推，与卡上的二分不是同一份实现）。
fn foot() -> Vec3 {
    Vec3::new(0.0, 1.0, 1.0)
}

/// 场景（沿用既有探针的几何；只有棱/时延/增益参与）。
fn scene<'a>(
    tris: &'a [[Vec3; 3]],
    top: &'a [[Vec3; 2]],
    coeff: &'a [f32],
    srcs: &'a [Vec3],
    dsts: &'a [Vec3],
) -> SceneIn<'a> {
    SceneIn {
        srcs,
        dsts,
        tris,
        absorb: &[],
        scatter: &[],
        edges: top,
        edge_coeff: coeff,
        speed: SPEED,
    }
}

/// **旧口径**手算：`(时延, 增益) = ((d₁+d₂)/c, coeff·(1+cosθ)/2 /(d₁d₂))`。
fn hand_old(p: Vec3, s: Vec3, d: Vec3, coeff: f32) -> (f32, f32) {
    let (v1, v2) = (p - s, d - p);
    let (d1, d2) = (v1.length(), v2.length());
    let cos_t = (v1.dot(v2) / (d1 * d2)).clamp(-1.0, 1.0);
    ((d1 + d2) / SPEED, coeff * 0.5 * (1.0 + cos_t) / (d1 * d2))
}

/// **独立参照**（`f64`）：C/S 走**数值积分**（Simpson）⇒ 与卡上的级数/渐近**不是同一份实现**；
/// 其余是 SPEC 口径（`utd`）的同式，只是换了精度与语言。A9a/A9b 的判据全在这里落地。
mod oracle {
    pub const PI: f64 = std::f64::consts::PI;

    /// `[x,y,z]` 点积 / 减法 / 数乘 / 叉积 / 范数。
    pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
    pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }
    pub fn mul(a: [f64; 3], k: f64) -> [f64; 3] {
        [a[0] * k, a[1] * k, a[2] * k]
    }
    pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }
    pub fn len(a: [f64; 3]) -> f64 {
        dot(a, a).sqrt()
    }
    pub fn normalize(a: [f64; 3]) -> [f64; 3] {
        let n = len(a);
        if n <= 0.0 {
            a
        } else {
            mul(a, 1.0 / n)
        }
    }

    /// Fresnel `C(x)/S(x)` = `∫₀ˣ cos/sin(πt²/2) dt`，**Simpson 数值积分**（奇函数 ⇒ 取 `|x|` 再翻回去）。
    pub fn cs(x: f64) -> (f64, f64) {
        const N: usize = 120_000;
        let b = x.abs();
        let h = b / N as f64;
        let (mut c, mut s) = (0.0f64, 0.0f64);
        for i in 0..=N {
            let w = if i == 0 || i == N {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };
            let t = i as f64 * h;
            let ph = 0.5 * PI * t * t;
            c += w * ph.cos();
            s += w * ph.sin();
        }
        let (c, s) = (c * h / 3.0, s * h / 3.0);
        if x < 0.0 {
            (-c, -s)
        } else {
            (c, s)
        }
    }

    /// 刀刃能量比 `½·[(½−C(ν))² + (½−S(ν))²]`；`ν = 0 ⇒ 1/4`。
    pub fn knife(nu: f64) -> f64 {
        let (c, s) = cs(nu);
        let (a, b) = (0.5 - c, 0.5 - s);
        0.5 * (a * a + b * b)
    }

    /// `F(χ) = 2j√χ·e^{jχ}·∫_{√χ}^∞ e^{−jt²}dt`（`χ ≤ 0 ⇒ 0`），用 `C/S` 闭环求值。
    fn transition(chi: f64) -> (f64, f64) {
        if chi <= 0.0 {
            return (0.0, 0.0);
        }
        let u = (2.0 * chi / PI).sqrt();
        let (c, s) = cs(u);
        let q = (0.5 * PI).sqrt();
        let (ire, iim) = (q * (0.5 - c), -q * (0.5 - s));
        let (ex, ey) = (chi.cos(), chi.sin());
        let r = chi.sqrt();
        (
            -2.0 * r * (ex * iim + ey * ire),
            2.0 * r * (ex * ire - ey * iim),
        )
    }

    /// `a±(β) = 2·cos²((2πnN∓ − β)/2)`，`N∓ = round((β ∓ π)/(2πn))`。
    fn apm(beta: f64, n: f64, plus: bool) -> f64 {
        let num = if plus { beta + PI } else { beta - PI };
        let nn = (num / (2.0 * n * PI)).round();
        let t = (2.0 * n * PI * nn - beta) / 2.0;
        2.0 * t * t
    }

    /// `cot`；`sin → 0` 给有限大数 `±1e12`（与 SPEC 同口径，避免 `0·∞ = NaN`）。
    fn cot(x: f64) -> f64 {
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

    /// `T(β) = cot((π+β)/(2n))F(kLa⁺) + cot((π−β)/(2n))F(kLa⁻)`。
    fn term(beta: f64, n: f64, k: f64, l: f64) -> (f64, f64) {
        let (ar, ai) = transition(k * l * apm(beta, n, true));
        let (br, bi) = transition(k * l * apm(beta, n, false));
        let c1 = cot((PI + beta) / (2.0 * n));
        let c2 = cot((PI - beta) / (2.0 * n));
        (c1 * ar + c2 * br, c1 * ai + c2 * bi)
    }

    /// KP 软边界绕射系数 `D = −e^{−jπ/4}/(2n√(2πk))·[T(β⁻) − T(β⁺)]`。
    fn soft(n: f64, phi: f64, phip: f64, k: f64, l: f64) -> (f64, f64) {
        let pre = 1.0 / (2.0 * n * (2.0 * PI * k).max(1e-12).sqrt());
        let fr = std::f64::consts::FRAC_PI_4;
        let (pr, pim) = (-fr.cos() * pre, fr.sin() * pre);
        let (ar, ai) = term(phi - phip, n, k, l);
        let (br, bi) = term(phi + phip, n, k, l);
        let (tr, ti) = (ar - br, ai - bi);
        (pr * tr - pim * ti, pr * ti + pim * tr)
    }

    /// `(m, q)`：`m = max(|re|,|im|)`、`|D| = m·q`（缩放幅值，乘除不溢出）。
    fn mag_scaled(v: (f64, f64)) -> (f64, f64) {
        let m = v.0.abs().max(v.1.abs());
        if m <= 0.0 {
            (0.0, 0.0)
        } else {
            let (nr, ni) = (v.0 / m, v.1 / m);
            (m, (nr * nr + ni * ni).sqrt())
        }
    }

    /// 楔角几何（`φ`、`φ'`、`n`、`L`），⊥ 棱平面内量角。
    pub struct Geo {
        pub phi: f64,
        pub phip: f64,
        pub n: f64,
        pub l: f64,
    }

    fn angle_in_plane(v: [f64; 3], ax: [f64; 3], bx: [f64; 3]) -> f64 {
        let t = dot(v, bx).atan2(dot(v, ax));
        if t < 0.0 {
            t + 2.0 * PI
        } else {
            t
        }
    }

    /// 从棱点/棱方向/声源/听者建框架；退化 ⇒ `None`（调用侧退回刀口）。
    pub fn frame(
        omega: f64,
        face0: [f64; 3],
        apex: [f64; 3],
        dir: [f64; 3],
        src: [f64; 3],
        dst: [f64; 3],
    ) -> Option<Geo> {
        let proj = sub(face0, mul(dir, dot(face0, dir)));
        if len(proj) < 1e-6 {
            return None;
        }
        let ax = normalize(proj);
        let bx = cross(dir, ax);
        let (vi, vr) = (sub(src, apex), sub(dst, apex));
        if len(vi) < 1e-3 || len(vr) < 1e-3 {
            return None;
        }
        let (ui, ur) = (
            sub(vi, mul(dir, dot(vi, dir))),
            sub(vr, mul(dir, dot(vr, dir))),
        );
        if len(ui) < 1e-3 || len(ur) < 1e-3 {
            return None;
        }
        let (d1, d2) = (len(vi), len(vr));
        Some(Geo {
            phi: angle_in_plane(ur, ax, bx),
            phip: angle_in_plane(ui, ax, bx),
            n: 2.0 - omega.clamp(0.0, PI) / PI,
            l: d1 * d2 / (d1 + d2),
        })
    }

    /// 楔角修正因子 `fac = min(|D_n|²/|D_2|², 4)`（`n = 2 ⇒ 1`、`n ≤ 1 ⇒ 0`）。
    pub fn factor(g: &Geo, k: f64) -> f64 {
        if g.n >= 2.0 {
            return 1.0;
        }
        if g.n <= 1.0 {
            return 0.0;
        }
        let (mn, qn) = mag_scaled(soft(g.n, g.phi, g.phip, k, g.l));
        let (m2, q2) = mag_scaled(soft(2.0, g.phi, g.phip, k, g.l));
        if m2 <= 0.0 {
            return if mn > 0.0 { 4.0 } else { 0.0 };
        }
        let t = (mn / m2) * (qn / q2);
        let fac = (t * t).min(4.0);
        if fac.is_nan() {
            4.0
        } else {
            fac
        }
    }
}

/// 波长（`hz ≤ 0 ⇒ 0` 哨兵）/ Fresnel 参数 `ν = 2√(Δ/λ)`。
fn lambda_of(hz: f64) -> f64 {
    if hz <= 0.0 {
        0.0
    } else {
        (SPEED as f64).max(1e-3) / hz
    }
}

fn nu_of(excess: f64, lambda: f64) -> f64 {
    if lambda <= 0.0 {
        0.0
    } else {
        2.0 * (excess.max(0.0) / lambda).sqrt()
    }
}

fn p3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

/// **参照入参**（收口成一份，避开本仓 `args-gate` 的形参软阈值）。
struct Ref {
    s: Vec3,
    d: Vec3,
    apex: Vec3,
    coeff: f32,
    hz: f32,
    dir: Vec3,
    /// `(Ω, face0)`；`None` ⇒ 纯刀口（`fac = 1`）。
    wedge: Option<(f32, Vec3)>,
}

/// **参照**（A9a/A9b 口径）：`coeff·knife(ν)·fac /(d₁d₂)`。
fn want_gain(r: &Ref) -> f32 {
    let (s, d, apex) = (p3(r.s), p3(r.d), p3(r.apex));
    let dir = oracle::normalize(p3(r.dir));
    let (v1, v2) = (oracle::sub(apex, s), oracle::sub(d, apex));
    let (d1, d2) = (oracle::len(v1), oracle::len(v2));
    let lambda = lambda_of(r.hz as f64);
    let excess = (d1 + d2) - oracle::len(oracle::sub(d, s));
    let nu = nu_of(excess, lambda);
    let mut fac = 1.0f64;
    if let Some((omega, face0)) = r.wedge {
        if let Some(g) = oracle::frame(omega as f64, p3(face0), apex, dir, s, d) {
            fac = oracle::factor(&g, 2.0 * oracle::PI / lambda);
        }
    }
    ((r.coeff as f64) * oracle::knife(nu) * fac / (d1 * d2)) as f32
}

/// 跑一趟 A9a/A9b 绕射核；起不来（设备/驱动问题）⇒ 打 `[SKIP]` 并给 `None`。
fn try_run_wedge(
    sc: &SceneIn<'_>,
    wedges: &[WedgeIn],
    hz: f32,
) -> Option<(String, Vec<DiffractOut>)> {
    match diffracted_wedge_on_adapter(0, sc, wedges, hz) {
        Ok(v) => Some(v),
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            None
        }
    }
}

/// 第 1 段：单棱、**无楔**（空楔表），给频带 ⇒ 卡上应落在参照的 `knife` 口径上；
/// 且**必须真的换了口径**（不该与旧口径几乎相等）；时延不受影响（仍是折线长 / c）。
fn stage_knife(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let sc = scene(tris, top, &coeff, srcs, dsts);
    let Some((name, got)) = try_run_wedge(&sc, &[], HZ) else {
        return;
    };
    assert_eq!(got.len(), 1, "一对声源/听者 ⇒ 一行读数");
    let p = foot();
    let want = want_gain(&Ref {
        s: srcs[0],
        d: dsts[0],
        apex: p,
        coeff: 1.0,
        hz: HZ,
        dir: edge_dir(top),
        wedge: None,
    });
    let old = hand_old(p, srcs[0], dsts[0], 1.0);
    println!(
        "[gpu] {name}: 刀刃 gain 卡上 {} vs 参照 {want}（旧口径 {}）",
        got[0].gain, old.1
    );
    assert!(got[0].hit, "阴影里必须产出绕射");
    assert!(
        (got[0].gain - want).abs() <= want * 1e-3,
        "真刀刃增益（卡上 {} vs 参照 {want}）",
        got[0].gain
    );
    assert!(
        (got[0].gain - old.1).abs() > old.1 * 1e-3,
        "给了频带就必须换口径（新/旧不该几乎相等：{} vs {}）",
        got[0].gain,
        old.1
    );
    assert!(
        (got[0].delay - old.0).abs() <= old.0 * 1e-4,
        "时延仍应是折线长 / c（卡上 {} vs 手算 {}）",
        got[0].delay,
        old.0
    );
}

/// 第 2 段：**平壁** `Ω = π ⇒ n = 1 ⇒ fac = 0` ⇒ 系数归零 ⇒ 不产出（`hit = false`）。
fn stage_flat_wall(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let sc = scene(tris, top, &coeff, srcs, dsts);
    let wedges = vec![WedgeIn {
        interior_angle: PI,
        face0: Vec3::new(0.0, 1.0, 0.0),
    }];
    let Some((name, got)) = try_run_wedge(&sc, &wedges, HZ) else {
        return;
    };
    println!("[gpu] {name}: 平壁 Ω=π ⇒ {:?}", got[0]);
    assert!(
        !got[0].hit && got[0].gain == 0.0,
        "Ω=π（平壁）⇒ 楔角系数 → 0 ⇒ 不得产出（实际 {:?}）",
        got[0]
    );
}

/// 第 3 段：**四分之一楔** `Ω = π/2`（`n = 1.5`）⇒ 卡上增益必须落在参照的 `factor` 口径上
/// （同式两份实现 ⇒ 卡上这份 WGSL 的 UTD/框架移植对不对就看这里）。
fn stage_quarter_wedge(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let sc = scene(tris, top, &coeff, srcs, dsts);
    let face0 = Vec3::new(0.0, 1.0, 0.0);
    let wedges = vec![WedgeIn {
        interior_angle: 0.5 * PI,
        face0,
    }];
    let Some((name, got)) = try_run_wedge(&sc, &wedges, HZ) else {
        return;
    };
    let want = want_gain(&Ref {
        s: srcs[0],
        d: dsts[0],
        apex: foot(),
        coeff: 1.0,
        hz: HZ,
        dir: edge_dir(top),
        wedge: Some((0.5 * PI, face0)),
    });
    let knife = want_gain(&Ref {
        s: srcs[0],
        d: dsts[0],
        apex: foot(),
        coeff: 1.0,
        hz: HZ,
        dir: edge_dir(top),
        wedge: None,
    });
    println!(
        "[gpu] {name}: 楔角 gain 卡上 {} vs 参照 {want}（无楔 {knife}）",
        got[0].gain
    );
    assert!(
        (got[0].gain - want).abs() <= want * 1e-3,
        "楔角修正后增益（卡上 {} vs 参照 {want}）",
        got[0].gain
    );
}

/// 第 4 段：**旧口径逐位不变** —— `diffracted_on_adapter`（旧薄壳）与 `band_hz = 0` /
/// `Ω = 0` 的表必须**逐位一致**（`lambda = 0` ⇒ 核内整段不参与）。
fn stage_legacy_bitwise(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let sc = scene(tris, top, &coeff, srcs, dsts);
    let Ok((name, a)) = diffracted_on_adapter(0, &sc) else {
        println!("[SKIP] 核起不来");
        return;
    };
    let zero = vec![WedgeIn {
        interior_angle: 0.0,
        face0: Vec3::ZERO,
    }];
    let Ok((_, b)) = diffracted_wedge_on_adapter(0, &sc, &[], 0.0) else {
        return;
    };
    let Ok((_, c)) = diffracted_wedge_on_adapter(0, &sc, &zero, 0.0) else {
        return;
    };
    println!("[gpu] {name}: 旧口径 {:?}", a[0]);
    assert_eq!(a, b, "旧薄壳与 `wedge 空 / band_hz=0` 必须逐位一致");
    assert_eq!(a, c, "`band_hz = 0` ⇒ Ω=0 表也走旧口径 ⇒ 逐位一致");
    // Ω=0（刀口）与空楔表在同一条 lane（都给频带）下也必须逐位一致。
    let Ok((_, d)) = diffracted_wedge_on_adapter(0, &sc, &[], HZ) else {
        return;
    };
    let Ok((_, e)) = diffracted_wedge_on_adapter(0, &sc, &zero, HZ) else {
        return;
    };
    assert_eq!(d, e, "Ω=0（刀口）与空楔表同口径 ⇒ 逐位一致");
}

/// 第 5 段：**楔角表给短了/给长了必须报错**（核内逐棱直接索引 ⇒ 不给设备越界读的机会）。
fn stage_table_mismatch(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let sc = scene(tris, top, &coeff, srcs, dsts);
    let w = WedgeIn {
        interior_angle: 0.5 * PI,
        face0: Vec3::ZERO,
    };
    assert!(
        diffracted_wedge_on_adapter(0, &sc, &[w, w], HZ).is_err(),
        "棱表 1 条、楔角表 2 条 ⇒ 必须报错"
    );
}

/// 五段顺序跑（见文件头注：**必须一条测试**）。
#[test]
fn diffraction_matches_independent_oracle_and_legacy_path_is_unchanged() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    let tris = barrier();
    let top = top_edge();
    let srcs = vec![Vec3::new(-0.5, 0.5, -1.0)];
    let dsts = vec![Vec3::new(0.5, 0.5, 3.0)];
    stage_knife(&tris, &top, &srcs, &dsts);
    stage_flat_wall(&tris, &top, &srcs, &dsts);
    stage_quarter_wedge(&tris, &top, &srcs, &dsts);
    stage_legacy_bitwise(&tris, &top, &srcs, &dsts);
    stage_table_mismatch(&tris, &top, &srcs, &dsts);
}
