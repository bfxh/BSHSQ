//! **A9a「刀刃真 Fresnel 绕射」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9 第 2 条）：
//! 逐带**显式声明频率**时，绕射的「方向因子」换成真 Fresnel 刀刃系数；默认（不给频率）退回 A4 旧口径。
//!
//! 分两段：**纯数学对拍**（`C/S` 对**独立数值积分**、奇对称、导数恒等式、`ν` 与经典刀口口径等价、
//! 阴影边界恰 −6.02 dB）与**端到端**（频率开关：旧口径逐带相等 / 声明后逐带单调、逐带比值与
//! **独立 Simpson 积分**算出的 Fresnel 链一致；零干扰、确定性）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, utd, AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Edge, Listener,
    PathKind, SoundPath, BANDS,
};

/// 测试侧**独立参照**的 Fresnel 积分（`f64` Simpson 直接积 `∫cos/sin(πt²/2)`）——不复用被测实现。
fn ref_cs(x: f32) -> (f64, f64) {
    let xf = x as f64;
    let n = 20000usize;
    let h = xf / n as f64;
    let (mut ac, mut as_) = (0.0f64, 0.0f64);
    for i in 1..n {
        let t = h * i as f64;
        let a = 0.5 * core::f64::consts::PI * t * t;
        let w = if i % 2 == 0 { 2.0 } else { 4.0 };
        ac += w * a.cos();
        as_ += w * a.sin();
    }
    (h / 3.0 * ac, h / 3.0 * as_)
}

/// 同一闭式，用**独立积分**算刀刃能量比 `|E_d/E₀|²`。
fn ref_knife(nu: f32) -> f64 {
    let (c, s) = ref_cs(nu);
    let (a, b) = (0.5 - c, 0.5 - s);
    0.5 * (a * a + b * b)
}

/// ① `C/S` 对**表值**（标准 Fresnel 表）：幂级数必须落在 4 位。
#[test]
fn fresnel_cs_matches_tables() {
    for (x, ce, se) in [
        (0.5f32, 0.49234423f64, 0.06473243f64),
        (1.0, 0.77989340, 0.43825915),
        (1.5, 0.44526118, 0.69750496),
        (2.0, 0.48825341, 0.34341568),
        (3.0, 0.60572079, 0.49631300),
    ] {
        let (c, s) = utd::fresnel_cs(x);
        assert!((c as f64 - ce).abs() <= 2e-4, "C({x}) = {c}，表值 {ce}");
        assert!((s as f64 - se).abs() <= 2e-4, "S({x}) = {s}，表值 {se}");
    }
}

/// ② **导数恒等式**（自洽、不引外部常数）：`C'(x) = cos(πx²/2)`、`S'(x) = sin(πx²/2)`（中心差分）。
/// ⚠️ 步长取 `1e-3`：大 `x` 处 `S''' ≈ −π²x²`（`x=5` 时 ~−247），`h=1e-2` 的光滑误差 `h²/6·|S'''|`
/// 有 `4e-3`（这是**差分法**的误差，不是被测量的误差）；`1e-3` 时降到 ~`4e-5`，仍远大于 `f32` 抵消噪声。
#[test]
fn fresnel_cs_derivative_is_the_integrand() {
    let h = 1e-3f32;
    for x in [0.3f32, 1.0, 2.0, 4.0, 5.0, 6.0] {
        let (cp, sp) = utd::fresnel_cs(x + h);
        let (cm, sm) = utd::fresnel_cs(x - h);
        let dc = (cp - cm) / (2.0 * h);
        let ds = (sp - sm) / (2.0 * h);
        let a = 0.5 * core::f32::consts::PI * x * x;
        assert!(
            (dc - a.cos()).abs() <= 1e-3,
            "C'({x}) = {dc}，应 {a_cos}",
            a_cos = a.cos()
        );
        assert!(
            (ds - a.sin()).abs() <= 1e-3,
            "S'({x}) = {ds}，应 {a_sin}",
            a_sin = a.sin()
        );
    }
}

/// ③ **奇对称**逐位：`C(−x) = −C(x)`、`S(−x) = −S(x)`（实现即按此规约 ⇒ 逐位相等）。
#[test]
fn fresnel_cs_is_odd() {
    for x in [0.5f32, 1.0, 2.0, 8.0] {
        let (c, s) = utd::fresnel_cs(x);
        let (cn, sn) = utd::fresnel_cs(-x);
        assert_eq!(cn, -c, "C(−{x}) 必须逐位为 −C({x})");
        assert_eq!(sn, -s, "S(−{x}) 必须逐位为 −S({x})");
    }
}

/// ④ **阴影边界恰 −6.02 dB**：`ν = 0 ⇒ |E/E₀|² = 1/4`，且随 `ν` 增大**严格单调降**、深处 ~`1/(2π²ν²)`。
#[test]
fn knife_boundary_is_minus_6db_and_monotone() {
    assert_eq!(utd::knife_gain(0.0), 0.25, "阴影边界必须**恰好** 1/4");
    let db = 10.0 * 0.25f32.log10();
    assert!((db + 6.0206).abs() <= 1e-3, "1/4 应恰 −6.02 dB，实际 {db}");
    let mut prev = f32::INFINITY;
    for nu in [0.5f32, 1.0, 2.0, 4.0, 8.0, 16.0] {
        let g = utd::knife_gain(nu);
        assert!(g < prev, "越深必须越弱（ν={nu}：{g} 不小于 {prev}）");
        prev = g;
        let r = ref_knife(nu);
        assert!(
            (g as f64 - r).abs() <= 5e-4,
            "ν={nu}：实现 {g} vs 独立积分 {r}"
        );
    }
    // 深处渐近：ν=16 ⇒ ~1/(2π²·16²)
    let deep = 1.0 / (2.0 * core::f32::consts::PI * core::f32::consts::PI * 256.0);
    assert!(
        (utd::knife_gain(16.0) / deep - 1.0).abs() <= 5e-2,
        "深处应 ~1/(2π²ν²)"
    );
    // 通视侧（ν < 0）⇒ 逼近 1
    assert!(utd::knife_gain(-8.0) > 0.95, "深处通视应逼近自由场");
    assert!(utd::knife_gain(-8.0) <= 1.0);
}

/// ⑤ **`ν` 与经典刀口口径等价**：`2√(Δ/λ)` 对上 `h·√(2(s+s')/(λ·s·s'))`（`Δ = h²(s+s')/(2·s·s')`）。
#[test]
fn nu_matches_classical_knife_edge() {
    let (h, s, sp, lam) = (1.0f32, 2.0f32, 3.0f32, 0.343f32);
    let excess = h * h * (s + sp) / (2.0 * s * sp);
    let classic = h * (2.0 * (s + sp) / (lam * s * sp)).sqrt();
    let mine = utd::fresnel_nu(excess, lam);
    assert!(
        (mine - classic).abs() <= classic * 1e-4,
        "ν 口径不等价：{mine} vs {classic}"
    );
    assert_eq!(utd::fresnel_nu(-1.0, lam), 0.0, "Δ < 0 必须夹到 0");
    assert_eq!(utd::fresnel_nu(excess, 0.0), 0.0, "λ ≤ 0（没频率）必须给 0");
    assert_eq!(utd::wavelength(343.0, 0.0), 0.0, "没频率 ⇒ 哨兵 0");
    assert!((utd::wavelength(343.0, 1000.0) - 0.343).abs() <= 1e-6);
}

/// 场景：地板 + 屏障（z=1、x∈[-1,1]、y∈[0,1]）+ 顶棱（同 A4 判据的几何）。
struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    edges: Vec<Edge>,
}

impl Scene {
    fn new() -> Self {
        let mut s = Self {
            verts: vec![
                Vec3::new(-4.0, 0.0, -4.0),
                Vec3::new(4.0, 0.0, -4.0),
                Vec3::new(4.0, 0.0, 4.0),
                Vec3::new(-4.0, 0.0, 4.0),
            ],
            tris: vec![[0u32, 1, 2], [0, 2, 3]],
            absorb: vec![[0.25f32; BANDS]; 2],
            trans: vec![[0.0f32; BANDS]; 2],
            edges: Vec::new(),
        };
        let base = s.verts.len() as u32;
        s.verts.extend([
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ]);
        s.tris
            .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        s.absorb.extend([[0.1f32; BANDS]; 2]);
        s.trans.extend([[0.0f32; BANDS]; 2]);
        s.edges.push(Edge {
            a: Vec3::new(-1.0, 1.0, 1.0),
            b: Vec3::new(1.0, 1.0, 1.0),
            coeff: [1.0; BANDS],
            wedge: None,
        });
        s
    }

    fn geom(&self) -> AcousticGeometry<'_> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            scattering: &[],
            portals: &[],
            edges: &self.edges,
            medium: None,
        }
    }
}

fn world_with_source() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.0, -0.25, -1.0), 0.25, 8, 1, 8);
    vol.fill_box(Vec3::new(-1.0, -0.25, -1.0), Vec3::new(1.0, 0.0, 1.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(0.0, -6.0, 0.0);
    for _ in 0..40 {
        w.step();
        if !w.impacts().is_empty() {
            break;
        }
    }
    w
}

fn cues(
    w: &World,
    geo: &AcousticGeometry<'_>,
    cfg: &AcousticConfig,
    pos: Vec3,
) -> (AcousticStats, Vec<SoundPath>) {
    let listener = Listener { pos, radius: 0.5 };
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener, cfg, &mut paths);
    (st, paths)
}

fn first_diffracted(paths: &[SoundPath]) -> Option<SoundPath> {
    paths
        .iter()
        .find(|p| p.kind == PathKind::Diffracted)
        .copied()
}

/// ⑥ **默认（不给频率）= A4 旧口径**：逐带相等，且等于 `coeff · (1+cosθ)/2 / (d1·d2)`（独立算几何）。
#[test]
fn without_frequency_it_is_the_old_angular_factor() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new();
    let g = scene.geom();
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let cfg = AcousticConfig::default();
    assert_eq!(cfg.band_hz, [0.0; BANDS], "默认必须是不给频率");
    let (st, paths) = cues(&w, &g, &cfg, lst);
    assert_eq!(st.direct, 0, "屏障必须先挡住直达");
    assert!(st.diffracted >= 1);
    let p = first_diffracted(&paths);
    assert!(p.is_some(), "必须有绕射路径（否则判据空跑）");
    let p = match p {
        Some(p) => p,
        None => return,
    };
    assert_eq!(p.bands[0], p.bands[1], "旧口径逐带相等（coeff 全 1）");
    assert_eq!(p.bands[1], p.bands[2]);
    // 独立算几何：直达距离、折线长（= delay·c）、Fermat 点处的 cosθ
    let src = w.impacts()[0].point;
    let fold = p.delay * cfg.speed_of_sound;
    // Fermat 点：棱上取 `|P−src|+|P−lst|` 最小 ⇒ 用折线长在两段上的分配反解
    let e = &scene.edges[0];
    let dir = (e.b - e.a).normalize();
    let mut best = (f32::INFINITY, e.a);
    for i in 0..=40000 {
        let q = e.a + dir * ((e.b - e.a).length() * (i as f32 / 40000.0));
        let l = (q - src).length() + (q - lst).length();
        if l < best.0 {
            best = (l, q);
        }
    }
    let q = best.1;
    let (d1, d2) = ((q - src).length(), (lst - q).length());
    let cos_t = (((q - src).dot(lst - q)) / (d1 * d2)).clamp(-1.0, 1.0);
    let want = 0.5 * (1.0 + cos_t) / (d1 * d2);
    assert!(
        (p.bands[0] - want).abs() <= want * 2e-3,
        "默认绕射必须就是旧口径 `(1+cosθ)/2/(d1·d2)`：实现 {} vs 独立 {want}（折线长 {fold} vs 独立 {})",
        p.bands[0],
        best.0
    );
}

/// ⑦ **声明频率 ⇒ 真 Fresnel**：逐带**严格单调降**（高频更闷），且**逐带比值**与独立 Simpson 积分一致。
#[test]
fn declared_frequency_switches_to_real_fresnel() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new();
    let g = scene.geom();
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let c = 343.0f32;
    let hz = [500.0f32, 1000.0, 2000.0];
    let cfg = AcousticConfig {
        band_hz: hz,
        ..AcousticConfig::default()
    };
    let (st, paths) = cues(&w, &g, &cfg, lst);
    assert_eq!(st.direct, 0, "屏障必须先挡住直达");
    assert!(st.diffracted >= 1);
    let p = first_diffracted(&paths);
    assert!(p.is_some(), "必须有绕射路径（否则判据空跑）");
    let p = match p {
        Some(p) => p,
        None => return,
    };
    assert!(
        p.bands[0] > p.bands[1] && p.bands[1] > p.bands[2],
        "声明频率后必须逐带递减（高频更闷），实际 {:?}",
        p.bands
    );
    // 独立参照：`Δ = 折线长 − 直连`（折线长 = delay·c），`ν_b = 2√(Δ/λ_b)`
    let src = w.impacts()[0].point;
    let excess = p.delay * c - (lst - src).length();
    assert!(excess > 0.0, "阴影里多余路程必须为正（{excess}）");
    let base = utd::fresnel_nu(excess, c / hz[0]);
    for (b, &f) in hz.iter().enumerate() {
        let nu = utd::fresnel_nu(excess, c / f);
        let want_ratio = ref_knife(nu) / ref_knife(base);
        let got_ratio = (p.bands[b] / p.bands[0]) as f64;
        assert!(
            (got_ratio - want_ratio).abs() <= 5e-3,
            "带 {b}：实现比值 {got_ratio} vs 独立积分 {want_ratio}（ν={nu}）"
        );
    }
}

/// ⑧ **零干扰 + 确定性**：声明频率时 `state_hash` 仍不变，同输入两跑逐字一致。
#[test]
fn frequency_mode_is_readonly_and_deterministic() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let scene = Scene::new();
    let g = scene.geom();
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let cfg = AcousticConfig {
        band_hz: [400.0, 800.0, 1600.0],
        ..AcousticConfig::default()
    };
    let a = cues(&w, &g, &cfg, lst);
    let b = cues(&w, &g, &cfg, lst);
    assert!(a.0.diffracted >= 1, "必须产出绕射（否则判据空跑）");
    assert_eq!(jsonl(&a.1), jsonl(&b.1), "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（A9a 也一样，只读）"
    );
}
