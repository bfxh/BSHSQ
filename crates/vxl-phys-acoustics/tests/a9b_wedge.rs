//! **A9b「一般楔角 UTD 修正」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）：
//! `Edge::wedge = Some(Ω)` 时按 Kouyoumjian–Pathak 系数算**楔角修正因子**
//! `fac = min(|D_n|²/|D_2|², WEDGE_FACTOR_MAX)`（`n = 2 − Ω/π`），乘到 A9a 刀刃口径上。
//!
//! 分两段：**纯因子对拍**（`n = 2` ⇒ **恰好 `1`**（逐位回到 A9a）、`n = 1`（平壁）⇒ **恰好 `0`**、
//! **有界且非 `NaN`**、**互易**、与**独立 KP 实现**（Python 侧 `f64`）数值一致）与**端到端**
//! （`Ω = 0` 时与 `wedge: None` **逐位相同**、默认（不给频率）忽略楔角、`Ω = π` 把路径能量**清零**、
//! 有界性、零干扰、确定性）。

use vxl_phys::{BodyBuildExt, DiagExt, ImpactExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::utd::{WedgeFrame, WEDGE_FACTOR_MAX};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticsExt, Edge, Listener, PathKind, SoundPath,
    Wedge, BANDS,
};

use core::f32::consts::PI;

/// 测试侧的 band 中心频率（Hz）——只需「显式声明了绝对频率」即可（`> 0`）。
const HZ: [f32; BANDS] = [500.0, 1000.0, 2000.0];

/// ⊥ 棱平面内的单位方向：从 `+X` 起、朝 `+Y` 转 `deg` 度（`b_axis = Z×X = +Y`）。
fn dir_at(deg: f32) -> Vec3 {
    let r = deg.to_radians();
    Vec3::new(r.cos(), r.sin(), 0.0)
}

/// 直接对**因子**取样：棱沿 `+Z`、`face0 = +X`、棱点在原点，声源在 `phip_deg`（距 `d1`）、
/// 听者在 `phi_deg`（距 `d2`）。
fn fac(interior: f32, phip_deg: f32, d1: f32, phi_deg: f32, d2: f32) -> f32 {
    let w = Wedge {
        interior_angle: interior,
        face0: Vec3::X,
    };
    let k = 2.0 * PI / 0.343;
    WedgeFrame::build(
        &w,
        Vec3::ZERO,
        Vec3::Z,
        dir_at(phip_deg) * d1,
        dir_at(phi_deg) * d2,
    )
    .map_or(1.0, |f| f.factor(k))
}

/// ① **`n = 2`（`Ω = 0`，刀口）⇒ 因子恰好 `1.0`**（逐位回到 A9a）。
#[test]
fn n_two_factor_is_exactly_one() {
    for &(phip, phi) in &[
        (90.0f32, 300.0f32),
        (90.0, 120.0),
        (10.0, 200.0),
        (45.0, 250.0),
    ] {
        for &(d1, d2) in &[(2.0f32, 3.0f32), (0.5, 4.0), (5.0, 0.7)] {
            let f = fac(0.0, phip, d1, phi, d2);
            assert_eq!(f, 1.0, "Ω=0 必须恰好 1.0（phip={phip} phi={phi}）");
        }
    }
}

/// ② **`n = 1`（`Ω = π`，平壁）⇒ 因子恰好 `0.0`**（无绕射）；越界内角按端点钳。
#[test]
fn flat_interior_factor_is_exactly_zero() {
    assert_eq!(fac(PI, 90.0, 2.0, 300.0, 3.0), 0.0);
    assert_eq!(fac(PI, 30.0, 1.0, 210.0, 1.0), 0.0);
    assert_eq!(fac(PI * 2.0, 90.0, 2.0, 260.0, 3.0), 0.0, "内角越界按 π 钳");
    assert_eq!(
        fac(-1.0, 90.0, 2.0, 260.0, 3.0),
        1.0,
        "负内角按 0 钳（刀口）"
    );
}

/// ③ **有界 `[0, WEDGE_FACTOR_MAX]`、非 `NaN`、且确有方向性**（`Ω = 90°` 时同时取到 `<1` 与 `>1`）。
#[test]
fn factor_is_bounded_finite_and_directional() {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for &ideg in &[20.0f32, 45.0, 90.0, 135.0, 160.0] {
        let interior = ideg.to_radians();
        for phi in (0..360).step_by(7) {
            let f = fac(interior, 90.0, 2.0, phi as f32, 3.0);
            assert!(!f.is_nan(), "因子不得为 NaN（Ω={ideg} φ={phi}）");
            assert!(
                (0.0..=WEDGE_FACTOR_MAX).contains(&f),
                "因子必须 ∈ [0, {WEDGE_FACTOR_MAX}]：{f}"
            );
            if ideg == 90.0 {
                lo = lo.min(f);
                hi = hi.max(f);
            }
        }
    }
    assert!(lo < 1.0, "Ω=90° 方向性：应有方向把能量压低（min={lo}）");
    assert!(hi > 1.0, "Ω=90° 方向性：应有方向把能量抬高（max={hi}）");
}

/// ④ **互易**：`fac(φ,φ') = fac(φ',φ)` —— 逐位相等（KP 系数对称、`L` 对称）。
#[test]
fn factor_is_reciprocal() {
    for &(a, da, b, db) in &[
        (90.0f32, 2.0f32, 210.0f32, 3.0f32),
        (30.0, 1.5, 300.0, 2.5),
        (0.5, 4.0, 250.0, 0.7),
    ] {
        let f1 = fac(PI / 2.0, a, da, b, db);
        let f2 = fac(PI / 2.0, b, db, a, da);
        assert_eq!(f1, f2, "互易必须逐位相等：{f1} vs {f2}");
    }
}

/// ⑤ **与独立 KP 实现（Python 侧 `f64`）数值一致**：`Ω=90°`、`φ'=90°`、`d₁=2`、`d₂=3`、`λ=0.343`。
///
/// 参考值来自 `f64` KP 软系数 + 真 `∫`（非本实现），故这是**独立对拍**（容差吸收 `f32` 求值差）。
#[test]
fn f32_factor_matches_independent_kp_reference() {
    for &(phi, want) in &[(120.0f32, 1.10674f32), (240.0, 2.55105), (60.0, 0.93489)] {
        let got = fac(PI / 2.0, 90.0, 2.0, phi, 3.0);
        assert!(
            (got - want).abs() <= want * 3e-2,
            "φ={phi}：实现 {got} vs 独立 KP {want}"
        );
    }
}

// ---------------------------------------------------------------------------
// 端到端：同一场景（屏障顶棱），只改 `edge.wedge`。
// ---------------------------------------------------------------------------

struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    edges: Vec<Edge>,
}

impl Scene {
    fn new(wedge: Option<Wedge>) -> Self {
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
            wedge,
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

fn diffracted_of(w: &World, geo: &AcousticGeometry<'_>, cfg: &AcousticConfig) -> Option<SoundPath> {
    let listener = Listener {
        pos: Vec3::new(0.0, 0.5, 2.0),
        radius: 0.5,
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener, cfg, &mut paths);
    assert_eq!(st.direct, 0, "屏障必须先挡住直达（否则判据空跑）");
    assert!(st.diffracted >= 1, "必须有绕射路径（否则判据空跑）");
    paths
        .iter()
        .find(|p| p.kind == PathKind::Diffracted)
        .copied()
}

/// ⑥ **`Ω = 0` ⟺ `wedge: None`：逐位相同**（声明频率下，因 `fac` 恰好 `1`）。
#[test]
fn zero_wedge_matches_none_bitwise() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let cfg = AcousticConfig {
        band_hz: HZ,
        ..AcousticConfig::default()
    };
    let s_none = Scene::new(None);
    let s_zero = Scene::new(Some(Wedge {
        interior_angle: 0.0,
        face0: Vec3::Z,
    }));
    let (p_none, p_zero) = match (
        diffracted_of(&w, &s_none.geom(), &cfg),
        diffracted_of(&w, &s_zero.geom(), &cfg),
    ) {
        (Some(a), Some(b)) => (a, b),
        _ => return,
    };
    assert_eq!(p_none.bands, p_zero.bands, "Ω=0 必须与 None 逐位相同");
    assert!(p_none.bands[0] > 0.0, "刀刃口径下带 0 必须为正");
}

/// ⑦ **默认（不给频率）忽略楔角**：`Ω = 90°` 与 `None` 在旧口径下逐位相同（`fac` 根本没进公式）。
#[test]
fn default_mode_ignores_wedge() {
    let w = world_with_source();
    let cfg = AcousticConfig::default();
    assert_eq!(cfg.band_hz, [0.0; BANDS], "默认必须是不给频率");
    let s_none = Scene::new(None);
    let s_wedged = Scene::new(Some(Wedge {
        interior_angle: PI / 2.0,
        face0: Vec3::Z,
    }));
    let (p_none, p_wedged) = match (
        diffracted_of(&w, &s_none.geom(), &cfg),
        diffracted_of(&w, &s_wedged.geom(), &cfg),
    ) {
        (Some(a), Some(b)) => (a, b),
        _ => return,
    };
    assert_eq!(p_none.bands, p_wedged.bands, "旧口径不应受楔角影响");
}

/// ⑧ **`Ω = π`（平壁）：声明频率后路径能量被清零**（`fac = 0`）；且**有界** `≤ WEDGE_FACTOR_MAX × 刀口`。
#[test]
fn flat_wedge_zeroes_path_and_wedge_is_bounded() {
    let w = world_with_source();
    let cfg = AcousticConfig {
        band_hz: HZ,
        ..AcousticConfig::default()
    };
    let s_none = Scene::new(None);
    let s_flat = Scene::new(Some(Wedge {
        interior_angle: PI,
        face0: Vec3::Z,
    }));
    let s_w = Scene::new(Some(Wedge {
        interior_angle: PI / 2.0,
        face0: Vec3::Z,
    }));
    let (p_none, p_flat, p_w) = match (
        diffracted_of(&w, &s_none.geom(), &cfg),
        diffracted_of(&w, &s_flat.geom(), &cfg),
        diffracted_of(&w, &s_w.geom(), &cfg),
    ) {
        (Some(a), Some(b), Some(c)) => (a, b, c),
        _ => return,
    };
    assert_eq!(p_flat.bands, [0.0; BANDS], "平壁 ⇒ 因子 0 ⇒ 路径能量清零");
    assert!(p_none.bands[0] > 0.0, "刀口基准必须 > 0");

    for b in 0..BANDS {
        assert!(p_w.bands[b] >= 0.0);
        assert!(
            p_w.bands[b] <= p_none.bands[b] * WEDGE_FACTOR_MAX + 1e-9,
            "带 {b}：楔角修正必须有界（{} ≤ {}×{WEDGE_FACTOR_MAX}）",
            p_w.bands[b],
            p_none.bands[b]
        );
    }
}

/// ⑨ **零干扰 + 确定性**：带楔角声明频率时 `state_hash` 不变、同输入两跑逐字一致。
#[test]
fn wedge_is_readonly_and_deterministic() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let scene = Scene::new(Some(Wedge {
        interior_angle: PI / 3.0,
        face0: Vec3::Z,
    }));
    let g = scene.geom();
    let listener = Listener {
        pos: Vec3::new(0.0, 0.5, 2.0),
        radius: 0.5,
    };
    let cfg = AcousticConfig {
        band_hz: HZ,
        ..AcousticConfig::default()
    };
    let run = |w: &World| {
        let mut paths: Vec<SoundPath> = Vec::new();
        let st = w.acoustic_cues(&g, &listener, &cfg, &mut paths);
        (st, jsonl(&paths))
    };
    let a = run(&w);
    let b = run(&w);
    assert!(a.0.diffracted >= 1, "必须产出绕射（否则判据空跑）");
    assert_eq!(a.1, b.1, "同输入两跑必须逐字一致");
    assert_eq!(w.state_hash(), before, "声学层必须零干扰（只读）");
}

/// `utd` 公开项存在性/常量锚（防误删）：`WEDGE_FACTOR_MAX = 4`。
#[test]
fn wedge_factor_max_is_four() {
    assert_eq!(WEDGE_FACTOR_MAX, 4.0);
}
