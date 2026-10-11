//! **A3「频带」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A3 行 + §4 原则 ③）：
//! 吸声 / 透射 / 介质吸收**逐带**（[`BANDS`] = 低 / 中 / 高），输出 [`SoundPath::bands`]。
//!
//! 三条硬指标：① **退化档**（三带同参）与单频带口径**逐位相同**；② **高频更闷** —— 逐带严格
//! 单调，且与解析式对拍；③ 确定性 + 零干扰（`jsonl` 也含 `bands`）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticStats, AcousticsExt,
    LayeredMedium, Listener, Medium, PathKind, SoundPath, BANDS,
};

/// 地板（两块三角）+ 可选的墙（z = 1.5、x∈[-3,3]、y∈[0,3]）；两张材质表**逐带**给。
struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
}

impl Scene {
    fn new(floor_absorb: [f32; BANDS], wall: Option<([f32; BANDS], [f32; BANDS])>) -> Self {
        let mut verts = vec![
            Vec3::new(-4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, 4.0),
            Vec3::new(-4.0, 0.0, 4.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
        let mut absorb = vec![floor_absorb, floor_absorb];
        let mut trans = vec![[0.0f32; BANDS]; 2];
        if let Some((a, t)) = wall {
            let base = verts.len() as u32;
            verts.extend([
                Vec3::new(-3.0, 0.0, 1.5),
                Vec3::new(3.0, 0.0, 1.5),
                Vec3::new(3.0, 3.0, 1.5),
                Vec3::new(-3.0, 3.0, 1.5),
            ]);
            tris.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
            absorb.extend([a, a]);
            trans.extend([t, t]);
        }
        Self {
            verts,
            tris,
            absorb,
            trans,
        }
    }

    fn geom<'a>(&'a self, medium: Option<&'a dyn AcousticMedium>) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            portals: &[],
            medium,
        }
    }
}

/// 一个"声源"：把一发冲击打进世界（落到体素地面上 ⇒ 声源 = 接触点）。
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

/// 听者在 z = 2.0（墙在 z = 1.5 ⇒ 直达被挡，只有透射/反射过来）。
fn listener() -> Listener {
    Listener {
        pos: Vec3::new(0.0, 0.5, 2.0),
        radius: 0.5,
    }
}

fn cues(w: &World, geo: &AcousticGeometry<'_>) -> (AcousticStats, Vec<SoundPath>) {
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener(), &AcousticConfig::default(), &mut paths);
    (st, paths)
}

/// 取（某声源、某类）第一条路径；没有就给零值占位（"真有没有"由 `AcousticStats` 断言）。
fn first_of(paths: &[SoundPath], kind: PathKind, src: u32) -> SoundPath {
    paths
        .iter()
        .find(|p| p.kind == kind && p.source == src)
        .copied()
        .unwrap_or(SoundPath {
            source: src,
            delay: 0.0,
            bands: [0.0; BANDS],
            gain: 0.0,
            dir: [0.0, 0.0, 0.0],
            kind,
        })
}

/// ① **退化档**：三带同参 ⇒ 三个带**逐位相同**、`gain` 就是中频带（与旧的标量口径一致）。
#[test]
fn uniform_bands_reduce_to_single_band() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new([0.25f32; BANDS], None);
    let g = scene.geom(None);
    let (st, paths) = cues(&w, &g);
    assert!(st.direct >= 1, "开阔场景必须产出直达（否则判据空跑）");
    let p = first_of(&paths, PathKind::Direct, 0);
    assert!(
        p.bands[0] == p.bands[1] && p.bands[1] == p.bands[2],
        "三带同参 ⇒ 三带必须逐位相同（实际 {:?}）",
        p.bands
    );
    assert!(
        p.gain == p.bands[BANDS / 2],
        "宽带口径必须就是中频带（实际 gain={} vs bands={:?}）",
        p.gain,
        p.bands
    );
    // 与 A1 的闭式口径一致：直达增益 = 1/d²
    let dist = (listener().pos - w.impacts()[0].point).length();
    assert!(
        (p.gain - 1.0 / (dist * dist)).abs() <= 1e-6 / (dist * dist),
        "退化档的 gain 必须回到 1/d²（实际 {}）",
        p.gain
    );
}

/// ② **介质分频带**：逐带衰减与 `exp(-α_b·L)/d²` 解析对拍，且**高频更闷**（严格单调）。
#[test]
fn band_dependent_medium_is_monotone_and_matches_closed_form() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new([0.25f32; BANDS], None);
    let field = LayeredMedium::split(
        1.0e9,
        Medium {
            speed_of_sound: 343.0,
            attenuation: [0.05, 0.2, 0.5],
        },
        Medium::AIR,
    );
    let g = scene.geom(Some(&field));
    let (st, paths) = cues(&w, &g);
    assert!(st.direct >= 1, "必须产出直达");
    let p = first_of(&paths, PathKind::Direct, 0);
    let dist = (listener().pos - w.impacts()[0].point).length();
    for (b, alpha) in field.medium_at_y(0.0).attenuation.iter().enumerate() {
        let expect = (-alpha * dist).exp() / (dist * dist);
        assert!(
            (p.bands[b] - expect).abs() <= expect * 1e-5,
            "第 {b} 带应为 exp(-α_b·L)/d²（实际 {}，期望 {expect}）",
            p.bands[b]
        );
    }
    assert!(
        p.bands[0] > p.bands[1] && p.bands[1] > p.bands[2],
        "吸收随频带递增 ⇒ 增益必须逐带递减（实际 {:?}）",
        p.bands
    );
}

/// ③ **透射分频带**：穿墙那条路径的三个带与 `τ_b/d²` 对拍，且高频更难透。
#[test]
fn band_dependent_transmission_is_monotone_and_matches_closed_form() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let tau = [0.4f32, 0.15, 0.03];
    let scene = Scene::new([0.25f32; BANDS], Some(([0.1f32; BANDS], tau)));
    let g = scene.geom(None);
    let (st, paths) = cues(&w, &g);
    assert!(
        st.leaked >= 1,
        "透射系数 > 0 的墙必须给出一条泄漏路径（否则判据空跑）"
    );
    assert_eq!(st.direct, 0, "被墙挡住的直达不得产出");
    let p = first_of(&paths, PathKind::Transmitted, 0);
    let dist = (listener().pos - w.impacts()[0].point).length();
    for (b, t) in tau.iter().enumerate() {
        let expect = t / (dist * dist);
        assert!(
            (p.bands[b] - expect).abs() <= expect * 1e-5,
            "第 {b} 带应为 τ_b/d²（实际 {}，期望 {expect}）",
            p.bands[b]
        );
    }
    assert!(
        p.bands[0] > p.bands[1] && p.bands[1] > p.bands[2],
        "隔声量随频率上升 ⇒ 高频带增益必须更低（实际 {:?}）",
        p.bands
    );
}

/// ④ **吸声分频带**：反射路径的**带比**必须等于面吸声给出的 `(1-a_b)` 比（解析值）。
#[test]
fn band_dependent_absorption_shows_up_in_reflections() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let a = [0.1f32, 0.5, 0.9];
    let scene = Scene::new(a, None);
    let g = scene.geom(None);
    let (st, paths) = cues(&w, &g);
    assert!(st.reflected >= 1, "地板必须给出一条反射（否则判据空跑）");
    let p = first_of(&paths, PathKind::Reflected, 0);
    // 几何与介质都与频带无关 ⇒ 带比就是 `(1-a_b)` 的比
    let expect_10 = (1.0 - a[1]) / (1.0 - a[0]);
    let expect_21 = (1.0 - a[2]) / (1.0 - a[1]);
    assert!(
        ((p.bands[1] / p.bands[0]) - expect_10).abs() <= expect_10 * 1e-5,
        "带 1/带 0 应为 (1-a₁)/(1-a₀)（实际 {}，期望 {expect_10}）",
        p.bands[1] / p.bands[0]
    );
    assert!(
        ((p.bands[2] / p.bands[1]) - expect_21).abs() <= expect_21 * 1e-5,
        "带 2/带 1 应为 (1-a₂)/(1-a₁)（实际 {}，期望 {expect_21}）",
        p.bands[2] / p.bands[1]
    );
}

/// ⑤ **确定性 + 零干扰**：`jsonl` 含 `bands` 也逐字一致；调用前后 `state_hash` 逐位不变。
#[test]
fn bands_are_deterministic_and_zero_interference() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let scene = Scene::new(
        [0.2f32, 0.6, 0.9],
        Some(([0.3f32, 0.8, 0.95], [0.4, 0.1, 0.02])),
    );
    let field = LayeredMedium::split(
        0.5,
        Medium {
            speed_of_sound: 330.0,
            attenuation: [0.02, 0.06, 0.2],
        },
        Medium::AIR,
    );
    let g = scene.geom(Some(&field));
    let a = cues(&w, &g);
    let b = cues(&w, &g);
    assert!(
        !a.1.is_empty(),
        "带介质与逐带材质的场景必须真的产出路径（否则判据空跑）"
    );
    assert!(
        jsonl(&a.1).contains("\"bands\":["),
        "jsonl 必须带逐带能量：{}",
        jsonl(&a.1)
    );
    assert_eq!(jsonl(&a.1), jsonl(&b.1), "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（逐带材质也一样，只读）"
    );
}
