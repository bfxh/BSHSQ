//! **A3 判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A3 行）：
//! **介质体积场**（声速 + 沿程吸收）。
//!
//! 文档给的两条硬指标就是这里的判据：① **穿越不同介质层的时延/衰减与解析式对拍**；
//! ② **同场景两跑逐字一致**（另加一条：介质层开着时 `state_hash` 仍逐位不变）。
//!
//! 场景：地板（只有声学几何、没有碰撞体）+ 一发冲击当声源；介质按 `y` 分层。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticStats, AcousticsExt,
    LayeredMedium, Listener, Medium, PathKind, SoundPath, BANDS,
};

/// 水：声速 1500 m/s + 明显吸收（0.2 /m）—— 只为一眼看得出差异的数值实验。
const WATER: Medium = Medium {
    speed_of_sound: 1500.0,
    attenuation: [0.2; BANDS],
};

/// 分界面高度：声源在 y ≈ 0.25、听者在 y = 1.5 ⇒ 直达路径在 `t = 0.2` 处穿过它。
const SPLIT: f32 = 0.5;

/// 只有声学几何的地板（两块三角）——"声学几何 ≠ 碰撞几何"的前提场景。
struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
}

impl Scene {
    fn new() -> Self {
        Self {
            verts: vec![
                Vec3::new(-4.0, 0.0, -4.0),
                Vec3::new(4.0, 0.0, -4.0),
                Vec3::new(4.0, 0.0, 4.0),
                Vec3::new(-4.0, 0.0, 4.0),
            ],
            tris: vec![[0u32, 1, 2], [0, 2, 3]],
            absorb: vec![[0.25f32; BANDS]; 2],
            trans: vec![[0.0f32; BANDS]; 2],
        }
    }

    fn geom<'a>(&'a self, medium: Option<&'a dyn AcousticMedium>) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            portals: &[],
            edges: &[],
            medium,
        }
    }
}

/// 只实现 `at`（**不给** `segment_exact`）的介质场 ⇒ 逼 [`seg_effect`] 走固定步长中点法回退分支。
struct SampledAirWater {
    split: f32,
}

impl AcousticMedium for SampledAirWater {
    fn at(&self, p: Vec3) -> Medium {
        if p.y < self.split {
            WATER
        } else {
            Medium::AIR
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

fn listener() -> Listener {
    Listener {
        pos: Vec3::new(0.0, 1.5, 2.0),
        radius: 0.5,
    }
}

/// 归约一次，取**声源 0** 的直达路径（判据只看它）。
fn direct_of(
    w: &World,
    geo: &AcousticGeometry<'_>,
    cfg: &AcousticConfig,
) -> (AcousticStats, SoundPath) {
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener(), cfg, &mut paths);
    let d = paths
        .iter()
        .find(|p| p.kind == PathKind::Direct && p.source == 0)
        .copied()
        .unwrap_or(SoundPath {
            source: 0,
            delay: 0.0,
            bands: [0.0; BANDS],
            gain: 0.0,
            dir: [0.0, 0.0, 0.0],
            kind: PathKind::Direct,
        });
    (st, d)
}

/// **均匀介质与解析式对拍**：整段都在水里 ⇒ 时延 = L/c、衰减 = `exp(-α·L)`（`1/d²` 之外）。
#[test]
fn uniform_medium_matches_closed_form() {
    let w = world_with_source();
    assert!(
        !w.impacts().is_empty(),
        "场景没产生声源（冲击）⇒ 判据会空洞"
    );
    let cfg = AcousticConfig::default();
    let scene = Scene::new();
    let src = w.impacts()[0].point;
    let l = listener().pos;
    let dist = (l - src).length();
    // 空气（无介质场）与全水（分层场的两层同参）各来一次
    let air = {
        let g = scene.geom(None);
        direct_of(&w, &g, &cfg)
    };
    let water_field = LayeredMedium::split(1.0e9, WATER, WATER);
    let water = {
        let g = scene.geom(Some(&water_field));
        direct_of(&w, &g, &cfg)
    };
    assert!(
        air.0.direct >= 1 && water.0.direct >= 1,
        "两种档都必须产出直达"
    );
    let expect_delay = dist / WATER.speed_of_sound;
    let expect_gain = (-WATER.attenuation[BANDS / 2] * dist).exp() / (dist * dist);
    assert!(
        (water.1.delay - expect_delay).abs() <= expect_delay * 1e-5,
        "全水时延应为 L/c（实际 {}，期望 {expect_delay}）",
        water.1.delay
    );
    assert!(
        (water.1.gain - expect_gain).abs() <= expect_gain * 1e-4,
        "全水增益应为 exp(-αL)/L²（实际 {}，期望 {expect_gain}）",
        water.1.gain
    );
    assert!(
        water.1.delay < air.1.delay,
        "声速更高的介质必须更快（水 {} vs 空气 {}）",
        water.1.delay,
        air.1.delay
    );
}

/// **穿层与解析式对拍**：下层水、上层空气 ⇒ 时延 = d₁/c₁ + d₂/c₂、衰减 = `exp(-α₁·d₁)`。
#[test]
fn two_layer_crossing_matches_closed_form() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let cfg = AcousticConfig::default();
    let scene = Scene::new();
    let src = w.impacts()[0].point;
    let l = listener().pos;
    let t = (SPLIT - src.y) / (l.y - src.y);
    assert!(t > 1e-3 && t < 1.0 - 1e-3, "分层必须真的被穿过（t={t}）");
    let hit = src + (l - src) * t;
    let (d1, d2) = ((hit - src).length(), (l - hit).length());
    let field = LayeredMedium::split(SPLIT, WATER, Medium::AIR);
    let g = scene.geom(Some(&field));
    let (st, p) = direct_of(&w, &g, &cfg);
    assert!(st.direct >= 1, "穿层场景必须产出直达");
    let expect_delay = d1 / WATER.speed_of_sound + d2 / Medium::AIR.speed_of_sound;
    let dist = (l - src).length();
    let expect_gain = (-WATER.attenuation[BANDS / 2] * d1).exp() / (dist * dist);
    assert!(
        (p.delay - expect_delay).abs() <= expect_delay * 1e-4,
        "穿层时延应为 d₁/c₁ + d₂/c₂（实际 {}，期望 {expect_delay}）",
        p.delay
    );
    assert!(
        (p.gain - expect_gain).abs() <= expect_gain * 1e-4,
        "穿层衰减应为 exp(-α₁d₁)/L²（实际 {}，期望 {expect_gain}）",
        p.gain
    );
}

/// **采样回退**：不给解析积分的场走固定步长中点法 —— 结果夹在"全水 / 全空气"之间，
/// 误差不超过**一个步长的弧长差**（原理性的界），且**步长越小越贴解析值**。
#[test]
fn sampled_fallback_brackets_and_approximates_exact() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let cfg = AcousticConfig::default();
    let scene = Scene::new();
    let sampled = SampledAirWater { split: SPLIT };
    let two_layer = LayeredMedium::split(SPLIT, WATER, Medium::AIR);
    let water_field = LayeredMedium::split(1.0e9, WATER, WATER);
    let s = {
        let g = scene.geom(Some(&sampled));
        direct_of(&w, &g, &cfg)
    };
    let e = {
        let g = scene.geom(Some(&two_layer));
        direct_of(&w, &g, &cfg)
    };
    let lo = {
        let g = scene.geom(Some(&water_field));
        direct_of(&w, &g, &cfg)
    };
    let hi = {
        let g = scene.geom(None);
        direct_of(&w, &g, &cfg)
    };
    assert!(
        s.1.delay > lo.1.delay && s.1.delay < hi.1.delay,
        "采样结果必须夹在\"全水/全空气\"之间（采样 {} / 水 {} / 空气 {}）",
        s.1.delay,
        lo.1.delay,
        hi.1.delay
    );
    // 中点法最多把**一个步长的弧长**记成另一种介质 ⇒ 时延误差 ≤ `step · span`（原理性的界）。
    let span = 1.0 / Medium::AIR.speed_of_sound - 1.0 / WATER.speed_of_sound;
    let e_coarse = (s.1.delay - e.1.delay).abs();
    assert!(
        e_coarse <= cfg.medium_step * span + 1e-9,
        "粗步长误差应 ≤ step·span（实测 {e_coarse}，界 {}）",
        cfg.medium_step * span
    );
    // 步长 10× 更小 ⇒ 误差必须更小（证明"步长真的在驱动精度"，采样循环不是空跑）。
    let fine_cfg = AcousticConfig {
        medium_step: 0.05,
        ..AcousticConfig::default()
    };
    let f = {
        let g = scene.geom(Some(&sampled));
        direct_of(&w, &g, &fine_cfg)
    };
    let e_fine = (f.1.delay - e.1.delay).abs();
    assert!(
        e_fine <= fine_cfg.medium_step * span + 1e-9,
        "细步长误差应 ≤ step·span（实测 {e_fine}，界 {}）",
        fine_cfg.medium_step * span
    );
    assert!(
        e_fine < e_coarse,
        "步长更小必须更贴解析值（细 {e_fine} vs 粗 {e_coarse}）"
    );
}

/// **确定性 + 零干扰**：介质层开着 ⇒ 两跑逐字一致；调用前后 `state_hash` 逐位不变。
#[test]
fn medium_is_deterministic_and_zero_interference() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let cfg = AcousticConfig::default();
    let scene = Scene::new();
    let field = LayeredMedium::split(SPLIT, WATER, Medium::AIR);
    let g = scene.geom(Some(&field));
    let a = direct_of(&w, &g, &cfg);
    let b = direct_of(&w, &g, &cfg);
    assert!(
        a.0.direct >= 1,
        "介质档必须真的产出路径（否则这条判据空跑）"
    );
    let (mut pa, mut pb) = (Vec::new(), Vec::new());
    pa.push(a.1);
    pb.push(b.1);
    assert_eq!(jsonl(&pa), jsonl(&pb), "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（介质场也一样，只读）"
    );
}
