//! **A4「衍射」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A4 行）：
//! **绕射（工程近似）** —— 棱上取 Fermat 点、只在阴影里产出。
//!
//! 文档给的硬指标是"**拐角后仍可闻**的能量-角度曲线对拍"。这里落成：① 阴影里必须有绕射路径，
//! 且折线长与**独立于实现的暴力扫棱**一致；② 阴影越深越弱（严格单调）；③ 棱的**端点**解也对；
//! ④ 逐带、预算、确定性。

use vxl_phys::{BodyBuildExt, DiagExt, ImpactExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Edge, Listener, PathKind,
    SoundPath, BANDS,
};

/// 场景：地板（y = 0）+ 屏障（z = 1.0、x∈[-1,1]、y∈[0,1]）+ 作者声明的**衍射棱**。
struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    edges: Vec<Edge>,
}

impl Scene {
    /// 屏障 + 顶棱（棱只取 `x ∈ span` 那一段：可以只给半截 ⇒ 逼出**端点解**）。
    fn new(span: (f32, f32)) -> Self {
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
        s.add_barrier(1.0, -1.0, 1.0, 0.0, 1.0);
        s.edges.push(Edge {
            a: Vec3::new(span.0, 1.0, 1.0),
            b: Vec3::new(span.1, 1.0, 1.0),
            coeff: [1.0; BANDS],
            wedge: None,
        });
        s
    }

    /// 再加一堵不透明屏障（`z` 处、`x∈[x0,x1]`、`y∈[y0,y1]`）——用来挡住绕射的某一条腿。
    fn add_barrier(&mut self, z: f32, x0: f32, x1: f32, y0: f32, y1: f32) {
        let base = self.verts.len() as u32;
        self.verts.extend([
            Vec3::new(x0, y0, z),
            Vec3::new(x1, y0, z),
            Vec3::new(x1, y1, z),
            Vec3::new(x0, y1, z),
        ]);
        self.tris
            .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        self.absorb.extend([[0.1f32; BANDS]; 2]);
        self.trans.extend([[0.0f32; BANDS]; 2]);
    }

    fn geom<'a>(
        &'a self,
        medium: Option<&'a dyn vxl_phys_acoustics::AcousticMedium>,
    ) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            scattering: &[],
            portals: &[],
            edges: &self.edges,
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

fn cues(w: &World, geo: &AcousticGeometry<'_>, pos: Vec3) -> (AcousticStats, Vec<SoundPath>) {
    let listener = Listener { pos, radius: 0.5 };
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener, &AcousticConfig::default(), &mut paths);
    (st, paths)
}

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

/// **独立参照**：沿棱暴力扫出的最小折线长 `min(|src−P| + |P−dst|)` 与 argmin 的参数 `u ∈ [0,1]`。
fn brute_force(e: &Edge, src: Vec3, dst: Vec3, n: usize) -> (f32, f32) {
    let (mut best, mut best_u) = (f32::INFINITY, 0.0f32);
    for i in 0..=n {
        let u = i as f32 / n as f32;
        let p = e.a + (e.b - e.a) * u;
        let len = (p - src).length() + (p - dst).length();
        if len < best {
            best = len;
            best_u = u;
        }
    }
    (best, best_u)
}

/// ① **拐角后仍可闻**：直达被挡（`direct == 0`），但绕射给了路径；折线长与暴力扫一致。
#[test]
fn diffraction_over_a_barrier_matches_brute_force() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new((-1.0, 1.0));
    let g = scene.geom(None);
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let (st, paths) = cues(&w, &g, lst);
    assert_eq!(st.direct, 0, "屏障必须先挡住直达（否则这条判据没意义）");
    assert!(st.blocked >= 1, "必须记到遮挡");
    assert!(st.diffracted >= 1, "阴影里必须有绕射路径（拐角后仍可闻）");
    let p = first_of(&paths, PathKind::Diffracted, 0);
    assert!(p.gain > 0.0 && p.delay > 0.0, "绕射路径要有正的时延与增益");
    let src = w.impacts()[0].point;
    let (best, _) = brute_force(&scene.edges[0], src, lst, 20001);
    let got = p.delay * AcousticConfig::default().speed_of_sound;
    assert!(
        (got - best).abs() <= best * 1e-4,
        "绕射折线长应为棱上的最小值（实现 {got}，暴力扫 {best}）"
    );
    assert!(
        jsonl(&paths).contains("\"kind\":\"diffracted\""),
        "jsonl 要有 diffracted 这一类"
    );
}

/// ② **阴影越深越弱**；走出阴影后**直达接管**（绕射不再产出）。
#[test]
fn deeper_shadow_is_quieter_and_direct_takes_over() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new((-1.0, 1.0));
    let g = scene.geom(None);
    let mut prev = f32::INFINITY;
    let mut gains = Vec::new();
    for z in [2.0f32, 2.5, 3.0] {
        let (st, paths) = cues(&w, &g, Vec3::new(0.0, 0.5, z));
        assert!(st.diffracted >= 1, "z={z} 仍应在阴影里（有绕射路径）");
        let g_b = first_of(&paths, PathKind::Diffracted, 0).gain;
        assert!(
            g_b < prev,
            "阴影越深必须越弱（z={z}：{g_b} 不小于上一档 {prev}，全表 {gains:?}）"
        );
        prev = g_b;
        gains.push(g_b);
    }
    // 走出阴影（y 抬到屏障顶以上 ⇒ 直达通视）：直达接管，绕射不再产出
    let (st_out, paths_out) = cues(&w, &g, Vec3::new(0.0, 2.5, 2.0));
    assert!(st_out.direct >= 1, "高出屏障后直达必须通视");
    assert_eq!(
        st_out.diffracted, 0,
        "不在阴影里就不该再产出绕射（避免与直达重复计能）"
    );
    assert_eq!(first_of(&paths_out, PathKind::Diffracted, 0).gain, 0.0);
}

/// ③ **端点解**：棱只给半截 ⇒ Fermat 点被夹到端点，折线长仍与暴力扫（同一段棱）一致。
#[test]
fn endpoint_diffraction_when_the_foot_is_outside() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::new((0.5, 1.0)); // 顶棱只留右半截
    let g = scene.geom(None);
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let (st, paths) = cues(&w, &g, lst);
    assert!(st.diffracted >= 1, "半截棱也要能绕");
    let src = w.impacts()[0].point;
    let (best, best_u) = brute_force(&scene.edges[0], src, lst, 20001);
    assert!(
        best_u <= 0.02,
        "这段几何的最优点应当就在棱的端点上（argmin u={best_u}）"
    );
    let got =
        first_of(&paths, PathKind::Diffracted, 0).delay * AcousticConfig::default().speed_of_sound;
    assert!(
        (got - best).abs() <= best * 1e-4,
        "端点解也要与暴力扫一致（实现 {got}，暴力扫 {best}）"
    );
}

/// ④ **腿被挡就没有绕射**（非空洞：同一场景去掉那堵墙就又有）。
#[test]
fn blocked_legs_kill_the_diffraction() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let clean = Scene::new((-1.0, 1.0));
    let (st_clean, _) = cues(&w, &clean.geom(None), lst);
    assert!(
        st_clean.diffracted >= 1,
        "干净场景必须有绕射（否则这条判据空跑）"
    );
    // 在绕射的"第二腿"上再插一堵墙（`z = 1.5`、`y∈[0,0.8]`）
    let mut blocked = Scene::new((-1.0, 1.0));
    blocked.add_barrier(1.5, -1.0, 1.0, 0.0, 0.8);
    let (st_blocked, paths) = cues(&w, &blocked.geom(None), lst);
    assert_eq!(
        st_blocked.diffracted,
        0,
        "绕射的腿被挡死 ⇒ 不得产出绕射路径（实际 {:?}）",
        jsonl(&paths)
    );
}

/// ⑤ **逐带 + 预算 + 确定性**：绕射系数逐带 ⇒ 带比就是系数比；`max_edges = 0` 关掉；两跑逐字一致。
#[test]
fn diffraction_bands_budget_and_determinism() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let mut scene = Scene::new((-1.0, 1.0));
    scene.edges[0].coeff = [1.0, 0.5, 0.2];
    let g = scene.geom(None);
    let a = cues(&w, &g, lst);
    let b = cues(&w, &g, lst);
    assert!(a.0.diffracted >= 1, "必须产出绕射（否则判据空跑）");
    let p = first_of(&a.1, PathKind::Diffracted, 0);
    assert!(
        p.bands[0] > p.bands[1] && p.bands[1] > p.bands[2],
        "绕射系数逐带 ⇒ 逐带增益必须递减（实际 {:?}）",
        p.bands
    );
    let expect = scene.edges[0].coeff[1] / scene.edges[0].coeff[0];
    assert!(
        ((p.bands[1] / p.bands[0]) - expect).abs() <= expect * 1e-5,
        "带比必须等于系数比（实际 {}，期望 {expect}）",
        p.bands[1] / p.bands[0]
    );
    assert_eq!(jsonl(&a.1), jsonl(&b.1), "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（绕射也一样，只读）"
    );
    // `max_edges = 0` ⇒ 关掉绕射（但不是"静默"：统计如实给 0）
    let off = AcousticConfig {
        max_edges: 0,
        ..AcousticConfig::default()
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: lst,
        radius: 0.5,
    };
    let st = w.acoustic_cues(&g, &listener, &off, &mut paths);
    assert_eq!(st.diffracted, 0, "max_edges = 0 时不得产出绕射路径");
}
