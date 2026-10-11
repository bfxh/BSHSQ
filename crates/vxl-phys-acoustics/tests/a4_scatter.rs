//! **A4「散射」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A4 行：散射系数）：
//! 镜面那条按 `(1 - s_b)` 打折，剩下的能量由**漫反射二级声源**（`Scattered`）接手。
//!
//! 文档给的硬指标是"**散射开/关的差异可测**"。这里落成：① 镜面与散射的**能量分工**；
//! ② 散射增益与闭式口径对拍、且对系数**线性**；③ **Lambertian**（同距离下绕离法向就变弱）；
//! ④ 腿被挡就没有；⑤ 逐带、预算、确定性。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticStats, AcousticsExt, Listener,
    PathKind, SoundPath, BANDS,
};

/// 场景：**单三角地板**（声源在它内部 ⇒ 散射面唯一，闭式好算）+ 可选的挡板。
struct Scene {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    scatter: Vec<[f32; BANDS]>,
}

impl Scene {
    /// 单三角地板 `(-4,0,-4) (4,0,-4) (0,0,4)`，逐面散射系数 `s`。
    fn floor(s: [f32; BANDS]) -> Self {
        Self {
            verts: vec![
                Vec3::new(-4.0, 0.0, -4.0),
                Vec3::new(4.0, 0.0, -4.0),
                Vec3::new(0.0, 0.0, 4.0),
            ],
            tris: vec![[0u32, 1, 2]],
            absorb: vec![[0.25f32; BANDS]],
            trans: vec![[0.0f32; BANDS]],
            scatter: vec![s],
        }
    }

    /// 水平挡板（`y` 处、`x,z ∈ [-2,2]`，散射 0）—— 用来挡住散射的腿。
    fn add_plate(&mut self, y: f32) {
        let base = self.verts.len() as u32;
        self.verts.extend([
            Vec3::new(-2.0, y, -2.0),
            Vec3::new(2.0, y, -2.0),
            Vec3::new(2.0, y, 2.0),
            Vec3::new(-2.0, y, 2.0),
        ]);
        self.tris
            .extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        self.absorb.extend([[0.1f32; BANDS]; 2]);
        self.trans.extend([[0.0f32; BANDS]; 2]);
        self.scatter.extend([[0.0f32; BANDS]; 2]);
    }

    /// 地板的中心与单位法向（测试自己算，不借实现的私有件）。
    ///
    /// 法向朝 **+y** 一侧（这两组测试场景的地板都在 y = 0 平面、声源在上方）。
    fn floor_center_normal(&self) -> (Vec3, Vec3) {
        let (a, b, c) = (self.verts[0], self.verts[1], self.verts[2]);
        let p = (a + b + c) * (1.0 / 3.0);
        let cross = (b - a).cross(c - a);
        let n = if cross.y < 0.0 { cross * -1.0 } else { cross };
        (p, n * (1.0 / n.length()))
    }

    fn geom<'a>(&'a self, medium: Option<&'a dyn AcousticMedium>) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            scattering: &self.scatter,
            portals: &[],
            edges: &[],
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

/// ① **能量分工**：`s = 0.6` ⇒ 镜面那条恰好是原来的 `0.4`，且多出散射路径。
#[test]
fn scattering_splits_the_specular_energy() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let lst = Vec3::new(0.0, 1.0, 1.0);
    let plain = Scene::floor([0.0f32; BANDS]);
    let rough = Scene::floor([0.6f32; BANDS]);
    let (st0, p0) = cues(&w, &plain.geom(None), lst);
    let (st1, p1) = cues(&w, &rough.geom(None), lst);
    assert!(st0.reflected >= 1 && st1.reflected >= 1, "地板必须给反射");
    assert_eq!(st0.scattered, 0, "散射系数 0 ⇒ 不得有散射路径（纯镜面）");
    assert_eq!(st1.scattered, 1, "单三角地板 ⇒ 恰好一条散射路径");
    let g0 = first_of(&p0, PathKind::Reflected, 0).gain;
    let g1 = first_of(&p1, PathKind::Reflected, 0).gain;
    assert!(
        (g1 - g0 * 0.4).abs() <= g0 * 1e-5,
        "镜面那条必须按 (1-s) 打折（s=0 时 {g0}，s=0.6 时 {g1}）"
    );
    assert!(g0 > 0.0, "基准增益必须为正");
    assert!(
        first_of(&p1, PathKind::Scattered, 0).gain > 0.0,
        "散射路径必须有正增益"
    );
}

/// ② **闭式对拍 + 对系数线性**：散射增益 = `s · cos_in · cos_out · A / (d1² d2²)`。
#[test]
fn scattered_gain_matches_closed_form_and_is_linear() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::floor([1.0f32; BANDS]);
    let (p, n) = scene.floor_center_normal();
    let lst = Vec3::new(0.0, 1.0, 1.0);
    let (st, paths) = cues(&w, &scene.geom(None), lst);
    assert!(st.scattered >= 1, "散射必须产出（否则判据空跑）");
    let got = first_of(&paths, PathKind::Scattered, 0).gain;
    // 手算口径（与实现同式，但独立写一遍）
    let src = w.impacts()[0].point;
    let (a, b, c) = (scene.verts[0], scene.verts[1], scene.verts[2]);
    let area = 0.5 * (b - a).cross(c - a).length();
    let (v1, v2) = (p - src, lst - p);
    let (d1, d2) = (v1.length(), v2.length());
    let cos_in = n.dot(v1 * (1.0 / d1)).abs();
    let cos_out = n.dot(v2 * (1.0 / d2)).abs();
    let expect = 1.0 * cos_in * cos_out * area / (d1 * d1 * d2 * d2);
    assert!(
        (got - expect).abs() <= expect * 1e-4,
        "散射增益应等于闭式口径（实际 {got}，期望 {expect}）"
    );
    // 线性：s 减半 ⇒ 增益减半
    let half = Scene::floor([0.5f32; BANDS]);
    let (_, ph) = cues(&w, &half.geom(None), lst);
    let got_half = first_of(&ph, PathKind::Scattered, 0).gain;
    assert!(
        (got_half - got * 0.5).abs() <= got * 1e-5,
        "散射增益必须与系数成正比（1.0 → {got}，0.5 → {got_half}）"
    );
}

/// ③ **Lambertian**：同距离下，绕离法向 `60°`（`cos_out = 0.5`）⇒ 恰好弱一半。
#[test]
fn scattered_energy_is_lambertian() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let scene = Scene::floor([1.0f32; BANDS]);
    let (p, n) = scene.floor_center_normal();
    let r = 1.0f32;
    let straight = p + n * r;
    // 取一条与法向正交的切线（地板平面内的方向即可）
    let tan = (Vec3::new(1.0, 0.0, 0.0) - n * n.x).normalize();
    let tilted = p + (n * 0.5 + tan * 0.8660254) * r;
    let (_, ps) = cues(&w, &scene.geom(None), straight);
    let (_, pt) = cues(&w, &scene.geom(None), tilted);
    let gs = first_of(&ps, PathKind::Scattered, 0).gain;
    let gt = first_of(&pt, PathKind::Scattered, 0).gain;
    assert!(gs > 0.0 && gt > 0.0, "两个位置都要有散射（否则判据空跑）");
    assert!(
        (gs - gt * 2.0).abs() <= gs * 1e-4,
        "同距离下 cos_out 减半 ⇒ 增益减半（法向 {gs} vs 斜 60° {gt}）"
    );
}

/// ④ **腿被挡就没有散射**（非空洞：同一场景去掉挡板就又有）。
#[test]
fn blocked_scatter_legs_produce_nothing() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let lst = Vec3::new(0.0, 1.5, 1.0);
    let clean = Scene::floor([1.0f32; BANDS]);
    let (st_clean, _) = cues(&w, &clean.geom(None), lst);
    assert!(
        st_clean.scattered >= 1,
        "干净场景必须有散射（否则这条判据空跑）"
    );
    let mut blocked = Scene::floor([1.0f32; BANDS]);
    blocked.add_plate(0.5);
    let (st_blocked, paths) = cues(&w, &blocked.geom(None), lst);
    assert_eq!(
        st_blocked.scattered,
        0,
        "散射的腿被挡死 ⇒ 不得产出散射路径（实际 {:?}）",
        jsonl(&paths)
    );
}

/// ⑤ **逐带 + 预算 + 确定性**：系数逐带 ⇒ 带比等于系数比；`max_scatters = 0` 关掉；两跑逐字一致。
#[test]
fn scatter_bands_budget_and_determinism() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let lst = Vec3::new(0.0, 1.0, 1.0);
    let scene = Scene::floor([0.2f32, 0.5, 0.9]);
    let g = scene.geom(None);
    let a = cues(&w, &g, lst);
    let b = cues(&w, &g, lst);
    assert!(a.0.scattered >= 1, "必须产出散射（否则判据空跑）");
    let p = first_of(&a.1, PathKind::Scattered, 0);
    assert!(
        p.bands[0] < p.bands[1] && p.bands[1] < p.bands[2],
        "散射系数逐带 ⇒ 逐带增益必须递增（实际 {:?}）",
        p.bands
    );
    let expect = 0.5 / 0.2;
    assert!(
        ((p.bands[1] / p.bands[0]) - expect).abs() <= expect * 1e-5,
        "带比必须等于系数比（实际 {}，期望 {expect}）",
        p.bands[1] / p.bands[0]
    );
    assert_eq!(jsonl(&a.1), jsonl(&b.1), "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（散射也一样，只读）"
    );
    // `max_scatters = 0` ⇒ 关掉散射
    let off = AcousticConfig {
        max_scatters: 0,
        ..AcousticConfig::default()
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: lst,
        radius: 0.5,
    };
    let st = w.acoustic_cues(&g, &listener, &off, &mut paths);
    assert_eq!(st.scattered, 0, "max_scatters = 0 时不得产出散射路径");
}
