//! **A2 判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A2 行）：
//! **portal 穿孔传声**（开口当二级声源）+ **无 portal 的墙按透射系数泄漏**。
//!
//! 文档给的两条硬指标就是这里的判据：① **关门 ⇒ 显著衰减 / 开门 ⇒ 恢复**；
//! ② **缝隙宽度 → 衰减单调**（严格单调 + 线性）。
//!
//! 场景：地板 + **z = 1 的实心墙**（门洞也照样用三角盖住），门洞上**叠一个声明的开口** ——
//! 开口是作者给的拓扑，不是从碰撞体或几何空洞里猜的（§4 设计原则 ②）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Listener, PathKind,
    Portal, SoundPath,
};

/// 门洞全宽（x ∈ [-1,1]）——开启比例 = 缝宽 / 本值。
const DOOR_W: f32 = 2.0;

/// 声学几何（顶点 / 三角 / 吸声 / 透射 / 开口）——打包起来，免得测试助手摊一长串形参。
struct Geo {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<f32>,
    trans: Vec<f32>,
    portals: Vec<Portal>,
}

/// 一次归约的结果（统计 + 路径）。
struct Cues {
    st: AcousticStats,
    paths: Vec<SoundPath>,
}

impl Geo {
    fn geom(&self) -> AcousticGeometry<'_> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            portals: &self.portals,
            medium: None,
        }
    }

    fn cues(&self, w: &World, cfg: &AcousticConfig) -> Cues {
        let mut paths: Vec<SoundPath> = Vec::new();
        let st = w.acoustic_cues(&self.geom(), &listener(), cfg, &mut paths);
        Cues { st, paths }
    }
}

/// 地板（8×8）+ 实心墙（z = 1、x∈[-3,3]、y∈[0,3]）+ 门洞上叠的开口。
fn scene(open: f32, transmission: f32) -> Geo {
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
        Vec3::new(-3.0, 0.0, 1.0),
        Vec3::new(3.0, 0.0, 1.0),
        Vec3::new(3.0, 3.0, 1.0),
        Vec3::new(-3.0, 3.0, 1.0),
    ];
    Geo {
        verts,
        tris: vec![[0u32, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
        absorb: vec![0.25f32, 0.25, 0.9, 0.9],
        trans: vec![0.0f32, 0.0, transmission, transmission],
        portals: vec![Portal {
            quad: [
                Vec3::new(-1.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 1.0),
                Vec3::new(1.0, 2.5, 1.0),
                Vec3::new(-1.0, 2.5, 1.0),
            ],
            open,
        }],
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
        pos: Vec3::new(0.0, 0.5, 2.0),
        radius: 0.5,
    }
}

/// 开口中心（= `Portal::quad` 四个角的算术平均，与实现同口径）。
fn door_center() -> Vec3 {
    Vec3::new(0.0, 1.25, 1.0)
}

/// 取（某声源、某类）的第一条路径；没有则给一条增益为 0 的占位 —— "到底有没有"由
/// `AcousticStats` 的计数断言。
fn first_of(paths: &[SoundPath], kind: PathKind, src: u32) -> SoundPath {
    paths
        .iter()
        .find(|p| p.kind == kind && p.source == src)
        .copied()
        .unwrap_or(SoundPath {
            source: src,
            delay: 0.0,
            gain: 0.0,
            dir: [0.0, 0.0, 0.0],
            kind,
        })
}

/// **关门 ⇒ 显著衰减 / 开门 ⇒ 恢复**：同一场景、同一听者，只翻开口的开启比例。
#[test]
fn closed_door_attenuates_and_open_door_restores() {
    let w = world_with_source();
    assert!(
        !w.impacts().is_empty(),
        "场景没产生声源（冲击）⇒ 判据会空洞"
    );
    let cfg = AcousticConfig::default();
    let shut = scene(0.0, 0.02).cues(&w, &cfg);
    let wide = scene(1.0, 0.02).cues(&w, &cfg);
    assert_eq!(shut.st.portals, 0, "关死（open=0）的开口不得产出穿孔路径");
    assert!(
        wide.st.portals >= 1,
        "全开的开口必须产出穿孔路径（否则判据空洞）"
    );
    assert_eq!(shut.st.direct, 0, "实心墙挡住的直达不得产出");
    assert!(shut.st.leaked >= 1, "关死时只剩墙的透射泄漏这条路径");
    let leak = first_of(&shut.paths, PathKind::Transmitted, 0);
    assert!(leak.gain > 0.0, "透射系数 0.02 的墙必须给出一条泄漏路径");
    // 全开：闭式对拍（增益 = 1/(d1+d2)²、时延 = (d1+d2)/c）
    let src = w.impacts()[0].point;
    let c = door_center();
    let reach = (c - src).length() + (listener().pos - c).length();
    let p = first_of(&wide.paths, PathKind::Portal, 0);
    let expect_gain = 1.0 / (reach * reach);
    assert!(
        (p.gain - expect_gain).abs() <= expect_gain * 1e-3,
        "开口增益应为 1/(d1+d2)²（实际 {}，期望 {expect_gain}）",
        p.gain
    );
    let expect_delay = reach / cfg.speed_of_sound;
    assert!(
        (p.delay - expect_delay).abs() <= expect_delay * 1e-3,
        "开口时延应按 (d1+d2)/c（实际 {}，期望 {expect_delay}）",
        p.delay
    );
    // 显著：开门 vs 关门（同一声源、同一听者）
    assert!(
        p.gain >= leak.gain * 20.0,
        "开门必须比关门显著更响（开门 {} vs 关门 {}）",
        p.gain,
        leak.gain
    );
}

/// **缝隙宽度 → 衰减单调**：缝越宽增益越高，且**正比**于缝宽（线性）。
#[test]
fn slit_width_to_attenuation_is_monotone_and_linear() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let cfg = AcousticConfig::default();
    let widths = [0.2f32, 0.5, 1.0, 2.0]; // 缝宽（米）；2.0 = 全开
    let mut gains: Vec<f32> = Vec::new();
    for wd in widths {
        let c = scene(wd / DOOR_W, 0.02).cues(&w, &cfg);
        assert!(c.st.portals >= 1, "缝宽 {wd} 必须有一条穿孔路径");
        gains.push(first_of(&c.paths, PathKind::Portal, 0).gain);
    }
    for i in 1..gains.len() {
        assert!(
            gains[i] > gains[i - 1],
            "缝越宽增益必须越高（实测 {gains:?}）"
        );
    }
    let full = gains[gains.len() - 1];
    assert!(full > 0.0, "全开必须是非零增益");
    for (i, wd) in widths.iter().enumerate() {
        let ratio = gains[i] / full;
        assert!(
            (ratio - wd / DOOR_W).abs() < 1e-5,
            "增益应正比于缝宽（缝宽 {wd} → 比值 {ratio}）"
        );
    }
}

/// **不透的墙不泄漏**：`transmission = 0` 时只有遮挡，没有泄漏路径（这条钉住 A2 的另一半）。
#[test]
fn opaque_wall_leaks_nothing() {
    let w = world_with_source();
    let c = scene(0.0, 0.0).cues(&w, &AcousticConfig::default());
    assert_eq!(c.st.direct, 0, "被挡的直达不得产出");
    assert_eq!(c.st.leaked, 0, "transmission = 0 的墙不得泄漏");
    assert_eq!(c.st.portals, 0, "关死的开口不得穿孔");
    assert!(c.st.blocked >= 1, "必须记到遮挡（否则这条判据是空跑）");
}

/// **预算**（路径数与开口数都有上界、顶格要上报）+ **确定性**（含 portal / transmitted 两类新路径）。
#[test]
fn portal_budget_reports_and_cues_are_deterministic() {
    let w = world_with_source();
    let g = scene(1.0, 0.02);
    let tight = AcousticConfig {
        max_paths: 1,
        ..AcousticConfig::default()
    };
    let c = g.cues(&w, &tight);
    assert!(
        c.paths.len() <= 1,
        "路径数必须 ≤ 预算（实际 {}）",
        c.paths.len()
    );
    assert!(c.st.dropped_budget >= 1, "预算必须被顶到过（否则判据空洞）");
    let no_portal = AcousticConfig {
        max_portals: 0,
        ..AcousticConfig::default()
    };
    assert_eq!(
        g.cues(&w, &no_portal).st.portals,
        0,
        "max_portals = 0 不得产出穿孔路径"
    );
    let a = g.cues(&w, &AcousticConfig::default());
    let b = g.cues(&w, &AcousticConfig::default());
    assert!(a.st.portals >= 1, "确定性判据要真的含穿孔路径（否则空跑）");
    assert_eq!(
        jsonl(&a.paths),
        jsonl(&b.paths),
        "同输入必须逐字一致的 cue 序列"
    );
}
