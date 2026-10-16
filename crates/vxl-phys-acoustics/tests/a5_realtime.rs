//! **A5 判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A5 行）：
//! **动态源/听者 + 预算与实时档**。
//!
//! 文档给的两条硬指标是：① **移动声源的 cue 连续性（无跳变）**；② **预算被顶满时上报而非静默丢**。
//! 另外补两条这一期引入的契约：③ 截断按**优先级（能量）**而不是产出顺序；④ 档位（[`AcousticTier`]）
//! 是**确定性**的工作量上界（与机器/负载/时间无关）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticStats, AcousticTier, AcousticsExt, Edge,
    Listener, PathKind, Portal, SoundPath, BANDS,
};

/// 房间的几何表：顶点 / 三角 / 逐带吸声 / 逐带透射 / 逐带散射。
type RoomTables = (
    Vec<Vec3>,
    Vec<[u32; 3]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
);

/// 一间"有内容"的声学房间：地板 + 带门洞的墙 + 衍射棱 + 高天花板 —— 五类路径都能出。
fn room() -> RoomTables {
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 1.5),
        Vec3::new(4.0, 0.0, 1.5),
        Vec3::new(4.0, 3.0, 1.5),
        Vec3::new(-4.0, 3.0, 1.5),
        Vec3::new(-4.0, 6.0, 1.5),
        Vec3::new(4.0, 6.0, 1.5),
        Vec3::new(4.0, 6.0, 4.0),
        Vec3::new(-4.0, 6.0, 4.0),
    ];
    let tris = vec![
        [0u32, 1, 2],
        [0, 2, 3],
        [4, 5, 6],
        [4, 6, 7],
        [8, 9, 10],
        [8, 10, 11],
    ];
    let absorb = vec![
        [0.25f32; BANDS],
        [0.25; BANDS],
        [0.6, 0.9, 0.95],
        [0.6, 0.9, 0.95],
        [0.4; BANDS],
        [0.4; BANDS],
    ];
    let trans = vec![
        [0.0f32; BANDS],
        [0.0; BANDS],
        [0.03, 0.02, 0.008],
        [0.03, 0.02, 0.008],
        [0.0; BANDS],
        [0.0; BANDS],
    ];
    let scatter = vec![
        [0.2f32; BANDS],
        [0.2; BANDS],
        [0.35, 0.5, 0.6],
        [0.35, 0.5, 0.6],
        [0.3; BANDS],
        [0.3; BANDS],
    ];
    (verts, tris, absorb, trans, scatter)
}

/// 门户 + 门框衍射棱（与 CLI 的房间同款）。
fn portal() -> Portal {
    Portal {
        quad: [
            Vec3::new(-1.0, 0.0, 1.5),
            Vec3::new(1.0, 0.0, 1.5),
            Vec3::new(1.0, 2.5, 1.5),
            Vec3::new(-1.0, 2.5, 1.5),
        ],
        open: 1.0,
        regions: None,
    }
}

fn edge() -> Edge {
    Edge {
        a: Vec3::new(1.0, 0.0, 1.5),
        b: Vec3::new(1.0, 2.5, 1.5),
        coeff: [0.5, 0.35, 0.2],
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

fn cues_at(
    w: &World,
    geo: &AcousticGeometry<'_>,
    pos: Vec3,
    cfg: &AcousticConfig,
) -> (AcousticStats, Vec<SoundPath>) {
    let listener = Listener { pos, radius: 0.5 };
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(geo, &listener, cfg, &mut paths);
    (st, paths)
}

/// ① **预算按优先级截断**：留下的必须是**最响的那几条**，且丢弃数如实上报。
#[test]
fn budget_keeps_the_loudest_paths() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let (verts, tris, absorb, trans, scatter) = room();
    let portals = [portal()];
    let edges = [edge()];
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &edges,
        medium: None,
    };
    let lst = Vec3::new(0.0, 1.0, 3.0);
    let wide = AcousticConfig {
        max_paths: 64,
        ..AcousticConfig::default()
    };
    let (st_all, all) = cues_at(&w, &geo, lst, &wide);
    assert!(
        all.len() >= 3,
        "这间房必须产出多条候选（实际 {}）",
        all.len()
    );
    let mut sorted: Vec<SoundPath> = all.clone();
    sorted.sort_by(|a, b| b.gain.total_cmp(&a.gain));
    for keep in 1..=3usize {
        let tight = AcousticConfig {
            max_paths: keep,
            ..AcousticConfig::default()
        };
        let (st, got) = cues_at(&w, &geo, lst, &tight);
        assert_eq!(got.len(), keep, "keep={keep} 时必须恰好留 {keep} 条");
        assert_eq!(
            jsonl(&got),
            jsonl(&sorted[..keep]),
            "留下的必须是**最响的** {keep} 条（keep={keep}）"
        );
        assert_eq!(
            st.dropped_budget,
            all.len() - keep,
            "丢弃数必须如实上报（keep={keep}）"
        );
    }
    assert_eq!(st_all.dropped_budget, 0, "宽预算下不该有丢弃");
}

/// ② **实时档是确定性的工作量上界**：档位越省，候选越少；`Minimal` 不算反射；顶满时上报。
#[test]
fn tiers_are_deterministic_work_caps() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let (verts, tris, absorb, trans, scatter) = room();
    let portals = [portal()];
    let edges = [edge()];
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &edges,
        medium: None,
    };
    let lst = Vec3::new(0.0, 1.0, 3.0);
    // 档位表本身（确定性预设）
    assert_eq!(AcousticTier::Minimal.config().max_reflections, 0);
    assert!(AcousticTier::Realtime.config().max_paths < AcousticTier::Offline.config().max_paths);
    assert_eq!(AcousticTier::Offline.config(), AcousticConfig::default());
    let mut counts = Vec::new();
    for tier in [
        AcousticTier::Offline,
        AcousticTier::Realtime,
        AcousticTier::Minimal,
    ] {
        let (st, paths) = cues_at(&w, &geo, lst, &tier.config());
        assert!(
            paths.len() <= tier.config().max_paths,
            "{tier:?} 的产出必须 ≤ 它的 max_paths"
        );
        if tier == AcousticTier::Minimal {
            assert_eq!(st.reflected, 0, "Minimal 档不算反射");
            assert!(
                st.dropped_budget >= 1,
                "Minimal 档必须真的顶到过预算（否则空跑）"
            );
        }
        counts.push(paths.len());
    }
    assert!(
        counts[0] >= counts[1] && counts[1] >= counts[2],
        "档越省候选越少（Offline/Realtime/Minimal = {counts:?}）"
    );
    // 同档两跑逐字一致（档位不引入不确定性）
    let a = cues_at(&w, &geo, lst, &AcousticTier::Realtime.config());
    let b = cues_at(&w, &geo, lst, &AcousticTier::Realtime.config());
    assert_eq!(jsonl(&a.1), jsonl(&b.1), "同档两跑必须逐字一致");
}

/// ③ **移动听者的连续性（无跳变）**：时延受几何界约束、增益受 Lipschitz 界约束，且路径数不抖。
#[test]
fn moving_listener_keeps_cues_continuous() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    // 开阔场景（只有地板）⇒ 直达一路都在，专测"连续"这件事
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let absorb = vec![[0.25f32; BANDS], [0.25; BANDS]];
    let trans = vec![[0.0f32; BANDS], [0.0; BANDS]];
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: None,
    };
    let cfg = AcousticConfig::default();
    let src = w.impacts()[0].point;
    let steps = 60usize;
    let step_len = 0.02f32;
    let mut prev: Option<SoundPath> = None;
    let mut count_changes = 0usize;
    let mut prev_count = 0usize;
    for i in 0..=steps {
        let pos = Vec3::new(0.0, 1.0, 1.0 + step_len * i as f32);
        let (_, paths) = cues_at(&w, &geo, pos, &cfg);
        let d = paths
            .iter()
            .find(|p| p.kind == PathKind::Direct && p.source == 0)
            .copied();
        assert!(d.is_some(), "开阔场景第 {i} 步必须还有直达");
        if let Some(p) = d {
            let dist = (pos - src).length();
            let bound_delay = step_len / cfg.speed_of_sound + 1e-6;
            let bound_gain = 2.0 / (dist - step_len).max(0.1).powi(3) * step_len + 1e-6;
            if let Some(q) = prev {
                assert!(
                    (p.delay - q.delay).abs() <= bound_delay,
                    "第 {i} 步时延跳了 {}（界 {bound_delay}）",
                    (p.delay - q.delay).abs()
                );
                assert!(
                    (p.gain - q.gain).abs() <= bound_gain,
                    "第 {i} 步增益跳了 {}（界 {bound_gain}）",
                    (p.gain - q.gain).abs()
                );
            }
            prev = Some(p);
        }
        if i > 0 && paths.len() != prev_count {
            count_changes += 1;
        }
        prev_count = paths.len();
    }
    let end_z = 1.0 + step_len * steps as f32;
    assert!(end_z > 2.0, "扫过的距离要够（终点 z = {}）", end_z);
    assert!(
        count_changes <= 3,
        "路径数不该抖（整段只变了 {count_changes} 次）"
    );
}

/// ④ **移动听者下逐位可复现**：同一条扫掠跑两遍 ⇒ 逐字一致；`state_hash` 不变。
#[test]
fn moving_listener_is_bitwise_reproducible() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let absorb = vec![[0.25f32; BANDS], [0.25; BANDS]];
    let trans = vec![[0.0f32; BANDS], [0.0; BANDS]];
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: None,
    };
    let cfg = AcousticTier::Realtime.config();
    let sweep = |w: &World| -> String {
        let mut s = String::new();
        for i in 0..=20 {
            let pos = Vec3::new(0.0, 1.0, 1.0 + 0.02 * i as f32);
            let (_, paths) = cues_at(w, &geo, pos, &cfg);
            s.push_str(&jsonl(&paths));
        }
        s
    };
    let (a, b) = (sweep(&w), sweep(&w));
    assert!(!a.is_empty(), "扫掠必须真的产出路径（否则判据空跑）");
    assert_eq!(a, b, "同一条扫掠两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（移动听者也一样，只读）"
    );
}
