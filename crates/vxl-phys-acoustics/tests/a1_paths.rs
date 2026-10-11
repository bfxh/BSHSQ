//! **A1 判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §5 的 A1 行）：
//! 直达 + 一次镜面反射（镜像源）+ 遮挡，单频带材质表；三条契约（零干扰 / 确定性 / 预算）。
//!
//! 场景刻意**只有声学几何、没有任何碰撞体**：这就是"声学几何 ≠ 碰撞几何"这条设计原则的可执行形式。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticsExt, Listener, PathKind, SoundPath, BANDS,
};

/// 一块 8×8 的地板（y = 0，两块三角）+ 一堵墙（可选，x = 0 平面、y∈[0,3]、z∈[-1,1]）。
/// 场景表：顶点 / 三角 / 逐带吸声 / 逐带透射。
type Tables = (
    Vec<Vec3>,
    Vec<[u32; 3]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
);

fn geo(with_wall: bool) -> Tables {
    let mut verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
    ];
    let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let mut absorb = vec![[0.25f32; BANDS]; 2];
    let mut trans = vec![[0.0f32; BANDS]; 2]; // A1 场景：面全不透（墙靠 `blocked` 丢）
    if with_wall {
        let base = verts.len() as u32;
        verts.extend([
            // 立在 z = 1 的 x-y 平面（声源在 z≈0、听者在 z=2 ⇒ 墙正好夹在中间）。
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 3.0, 1.0),
            Vec3::new(-1.0, 3.0, 1.0),
        ]);
        tris.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        absorb.extend([[0.9f32; BANDS]; 2]); // 强吸声的墙（挡直达，反射也弱）
        trans.extend([[0.0f32; BANDS]; 2]);
    }
    (verts, tris, absorb, trans)
}

/// 一个"声源"：把一发冲击打进世界（`impacts` 是声学层的声源来源——与破坏/感官同源）。
fn world_with_source(at: Vec3) -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.0, -0.25, -1.0), 0.25, 8, 1, 8);
    vol.fill_box(Vec3::new(-1.0, -0.25, -1.0), Vec3::new(1.0, 0.0, 1.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        at,
        Quat::IDENTITY,
        1000.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(0.0, -6.0, 0.0);
    // 落到地面上为止（声源 = `impacts` 的接触点；落地前没有冲击 ⇒ 判据会空洞）。
    for _ in 0..40 {
        w.step();
        if !w.impacts().is_empty() {
            break;
        }
    }
    w
}

fn listener_at(z: f32) -> Listener {
    Listener {
        pos: Vec3::new(0.0, 0.5, z),
        radius: 0.5,
    }
}

/// **闭式对拍**：直达时延 = 距离/声速、增益 = 1/d²；反射路径长度 = 镜像距离（镜像源法的解析值）。
#[test]
fn closed_form_direct_and_first_reflection() {
    let (verts, tris, absorb, trans) = geo(false);
    let g = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        portals: &[],
        medium: None,
    };
    let w = world_with_source(Vec3::new(0.0, 1.0, 0.0));
    let cfg = AcousticConfig::default();
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(&g, &listener_at(2.0), &cfg, &mut paths);
    assert!(st.sources > 0, "场景没产生声源（冲击）⇒ 判据会空洞");
    assert!(st.direct > 0, "开阔场景必须至少有一条直达路径");
    // 直达：方向 +z（从听者指向声源）、时延/增益按解析式
    let d = paths
        .iter()
        .find(|p| p.kind == PathKind::Direct)
        .copied()
        .unwrap_or(SoundPath {
            source: 0,
            delay: 0.0,
            bands: [0.0; BANDS],
            gain: 0.0,
            dir: [0.0, 0.0, 0.0],
            kind: PathKind::Direct,
        });
    assert!(d.dir[2] < -0.9, "直达方向应指向 -z（实际 {:?}）", d.dir);
    // 反射：镜像源法给 (d1+d2) = |源 - 镜像听者|（地板 y=0 ⇒ 镜像听者 y = -0.5）
    let refl: Vec<&SoundPath> = paths
        .iter()
        .filter(|p| p.kind == PathKind::Reflected)
        .collect();
    assert!(!refl.is_empty(), "地板必须给出一条一次反射");
    for p in refl {
        assert!(p.delay > d.delay, "反射时延必须大于直达（几何上必然更远）");
        assert!(p.gain > 0.0 && p.gain < d.gain, "反射更弱（距离 + 吸声）");
    }
    // 确定性：同输入重跑 ⇒ 逐字一致
    let mut again: Vec<SoundPath> = Vec::new();
    w.acoustic_cues(&g, &listener_at(2.0), &cfg, &mut again);
    assert_eq!(
        jsonl(&paths),
        jsonl(&again),
        "同输入必须逐字一致的 cue 序列"
    );
}

/// **遮挡**：墙立在声源与听者之间 ⇒ 直达被丢（`blocked > 0` 且没有 `Direct` 路径）。
#[test]
fn wall_blocks_the_direct_path() {
    let (verts, tris, absorb, trans) = geo(true);
    let g = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        portals: &[],
        medium: None,
    };
    let w = world_with_source(Vec3::new(0.0, 1.0, 0.0));
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(
        &g,
        &listener_at(2.0),
        &AcousticConfig::default(),
        &mut paths,
    );
    assert!(st.blocked > 0, "墙后必须记到被遮挡的直达路径");
    assert_eq!(st.direct, 0, "被挡的直达不得产出路径");
}

/// **预算**：`max_paths` 顶格时按序截断并**上报**（`dropped_budget > 0`）——非空洞。
#[test]
fn budget_truncates_and_reports() {
    let (verts, tris, absorb, trans) = geo(false);
    let g = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        portals: &[],
        medium: None,
    };
    let w = world_with_source(Vec3::new(0.0, 1.0, 0.0));
    let cfg = AcousticConfig {
        max_paths: 1,
        ..AcousticConfig::default()
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(&g, &listener_at(2.0), &cfg, &mut paths);
    assert!(
        paths.len() <= 1,
        "路径数必须 ≤ 预算（实际 {}）",
        paths.len()
    );
    assert!(st.dropped_budget > 0, "预算必须被顶到过（否则判据空洞）");
}

/// **零干扰**：开/关声学层，`state_hash` 逐位不变（只读消费者）。
#[test]
fn zero_interference_state_hash_is_bit_identical() {
    let (verts, tris, absorb, trans) = geo(false);
    let g = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        portals: &[],
        medium: None,
    };
    let mut w = world_with_source(Vec3::new(0.0, 1.0, 0.0));
    let before = w.state_hash();
    let mut paths: Vec<SoundPath> = Vec::new();
    w.acoustic_cues(
        &g,
        &listener_at(2.0),
        &AcousticConfig::default(),
        &mut paths,
    );
    assert!(
        paths.iter().any(|p| p.kind == PathKind::Reflected),
        "声学层得真的产出过路径（否则这条零干扰判据是空跑）"
    );
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（开/关 state_hash 逐位不变）"
    );
}
