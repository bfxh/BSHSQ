//! **A5「GPU 档」基础件判据**：**批处理入口**与它必须满足的**等价契约**。
//!
//! 设备内核（跑在 wgpu 上的计算着色器）不在本 crate —— 它属于 `vxl-phys-gpu`，而且 CI 的运行器
//! 没有 GPU（既有口径：`cargo test --workspace --exclude vxl-phys-gpu`）。本层能钉住的是
//! **接进来就不改数值**的那条契约：批处理的结果必须与逐个标量调用**逐位相同**。

use vxl_phys::{BodyBuildExt, DiagExt, ImpactExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Listener, Portal,
    SoundPath, BANDS,
};

/// 房间的几何表：顶点 / 三角 / 逐带吸声 / 逐带透射 / 逐带散射 / 开口。
type RoomTables = (
    Vec<Vec3>,
    Vec<[u32; 3]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
    Vec<Portal>,
);

/// 一间有内容的房间：地板 + 带门洞的墙（透射/散射都有）—— 每个听者都能产出多条路径。
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
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]];
    let absorb = vec![
        [0.25f32; BANDS],
        [0.25; BANDS],
        [0.6, 0.9, 0.95],
        [0.6, 0.9, 0.95],
    ];
    let trans = vec![
        [0.0f32; BANDS],
        [0.0; BANDS],
        [0.03, 0.02, 0.008],
        [0.03, 0.02, 0.008],
    ];
    let scatter = vec![
        [0.2f32; BANDS],
        [0.2; BANDS],
        [0.35, 0.5, 0.6],
        [0.35, 0.5, 0.6],
    ];
    let portals = vec![Portal {
        quad: [
            Vec3::new(-1.0, 0.0, 1.5),
            Vec3::new(1.0, 0.0, 1.5),
            Vec3::new(1.0, 2.5, 1.5),
            Vec3::new(-1.0, 2.5, 1.5),
        ],
        open: 1.0,
        regions: None,
    }];
    (verts, tris, absorb, trans, scatter, portals)
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

/// 三个听者（都在墙外侧）。
fn listeners() -> Vec<Listener> {
    vec![
        Listener {
            pos: Vec3::new(0.0, 0.5, 2.0),
            radius: 0.5,
        },
        Listener {
            pos: Vec3::new(0.5, 0.8, 2.5),
            radius: 0.5,
        },
        Listener {
            pos: Vec3::new(-0.5, 0.4, 3.0),
            radius: 0.5,
        },
    ]
}

/// ① **逐位等价**：批处理 == 把每个听者单独跑一遍再拼起来（含分段边界 = `paths()`）。
#[test]
fn batch_matches_scalar_calls_bitwise() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let (verts, tris, absorb, trans, scatter, portals) = room();
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &[],
        medium: None,
    };
    let cfg = AcousticConfig::default();
    let ls = listeners();
    let mut out: Vec<SoundPath> = Vec::new();
    let mut stats: Vec<AcousticStats> = Vec::new();
    w.acoustic_cues_batch(&geo, &ls, &cfg, &mut out, &mut stats);
    assert_eq!(stats.len(), ls.len(), "stats 必须与 listeners 一一对应");
    assert!(
        out.len() >= 3,
        "这个房间必须产出多条路径（实际 {}）",
        out.len()
    );
    // 逐个跑一遍再拼
    let mut want_out: Vec<SoundPath> = Vec::new();
    let mut want_stats: Vec<AcousticStats> = Vec::new();
    for l in ls.iter() {
        let st = w.acoustic_cues(&geo, l, &cfg, &mut want_out);
        want_stats.push(st);
    }
    assert_eq!(
        jsonl(&out),
        jsonl(&want_out),
        "批处理必须与逐个标量调用逐位相同"
    );
    assert_eq!(stats, want_stats, "统计也必须一致");
    // 分段边界 = 前缀和
    let mut acc = 0usize;
    for st in stats.iter() {
        acc += st.paths();
    }
    assert_eq!(acc, out.len(), "各段之和必须等于产出总数");
}

/// ② **逐听者预算**：`max_paths` 不跨听者共享（否则后面的听者会被前面的饿死）。
#[test]
fn budget_is_per_listener() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let (verts, tris, absorb, trans, scatter, portals) = room();
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &[],
        medium: None,
    };
    let ls = listeners();
    // 先拿"宽预算"的候选数，确认每个听者都 ≥ 2 条（否则这条判据空跑）
    let wide = AcousticConfig {
        max_paths: 64,
        ..AcousticConfig::default()
    };
    let mut wide_out: Vec<SoundPath> = Vec::new();
    let mut wide_stats: Vec<AcousticStats> = Vec::new();
    w.acoustic_cues_batch(&geo, &ls, &wide, &mut wide_out, &mut wide_stats);
    assert!(
        wide_stats.iter().all(|s| s.paths() >= 2),
        "每个听者都要有 ≥ 2 条候选（实际 {:?}）",
        wide_stats.iter().map(|s| s.paths()).collect::<Vec<_>>()
    );
    let tight = AcousticConfig {
        max_paths: 1,
        ..AcousticConfig::default()
    };
    let mut out: Vec<SoundPath> = Vec::new();
    let mut stats: Vec<AcousticStats> = Vec::new();
    w.acoustic_cues_batch(&geo, &ls, &tight, &mut out, &mut stats);
    for (i, st) in stats.iter().enumerate() {
        assert_eq!(st.paths(), 1, "第 {i} 个听者必须各自拿到自己的 1 条额度");
        assert!(st.dropped_budget >= 1, "第 {i} 个听者必须上报被丢的候选");
    }
    assert_eq!(
        out.len(),
        ls.len(),
        "总产出 = 每听者上限之和（共享预算的话后面几个听者会被饿死）"
    );
}

/// ③ **空批与单元素批**：空批不动输出；单元素批与标量调用逐位相同。
#[test]
fn empty_and_single_element_batches() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let (verts, tris, absorb, trans, scatter, portals) = room();
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &[],
        medium: None,
    };
    let cfg = AcousticConfig::default();
    let mut out: Vec<SoundPath> = Vec::new();
    let mut stats: Vec<AcousticStats> = Vec::new();
    w.acoustic_cues_batch(&geo, &[], &cfg, &mut out, &mut stats);
    assert!(out.is_empty() && stats.is_empty(), "空批不该动输出");
    let one = [listeners()[0]];
    w.acoustic_cues_batch(&geo, &one, &cfg, &mut out, &mut stats);
    let mut scalar: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(&geo, &one[0], &cfg, &mut scalar);
    assert_eq!(jsonl(&out), jsonl(&scalar), "单元素批必须与标量调用一致");
    assert_eq!(stats[0], st);
}

/// ④ **确定性 + 零干扰**：批处理两跑逐字一致；调用前后 `state_hash` 逐位不变。
#[test]
fn batch_is_deterministic_and_zero_interference() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let (verts, tris, absorb, trans, scatter, portals) = room();
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &scatter,
        portals: &portals,
        edges: &[],
        medium: None,
    };
    let cfg = AcousticConfig::default();
    let ls = listeners();
    let run = |w: &World| -> String {
        let mut out: Vec<SoundPath> = Vec::new();
        let mut stats: Vec<AcousticStats> = Vec::new();
        w.acoustic_cues_batch(&geo, &ls, &cfg, &mut out, &mut stats);
        jsonl(&out)
    };
    let (a, b) = (run(&w), run(&w));
    assert!(!a.is_empty(), "批处理必须真的产出路径（否则判据空跑）");
    assert_eq!(a, b, "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（批处理也一样，只读）"
    );
}
