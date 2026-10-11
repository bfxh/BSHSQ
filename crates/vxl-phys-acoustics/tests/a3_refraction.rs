//! **A3 第二半判据：折射（Snell 折转）**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §3 原则 ③ + §5 的 A3 行）。
//!
//! 三条硬指标：① 折点上的**角度满足 Snell 定律**（`sinθ/c` 沿声线守恒）；② 折线是**旅行时最小**
//! 的那条（Fermat）—— 与独立参照的"暴力扫界面点"对拍，且比直线积分更快；③ 均匀介质**不折**
//! （避免与直达重复计能）。外加逐段遮挡与确定性。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticStats, AcousticsExt,
    LayeredMedium, Listener, Medium, PathKind, SoundPath, BANDS,
};

/// 水（快、吸收明显）——与空气一起构成一个层界。
const WATER: Medium = Medium {
    speed_of_sound: 1500.0,
    attenuation: [0.2; BANDS],
};

/// 层界高度（声源在它下面 ⇒ 水里；听者在它上面 ⇒ 空气里）。
const SPLIT: f32 = 0.3;

fn field() -> LayeredMedium {
    LayeredMedium::split(SPLIT, WATER, Medium::AIR)
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

/// ① **Snell 定律**：折点落在界面上、两侧 `sinθ/c` 相等；折线是旅行时最小（暴力扫界面点对拍）。
#[test]
fn snell_holds_and_the_kink_minimizes_travel_time() {
    let m = field();
    // 水里 → 空气里（z 都取 0 ⇒ 折点就在 x-y 平面里，方便暴力扫）
    let src = Vec3::new(0.0, -2.0, 0.0);
    let dst = Vec3::new(3.0, 2.0, 0.0);
    let (kinks, delay, _) = m
        .refracted(src, dst, 64)
        .unwrap_or((Vec::new(), 0.0, [0.0; BANDS]));
    assert_eq!(kinks.len(), 3, "两段折线必须有 3 个折点（含首尾）");
    assert!(
        (kinks[0] - src).length() < 1e-6 && (kinks[2] - dst).length() < 1e-6,
        "折点表必须含首尾"
    );
    let p1 = kinks[1];
    assert!(
        (p1.y - SPLIT).abs() < 1e-5,
        "折点必须落在界面上（实际 y={}）",
        p1.y
    );
    // Snell：sinθ/c 沿声线守恒
    let a1 = p1 - src;
    let a2 = dst - p1;
    let sin1 = a1.x.hypot(a1.z) / a1.length();
    let sin2 = a2.x.hypot(a2.z) / a2.length();
    let (s1, s2) = (
        sin1 / WATER.speed_of_sound,
        sin2 / Medium::AIR.speed_of_sound,
    );
    assert!(
        (s1 - s2).abs() <= s1.abs().max(s2.abs()) * 1e-3,
        "Snell 必须成立（sinθ₁/c₁={s1} vs sinθ₂/c₂={s2}）"
    );
    // 独立参照：沿界面暴力扫折点，取旅行时最小
    let mut best = f32::INFINITY;
    for i in 0..=20000 {
        let x = -6.0 + 14.0 * (i as f32 / 20000.0);
        let p = Vec3::new(x, SPLIT, 0.0);
        let t = (p - src).length() / WATER.speed_of_sound
            + (dst - p).length() / Medium::AIR.speed_of_sound;
        if t < best {
            best = t;
        }
    }
    assert!(
        delay <= best * (1.0 + 1e-4) && delay >= best * (1.0 - 1e-4),
        "折线的旅行时应等于界面上的最小值（实现 {delay}，暴力扫 {best}）"
    );
    // Fermat：折射比"直线积分"更快（后者不是极值路径）
    let straight = m
        .segment_exact(src, dst)
        .unwrap_or((f32::INFINITY, [0.0; BANDS]))
        .0;
    assert!(
        delay < straight,
        "折射必须比直线更快（折射 {delay}，直线 {straight}）"
    );
}

/// ② **路径级**：`Refracted` 路径存在，时延/衰减/方向与介质给的一致。
#[test]
fn refracted_path_matches_the_medium() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let f = field();
    let geo = AcousticGeometry {
        verts: &[],
        tris: &[],
        absorption: &[],
        transmission: &[],
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: Some(&f),
    };
    let lst = Vec3::new(0.0, 1.0, 2.0);
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: lst,
        radius: 0.5,
    };
    let st: AcousticStats =
        w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
    assert!(st.refracted >= 1, "层界两侧的声源/听者必须产出折射路径");
    let p = paths
        .iter()
        .find(|p| p.kind == PathKind::Refracted && p.source == 0)
        .copied()
        .unwrap_or(SoundPath {
            source: 0,
            delay: 0.0,
            bands: [0.0; BANDS],
            gain: 0.0,
            dir: [0.0, 0.0, 0.0],
            kind: PathKind::Refracted,
        });
    let src = w.impacts()[0].point;
    let (kinks, delay, atten) = f
        .refracted(src, lst, AcousticConfig::default().refraction_iters)
        .unwrap_or((Vec::new(), 0.0, [0.0; BANDS]));
    assert!(
        (p.delay - delay).abs() <= delay * 1e-5,
        "路径时延必须等于介质给的（路径 {} vs 介质 {delay}）",
        p.delay
    );
    let dist = (lst - src).length();
    for (b, a) in atten.iter().enumerate() {
        let expect = a / (dist * dist);
        assert!(
            (p.bands[b] - expect).abs() <= expect * 1e-5,
            "第 {b} 带应为 介质衰减/d²（实际 {}，期望 {expect}）",
            p.bands[b]
        );
    }
    // 到达方向 = 从听者指向最后一个折点
    let prev = kinks[kinks.len() - 2];
    let want = (lst - prev).normalize();
    assert!(
        (p.dir[0] - want.x).abs() < 1e-5
            && (p.dir[1] - want.y).abs() < 1e-5
            && (p.dir[2] - want.z).abs() < 1e-5,
        "到达方向必须指向最后一个折点（实际 {:?}，期望 {want:?}）",
        p.dir
    );
}

/// ③ **均匀介质不折射**（直线就是解 ⇒ 交给直达那条，避免重复计能）。
#[test]
fn uniform_medium_does_not_refract() {
    let uniform = LayeredMedium::split(0.0, WATER, WATER);
    assert!(
        uniform
            .refracted(Vec3::new(0.0, -1.0, 0.0), Vec3::new(3.0, 1.0, 0.0), 64)
            .is_none(),
        "两侧同速 ⇒ 不该产出折射折线"
    );
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let geo = AcousticGeometry {
        verts: &[],
        tris: &[],
        absorption: &[],
        transmission: &[],
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: Some(&uniform),
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: Vec3::new(0.0, 1.0, 2.0),
        radius: 0.5,
    };
    let st = w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
    assert_eq!(
        st.refracted, 0,
        "均匀介质下不得出现 Refracted（否则与直达重复计能）"
    );
}

/// ④ **逐段遮挡**：折线里有一段被挡 ⇒ 不产出；去掉挡板就又有。
#[test]
fn blocked_refraction_leg_produces_nothing() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let f = field();
    let lst = Vec3::new(0.0, 1.0, 2.0);
    let quiet = AcousticGeometry {
        verts: &[],
        tris: &[],
        absorption: &[],
        transmission: &[],
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: Some(&f),
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: lst,
        radius: 0.5,
    };
    let clean = w.acoustic_cues(&quiet, &listener, &AcousticConfig::default(), &mut paths);
    assert!(
        clean.refracted >= 1,
        "干净场景必须有折射（否则这条判据空跑）"
    );
    // 在空气段上插一块不透明板（y = 0.6、x,z ∈ [-2,2]）
    let verts = vec![
        Vec3::new(-2.0, 0.6, -2.0),
        Vec3::new(2.0, 0.6, -2.0),
        Vec3::new(2.0, 0.6, 2.0),
        Vec3::new(-2.0, 0.6, 2.0),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let absorb = vec![[0.1f32; BANDS]; 2];
    let trans = vec![[0.0f32; BANDS]; 2];
    let blocked = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: Some(&f),
    };
    let mut paths2: Vec<SoundPath> = Vec::new();
    let st = w.acoustic_cues(&blocked, &listener, &AcousticConfig::default(), &mut paths2);
    assert_eq!(
        st.refracted,
        0,
        "折线的一段被挡死 ⇒ 不得产出折射路径（实际 {:?}）",
        jsonl(&paths2)
    );
}

/// ⑤ **确定性 + 零干扰**。
#[test]
fn refraction_is_deterministic_and_zero_interference() {
    let mut w = world_with_source();
    let before = w.state_hash();
    let f = field();
    let geo = AcousticGeometry {
        verts: &[],
        tris: &[],
        absorption: &[],
        transmission: &[],
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: Some(&f),
    };
    let lst = Vec3::new(0.0, 1.0, 2.0);
    let run = |w: &World| -> String {
        let mut paths: Vec<SoundPath> = Vec::new();
        let listener = Listener {
            pos: lst,
            radius: 0.5,
        };
        w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
        jsonl(&paths)
    };
    let (a, b) = (run(&w), run(&w));
    assert!(
        a.contains("\"kind\":\"refracted\""),
        "流里必须有折射路径：{a}"
    );
    assert_eq!(a, b, "同输入两跑必须逐字一致");
    assert_eq!(
        w.state_hash(),
        before,
        "声学层必须零干扰（折射也一样，只读）"
    );
}
