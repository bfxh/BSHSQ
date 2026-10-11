//! **自动认边判据**（A4 的兜底路线，设计文档 §8）：从三角网格里认出**边界棱**与**凸棱**。
//!
//! 钉四件事：① 一个盒子恰好 12 条凸棱（6 条**面对角线**是共面的，不算）；② 平板的 4 条边框在、
//! 中间对角线不在；③ **凹**棱不算（墙-地那种接缝）；④ 浅折可由 `min_wedge_deg` 卡；外加
//! ⑤ 认出来的棱直接能驱动 A4 绕射（与独立参照的**暴力扫棱**一致）。

use std::collections::BTreeSet;
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Edge, EdgeDetect, Listener,
    PathKind, SoundPath,
};

/// 一条棱的**无向键**（两端排序 ⇒ 与方向无关）；坐标取整后比较是精确的。
type EdgeKey = ((i32, i32, i32), (i32, i32, i32));

/// 顶点坐标取整（这些判据用的都是小整数坐标，取整后比较是精确的）。
fn key(p: Vec3) -> (i32, i32, i32) {
    (p.x as i32, p.y as i32, p.z as i32)
}

/// 一条棱的无向键（两端排序 ⇒ 与方向无关）。
fn ekey(e: &Edge) -> EdgeKey {
    let (a, b) = (key(e.a), key(e.b));
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn keys(edges: &[Edge]) -> BTreeSet<EdgeKey> {
    edges.iter().map(ekey).collect()
}

/// 在"只有顶点/三角"的几何上跑认边（其余表留空 —— 认边不碰它们）。
fn detect(verts: &[Vec3], tris: &[[u32; 3]], d: &EdgeDetect) -> Vec<Edge> {
    let geo = AcousticGeometry {
        verts,
        tris,
        absorption: &[],
        transmission: &[],
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: None,
    };
    geo.detect_edges(d)
}

/// 单位盒子（8 顶点 12 三角，**绕序朝外**）—— 顺带在判据里自查这条前提。
fn box_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let verts = vec![
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(1.0, -1.0, -1.0),
        Vec3::new(1.0, 1.0, -1.0),
        Vec3::new(-1.0, 1.0, -1.0),
        Vec3::new(-1.0, -1.0, 1.0),
        Vec3::new(1.0, -1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, 1.0, 1.0),
    ];
    let tris = vec![
        [0u32, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    (verts, tris)
}

/// ① **盒子：恰好 12 条凸棱**（6 条面对角线共面 ⇒ 不算），且两跑逐位一致。
#[test]
fn box_has_exactly_twelve_convex_edges() {
    let (verts, tris) = box_mesh();
    // 自查测试数据：绕序朝外（外法向与"面心方向"同向）—— 认边的前提
    for tri in tris.iter() {
        let (a, b, c) = (
            verts[tri[0] as usize],
            verts[tri[1] as usize],
            verts[tri[2] as usize],
        );
        let n = (b - a).cross(c - a);
        assert!(
            n.dot((a + b + c) * (1.0 / 3.0)) > 0.0,
            "测试自己的盒子绕序必须朝外"
        );
    }
    let d = EdgeDetect::default();
    let got = detect(&verts, &tris, &d);
    let again = detect(&verts, &tris, &d);
    assert_eq!(got.len(), 12, "盒子必须恰好 12 条凸棱（对角线不算）");
    assert_eq!(keys(&got), keys(&again), "认边必须逐位可复现");
    let want: BTreeSet<_> = [
        (0u32, 1u32),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    .iter()
    .map(|&(i, j)| {
        let (a, b) = (key(verts[i as usize]), key(verts[j as usize]));
        if a <= b {
            (a, b)
        } else {
            (b, a)
        }
    })
    .collect();
    assert_eq!(keys(&got), want, "认出来的必须正好是盒子的 12 条几何棱");
}

/// ② **平板：4 条边框在、中间对角线不在**（对角线是共面的共享棱）。
#[test]
fn flat_plate_keeps_rim_but_not_the_diagonal() {
    let verts = vec![
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, 1.0),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3]]; // 共享对角线 (0,2)
    let got = detect(&verts, &tris, &EdgeDetect::default());
    assert_eq!(got.len(), 4, "平板只有 4 条边框（对角线是共面棱）");
    let ks = keys(&got);
    let (a, b) = (key(verts[0]), key(verts[2]));
    let diag = if a <= b { (a, b) } else { (b, a) };
    assert!(!ks.contains(&diag), "中间对角线不该被认成衍射棱");
    for (i, j) in [(0usize, 1usize), (1, 2), (2, 3), (3, 0)] {
        let (p, q) = (key(verts[i]), key(verts[j]));
        let k = if p <= q { (p, q) } else { (q, p) };
        assert!(ks.contains(&k), "边框 ({i},{j}) 必须在");
    }
}

/// ③ **凹棱不算**：墙-地那种接缝（90° 是够"折"的，但方向是**凹**的）。
#[test]
fn concave_joint_is_not_an_edge() {
    // 共享棱 a→b；地板法向 +y、墙法向 +z ⇒ 从外侧看是凹角
    let a = Vec3::new(0.0, 0.0, 0.0);
    let b = Vec3::new(1.0, 0.0, 0.0);
    let verts = vec![a, b, Vec3::new(1.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 0.0)];
    let tris = vec![[0u32, 2, 1], [0, 1, 3]];
    let got = detect(&verts, &tris, &EdgeDetect::default());
    assert_eq!(got.len(), 4, "只该认出 4 条方位边界棱（共享的凹棱不算）");
    let (p, q) = (key(a), key(b));
    let shared = if p <= q { (p, q) } else { (q, p) };
    assert!(
        !keys(&got).contains(&shared),
        "凹接缝不该被认成衍射棱（凸凹判据要起作用）"
    );
}

/// ④ **浅折由 `min_wedge_deg` 卡**：先量出实际二面角，再在它两侧各取一个阈值。
#[test]
fn shallow_fold_is_gated_by_min_wedge_deg() {
    let a = Vec3::new(0.0, 0.0, 0.0);
    let b = Vec3::new(1.0, 0.0, 0.0);
    let c1 = Vec3::new(0.5, -0.1, 1.0);
    let c2 = Vec3::new(0.5, 0.6, -1.0);
    let verts = vec![a, b, c1, c2];
    // 与实现**同源**的绕序公式：(v₁−v₀)×(v₂−v₀)
    let cross_of = |t: [u32; 3], vs: &[Vec3]| {
        let (v0, v1, v2) = (vs[t[0] as usize], vs[t[1] as usize], vs[t[2] as usize]);
        (v1 - v0).cross(v2 - v0)
    };
    let t1 = [0u32, 1, 2];
    let t2 = [0u32, 3, 1];
    // 按实现的口径把第二个面**调成凸**（凸凹判据：第三点要在第一个面平面的外侧）
    let n1 = cross_of(t1, &verts);
    let t2 = if n1.dot(c2 - a) < 0.0 {
        t2
    } else {
        [t2[0], t2[2], t2[1]]
    };
    let tris = vec![t1, t2];
    // 量二面角（同一口径，测试独立算一遍）
    let (n1, n2) = (cross_of(t1, &verts), cross_of(t2, &verts));
    let ang = (n1.dot(n2) / (n1.length() * n2.length()))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();
    assert!(
        ang > 1.0 && ang < 179.0,
        "阈值两侧要留得下 ±5°（实际折角 {ang}）"
    );
    let loose = EdgeDetect {
        min_wedge_deg: ang - 5.0,
        ..EdgeDetect::default()
    };
    let tight = EdgeDetect {
        min_wedge_deg: ang + 5.0,
        ..EdgeDetect::default()
    };
    let (p, q) = (key(a), key(b));
    let shared = if p <= q { (p, q) } else { (q, p) };
    let loose_ks = keys(&detect(&verts, &tris, &loose));
    let tight_ks = keys(&detect(&verts, &tris, &tight));
    assert!(
        loose_ks.contains(&shared),
        "阈值放宽到折角以下时必须认出这条凸棱"
    );
    assert!(!tight_ks.contains(&shared), "阈值收到折角以上时必须滤掉它");
}

/// ⑤ **认出来的棱能直接驱动绕射**：屏幕上自动认出的棱拿去跑 A4，与独立参照（暴力扫棱）一致。
#[test]
fn detected_edges_drive_diffraction() {
    // 地板（y = 0）+ 立在它上面的薄屏（z = 1.0、x∈[-0.5,0.5]、y∈[0,1]）
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
        Vec3::new(-0.5, 0.0, 1.0),
        Vec3::new(0.5, 0.0, 1.0),
        Vec3::new(0.5, 1.0, 1.0),
        Vec3::new(-0.5, 1.0, 1.0),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]];
    let detected = detect(&verts, &tris, &EdgeDetect::default());
    assert!(
        detected.len() >= 7,
        "地板 4 条方位棱 + 屏的顶边与两条竖边（底边也在地板上，共 8 条起）—— 实际 {}",
        detected.len()
    );
    // 独立参照：在**这些棱**上暴力扫，取全局最短折线
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let src = w.impacts()[0].point;
    let lst = Vec3::new(0.0, 0.5, 2.0);
    let mut best = f32::INFINITY;
    for e in detected.iter() {
        for i in 0..=2000 {
            let u = i as f32 / 2000.0;
            let p = e.a + (e.b - e.a) * u;
            let len = (p - src).length() + (lst - p).length();
            if len < best {
                best = len;
            }
        }
    }
    // 用自动认出来的棱当衍射棱，跑一次归约
    let absorb = vec![[0.25f32; 3], [0.25; 3], [0.1; 3], [0.1; 3]];
    let trans = vec![[0.0f32; 3]; 4];
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &[],
        edges: &detected,
        medium: None,
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let listener = Listener {
        pos: lst,
        radius: 0.5,
    };
    let st: AcousticStats =
        w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
    assert!(st.diffracted >= 1, "认出来的棱必须真的产出绕射路径");
    let got = paths
        .iter()
        .filter(|p| p.kind == PathKind::Diffracted)
        .map(|p| p.delay * AcousticConfig::default().speed_of_sound)
        .fold(f32::INFINITY, f32::min);
    assert!(
        (got - best).abs() <= best * 1e-4,
        "最短的绕射折线应与暴力扫一致（实现 {got}，暴力扫 {best}）"
    );
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
