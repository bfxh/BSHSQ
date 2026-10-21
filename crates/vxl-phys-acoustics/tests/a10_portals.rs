//! **体素空洞自动识别判据**（A10，设计文档 §8 待裁项①的兜底路线）。
//!
//! 一句话口径：**空 = `!占据`**；先按 **6 连通**把空域分成「房间」；分房前按切比雪夫半径
//! `neck_cells` **腐蚀**断颈；两房标签**相接**的那层空格就是**开口**。钉五件事：
//! ① 一格窄门把两块空域切成两房并在其间出一扇 portal（`regions` 填两侧 id）；
//! ② 墙封死 ⇒ 无 portal；③ 宽于 `2·neck` 的开口把两房并成一房 ⇒ 无 portal；
//! ④ 同输入**逐位可复现**；⑤ 认出来的 portal 能直接驱动 A2 的穿孔路径。
//!
//! 外加：`neck_cells = 0` 时永不分裂（整个连通空域并成一房）。

use vxl_phys::{BodyBuildExt, ImpactExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    AcousticConfig, AcousticGeometry, AcousticStats, AcousticsExt, Listener, Portal, PortalDetect,
    VoxelOccupancy,
};

/// 行主序格索引（与 `VoxelOccupancy::occupied` 同口径）。
fn occ_index(nx: u32, ny: u32, ix: u32, iy: u32, iz: u32) -> usize {
    (ix + nx * (iy + ny * iz)) as usize
}

/// 造一块体素占据：`ix == wall` 那层是实墙，两侧是**空**房间；`doors` 给出墙上挖开的格子 `(iy, iz)`。
fn rooms(nx: u32, ny: u32, nz: u32, wall: u32, doors: &[(u32, u32)]) -> Vec<bool> {
    let mut occ = vec![true; (nx * ny * nz) as usize];
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                if ix != wall {
                    occ[occ_index(nx, ny, ix, iy, iz)] = false;
                }
            }
        }
    }
    for &(dy, dz) in doors {
        occ[occ_index(nx, ny, wall, dy, dz)] = false;
    }
    occ
}

/// 在 `origin = 0`、`step = 1` 的网格上跑一次识别（这几条判据用的都是小整数格）。
fn detect(
    nx: u32,
    ny: u32,
    nz: u32,
    wall: u32,
    doors: &[(u32, u32)],
    cfg: &PortalDetect,
) -> Vec<Portal> {
    let occ = rooms(nx, ny, nz, wall, doors);
    let vo = VoxelOccupancy {
        origin: Vec3::new(0.0, 0.0, 0.0),
        step: 1.0,
        dims: [nx, ny, nz],
        occupied: &occ,
    };
    vo.detect_portals(cfg)
}

/// ① **一格窄门**：两块空域被切成两房，其间正好一扇 portal，法向沿 x、两侧 id = `[0, 1]`。
#[test]
fn doorway_between_two_rooms_yields_one_portal() {
    let got = detect(9, 5, 5, 4, &[(2, 2)], &PortalDetect::default());
    assert_eq!(got.len(), 1, "一格窄门应正好产出一扇 portal");
    let p = got[0];
    assert_eq!(p.regions, Some([0, 1]), "portal 必须填两侧房间 id（升序）");
    assert!((p.open - 1.0).abs() < 1e-6, "默认开启比例 = 全开");
    for c in p.quad.iter() {
        assert!((c.x - 5.0).abs() < 1e-4, "法向轴 x 居中（实际 {}）", c.x);
        assert!(c.y >= 2.0 - 1e-4 && c.y <= 3.0 + 1e-4, "开口覆盖 y ∈ [2,3]");
        assert!(c.z >= 2.0 - 1e-4 && c.z <= 3.0 + 1e-4, "开口覆盖 z ∈ [2,3]");
    }
}

/// ② **墙封死**：两侧空域不相接 ⇒ 无 portal。
#[test]
fn sealed_wall_yields_no_portal() {
    let got = detect(9, 5, 5, 4, &[], &PortalDetect::default());
    assert!(got.is_empty(), "墙封死 ⇒ 两房不相接 ⇒ 无 portal");
}

/// ③ **宽开口**：宽于 `2·neck_cells` 的通道在腐蚀后仍留着 ⇒ 两房并成一房 ⇒ 无 portal。
#[test]
fn wide_opening_merges_rooms_so_no_portal() {
    let mut doors = Vec::new();
    for dy in 1..=3 {
        for dz in 1..=3 {
            doors.push((dy, dz));
        }
    }
    let got = detect(9, 5, 5, 4, &doors, &PortalDetect::default());
    assert!(got.is_empty(), "3×3 宽口把两房并成一个 ⇒ 无 portal");
}

/// ⑥ **`neck_cells = 0` 永不分裂**：不腐蚀 ⇒ 整个连通空域并成一房 ⇒ 无 portal。
#[test]
fn zero_neck_never_splits() {
    let cfg = PortalDetect {
        neck_cells: 0,
        ..PortalDetect::default()
    };
    let got = detect(9, 5, 5, 4, &[(2, 2)], &cfg);
    assert!(got.is_empty(), "neck = 0 ⇒ 整片连通空域一房 ⇒ 无 portal");
}

/// ④ **逐位可复现**：同输入两次识别结果逐位相同。
#[test]
fn detection_is_bitwise_reproducible() {
    let cfg = PortalDetect::default();
    let a = detect(9, 5, 5, 4, &[(2, 2)], &cfg);
    let b = detect(9, 5, 5, 4, &[(2, 2)], &cfg);
    assert_eq!(a, b, "同输入必须逐位可复现");
}

/// 把四边形在**自身平面内**按中心放大 `f` 倍（用于从识别出的门造一面更大的同平面墙）。
fn expand_quad(q: &[Vec3; 4], f: f32) -> Vec<Vec3> {
    let c = (q[0] + q[1] + q[2] + q[3]) * 0.25;
    q.iter().map(|&p| c + (p - c) * f).collect()
}

/// ⑤ **识别出的 portal 能直接驱动穿孔路径**：立一面同平面墙挡住直线，门心落在墙上 ⇒ 出 `Portal` 路径；
/// 撤掉 portal ⇒ 无（否证）。
#[test]
fn detected_portal_drives_portal_path() {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let src = w.impacts()[0].point;
    let wall_x = src.x + 2.0;

    // 0.5 m 网格：把墙上那一格门对准 `(wall_x, src.y, src.z)`（门格 = 局部 (5,5,5)）
    let step = 0.5f32;
    let occ = rooms(11, 11, 11, 5, &[(5, 5)]);
    let vo = VoxelOccupancy {
        origin: Vec3::new(wall_x - 5.5 * step, src.y - 5.5 * step, src.z - 5.5 * step),
        step,
        dims: [11, 11, 11],
        occupied: &occ,
    };
    let detected = vo.detect_portals(&PortalDetect::default());
    assert_eq!(detected.len(), 1, "应认出恰好一扇门");
    let portal = detected[0];
    assert_eq!(portal.regions, Some([0, 1]), "门两侧应识别为两房");

    // 用识别出的门造一面**同平面**墙（平面内放大 3 倍 ⇒ 直线被挡、门心落在墙上）；双面绕序防背面剔除。
    let verts = expand_quad(&portal.quad, 3.0);
    let wall_tris = vec![[0u32, 1, 2], [0, 2, 3], [0, 2, 1], [0, 3, 2]];
    let absorb = vec![[0.2f32; 3]; 4];
    let trans = vec![[0.0f32; 3]; 4];
    let listener = Listener {
        pos: src + Vec3::new(4.0, 0.0, 0.0),
        radius: 0.5,
    };

    let mut paths = Vec::new();
    let geo = AcousticGeometry {
        verts: &verts,
        tris: &wall_tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &detected,
        edges: &[],
        medium: None,
    };
    let st: AcousticStats =
        w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
    assert!(st.portals >= 1, "识别出的门必须真的产出 portal 路径");
    assert_eq!(
        st.leaked, 0,
        "墙完全不透 ⇒ 不该有透射泄漏（佐证直线确实被挡）"
    );

    // 否证：撤掉 portal ⇒ 没有穿孔路径
    let mut paths2 = Vec::new();
    let geo2 = AcousticGeometry {
        portals: &[],
        ..geo
    };
    let st2 = w.acoustic_cues(&geo2, &listener, &AcousticConfig::default(), &mut paths2);
    assert_eq!(st2.portals, 0, "没登记 portal ⇒ 不该有穿孔路径");
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
