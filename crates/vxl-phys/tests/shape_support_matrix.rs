//! **形状支持矩阵判据**（`docs/SURVEY-SHAPE-SUPPORT-MATRIX.md` §1/§2）：`Shape` **每一臂**在各域的
//! 姿态逐个登记与断言，让"新增一类形状"**不可能静默通过**。
//!
//! 核心机制：`posture()` 的 **穷尽 `match`**（无 `_` 兜底）——给 `Shape` 加第 11 臂却不到这里
//! 登记 ⇒ **编译报错**。"不支持"必须是**有人做过的决定**，不是漏写（本仓的"有形状、无接触"缺口
//! 已栽过：`provider_shape_coverage.rs` 记的胶囊/圆柱/圆锥穿过提供者那三条）。
//!
//! 三类断言：① 穷尽登记（每臂的域姿态）；② 可用性（**行为**：动态体质量、宽相 AABB 逐顶点包含、
//! 薄壳解析对拍）；③ **已知缺口钉住**（三角网 × {盒, 提供者} 暂不产接触 ⇒ **T1b 落地后翻面**）。

use vxl_phys::*;
use vxl_phys_broad::shape_aabb;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::mesh::TriMesh;

const TICKS: usize = 240;

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 **y = 0 平面**）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// 平地板（y = 0，8×8 格、±4 m；与 `provider_shape_coverage.rs` 同一套三角化顺序）。
fn flat_mesh() -> TriMesh {
    const N: usize = 8;
    const S: f32 = 4.0;
    let (verts, tris) = plate(N, S);
    TriMesh::new(verts, tris)
}

/// 形状在某域的**姿态**（矩阵表的一行）。字段刻意只有两个：本文件判的是"能不能"这类
/// **二值**事实，不重述源码里的公式（公式由各自的解析判据守）。
#[derive(Debug, Clone, Copy, PartialEq)]
struct Posture {
    /// 有**局部 AABB** 语义（宽相用局部半长；地形类由 provider 侧给 bounds）。
    local_aabb: bool,
    /// 可作**动态体**（质量 > 0）。
    dynamic_ok: bool,
}

/// **登记函数**：每一臂的域姿态。这是矩阵的唯一真值表——加臂必须来这里做决定。
fn posture(shape: &Shape) -> Posture {
    match *shape {
        // 地形 / 提供者：宽相 AABB 由 provider 侧给（`hf_bounds`/`provider_bounds`），且
        // **无质量属性**（只作静态地形，见 `core::mass::mass_props` 尾臂）。
        Shape::HeightField(_) | Shape::Provider(_) => Posture {
            local_aabb: false,
            dynamic_ok: false,
        },
        // 有界几何：局部半长 + 可动态。三角网与外壳/复合体同款（core 给 AABB 盒兜底，
        // 三角网的真值由门面按**薄壳**覆写，见 `spawn_trimesh_body`）。
        Shape::Box { .. }
        | Shape::Sphere { .. }
        | Shape::Cylinder { .. }
        | Shape::Capsule { .. }
        | Shape::Cone { .. }
        | Shape::ConvexHull { .. }
        | Shape::Compound { .. }
        | Shape::TriMesh { .. } => Posture {
            local_aabb: true,
            dynamic_ok: true,
        },
    }
}

/// 每一臂一个代表形状（**全部 10 臂**）。
fn representatives() -> Vec<(&'static str, Shape)> {
    let h = Vec3::splat(0.3);
    vec![
        ("box", Shape::Box { half: h }),
        ("sphere", Shape::Sphere { radius: 0.3 }),
        (
            "cylinder",
            Shape::Cylinder {
                half_height: 0.3,
                radius: 0.3,
            },
        ),
        (
            "capsule",
            Shape::Capsule {
                half_height: 0.3,
                radius: 0.3,
            },
        ),
        (
            "cone",
            Shape::Cone {
                half_height: 0.3,
                radius: 0.3,
            },
        ),
        ("heightfield", Shape::HeightField(0)),
        ("provider", Shape::Provider(0)),
        ("hull", Shape::ConvexHull { hull: 0, half: h }),
        (
            "compound",
            Shape::Compound {
                compound: 0,
                half: h,
            },
        ),
        ("trimesh", Shape::TriMesh { mesh: 0, half: h }),
    ]
}

/// **① 穷尽登记 + ② 动态性（行为）**：登记表与实测一致。
#[test]
fn every_arm_is_registered_and_dynamic_posture_holds() {
    for (name, shape) in representatives() {
        let want = posture(&shape);
        assert_eq!(
            shape.bounding_sphere_radius().is_finite(),
            want.local_aabb,
            "{name}: 局部 AABB 姿态与登记表不符"
        );
        let mut w = World::new(PhysConfig::default());
        let b = w.add_dynamic(shape, Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
        assert_eq!(
            w.bodies.inv_mass[b] > 0.0,
            want.dynamic_ok,
            "{name}: 动态体姿态与登记表不符"
        );
    }
}

/// **T1a-①**：三角网体的宽相 AABB（**任意姿态**）**逐顶点包含**整张网。
///
/// 走的是宽相唯一入口 `shape_aabb`（不是"照抄一遍同样的公式"）：判据必须压在被测代码
/// 本来走的那条路上；姿态取**斜置**（绕 (1,1,0.5) 转 40°）以免对称掩盖符号错误。
#[test]
fn trimesh_aabb_contains_every_vertex_under_rotation() {
    let (pts, tris) = plate(2, 1.0);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts.clone(), tris);
    let rot = Quat::from_axis_angle(Vec3::new(1.0, 1.0, 0.5).normalize(), 0.7);
    let body = w.spawn_trimesh_body(mesh, Vec3::new(0.3, 0.7, -0.2), rot, 1000.0, 0.01) as usize;
    let (pos, rot) = (w.bodies.position[body], w.bodies.rot(body));
    let bb = shape_aabb(&w.bodies.shape[body], pos, rot, 0.0, &[], &[]);
    for p in &pts {
        let q = pos + rot.rotate_vec3(*p);
        assert!(
            q.x >= bb.min.x - 1e-5
                && q.y >= bb.min.y - 1e-5
                && q.z >= bb.min.z - 1e-5
                && q.x <= bb.max.x + 1e-5
                && q.y <= bb.max.y + 1e-5
                && q.z <= bb.max.z + 1e-5,
            "顶点 {q:?} 落在宽相 AABB {bb:?} 外 ⇒ 宽相会漏对"
        );
    }
}

/// **T1a-②**：门面把**薄壳**质量属性真的写进了体（不是只算不用）。
/// 解析式：`±1` 方板 `m = ρ·t·A = 7·0.02·4`、`I_y = 2m/3`（绕法向）。
#[test]
fn trimesh_body_gets_thin_shell_props() {
    let (pts, tris) = plate(2, 1.0);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts, tris);
    let body = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 7.0, 0.02) as usize;
    let m = 7.0 * 0.02 * 4.0;
    let got = 1.0 / w.bodies.inv_mass[body];
    assert!((got - m).abs() / m < 1e-4, "薄壳质量 {got} ≠ ρtA = {m}");
    let iy = 1.0 / w.bodies.local_inv_inertia[body].y;
    assert!(
        (iy - 2.0 * m / 3.0).abs() / iy < 1e-4,
        "绕法向惯量 {iy} ≠ 2m/3"
    );
}

/// **③ 已知缺口（T1b 关闭）· 三角网 × 静态盒**：逐顶点采样未落地 ⇒ 薄壳**穿过**盒子。
///
/// 本判据**钉住现状**（不是"期望它坏"）：T1b 落地后它会红 ⇒ 那时把断言翻面成"被托住"。
/// 缺口因此有名字、有断言、有翻面条件——不静默。
#[test]
fn gap_trimesh_passes_through_box_until_t1b() {
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts, tris);
    w.add_static(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b =
        w.spawn_trimesh_body(mesh, Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY, 1000.0, 0.01) as usize;
    for _ in 0..TICKS {
        w.step();
    }
    assert!(
        w.bodies.position[b].y < -1.0,
        "三角网被盒托住了 —— T1b 似乎已落地：请把本判据翻面成『停住』并更新支持矩阵"
    );
}

/// **③ 已知缺口（T1b 关闭）· 三角网 × 提供者（三角网地板）**：同一缺口的第二条腿
/// （判据 ① 的原场景）。翻面条件同上。
#[test]
fn gap_trimesh_passes_through_provider_floor_until_t1b() {
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let _floor = w.add_mesh(flat_mesh());
    let mesh = w.add_trimesh(pts, tris);
    let b =
        w.spawn_trimesh_body(mesh, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0, 0.01) as usize;
    for _ in 0..TICKS {
        w.step();
    }
    assert!(
        w.bodies.position[b].y < -1.0,
        "三角网停在地板上了 —— T1b 似乎已落地：请把本判据翻面成『逐顶点间隙 ≈ 0』"
    );
}
