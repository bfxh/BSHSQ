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

/// **判据 ①（凸体腿，T1b-2 已落地）· 三角网 × 静态盒**：布片落到盒顶 ⇒ **被托住**。
///
/// 口径同 provider 腿（取窗口均值）：高度贴盒顶 + 末窗 `|v|` 均值收敛。
/// 盒顶 2×2 大于布片 1×1 ⇒ 落点完全在支撑面内（避开"滑出边缘"这类与接触无关的失败）。
#[test]
fn trimesh_is_held_by_box() {
    const TOP: f32 = 1.0;
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts, tris);
    w.add_static(
        Shape::Box {
            half: Vec3::splat(TOP),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b =
        w.spawn_trimesh_body(mesh, Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY, 1000.0, 0.01) as usize;
    let mut vsum = 0.0f32;
    let mut n = 0.0f32;
    for tick in 0..600 {
        w.step();
        if tick + 60 >= 600 {
            vsum += w.bodies.linvel[b].length();
            n += 1.0;
        }
    }
    let y = w.bodies.position[b].y;
    let v = vsum / n;
    println!("[判据①-box] y={y:+.5}（盒顶 {TOP}） |v|末窗均值={v:.5}");
    assert!(
        (y - TOP).abs() < 0.05,
        "布片应被托在盒顶（体心 y 实得 {y:+.5}，盒顶 {TOP}）"
    );
    assert!(v < 0.1, "末窗 |v| 均值 {v:.5} 未收敛");
}

/// **判据 ①（凸体腿）· 三角网 × 静态球**：布片落在球上 ⇒ **全程不深穿透**。
///
/// 薄片坐在球顶上会**翻落**（真物理，不该当失败）⇒ 这条判的是**接触的承诺**：
/// 逐 tick 取 9 个顶点里最深的 `|v_i − c| − r`，全程 ≥ `−0.05`。
/// 口径前提：球径（1.0）与顶点间距（0.5）可比 ⇒ 球**钻不过顶点之间**（顶点采样的已知边界，
/// 见 `mesh_pair.rs` 文件头）。
#[test]
fn trimesh_never_penetrates_sphere() {
    const R: f32 = 0.5;
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let mesh = w.add_trimesh(pts.clone(), tris);
    w.add_static(Shape::Sphere { radius: R }, Vec3::ZERO, Quat::IDENTITY);
    let b =
        w.spawn_trimesh_body(mesh, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0, 0.01) as usize;
    let mut worst = f32::MAX;
    for _ in 0..600 {
        w.step();
        let (p, q) = (w.bodies.position[b], w.bodies.rot(b));
        for v in &pts {
            worst = worst.min((p + q.rotate_vec3(*v)).length() - R);
        }
    }
    println!("[判据①-sphere] 全程最深顶点穿透 = {worst:+.5}（判据 ≥ −0.05）");
    assert!(
        worst > -0.05,
        "顶点穿透球体过深（最深 {worst:+.5}）⇒ 球腿的接触没守住（没有它，布片会直接落下穿）"
    );
}

/// **判据 ①（provider 腿，T1b-1 已落地）· 三角网 × 提供者（三角网地板）**：
/// 布片落到三角网地板上 ⇒ **被托住**（口径见下；与 `provider_shape_coverage.rs` 同族但**不卡绝对值**）。
///
/// 流形 = **逐顶点采样**（`mesh_pair.rs::mesh_provider_contacts`）：顶点即样本、法线取提供者
/// 在该点的外法线 ⇒ 平面地板上多个顶点同法线 ⇒ 一条流形多点支撑。
///
/// **两条口径**（都按本仓测量协议：决定量取**窗口均值**，单点差可以只是相位）：
/// ① **不下穿**：末体心高度有界（实测 −0.0086 m）；
/// ② **已收敛**：末 60 tick 的 `|v|` 均值 < 0.1（实测 0.047）。
/// 残留微幅振荡是**点采样路线**的既有性质（外壳路同量级）⇒ 由对拍判据
/// `trimesh_route_matches_hull_route_on_provider` 守；`就位间隙 ≈ 0` 的收紧另立一片。
#[test]
fn trimesh_is_held_by_provider_floor() {
    let (_, mesh_v, mesh_y) = drop_on_floor(true, 600);
    println!("[判据①] mesh: y={mesh_y:+.5} |v|末窗均值={mesh_v:.5}");
    assert!(
        mesh_y.abs() < 0.05,
        "布片应被托在地板附近（体心 y 实得 {mesh_y:+.5}）"
    );
    assert!(
        mesh_v < 0.1,
        "末窗 |v| 均值 {mesh_v:.5} 未收敛（真掉穿/自由落体会 ≫ 0.1）"
    );
}

/// **路线对拍（比值判据）**：同几何、同场景、只换**接触路线**——
/// **三角网顶点**（本片新路）vs **既有凸体外壳点云**（`hull_provider_contacts`，点查询同款采样）。
///
/// 判据：两条路都**被托住**，且新路的末窗 `|v|` 均值 **不超过老路的 2 倍**（实测 1.2×）。
/// 为什么用比值（本仓 §9 口径：耦合横向量取比值、不卡绝对值）：两条路的残留都是
/// **点采样 + 预测接触**的微幅振荡（实测 0.047 / 0.039），绝对值随机器/参数微动；比值只问
/// "新路有没有比既有路差一个量级"。
///
/// ⚠️ 这是**弱对照**（质量差 5 个量级：薄壳 `ρ·t·ΣA = 1000·0.01·1 ≈ 10 kg`，
/// 外壳走 AABB 盒兜底 ⇒ 半长 y=0 ⇒ `≈1e-4 kg`）——正因如此它同时说明：
/// 这层微振荡**与质量/惯量无关**（两路在 5 个量级差下同量级）⇒ 不是本片引入的伪影。
#[test]
fn trimesh_route_matches_hull_route_on_provider() {
    let (_, mesh_v, mesh_y) = drop_on_floor(true, 600);
    let (_, hull_v, hull_y) = drop_on_floor(false, 600);
    println!("[对拍] mesh y={mesh_y:+.5} |v|={mesh_v:.5} ／ hull y={hull_y:+.5} |v|={hull_v:.5}");
    assert!(
        mesh_y.abs() < 0.05 && hull_y.abs() < 0.05,
        "两条路都该被托住（mesh y={mesh_y:+.5} / hull y={hull_y:+.5}）"
    );
    assert!(
        mesh_v < hull_v * 2.0 + 1e-3,
        "三角网路的末窗 |v|（{mesh_v:.5}）比既有外壳路（{hull_v:.5}）差一个量级 ⇒ 新路有问题"
    );
}

/// 把「平铺网格」从 `y = 2` 掉到**三角网地板**上，跑 `ticks`；返回 `(体号, 末 60 tick 的 |v| 均值, 末体心 y)`。
///
/// `as_mesh = true` ⇒ 走三角网（`Shape::TriMesh`，顶点点查询）；`false` ⇒ 走既有凸体外壳
/// （同几何点云，`add_hull` + `spawn_hull_body`）⇒ **只换路线、不换几何**。
fn drop_on_floor(as_mesh: bool, ticks: usize) -> (usize, f32, f32) {
    let (pts, tris) = plate(2, 0.5);
    let mut w = World::new(PhysConfig::default());
    let _floor = w.add_mesh(flat_mesh());
    let body = if as_mesh {
        let m = w.add_trimesh(pts, tris);
        w.spawn_trimesh_body(m, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0, 0.01)
    } else {
        let h = w.add_hull(pts);
        w.spawn_hull_body(h, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0)
    } as usize;
    let mut vsum = 0.0f32;
    let mut n = 0.0f32;
    for tick in 0..ticks {
        w.step();
        if tick + 60 >= ticks {
            vsum += w.bodies.linvel[body].length();
            n += 1.0;
        }
    }
    (body, vsum / n, w.bodies.position[body].y)
}
