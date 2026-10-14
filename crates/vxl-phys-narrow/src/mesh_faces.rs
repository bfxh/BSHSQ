//! **三角网 × 凸体的采样本体**（从 `mesh_pair.rs` 迁入并**扩出"另一向"**）。
//!
//! **为什么另起一个文件**：`mesh_pair.rs` 受 god 门**行数棘轮**（只准减）—— 这族采样还会继续长
//! （其余形状的同款补向、面-面裁剪…），按 `mesh_mesh.rs` / `mesh_grid.rs` 的先例另起域文件，
//! `mesh_pair.rs` 只留薄入口。顶点向的两遍采样是**纯搬移**（逐字未改）。
//!
//! **三向采样**（P19 体侧的根因，2026-10-14 实测）：
//!
//! - **向 1（原有）：网顶点 × 对方形状**（`point_shape` 解析最近点）。网顶点即样本 ⇒ 一个顶点
//!   一个候选。**只有这一向时有一族漏配**：体**骑在网面两侧**（薄壳的典型形态）时，网的顶点
//!   全在体外、体的角点离面又有半个体长 ⇒ **两条路都零接触** ⇒ 直穿（实测 1 m 盒 60 m/s 撞
//!   4×4 m 单页薄壳：末态 x = **395 m**、`|v|` 一路 60 不变）。⚠️ 这不是"接触没进解算集"，
//!   是**该对整场零候选**（探针逐 tick 读流形表：`pair=true` 而流形表恒空）。
//! - **向 2（新增，仅盒）：盒的 14 个表面点（8 角 + 6 面心）× 网三角** —— 接近量，
//!   窗口 = **点到三角的真空距 ≤ skin**，`depth` 取**带符号的真距离**（正 = 已越过面）。
//! - **向 3（新增，仅盒）：盒的 12 条棱 × 网三角**（Möller–Trumbore，与
//!   `vxl-phys-terrain/src/mesh_cross.rs` 同式）—— 覆盖"**已经穿过去**"的深嵌档
//!   （此时 14 个表面点全部离面 > skin，向 2 看不见它）。
//!
//! 这三向与**提供者侧**（`TriMesh::contacts_box` = 14 点 + `surface_crossing`）同口径：
//! 体侧原来缺的正是后两向 ⇒ 同一格几何在 provider 路（`add_mesh`）拦得住、在体-体路
//! （`Shape::TriMesh` 体）拦不住。
//!
//! ⚠️ **向 2 只在"向 1 一无所获"时补**（`dom.is_none()`）：它是那一族**整场零候选**的补盲，
//! 不是给已经看得见的对**加冗余** —— 实测无条件加会在 `trimesh_is_held_by_compound` 上把
//! 末窗 `|v|` 从 <0.1 顶到 **0.106**（多余的冗余面接触改了收敛形状）。
//! ⚠️ **窗口必须用"点到三角的真距离"**，不能用 `point_triangle` 的**平面带符号距离**：薄壳
//! 无内外判定，平面距离对"面背后任意远处"也给正号 ⇒ 会把 1.5 m 外的样本也算成穿透，实测
//! 直接把 CCD 锁死在离面 **0.98 m** 处（首版踩过，别再改回去）。
//! ⚠️ 法线符号由"**盒心在三角哪一侧**"定 —— 不引入内外判定，但保证体**从哪一侧来就被推回
//! 哪一侧**（提供者侧 `surface_crossing` 同款口径）。
use super::{point_shape, point_triangle};
use crate::prims::select_contacts;
use crate::{ContactPoint, ContactPoints, DefaultNarrowPhase, Manifold};
use vxl_phys_core::{Mat3, Quat, Shape, Vec3};

/// 一次"三角网 × 凸体"采样的输入（收成结构体：`args-gate` 软阈值 7 形参）。
pub(crate) struct Sampler<'a> {
    /// 网在哪一侧（0 = 对侧 a，1 = 侧 b）——`np.ws.hull_pts[side]` 是**网顶点的世界点**。
    pub(crate) side: usize,
    pub(crate) mesh_shape: &'a Shape,
    pub(crate) other_shape: &'a Shape,
    pub(crate) other_pos: Vec3,
    pub(crate) other_rot: Quat,
    pub(crate) mesh_is_a: bool,
    pub(crate) a: u32,
    pub(crate) b: u32,
}

/// 盒的 12 条棱（角点下标按 `sx,sy,sz` 的字典序 ⇒ 只差一位的两个角相连）。
const BOX_EDGES: [(usize, usize); 12] = [
    (0, 1),
    (0, 2),
    (0, 4),
    (1, 3),
    (1, 5),
    (2, 3),
    (2, 6),
    (3, 7),
    (4, 5),
    (4, 6),
    (5, 7),
    (6, 7),
];

/// **一次完整的采样 + 装配**：主导面 → 同面候选（顶点向 + 盒的两向补采样）→ 流形。
///
/// 法线口径（从求解器定义反推）：求解器把 `+n·λ` 给 b、`−n·λ` 给 a，而每条候选给的 `n_o` 是
/// "把**样本**推出去"的方向 ⇒ 网的顶点是样本时，网在 a 侧 ⇒ `n(a→b) = −n_o`；网在 b 侧
/// ⇒ `n(a→b) = +n_o`。盒的两向里样本是**盒**（不在 `side` 槽上）⇒ 同样按"样本是哪一侧"折算，
/// 见 [`box_faces`] 的 `n_o` 定义。
pub(crate) fn mesh_vs_shape(np: &mut DefaultNarrowPhase, q: &Sampler<'_>, out: &mut Vec<Manifold>) {
    let mut dom = vertex_dominant(np, q);
    let mut extra: Vec<(Vec3, f32, Vec3, u32)> = Vec::new();
    if dom.is_none() {
        complement_faces(np, q, &mut dom, &mut extra);
    }
    let Some((n_dom, _)) = dom else {
        return;
    };
    collect_vertex(np, q, n_dom);
    collect_extra(&extra, n_dom, &mut np.ws.cand);
    if np.ws.cand.is_empty() || !select_contacts(np) {
        return;
    }
    out.push(Manifold {
        a: q.a,
        b: q.b,
        normal: if q.mesh_is_a { -n_dom } else { n_dom },
        points: ContactPoints::from_slice(&np.ws.cand),
    });
}

/// **向 1 的主导面** = 最深候选的法线（并列取先出现者 ⇒ 确定性）。
fn vertex_dominant(np: &DefaultNarrowPhase, q: &Sampler<'_>) -> Option<(Vec3, f32)> {
    let mut dom: Option<(Vec3, f32)> = None;
    for k in 0..np.ws.hull_pts[q.side].len() {
        let Some((n_o, depth, _)) = vertex_probe(np, q, k) else {
            continue;
        };
        if depth > -np.skin && dom.is_none_or(|(_, d)| depth > d) {
            dom = Some((n_o, depth));
        }
    }
    dom
}

/// **向 1 的同面候选**（点-盒角点可能同时落在两张面上 ⇒ 只留主导面，免得多法线混进同一流形）。
fn collect_vertex(np: &mut DefaultNarrowPhase, q: &Sampler<'_>, n_dom: Vec3) {
    np.ws.cand.clear();
    for k in 0..np.ws.hull_pts[q.side].len() {
        let Some((n_o, depth, hit)) = vertex_probe(np, q, k) else {
            continue;
        };
        if depth > -np.skin && n_o.dot(n_dom) > 0.9 {
            np.ws.cand.push(ContactPoint {
                point: hit,
                depth,
                feature: k as u32 + 1, // 顶点序稳定 ⇒ 跨帧可续接 warm
            });
        }
    }
}

/// 单个网顶点的解析最近点查询（`(对方外法线, 深度, 对方表面上的点)`）。
fn vertex_probe(np: &DefaultNarrowPhase, q: &Sampler<'_>, k: usize) -> Option<(Vec3, f32, Vec3)> {
    point_shape(
        np.ws.hull_pts[q.side][k],
        q.other_shape,
        q.other_pos,
        q.other_rot,
    )
}

/// 把补采样候选按主导面过滤后并进 `cand`（`n_dom` 之外的朝向不混进同一流形）。
fn collect_extra(extra: &[(Vec3, f32, Vec3, u32)], n_dom: Vec3, cand: &mut Vec<ContactPoint>) {
    for &(n_o, depth, point, feature) in extra {
        if n_o.dot(n_dom) > 0.9 {
            cand.push(ContactPoint {
                point,
                depth,
                feature,
            });
        }
    }
}

/// **对方形状的"另一向"补采样**（只在顶点向一无所获时调用）—— 按形状分派 + 把候选并进 `dom`。
///
/// 两族的分工（都是"**体自身的几何 × 网三角**"这半向，顶点向补不了）：
/// - **盒**（无半径）⇒ 必须拿形状自身的**表面点与棱**去撞网面（14 点 + 12 棱，见 [`box_faces`]）；
/// - **球/胶囊**（自带半径）⇒ 沿轴采样点 × 三角即可：半径感知的窗口天然覆盖"体跨过网面"那一档，
///   与 provider 侧 `contacts_sphere`（`depth = radius − |sd|` + 法线折向球心）**同一把尺**。
///
/// 其余形状（圆柱/锥）**如实不受理**：它们的采样点在**表面**上、窗口只有皮肤带那么宽
/// （实测 ⑥ 形状矩阵里那两族在 provider 路与体侧路都还是直穿）；补齐要一条"体积向"，
/// 属下一片（登记在 `OPEN-PROBLEMS.md` P19）。
fn complement_faces(
    np: &mut DefaultNarrowPhase,
    q: &Sampler<'_>,
    dom: &mut Option<(Vec3, f32)>,
    out: &mut Vec<(Vec3, f32, Vec3, u32)>,
) {
    match *q.other_shape {
        Shape::Box { .. } => box_faces(np, q, out),
        Shape::Sphere { radius } => round_faces(np, q, &[q.other_pos], radius, out),
        Shape::Capsule {
            half_height,
            radius,
        } => {
            // 5 点沿轴均布（与 provider 侧 `capsule_provider_contacts` 同数同布）
            let axis = Mat3::from_quat(q.other_rot).mul_vec3(Vec3::Y);
            let mut s = [Vec3::ZERO; 5];
            for (k, p) in s.iter_mut().enumerate() {
                let t = k as f32 * 0.25;
                *p = q.other_pos + axis * (half_height * (2.0 * t - 1.0));
            }
            round_faces(np, q, &s, radius, out);
        }
        // 圆柱/圆锥：**表面点采样**（与 provider 侧 `ring_provider_contacts` 同集合：3 环 × 4 方位
        // + 圆锥顶点）—— 深度取"带符号真距离"、法线按**体心**折，见 [`surface_hits`]。
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let mut s = [Vec3::ZERO; 12];
            ring_samples(
                q,
                &[(-half_height, radius), (0.0, radius), (half_height, radius)],
                &mut s,
            );
            surface_hits(np, q, &s, out);
            let mut segs = Vec::new();
            revolution_edges(q, half_height, radius, false, &mut segs);
            edge_hits_of(np, q, &segs, out);
        }
        Shape::Cone {
            half_height,
            radius,
        } => {
            let mut s = [Vec3::ZERO; 13];
            let rings = [
                (-half_height, radius),
                (-half_height * 0.5, radius * 0.75),
                (0.0, radius * 0.5),
            ];
            ring_samples(q, &rings, &mut s[..12]);
            let axis = Mat3::from_quat(q.other_rot).mul_vec3(Vec3::Y);
            s[12] = q.other_pos + axis * half_height; // 圆锥顶点（与 provider 侧的 `tip` 同位）
            surface_hits(np, q, &s, out);
            let mut segs = Vec::new();
            revolution_edges(q, half_height, radius, true, &mut segs);
            edge_hits_of(np, q, &segs, out);
        }
        _ => {}
    }
    for &(n_o, depth, _, _) in out.iter() {
        if dom.is_none_or(|(_, d)| depth > d) {
            *dom = Some((n_o, depth));
        }
    }
}

/// **球/胶囊的"另一向"**：沿轴采样点（每个都是半径 `radius` 的球）× 网三角。
///
/// 口径与 provider 侧 `contacts_sphere` **同一把尺**：`d` = 采样点到三角的**真距离**、
/// 法线 = "**从面指向采样点**"（= 采样点在面哪一侧，**不引入内外判定**）、
/// 深度 = `radius − d`（正 = 球面已越过面；负 = 预期接触，与 `depth > -skin` 的窗口同口径）。
/// ⚠️ 不能用 `point_triangle` 的平面带符号距离当 `d`（那是旧 `contacts_sphere` 的坑：
/// 背面来时报 `radius + 距离`）。
fn round_faces(
    np: &DefaultNarrowPhase,
    q: &Sampler<'_>,
    samples: &[Vec3],
    radius: f32,
    out: &mut Vec<(Vec3, f32, Vec3, u32)>,
) {
    let Shape::TriMesh { mesh, .. } = *q.mesh_shape else {
        return;
    };
    let pts = &np.ws.hull_pts[q.side];
    let tris = np.meshes.tris(mesh);
    let skin = np.skin;
    for (si, c) in samples.iter().enumerate() {
        for (ti, tri) in tris.iter().enumerate() {
            let (w0, w1, w2) = world_tri(pts, tri);
            let Some((n_tri, _, hit)) = point_triangle(*c, w0, w1, w2) else {
                continue;
            };
            let d = (*c - hit).length();
            if d > radius + skin {
                continue;
            }
            let n_out = if d > 1e-6 {
                (*c - hit) * (1.0 / d)
            } else {
                n_tri
            };
            let feature = (3u32 << 20) + (si as u32) * (1 << 12) + ti as u32;
            out.push((-n_out, radius - d, hit, feature));
        }
    }
}

/// **盒 × 网**的"另一向"采样：候选压进 `out`（主导面的合并由 [`complement_faces`] 统一做）。
///
/// 返回的 `n_o` 与顶点向**同约定**（"把样本推出去"）：这里的样本是**盒** ⇒ 先把方向定义成
/// "把盒推出去"（`n_tri × sign`，`sign` 由盒心在三角哪一侧定），再取反 —— 装配处按"网在哪
/// 一侧"折算 `a→b`，而盒总是**不在网的槽上**（推导见 [`mesh_vs_shape`]）。
fn box_faces(np: &mut DefaultNarrowPhase, q: &Sampler<'_>, out: &mut Vec<(Vec3, f32, Vec3, u32)>) {
    let (Shape::Box { half }, Shape::TriMesh { .. }) = (*q.other_shape, *q.mesh_shape) else {
        return;
    };
    let rot = Mat3::from_quat(q.other_rot);
    let cs = box_corners(q.other_pos, &rot, half);
    let samples = box_surface_points(q.other_pos, &rot, half, &cs);
    surface_hits(np, q, &samples, out);
    let segs: Vec<(Vec3, Vec3)> = BOX_EDGES.iter().map(|&(u, v)| (cs[u], cs[v])).collect();
    edge_hits_of(np, q, &segs, out);
}

/// [`edge_hits`] 的门面（从 `np`/`q` 取网顶点缓存、三角表与皮肤带）。
fn edge_hits_of(
    np: &DefaultNarrowPhase,
    q: &Sampler<'_>,
    segs: &[(Vec3, Vec3)],
    out: &mut Vec<(Vec3, f32, Vec3, u32)>,
) {
    let Shape::TriMesh { mesh, .. } = *q.mesh_shape else {
        return;
    };
    let pts = &np.ws.hull_pts[q.side];
    edge_hits(pts, np.meshes.tris(mesh), segs, q.other_pos, np.skin, out);
}

/// 盒的 8 个角（世界点；下标按 `sx,sy,sz` 的字典序 ⇒ 与 [`BOX_EDGES`] 同序）。
fn box_corners(pos: Vec3, rot: &Mat3, half: Vec3) -> [Vec3; 8] {
    let mut cs = [Vec3::ZERO; 8];
    let mut i = 0usize;
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let h = Vec3::new(sx * half.x, sy * half.y, sz * half.z);
                cs[i] = pos + rot.mul_vec3(h);
                i += 1;
            }
        }
    }
    cs
}

/// 盒的 **14 个表面点** = 8 角 + 6 面心（与提供者侧 `contacts_box` 的采样面一致）。
fn box_surface_points(pos: Vec3, rot: &Mat3, half: Vec3, cs: &[Vec3; 8]) -> [Vec3; 14] {
    let fc = [
        Vec3::new(half.x, 0.0, 0.0),
        Vec3::new(-half.x, 0.0, 0.0),
        Vec3::new(0.0, half.y, 0.0),
        Vec3::new(0.0, -half.y, 0.0),
        Vec3::new(0.0, 0.0, half.z),
        Vec3::new(0.0, 0.0, -half.z),
    ];
    let mut s = [Vec3::ZERO; 14];
    s[..8].copy_from_slice(cs);
    for (i, h) in fc.iter().enumerate() {
        s[8 + i] = pos + rot.mul_vec3(*h);
    }
    s
}

/// **向 2**：体表面点 × 三角（接近量；窗口 = **点到三角的真距离** ≤ skin，深度 = **带符号**真距离）。
///
/// 法线按 **体心在哪一侧**折（`side_sign`）—— 采样点自己的那一侧在"正好贴上"时会来回跳
/// （体心不会）⇒ 用体心。这条口径对**盒**（14 点）与**圆柱/锥**（环采样点）共用：
/// 两者都是"无半径的表面点"⇒ 深度取带符号真距离（正 = 已越过面）；球/胶囊有半径 ⇒ 走 [`round_faces`]。
fn surface_hits(
    np: &DefaultNarrowPhase,
    q: &Sampler<'_>,
    samples: &[Vec3],
    out: &mut Vec<(Vec3, f32, Vec3, u32)>,
) {
    let Shape::TriMesh { mesh, .. } = *q.mesh_shape else {
        return;
    };
    let pts = &np.ws.hull_pts[q.side];
    let tris = np.meshes.tris(mesh);
    let (center, skin) = (q.other_pos, np.skin);
    for (pi, p) in samples.iter().enumerate() {
        for (ti, tri) in tris.iter().enumerate() {
            let (w0, w1, w2) = world_tri(pts, tri);
            let Some((n_tri, _, hit)) = point_triangle(*p, w0, w1, w2) else {
                continue;
            };
            if (*p - hit).length() > skin {
                continue;
            }
            let n_out = n_tri * side_sign(n_tri, center, w0);
            let depth = -(*p - hit).dot(n_out); // 正 = 已越过面（穿透）
            let feature = (1u32 << 20) + (pi as u32) * (1 << 12) + ti as u32;
            out.push((-n_out, depth, hit, feature));
        }
    }
}

/// **三环 × 4 方位**（环平面内 `+X` 起算 90° 均布）—— 与 provider 侧 `ring_provider_contacts`
/// 同集合（局部系轴 = +Y：`(轴向位置, 环半径)`）。方位起点用**局部 +X** 而不是"世界最低方向"：
/// 那条起算规则是为**支撑面**（重力方向）定的，网格碰撞的方向由网面决定 ⇒ 这里只需覆盖环上的极值。
fn ring_samples(q: &Sampler<'_>, rings: &[(f32, f32); 3], out: &mut [Vec3]) {
    let m = Mat3::from_quat(q.other_rot);
    for (ri, (cy, rho)) in rings.iter().enumerate() {
        for k in 0..4 {
            let th = std::f32::consts::FRAC_PI_2 * k as f32;
            let h = Vec3::new(rho * th.cos(), *cy, rho * th.sin());
            out[ri * 4 + k] = q.other_pos + m.mul_vec3(h);
        }
    }
}

/// **向 3**：**体的棱（线段表）× 三角**（穿越量）。深嵌档只有它能看见 ⇒ 深度记满皮肤带
/// （与提供者侧 `mesh_cross` 的交叉通道同口径）。
///
/// 段表由调用方给：**盒** = 12 条棱（精确）；**圆柱/圆锥** = 端面圆折线 + 母线（[`revolution_edges`]，
/// 见那里的完备性说明）。
fn edge_hits(
    pts: &[Vec3],
    tris: &[[u32; 3]],
    segs: &[(Vec3, Vec3)],
    center: Vec3,
    skin: f32,
    out: &mut Vec<(Vec3, f32, Vec3, u32)>,
) {
    for (ti, tri) in tris.iter().enumerate() {
        let (w0, w1, w2) = world_tri(pts, tri);
        let crossed = segs
            .iter()
            .any(|&(a, b)| seg_tri(a, b, w0, w1, w2).is_some());
        let Some((n_tri, _, hit)) = point_triangle(center, w0, w1, w2) else {
            continue;
        };
        if !crossed {
            continue;
        }
        let n_out = n_tri * side_sign(n_tri, center, w0);
        out.push((-n_out, skin, hit, (2u32 << 20) + ti as u32));
    }
}

/// **回转体（圆柱/圆锥）的"棱"**：端面圆折线（8 等分）+ 母线（线段）—— 与盒的 12 棱同口径的
/// 穿越通道（薄壳穿过回转体侧面/端面时用它，而不是靠"采样点落在皮肤带内"那条空间窗口）。
///
/// ```text
/// 圆柱：y = ±h 两条端面圆（各 8 段）+ 8 条平行于轴的母线
/// 圆锥：y = −h 的底圈（8 段）+ 8 条从底圈到顶点的母线
/// ```
///
/// ⚠️ **完备性（如实登记）**：方位取 8 等分 ⇒ 侧面上**两条相邻折线之间**那段曲面只有弦近似；
/// 薄壳正好从那个"缝"里穿、且**两条端面圆都不与它相交**时仍可能漏（盒那条 12 棱是精确的）。
/// 这是"采样完备性"的取舍：8 等分把弦长压到 `2r·sin(22.5°) ≈ 0.77r`，实测（`p19_body_side_probe`
/// 的相位扫描）圆柱 6–9.9 m/s 的 64 相位从 **13–33 档穿**降到 **0 档**。
fn revolution_edges(
    q: &Sampler<'_>,
    half_height: f32,
    radius: f32,
    cone: bool,
    out: &mut Vec<(Vec3, Vec3)>,
) {
    const N: usize = 8;
    let m = Mat3::from_quat(q.other_rot);
    let ring = |y: f32, r: f32, k: usize| {
        let th = std::f32::consts::TAU * k as f32 / N as f32;
        q.other_pos + m.mul_vec3(Vec3::new(r * th.cos(), y, r * th.sin()))
    };
    // 端面圆折线：圆柱两条（±h）、圆锥只有底圈（−h）
    let levels: &[f32] = if cone {
        &[-half_height]
    } else {
        &[-half_height, half_height]
    };
    for &y in levels {
        for k in 0..N {
            out.push((ring(y, radius, k), ring(y, radius, k + 1)));
        }
    }
    // 母线：圆柱上下同方位相连；圆锥底圈 → 顶点
    let tip = q.other_pos + m.mul_vec3(Vec3::Y * half_height);
    for k in 0..N {
        let a = ring(-half_height, radius, k);
        let b = if cone {
            tip
        } else {
            ring(half_height, radius, k)
        };
        out.push((a, b));
    }
}

/// 网三角的世界点（`pts` = 网顶点的世界缓存）。
fn world_tri(pts: &[Vec3], tri: &[u32; 3]) -> (Vec3, Vec3, Vec3) {
    (
        pts[tri[0] as usize],
        pts[tri[1] as usize],
        pts[tri[2] as usize],
    )
}

/// "盒心在三角哪一侧"的符号（`+1` = 与环绕法线同侧）⇒ 法线乘它即"把盒推回去"。
fn side_sign(n_tri: Vec3, box_center: Vec3, tri_p0: Vec3) -> f32 {
    if n_tri.dot(box_center - tri_p0) >= 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// **段-三角相交**（Möller–Trumbore 闭式解）：线段 `p0→p1` 与三角 `(a,b,c)`，命中返回线段参数
/// `t ∈ [0,1]`；平行 / 退化（`p0 == p1`、零面积三角）/ 未命中 ⇒ `None`。
/// 与 `vxl-phys-terrain/src/mesh_cross.rs` 的 `seg_tri` 同式（本 crate 不依赖 terrain ⇒ 各留一份）。
fn seg_tri(p0: Vec3, p1: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    const EPS: f32 = 1e-9;
    let (d, e1, e2) = (p1 - p0, b - a, c - a);
    let h = d.cross(e2);
    let det = e1.dot(h);
    if det.abs() < EPS {
        return None;
    }
    let inv = 1.0 / det;
    let s = p0 - a;
    let u = s.dot(h) * inv;
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    let t = e2.dot(q) * inv;
    let inside = (0.0..=1.0).contains(&u) && v >= 0.0 && u + v <= 1.0 && (0.0..=1.0).contains(&t);
    inside.then_some(t)
}
