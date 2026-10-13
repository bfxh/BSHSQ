//! **面穿过体**（P19）：网格接触的"12 条棱 × 三角形"相交补丁 —— `mesh.rs` 的子模块。
//!
//! **为什么拆出来**：`mesh.rs` 受 god 门**文件行数棘轮**（只准减）—— 这块 ~100 行留在那里会超窗。
//! 先例与理由同 `mesh_mesh.rs → mesh_grid.rs`：**还会继续长的采样代码另起域文件**。
//!
//! **为什么需要**：`contacts_box` 的 14 点采样（8 角 + 6 面心）量的都是"**点到面**的距离"。
//! 体**骑在网格面两侧**时（薄壳的典型形态），面心到面的距离 = `|体心到面|` ⇒ 只有当
//! `|体心| ≤ skin(0.02)` 才出接触 ⇒ **接触窗口 = 皮肤带宽**（0.04 m），而采样步长是
//! `0.5×最小半长`（1 m 盒 = 0.25 m）、每 tick 还走 1 m ⇒ **离散管线与 CCD 都整段跳过**
//! （实测 60 m/s 的盒撞 4×4 m 单页薄壳**直穿 395 m**；金丝雀曾是 `tests/ccd_thin_shell.rs` ④，
//! 现已翻面成"必须停住"）。补一遍相交测试后，窗口回到"**体沿法向尺寸**"（1 m 盒 ⇒ ≥4 个采样步）
//! ⇒ 离散与 CCD 都能接住。
//!
//! ⚠️ 本档只覆盖**盒**；球/胶囊/圆柱/锥的"面穿过体"仍缺，登记在
//! `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md` 与 `crates/vxl-phys/tests/shape_support_matrix.rs`
//! 的矩阵/金丝雀里（**不是静默**）。
use super::{closest_on_tri, TriMesh};
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Mat3, Quat, Vec3};

/// 一次盒查询的入参（收成结构体：`args-gate` 的软阈值 ≤7 参，盒子查询自带 5 个）。
pub(super) struct BoxQuery {
    pub id: u32,
    pub half: Vec3,
    pub pos: Vec3,
    pub rot: Quat,
    pub skin: f32,
}

/// 盒的 12 条棱（角点下标按 `sx,sy,sz` 的字典序 ⇒ 只差一位的两个角相连）。
const EDGES: [(usize, usize); 12] = [
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

/// **段-三角相交**（Möller–Trumbore 闭式解）：线段 `p0→p1` 与三角 `(a,b,c)`，命中返回线段参数
/// `t ∈ [0,1]`；平行 / 退化（`p0 == p1`、零面积三角）/ 未命中 ⇒ `None`。
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
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (0.0..=1.0).contains(&t).then_some(t)
}

/// **盒 × 网格**的整条接触查询（8 角 + 6 面心 + **12 棱相交**）—— `mesh.rs` 的 `contacts_box`
/// 现在只留一行调用，逻辑与口径全在这里。
///
/// **接触口径（相交那一路）**：`point` = 体心到该三角的最近点（喂 warm/破坏管线；不用棱上的几何
/// 交点 —— 那会让接触点/法线对不齐）、`normal` = 三角环绕法线 × `sign`（`sign` 由"体心在三角哪
/// 一侧"定 ⇒ 把体**推回它来的一侧**，**不引入内外判定**）、`depth = skin`（位置级去穿透交求解器
/// erp，与皮肤带口径一致：薄壳无内外判定，沿法向的"穿透量"没有唯一值）。
/// `feature = (1<<20) + ti`（与点采样的 `ti+1` 错开，免跨帧误配 warm）。
pub(super) fn contacts_box(mesh: &TriMesh, q: &BoxQuery, out: &mut Vec<InteropContact>) -> bool {
    let (id, half, pos, rot, skin) = (q.id, q.half, q.pos, q.rot, q.skin);
    let m = Mat3::from_quat(rot);
    let mut any = false;
    let mut cs = [Vec3::ZERO; 8];
    let mut ci = 0usize;
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let w = pos + m.mul_vec3(Vec3::new(sx * half.x, sy * half.y, sz * half.z));
                cs[ci] = w;
                ci += 1;
                any |= mesh.contacts_point(id, w, skin, out);
            }
        }
    }
    for axis in 0..3 {
        let h = match axis {
            0 => Vec3::new(half.x, 0.0, 0.0),
            1 => Vec3::new(0.0, half.y, 0.0),
            _ => Vec3::new(0.0, 0.0, half.z),
        };
        for s in [-1.0f32, 1.0] {
            any |= mesh.contacts_point(id, pos + m.mul_vec3(h * s), skin, out);
        }
    }
    // ⚠️ 候选半径只取**几何覆盖**（外接球半径 + 一个**有界**余量）：`skin` 形参在高速下是**速度
    // 自适应接触带**（`velocity_band`，实测 1920 m/s 时 48 m）——那是接触余量、不是覆盖需求。
    // 用满 band 会让 `r` 越界成百上千格，反过来把这条判据自己关掉（实测 ≥1920 m/s 档
    // `[cross]` 零调用 ⇒ 又穿）。覆盖论证只需要：**穿过体的三角形必在体外接球内**。
    let r = half.length() + skin.min(0.5);
    any |= surface_crossing(mesh, &cs, pos, r, skin, out);
    any
}

/// **面穿过体**：盒的 12 条棱 × 网格三角形 的相交测试（见 [`seg_tri`]）。
///
/// 候选 = 体心 ± `r`（= 外接球半径 + skin）覆盖的格子里的三角 —— 面**穿过体**时该三角必在
/// 外接球内 ⇒ 这一遍是**完备**的。体远大于格边（`r > 8·bin`）时候选会爆 ⇒ 直接跳过：那一类形态
/// （网格顶点落进体里）由点采样那一路覆盖，是**另一类**接触。
fn surface_crossing(
    mesh: &TriMesh,
    cs: &[Vec3; 8],
    center: Vec3,
    r: f32,
    skin: f32,
    out: &mut Vec<InteropContact>,
) -> bool {
    let Some(g) = &mesh.grid else {
        return false;
    };
    if r > 8.0 * g.bin {
        return false;
    }
    let bin_of = |p: f32, o: f32| -> i64 { ((p - o) / g.bin).floor() as i64 };
    let (x0, x1) = (
        bin_of(center.x - r, g.origin.x),
        bin_of(center.x + r, g.origin.x),
    );
    let (y0, y1) = (
        bin_of(center.y - r, g.origin.y),
        bin_of(center.y + r, g.origin.y),
    );
    let (z0, z1) = (
        bin_of(center.z - r, g.origin.z),
        bin_of(center.z + r, g.origin.z),
    );
    let mut any = false;
    for x in x0.max(0)..=x1.min(g.dims.0 as i64 - 1) {
        for y in y0.max(0)..=y1.min(g.dims.1 as i64 - 1) {
            for z in z0.max(0)..=z1.min(g.dims.2 as i64 - 1) {
                let i = ((x as usize) * g.dims.1 + y as usize) * g.dims.2 + z as usize;
                for &ti in &g.bins[i] {
                    let t = mesh.tris[ti as usize];
                    let (a, b, c) = (
                        mesh.verts[t[0] as usize],
                        mesh.verts[t[1] as usize],
                        mesh.verts[t[2] as usize],
                    );
                    if !EDGES
                        .iter()
                        .any(|&(u, v)| seg_tri(cs[u], cs[v], a, b, c).is_some())
                    {
                        continue;
                    }
                    let n = mesh.normals[ti as usize];
                    let sign = if n.dot(center - a) >= 0.0 { 1.0 } else { -1.0 };
                    out.push(InteropContact {
                        point: closest_on_tri(center, a, b, c),
                        normal: n * sign,
                        depth: skin,
                        feature: (1 << 20) + ti,
                    });
                    any = true;
                }
            }
        }
    }
    any
}
