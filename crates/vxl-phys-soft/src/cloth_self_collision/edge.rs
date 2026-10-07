//! **点-边自碰撞**（T3 登记的进阶档）：补点-点对"从两粒子**之间**穿过"的盲区。
//!
//! **为什么点-点有盲区**：位置投影采的是**离散时刻**；粒子从一条边的**中段**穿过时，到两个
//! 端点的距离都 `> d_c`，点-点**永远不触发**（本仓尺度：网格间距 0.125 m vs 接触带 `d_c`
//! 0.01–0.07 m ⇒ 盲区是常态）。点-边改测**到线段**的距离：`< d_c` 就沿法向推开（逆质量
//! 分担，投影点重心坐标 `u` 的杠杆权重 `(1−u)²/u²`——端点处退化为点-点同式）。
//!
//! **开关（默认关 ⇒ 零换代）**：子开关 [`PointEdge::enabled`] 首行短路 ⇒ T3 的每条读数
//! **逐位不变**。
//!
//! **边界（写清，不是漏）**：仍是离散投影 ⇒ **不承诺 CCS**（与 T3 同一句话）；退化边（两端点
//! 重合）留给点-点；排除集 = 端点与粒子成**网格邻居**（结构/剪切/弯曲同一份 `forbidden`
//! ⇒ 不与折痕处的弹簧对顶）。
//! **自摩擦已落地**（2026-10-07，[`crate::cloth_edge_friction`]）：与点-点自摩擦**同一条**
//! 库仑锥、同一个 `μ`（`SelfCollision::friction`，默认 `0` = 关）；调用点在 `project` 里只占
//! 一行 —— 那个函数卡在 god 门的最长函数棘轮上，故实现与判据都在独立模块里。
//!
//! **确定性**：边按 `cons` 升序建桶 ⇒ 桶内升序；逐粒子候选经 `seen` 去重（复用 ⇒ 热路径
//! 零分配）；每个（粒子, 边）对每子步**恰好解算一次**。
//!
//! **布局**：本文件是 `cloth_self_collision` 的**同名目录式子模块**（`mod edge;`）。父文件
//! 受行数棘轮，`SelfContacts::new` 的两个小构建器（默认值/禁止集）也一并收进这里 ⇒ 父文件
//! 回到基线以下（god 门"可收紧"）。
use super::cell_of;
use crate::cloth::ClothSheet;
use crate::params::SelfCollision;
use std::collections::BTreeMap;
use vxl_phys_core::Vec3;

/// **点-边子系统状态**（边表 + 空间哈希 + 开关；挂在 `SelfContacts.edges`；**默认关**）。
#[derive(Clone, Default)]
pub struct PointEdge {
    /// 子开关（**默认 `false`** ⇒ 首行短路 ⇒ T3 读数逐位不变；打开是有意的行为变化）。
    pub enabled: bool,
    /// 三角网唯一边集（= `cons` 的顺序 ⇒ 确定性；`topology()` 的注释钉了这一口径）。
    list: Vec<[u32; 2]>,
    /// 空间哈希（边 AABB → 桶；桶内 = 插入序 = 边升序）。
    cells: BTreeMap<(i32, i32, i32), Vec<u32>>,
    /// 逐粒子候选去重的 scratch（复用 ⇒ 热路径零分配）。
    seen: Vec<u32>,
    /// 最近一个子步实际解算的点-边对数（诊断/判据读）。
    pub pairs: u32,
}

/// 一条边允许跨的格子数上限（病态大边保护；超过就整条跳过 ⇒ 不挂死，确定性不变）。
const MAX_CELLS_PER_EDGE: i64 = 1 << 12;

impl PointEdge {
    pub(crate) fn new(cons: &[[u32; 2]]) -> Self {
        Self {
            enabled: false,
            list: cons.to_vec(),
            cells: BTreeMap::new(),
            seen: Vec::new(),
            pairs: 0,
        }
    }

    /// **建格**：边 AABB 覆盖到的每个格子收一份边号（格边长 = 碰撞直径，与点-点同一个
    /// `inv` ⇒ 粒子的 3×3×3 邻域必覆盖到 `d_c` 内的全部边）。
    fn rebuild(&mut self, pos: &[Vec3], inv: f32) {
        self.cells.clear();
        for e in 0..self.list.len() {
            let [a, b] = self.list[e];
            let (pa, pb) = (pos[a as usize], pos[b as usize]);
            let lo = (
                (pa.x.min(pb.x) * inv).floor() as i32,
                (pa.y.min(pb.y) * inv).floor() as i32,
                (pa.z.min(pb.z) * inv).floor() as i32,
            );
            let hi = (
                (pa.x.max(pb.x) * inv).floor() as i32,
                (pa.y.max(pb.y) * inv).floor() as i32,
                (pa.z.max(pb.z) * inv).floor() as i32,
            );
            let span =
                (hi.0 - lo.0 + 1) as i64 * (hi.1 - lo.1 + 1) as i64 * (hi.2 - lo.2 + 1) as i64;
            if span > MAX_CELLS_PER_EDGE {
                continue; // 病态大边（网格坏了）⇒ 跳过这条，别把建格挂死
            }
            for x in lo.0..=hi.0 {
                for y in lo.1..=hi.1 {
                    for z in lo.2..=hi.2 {
                        self.cells.entry((x, y, z)).or_default().push(e as u32);
                    }
                }
            }
        }
    }
}

/// `SelfContacts::new` 的**默认值构建器**（父文件行数棘轮 ⇒ 收进本文件）。
pub(super) fn default_cfg(particle_radius: f32) -> SelfCollision {
    SelfCollision {
        enabled: false,
        particle_radius,
        ..SelfCollision::default()
    }
}

/// `SelfContacts::new` 的**禁止集构建器**（结构/剪切 + 弯曲；排序去重 ⇒ 二分）。
pub(super) fn build_forbidden(cons: &[[u32; 2]], bend: &[[u32; 2]]) -> Vec<[u32; 2]> {
    let mut f: Vec<[u32; 2]> = cons.iter().chain(bend.iter()).copied().collect();
    f.sort_unstable();
    f.dedup();
    f
}

impl ClothSheet {
    /// **自碰撞收尾**（`project_self_contacts` 尾行 1:1 替换"写回 pairs"⇒ 那个函数的
    /// 行数棘轮不动）：写回点-点对数 + **点-边趟**（子开关默认关 ⇒ 整段跳过）。
    pub(crate) fn finish_self_contacts(&mut self, point_pairs: u32) {
        self.self_contacts.pairs = point_pairs;
        if !self.self_contacts.edges.enabled {
            return; // 点-边关（默认）⇒ T3 读数逐位不变
        }
        let d_c = 2.0 * self.self_contacts.cfg.particle_radius;
        let inv = 1.0 / d_c;
        let pairs = project(self, d_c, inv);
        self.self_contacts.edges.pairs = pairs;
    }
}

/// **候选收集**（粒子 `i` 的 27 邻格边号，去重后写进 `seen`，返回条数）—— 从 `project`
/// 抽出：那里卡在 god 门的**最长函数**棘轮上，本片（加摩擦）必须腾出降幅才能合法涨行；
/// 桶内是插入序 = 边升序 ⇒ 去重后的次序确定（与原内联实现逐条一致）。
fn collect(sheet: &mut ClothSheet, i: usize, inv: f32) -> usize {
    let ci = cell_of(sheet.pos[i], inv);
    let seen = &mut sheet.self_contacts.edges.seen;
    seen.clear();
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                let key = (ci.0 + dx, ci.1 + dy, ci.2 + dz);
                let Some(bucket) = sheet.self_contacts.edges.cells.get(&key) else {
                    continue;
                };
                for &e in bucket {
                    if !seen.contains(&e) {
                        seen.push(e); // 同一条边落在多个格 ⇒ 去重
                    }
                }
            }
        }
    }
    seen.len()
}

/// **点-边投影**（点-点趟之后的顺序 Gauss-Seidel 一遍）：按自由粒子升序 × 27 邻格内
/// 去重后的边候选，每个（粒子, 边）恰好解算一次。
fn project(sheet: &mut ClothSheet, d_c: f32, inv: f32) -> u32 {
    let n = sheet.pos.len();
    let mut pairs = 0u32;
    let friction = sheet.self_contacts.cfg.friction;
    sheet.self_contacts.edges.rebuild(&sheet.pos, inv);
    for i in 0..n {
        if sheet.inv_mass[i] == 0.0 {
            continue; // 钉住粒子不发起（与点-点同惯例）
        }
        // ②a 候选收集（抽成 `collect`，见它的注）。
        let cand_len = collect(sheet, i, inv);
        // ②b 解算（候选表此后只读；`pos` 是别的字段 ⇒ 整体借用相容）。
        for ei in 0..cand_len {
            let e = sheet.self_contacts.edges.seen[ei];
            let [j32, k32] = sheet.self_contacts.edges.list[e as usize];
            if sheet.self_contacts.is_forbidden(i as u32, j32)
                || sheet.self_contacts.is_forbidden(i as u32, k32)
            {
                continue; // 端点与粒子成网格邻居 ⇒ 不解算（不与折痕处弹簧对顶）
            }
            let (j, k) = (j32 as usize, k32 as usize);
            let (pj, pk) = (sheet.pos[j], sheet.pos[k]);
            let ab = pk - pj;
            let ab2 = ab.length_squared();
            if ab2 < 1e-12 {
                continue; // 退化边（两端点重合）⇒ 留给点-点
            }
            let pi = sheet.pos[i];
            // 线段最近点（Ericson）：`u` 钳到 [0,1] ⇒ 端点处自然退化为点-点同式。
            let u = ((pi - pj).dot(ab) / ab2).clamp(0.0, 1.0);
            let c = pj + ab * u;
            let d = pi - c;
            let len = d.length();
            if len >= d_c {
                continue; // 不在接触距离内
            }
            if len < 1e-9 {
                continue; // 同心退化：方向无定义（与点-点同惯例）
            }
            let w_i = sheet.inv_mass[i];
            let (w_j, w_k) = (sheet.inv_mass[j], sheet.inv_mass[k]);
            // 杠杆权重：投影点随 `u` 移动的有效逆质量（XPBD 边接触标准式）。
            let w = w_i + w_j * (1.0 - u) * (1.0 - u) + w_k * u * u;
            if w <= 0.0 {
                continue; // j、k 都钉住（i 自由 ⇒ 保底）
            }
            let nrm = d * (1.0 / len);
            let lam = (d_c - len) / w;
            sheet.pos[i] += nrm * (w_i * lam);
            sheet.pos[j] -= nrm * (w_j * (1.0 - u) * lam);
            sheet.pos[k] -= nrm * (w_k * u * lam);
            crate::cloth::cloth_edge_friction::resist_edge_slip(
                (&mut sheet.pos, &sheet.prev, &sheet.inv_mass),
                friction,
                (i, j, k),
                (u, w),
                (nrm, d_c - len),
            );
            pairs += 1;
        }
    }
    pairs
}

impl super::SelfContacts {
    /// 最近一个子步解算的**点-边对数**（诊断/判据读；字段 crate 内私有 ⇒ 走访问器）。
    pub fn edge_pairs(&self) -> u32 {
        self.edges.pairs
    }

    /// **点-边子开关**（默认 `false` ⇒ T3 读数逐位不变；打开是有意的行为变化）。
    pub fn set_point_edge(&mut self, on: bool) {
        self.edges.enabled = on;
    }
}
