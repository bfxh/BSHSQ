//! **ClothSelfCollideExt**：`ClothSheet` 的**自碰撞约束族**——本族**不往 `ClothSheet` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——36 个
//! 方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在同一片上，
//! 处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，判据是
//! 「新加一族约束**不必再往本类型添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `ClothSheet` 的方法数**不计入本类型**（门账 36 → 减本族 2 个）。
//!
//! **先例**：同判据的既有落法见 `crates/vxl-phys/src/world_struct/ext/`（`World` 76 → 22）。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
//!
//! **本族跨两个文件**：点-点趟住在 `cloth_self_collide_pass.rs`（`#[path]` 子模块）、
//! 点-边趟住在 `cloth_self_collision/edge.rs` ⇒ 两个方法搬进**同一个** trait
//! （同族同面 ⇒ 同一个扩展点），实现体也一并搬来（`cell_of` 仍在
//! `cloth_self_collision.rs` 里，**私有** ⇒ 本文件用 `super::` 路径访问）。
use crate::cloth::ClothSheet;
use crate::cloth_self_collision::cell_of;

/// **ClothSelfCollideExt**：`ClothSheet` 的**自碰撞约束族**（布片对自己）。
pub(crate) trait ClothSelfCollideExt {
    /// **自碰撞投影**（每子步一次，在建格位置上做一遍顺序 Gauss-Seidel）。
    ///
    /// **钉住粒子**：它与自由粒子的对**由自由粒子那一侧发起**（`inv_mass[j] == 0` 时不受
    /// `j > i` 限制）⇒ 自由粒子会被完整推开（`w = w_i` ⇒ `λ = depth/w_i`），钉住粒子不动。
    ///
    /// **格子过期**：建格用**建格那一刻**的位置，之后本子步内的推出不再重建 ⇒ 候选集略旧；
    /// 但"一格边长 = 碰撞直径、查 3×3×3 邻域"留了富余，且 8 个子步每步都重建一次。
    fn project_self_contacts(&mut self);

    /// **自碰撞收尾**（`project_self_contacts` 尾行 1:1 替换"写回 pairs"⇒ 那个函数的
    /// 行数棘轮不动）：写回点-点对数 + **点-边趟**（子开关默认关 ⇒ 整段跳过）。
    fn finish_self_contacts(&mut self, point_pairs: u32);
}

impl ClothSelfCollideExt for ClothSheet {
    fn project_self_contacts(&mut self) {
        let r = self.self_contacts.cfg.particle_radius;
        if !r.is_finite() || r <= 0.0 {
            return; // 半径非正/非有限 ⇒ 关（与 `ClothSheet::radius < 0` 同惯例；NaN 也走这条）
        }
        let d_c = 2.0 * r;
        let inv = 1.0 / d_c;
        // **自摩擦**（`0` = 关；默认 0 ⇒ 整块跳过 ⇒ T3 的读数逐位不变）
        let friction = self.self_contacts.cfg.friction;
        // ① 建格（**含钉住粒子**：它们要能被"别人推开"的那一侧看见）。
        self.self_contacts.cells.clear();
        for i in 0..self.pos.len() {
            let key = cell_of(self.pos[i], inv);
            self.self_contacts
                .cells
                .entry(key)
                .or_default()
                .push(i as u32);
        }
        // ② 逐自由粒子 × 27 邻格。
        let n = self.pos.len();
        let mut pairs = 0u32;
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                continue; // 钉住粒子不发起（它的对会由对面那只自由粒子发起）
            }
            let ci = cell_of(self.pos[i], inv);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let key = (ci.0 + dx, ci.1 + dy, ci.2 + dz);
                        let Some(bucket) = self.self_contacts.cells.get(&key) else {
                            continue;
                        };
                        for &jj in bucket {
                            let j = jj as usize;
                            if j == i || (j < i && self.inv_mass[j] > 0.0) {
                                continue; // 自由-自由对只由小号发起 ⇒ 每对恰好一次
                            }
                            if self.self_contacts.is_forbidden(i as u32, jj) {
                                continue; // 网格邻居（结构/剪切/弯曲）不解算
                            }
                            let d = self.pos[j] - self.pos[i];
                            let len = d.length();
                            if len >= d_c {
                                continue; // 不在碰撞距离内
                            }
                            if len < 1e-9 {
                                continue; // 同心退化：方向无定义（与 `shape_penetration` 同惯例）
                            }
                            let w_i = self.inv_mass[i];
                            let w_j = self.inv_mass[j];
                            let w = w_i + w_j;
                            if w <= 0.0 {
                                continue; // 两只都钉住（前者已排除 ⇒ 保底）
                            }
                            let nrm = d * (1.0 / len);
                            let lam = (d_c - len) / w;
                            self.pos[i] -= nrm * (w_i * lam);
                            self.pos[j] += nrm * (w_j * lam);
                            // **自摩擦**（切向库仑锥；`μ = 0` 默认 ⇒ 首行返回 ⇒ T3 读数逐位不变）。
                            // 口径、以及"为什么预算取 `depth` 而不是 `w_p·λ`"见
                            // `cloth_self_friction::resist_slip` 的注。
                            crate::cloth_self_friction::resist_slip(
                                &mut self.pos,
                                self.prev.as_slice(),
                                i,
                                j,
                                w_i,
                                w_j,
                                nrm,
                                d_c - len,
                                friction,
                            );
                            pairs += 1;
                        }
                    }
                }
            }
        }
        self.finish_self_contacts(pairs);
    }

    fn finish_self_contacts(&mut self, point_pairs: u32) {
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
