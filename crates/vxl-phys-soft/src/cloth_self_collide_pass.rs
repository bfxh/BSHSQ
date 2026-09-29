//! **自碰撞的推进步**（`ClothSheet::project_self_contacts`）—— `cloth_self_collision` 的**子模块**
//! （`#[path]` 指向同目录本文件）。**为什么拆**：`cloth_self_collision.rs` 受 god 门**文件行数
//! 棘轮**，自摩擦那片让它 167 → 184 行、最长函数 66 → 82 ⇒ 按"按域拆文件"的先例把**这一个
//! 函数**整块搬走（子模块能看见父模块的私有 `cell_of`，故不必改它的可见性）。
//!
//! **顺序与确定性**：建格按粒子 `i` **升序**插入 ⇒ 桶内升序；解算只从自由粒子发起、取
//! `j > i`（钉住粒子例外，由对面那只自由粒子发起）⇒ **每对恰好解算一次**、顺序固定 ⇒ 逐位可复现。
use super::cell_of;
use crate::cloth::ClothSheet;

impl ClothSheet {
    /// **自碰撞投影**（每子步一次，在建格位置上做一遍顺序 Gauss-Seidel）。
    ///
    /// **钉住粒子**：它与自由粒子的对**由自由粒子那一侧发起**（`inv_mass[j] == 0` 时不受
    /// `j > i` 限制）⇒ 自由粒子会被完整推开（`w = w_i` ⇒ `λ = depth/w_i`），钉住粒子不动。
    ///
    /// **格子过期**：建格用**建格那一刻**的位置，之后本子步内的推出不再重建 ⇒ 候选集略旧；
    /// 但"一格边长 = 碰撞直径、查 3×3×3 邻域"留了富余，且 8 个子步每步都重建一次。
    pub(crate) fn project_self_contacts(&mut self) {
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
}
