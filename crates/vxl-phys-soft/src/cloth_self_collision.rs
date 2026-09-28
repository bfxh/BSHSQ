//! **布片自碰撞**（切片 T3）——`ClothSheet` 的自接触段（从 `cloth.rs` 拆出，理由同 `cloth_coupling.rs`：
//! `cloth.rs` 受 god 门**文件行数棘轮**）。
//!
//! **口径**（最小档，与提供者/刚体接触同族）：**位置口径 + α=0 + 两体按逆质量分担** ——
//! 两个粒子靠近到 `2·particle_radius`（= 壳厚）以内就沿心线方向推开，各让 `w·λ`。
//! 没有切向摩擦、没有速度项（`v = (x − x_prev)/h` 反推 ⇒ 位置修正自动进速度，与全仓同口径）。
//!
//! **三条边界（写清，不是漏）**：
//! 1. **点-点对**（点-边/点-三角属进阶档）⇒ 极薄/高速的相对滑动可能**穿过**格缝，本档不承诺 CCS；
//! 2. **无自摩擦** ⇒ 两层之间可以自由滑（"折起来不粘"）；
//! 3. **网格邻居不解算**（[`ClothSheet::self_contacts`] 的 `forbidden`）：折痕处二环对天然趋近 0，
//!    解它就会与弯曲弹簧对顶。
//!
//! **关闭时（默认）**：`cfg.enabled = false` ⇒ 调用点首行短路，本文件不产生任何效果
//! ⇒ 既有场景**逐位不变**（三条冻结哈希与 `gold` 门不动）。
use crate::cloth::ClothSheet;
use crate::params::SelfCollision;
use std::collections::BTreeMap;
use vxl_phys_core::Vec3;

/// **自碰撞状态**（切片 T3：空间哈希 + 粒子-粒子位置投影；**默认关**）。
///
/// **为什么收成一个结构**：`ClothSheet` 受 god 门**成员棘轮**（22 成员顶格，阈值 24）——
/// 参数 / 哈希 / 禁止对一起进来就顶格 ⇒ 按先例（`rope::VirtRot` / `BodyCoupling`）合并成一个
/// （`ClothSheet` 成员 22 → 23）。**定义在本文件**（与算法同域）：`cloth.rs` 同时受
/// **文件行数棘轮**，搬走后它才回到交换窗内。
///
/// **默认关**（`cfg.enabled = false`）⇒ `project_self_contacts` 首行短路 ⇒ 既有场景**逐位不变**
/// （三条冻结哈希与 `gold` 门不动；本片是**开关**，见 `PLAN-triangle-first-class.md` 风险 B）。
#[derive(Clone, Default)]
pub struct SelfContacts {
    /// 参数（骨架 [`crate::params::SelfCollision`]）：`particle_radius` = **粒子的碰撞球半径**，
    /// 与 `ClothSheet::radius` 同口径 ⇒ 两层心线的最小间距 = `2·particle_radius` = **壳厚**
    /// （`new` 里默认取 `t·0.5`；骨架自带的 0.05 是给"独立粒子"的，布片按壳厚更自然）。
    pub cfg: SelfCollision,
    /// **不可解算的对**（结构/剪切 + 弯曲邻居；`[min,max]` 升序 ⇒ 二分查找）：网格自己的弹簧
    /// 在**折痕**处天然会把二环对拉到很近（平铺网格折 180° 时二环对的距离 → 0）⇒ 若不禁，
    /// 自碰撞会与弯曲弹簧**对顶**。判据里的"最小间距"也必须用同一份排除集（同口径）。
    forbidden: Vec<[u32; 2]>,
    /// **空间哈希**（格子边长 = 碰撞直径 `2r` ⇒ 3×3×3 邻格覆盖全部候选）。
    /// 用 `BTreeMap` 而不是 `HashMap`：**迭代序确定**（本仓的确定性纪律 —— 本片只做查找，
    /// 但别给未来的"遍历格子"留坑）。桶内序 = 插入序 = **粒子升序**（建格时按 `i` 升序）。
    cells: BTreeMap<(i32, i32, i32), Vec<u32>>,
    /// 最近一个子步实际解算的对数（诊断/判据读；不参与动力学）。
    pub pairs: u32,
}

impl SelfContacts {
    /// 默认**关**；`particle_radius` 由 `ClothSheet::new` 按壳厚给（见字段注）。
    pub(crate) fn new(particle_radius: f32) -> Self {
        Self {
            cfg: SelfCollision {
                enabled: false,
                particle_radius,
            },
            forbidden: Vec::new(),
            cells: BTreeMap::new(),
            pairs: 0,
        }
    }

    /// 登记"不许自碰撞解算"的对（结构/剪切 + 弯曲；排序去重 ⇒ 二分）。
    pub(crate) fn set_forbidden(&mut self, cons: &[[u32; 2]], bend: &[[u32; 2]]) {
        let mut f: Vec<[u32; 2]> = cons.iter().chain(bend.iter()).copied().collect();
        f.sort_unstable();
        f.dedup();
        self.forbidden = f;
    }

    /// 该对是否**不可解算**（网格邻居；判据也用同一份集合）。
    pub fn is_forbidden(&self, i: u32, j: u32) -> bool {
        let key = if i < j { [i, j] } else { [j, i] };
        self.forbidden.binary_search(&key).is_ok()
    }

    /// 禁止对的条数（诊断/判据读）。
    pub fn forbidden_count(&self) -> usize {
        self.forbidden.len()
    }
}

/// **位置 → 格子**（边长 = 碰撞直径的倒数 `inv`）。`floor` + `as i32`（饱和转换）⇒ 确定性；
/// 极端坐标会被饱和到 `i32` 边界（同一格里 ⇒ 只影响性能不影响判据）。
fn cell_of(p: Vec3, inv: f32) -> (i32, i32, i32) {
    (
        (p.x * inv).floor() as i32,
        (p.y * inv).floor() as i32,
        (p.z * inv).floor() as i32,
    )
}

impl ClothSheet {
    /// **自碰撞投影**（每子步一次，在建格位置上做一遍顺序 Gauss-Seidel）。
    ///
    /// **顺序与确定性**：建格按粒子 `i` **升序**插入 ⇒ 桶内升序；解算只从自由粒子发起、取
    /// `j > i`（钉住粒子例外，见下）⇒ **每对恰好解算一次**，且顺序是 `(i 升序, 桶内序)`
    /// 的固定序 ⇒ 同一份输入逐位可复现。
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
                            pairs += 1;
                        }
                    }
                }
            }
        }
        self.self_contacts.pairs = pairs;
    }
}
