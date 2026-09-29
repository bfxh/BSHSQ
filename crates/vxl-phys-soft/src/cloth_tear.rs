//! **布料撕裂**（`SPEC.md` §4.6 / 骨架 [`crate::params::TearStrain`] 的落点）——
//! `ClothSheet` 的撕裂段（从 `cloth.rs` 拆出：那里受 god 门**文件行数棘轮**，本域一进来就超窗；
//! 拆法与 `cloth_coupling.rs` / `cloth_self_collision.rs` / `cloth_aero.rs` 同款）。
//!
//! **口径**：`eps` = **应变阈值**（`|len − rest| / rest`；骨架档位 `E03/E05/E10/None` =
//! 0.3 / 0.5 / 1.0 / **`∞`**）。超阈的**距离约束**（唯一边**与**弯曲对 —— 两者同一条规则）
//! 被标记为**已撕裂** ⇒ 该约束不再参与投影。
//!
//! **"标记"而不是"移除"**：索引稳定 ⇒ ① 确定性（不重排数组）② `rest`/`lambda` 等并行数组不必
//! 重分配 ③ 判据里的 warm id 不漂。
//!
//! **默认关闭**：`eps = ∞`（[`Tearing::default`]）⇒ `tear_check` 首行短路、跳过分支永不命中
//! ⇒ 既有场景**逐位不变**（判据实测：默认档与显式 `∞` 逐位相同）。
use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

/// **撕裂状态**。
///
/// **关闭语义**（与 `bend_compliance = ∞` 同族先例）：`eps = f32::INFINITY`（**默认**）⇒
/// `tear_check` 首行短路 ⇒ 默认档零成本、逐位不变。
#[derive(Clone)]
pub struct Tearing {
    /// 撕裂应变阈值（`|len − rest| / rest`；**非有限 = 关**）。
    pub eps: f32,
    /// 与 `cons` 同序的**已撕裂**标记（长度不匹配时按"全未撕"处理 ⇒ 默认构造即可用）。
    pub(crate) torn: Vec<bool>,
    /// 与 `bend` 同序的**已撕裂**标记。
    ///
    /// **为什么弯曲对也要撕**（2026-09-29 实测教训）：只撕 `cons` 时，撕开处仍被**二环（弯曲）
    /// 距离约束**连着 ⇒ 布片"撕了但不松"（判据实测：单角钉住撕开 10 条边后只移动 1.53 m，
    /// 而完全自由的布片该飞走 40 m）。物理上撕口两侧的弯曲刚度也该消失 ⇒ **同一阈值也作用于
    /// `bend`**（两者都是距离约束 ⇒ 同一条"应变超阈即断"的规则）。
    pub(crate) torn_bend: Vec<bool>,
}

impl Default for Tearing {
    /// ⚠️ **默认必须是 `∞`（关）** —— 若用 `#[derive(Default)]` 会得到 `0.0`，
    /// 那等于"一有应变就撕"，会把所有既有场景撕碎。
    fn default() -> Self {
        Self {
            eps: f32::INFINITY,
            torn: Vec::new(),
            torn_bend: Vec::new(),
        }
    }
}

/// **把"应变超阈"的对标成已撕裂**（`cons` 与 `bend` 共用同一条规则：两者都是距离约束）。
/// `rest ≤ 0` 退化（注册期已滤，这里兜底）⇒ 跳过（避免除零）。
fn mark_torn(pairs: &[[u32; 2]], rest: &[f32], torn: &mut Vec<bool>, pos: &[Vec3], eps: f32) {
    if torn.len() != pairs.len() {
        *torn = vec![false; pairs.len()];
    }
    for (k, [a, b]) in pairs.iter().enumerate() {
        if torn[k] || rest[k] <= 0.0 {
            continue;
        }
        let len = (pos[*b as usize] - pos[*a as usize]).length();
        if (len - rest[k]).abs() / rest[k] > eps {
            torn[k] = true;
        }
    }
}

impl ClothSheet {
    /// **撕裂检查**（每子步一次、投影之后）：应变超阈 ⇒ 标记（**同一阈值同时作用于
    /// 结构/剪切边与弯曲对**）。`eps` 非有限 ⇒ **显式短路**（与 `bend_compliance = ∞` 同族先例）。
    pub(crate) fn tear_check(&mut self) {
        if !self.damage.tear.eps.is_finite() {
            return;
        }
        let eps = self.damage.tear.eps;
        let Self {
            pos,
            cons,
            rest,
            bend,
            bend_rest,
            damage,
            ..
        } = self;
        mark_torn(cons, rest, &mut damage.tear.torn, pos, eps);
        mark_torn(bend, bend_rest, &mut damage.tear.torn_bend, pos, eps);
    }

    /// **唯一边（结构/剪切）的距离投影**（一遍；从 `project_constraints` 抽出 —— 那边是本地
    /// 最长函数，god 门"合法交换"要求最长函数**严格下降**）。**已撕裂的边跳过**。
    pub(crate) fn project_edges(&mut self, a_tilde: f32) {
        for (k, [i, j]) in self.cons.iter().enumerate() {
            if self.is_torn(k) {
                continue; // **已撕裂**：不再是约束（`torn` 未初始化/短于 `cons` ⇒ 视作未撕）
            }
            let (i, j) = (*i as usize, *j as usize);
            let w = self.inv_mass[i] + self.inv_mass[j];
            if w <= 0.0 {
                continue;
            }
            let d = self.pos[j] - self.pos[i];
            let len = d.length();
            if len < 1e-9 {
                continue;
            }
            let dir = d * (1.0 / len);
            let c = len - self.rest[k];
            let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
            self.lambda[k] += dl;
            self.pos[i] -= dir * (self.inv_mass[i] * dl);
            self.pos[j] += dir * (self.inv_mass[j] * dl);
        }
    }

    /// 第 `k` 条唯一边是否**已撕裂**（`torn` 未初始化/短于 `cons` ⇒ `false`）。
    pub fn is_torn(&self, k: usize) -> bool {
        self.damage.tear.torn.get(k).copied().unwrap_or(false)
    }

    /// 第 `k` 条**弯曲对**是否已撕裂。
    pub fn bend_is_torn(&self, k: usize) -> bool {
        self.damage.tear.torn_bend.get(k).copied().unwrap_or(false)
    }

    /// **已撕裂的边数**（判据/诊断用；`0` = 没撕或撕裂功能关）。
    pub fn torn_count(&self) -> usize {
        self.damage.tear.torn.iter().filter(|t| **t).count()
    }

    /// 已撕裂的**弯曲对**数。
    pub fn bend_torn_count(&self) -> usize {
        self.damage.tear.torn_bend.iter().filter(|t| **t).count()
    }

    /// 第 `k` 条**唯一边**的两端与注册长度（判据仪器：撕裂后的读数要能逐边取）。
    pub fn edge(&self, k: usize) -> ([u32; 2], Vec3, Vec3, f32) {
        let [a, b] = self.cons[k];
        (
            [a, b],
            self.pos[a as usize],
            self.pos[b as usize],
            self.rest[k],
        )
    }

    /// 第 `k` 条唯一边的当前**应变** `|len − rest| / rest`（判据仪器）。
    pub fn edge_strain(&self, k: usize) -> f32 {
        let (_, pa, pb, rest) = self.edge(k);
        if rest <= 0.0 {
            return 0.0;
        }
        ((pb - pa).length() - rest).abs() / rest
    }
}
