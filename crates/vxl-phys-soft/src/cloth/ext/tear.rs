//! **ClothTearExt**：`ClothSheet` 的**撕裂约束族**——本族**不往 `ClothSheet` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——36 个
//! 方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在同一片上，
//! 处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，判据是
//! 「新加一族约束**不必再往本类型添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `ClothSheet` 的方法数**不计入本类型**（门账 36 → 减本族 8 个）。
//!
//! **先例**（同一判据的既有落法）：`crates/vxl-phys/src/world_struct/ext/` 的 11 个
//! `World` 域侧扩展 trait（`World` 76 → 22）、`crates/vxl-phys/src/world_soft_ext.rs`
//! 的 `SoftExt`。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
//!
//! **可见性**：原先 `pub fn` 的判据仪器（`is_torn` / `bend_is_torn` / `torn_count` /
//! `bend_torn_count` / `edge` / `edge_strain`）在 trait 里写 `fn`（Rust **不允许** trait
//! 方法带 `pub`）⇒ 由 **trait 自身的 `pub`** 提供外部可见性 ⇒ 调用方 `use` 本 trait 后
//! 点号语法照旧可用（`crates/vxl-phys-soft/tests/cloth_tear.rs` 等已改引本 trait）。
use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

/// **ClothTearExt**：`ClothSheet` 的**撕裂约束族**（应变超阈 ⇒ 标记该约束不再参与投影）。
pub trait ClothTearExt {
    /// **撕裂检查**（每子步一次、投影之后）：应变超阈 ⇒ 标记（**同一阈值同时作用于
    /// 结构/剪切边与弯曲对**）。`eps` 非有限 ⇒ **显式短路**（与 `bend_compliance = ∞` 同族先例）。
    fn tear_check(&mut self);

    /// **唯一边（结构/剪切）的距离投影**（一遍；从 `project_constraints` 抽出 —— 那边是本地
    /// 最长函数，god 门"合法交换"要求最长函数**严格下降**）。**已撕裂的边跳过**。
    fn project_edges(&mut self, a_tilde: f32);

    /// 第 `k` 条唯一边是否**已撕裂**（`torn` 未初始化/短于 `cons` ⇒ `false`）。
    fn is_torn(&self, k: usize) -> bool;

    /// 第 `k` 条**弯曲对**是否已撕裂。
    fn bend_is_torn(&self, k: usize) -> bool;

    /// **已撕裂的边数**（判据/诊断用；`0` = 没撕或撕裂功能关）。
    fn torn_count(&self) -> usize;

    /// 已撕裂的**弯曲对**数。
    fn bend_torn_count(&self) -> usize;

    /// 第 `k` 条**唯一边**的两端与注册长度（判据仪器：撕裂后的读数要能逐边取）。
    fn edge(&self, k: usize) -> ([u32; 2], Vec3, Vec3, f32);

    /// 第 `k` 条唯一边的当前**应变** `|len − rest| / rest`（判据仪器）。
    fn edge_strain(&self, k: usize) -> f32;
}

impl ClothTearExt for ClothSheet {
    fn tear_check(&mut self) {
        if !self.damage.tear.eps.is_finite() {
            return;
        }
        let eps = self.damage.tear.eps;
        let Self {
            pos,
            cons,
            rest,
            bend,
            damage,
            ..
        } = self;
        let (bend, bend_rest) = bend.parts();
        mark_torn(cons, rest, &mut damage.tear.torn, pos, eps);
        mark_torn(bend, bend_rest, &mut damage.tear.torn_bend, pos, eps);
    }

    fn project_edges(&mut self, a_tilde: f32) {
        for (k, [i, j]) in self.cons.iter().enumerate() {
            if self.is_torn(k) {
                continue; // **已撕裂**：不再是约束（`torn` 未初始化/短于 `cons` ⇒ 视作未撕）
            }
            let (i, j) = (*i as usize, *j as usize);
            let w = self.inv_mass[i] + self.inv_mass[j];
            let d = self.pos[j] - self.pos[i];
            let len = d.length();
            if w <= 0.0 || len < 1e-9 {
                continue; // 双边都钉住 / 退化对（两条早退在**任何写之前** ⇒ 合并判据逐位无差）
            }
            let dir = d * (1.0 / len);
            let c = len - self.rest[k];
            let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
            self.lambda[k] += dl;
            self.pos[i] -= dir * (self.inv_mass[i] * dl);
            self.pos[j] += dir * (self.inv_mass[j] * dl);
        }
    }

    fn is_torn(&self, k: usize) -> bool {
        self.damage.tear.torn.get(k).copied().unwrap_or(false)
    }

    fn bend_is_torn(&self, k: usize) -> bool {
        self.damage.tear.torn_bend.get(k).copied().unwrap_or(false)
    }

    fn torn_count(&self) -> usize {
        self.damage.tear.torn.iter().filter(|t| **t).count()
    }

    fn bend_torn_count(&self) -> usize {
        self.damage.tear.torn_bend.iter().filter(|t| **t).count()
    }

    fn edge(&self, k: usize) -> ([u32; 2], Vec3, Vec3, f32) {
        let [a, b] = self.cons[k];
        (
            [a, b],
            self.pos[a as usize],
            self.pos[b as usize],
            self.rest[k],
        )
    }

    fn edge_strain(&self, k: usize) -> f32 {
        let (_, pa, pb, rest) = self.edge(k);
        if rest <= 0.0 {
            return 0.0;
        }
        ((pb - pa).length() - rest).abs() / rest
    }
}

/// **把"应变超阈"的对标成已撕裂**（`cons` 与 `bend` 共用同一条规则：两者都是距离约束）。
/// `rest ≤ 0` 退化（注册期已滤，这里兜底）⇒ 跳过（避免除零）。
pub(crate) fn mark_torn(
    pairs: &[[u32; 2]],
    rest: &[f32],
    torn: &mut Vec<bool>,
    pos: &[Vec3],
    eps: f32,
) {
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
