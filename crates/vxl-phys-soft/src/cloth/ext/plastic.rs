//! **ClothPlasticExt**：`ClothSheet` 的**塑性约束族**——本族**不往 `ClothSheet` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——36 个
//! 方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在同一片上，
//! 处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，判据是
//! 「新加一族约束**不必再往本类型添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `ClothSheet` 的方法数**不计入本类型**（门账 36 → 减本族 4 个）。
//!
//! **先例**：同判据的既有落法见 `crates/vxl-phys/src/world_struct/ext/`（`World` 76 → 22）。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
//!
//! **可见性**：原先 `pub fn` 的判据仪器（`rest_of` / `bend_rest_of` / `plastic_of`）在 trait
//! 里写 `fn`（Rust **不允许** trait 方法带 `pub`）⇒ 由 **trait 自身的 `pub`** 提供外部可见性
//! ⇒ 调用方 `use` 本 trait 后点号语法照旧可用。
use crate::cloth::ext::ClothTearExt;
use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

/// **ClothPlasticExt**：`ClothSheet` 的**塑性约束族**（`rest` 长度永久生长 ⇒ 卸载后留永久变形）。
pub trait ClothPlasticExt {
    /// **塑性流动**（每子步一次、投影之后、撕裂检查之前 —— 见 `cloth::damage::damage_step`）：
    /// 超阈应变按 `rate` 并入 `rest`（结构/剪切；弯曲对由子开关 `bend` 控制）。
    /// `yield_strain` 非有限 / `rate ≤ 0` ⇒ **显式短路**（与 `tear_check`、
    /// `bend_compliance = ∞` 同族先例）。
    fn plastic_flow(&mut self);

    /// 第 `k` 条结构/剪切边的**当前注册长度** `rest`（判据仪器：塑性的永久变化就写在这里）。
    fn rest_of(&self, k: usize) -> f32;

    /// 第 `k` 条**弯曲对（二环对）**的当前注册长度（判据仪器）。
    fn bend_rest_of(&self, k: usize) -> f32;

    /// 第 `k` 条边的**累计塑性应变**（诊断/判据读；未流动过 ⇒ `0`）。
    fn plastic_of(&self, k: usize) -> f32;
}

impl ClothPlasticExt for ClothSheet {
    fn plastic_flow(&mut self) {
        let gy = self.damage.plastic.yield_strain;
        if !gy.is_finite() || gy < 0.0 {
            return; // 默认关（∞）；负阈无物理意义 ⇒ 同短路
        }
        let rate = self.damage.plastic.rate;
        if rate.is_nan() || rate <= 0.0 {
            return; // `rate ≤ 0` / NaN ⇒ 关（不流动）
        }
        let n = self.cons.len();
        if self.damage.plastic.strain.len() != n {
            self.damage.plastic.strain = vec![0.0; n];
        }
        for k in 0..n {
            if self.is_torn(k) {
                continue; // 已撕裂：约束不再解算，`rest` 无意义
            }
            let [i, j] = self.cons[k];
            let step = flow_pair(
                &self.pos,
                &mut self.rest[k],
                i as usize,
                j as usize,
                gy,
                rate,
            );
            self.damage.plastic.strain[k] += step;
        }
        if self.damage.plastic.bend {
            for k in 0..self.bend.pairs.len() {
                if self.damage.tear.torn_bend.get(k).copied().unwrap_or(false) {
                    continue; // 已撕裂的弯曲对：与 `cons` 同口径
                }
                let [i, j] = self.bend.pairs[k];
                flow_pair(
                    &self.pos,
                    &mut self.bend.rest[k],
                    i as usize,
                    j as usize,
                    gy,
                    rate,
                );
            }
        }
    }

    fn rest_of(&self, k: usize) -> f32 {
        self.rest[k]
    }

    fn bend_rest_of(&self, k: usize) -> f32 {
        self.bend.rest[k]
    }

    fn plastic_of(&self, k: usize) -> f32 {
        self.damage.plastic.strain.get(k).copied().unwrap_or(0.0)
    }
}

/// **一对距离约束的塑性流动**：`rest` ← 超阈部分（`|ε| − yield`）按 `rate` 并入；
/// 返回**本遍的塑性增量**（`0` = 没流动）。`cons`（结构/剪切）与 `bend`（二环对）共用同一式
/// —— 两者都是距离约束（与撕裂的"同一阈值同时作用于两者"同口径）。
fn flow_pair(pos: &[Vec3], rest: &mut f32, i: usize, j: usize, gy: f32, rate: f32) -> f32 {
    let r = *rest;
    if r <= 0.0 {
        return 0.0; // 退化（注册期已滤，这里兜底 ⇒ 避免除零）
    }
    let len = (pos[j] - pos[i]).length();
    let strain = (len - r) / r;
    let excess = strain.abs() - gy;
    if excess.is_nan() || excess <= 0.0 {
        return 0.0; // 亚阈（NaN 也走这条）
    }
    let step = rate * excess;
    *rest = r * (1.0 + strain.signum() * step);
    step
}
