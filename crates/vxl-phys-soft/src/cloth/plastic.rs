//! **布片塑性**（`docs/PLAN-plasticity.md` 的落点）：XPBD 距离约束（结构/剪切）的 `rest`
//! 长度**永久**生长 —— 应变 `|len − rest| / rest` 超过**屈服阈**的部分，按**塑性率** `rate`
//! 每子步并入 `rest`（`rest ← rest·(1 + sign(ε)·rate·(|ε| − yield))`）⇒ 卸载后留**永久变形**。
//!
//! **关闭语义**（与 `Tearing::eps` 同一口径）：`yield_strain = ∞`（**默认**）⇒ `plastic_flow`
//! 首行短路 ⇒ 默认档**逐位不变**（零换代）。规格书**没有规定塑性** ⇒ 系数是本片选的默认
//! （同自摩擦 μ 的处境）。
//!
//! **边界（写清，不是漏）**：只做 `cons`（结构/剪切；弯曲对的塑性 = 折痕"记忆"属后续片）；
//! 无硬化/软化、无各向异性、无"塑性损伤累积进撕裂阈"（见 PLAN §五）。
use crate::cloth::ClothSheet;

/// **塑性状态**（默认关：`yield_strain = ∞`）。
///
/// ⚠️ **`Default` 手写不可 derive**（derive 给 `0.0` = "全塑"，会把既有场景一律拉长 ——
/// `Tearing` 同款教训）。
#[derive(Clone)]
pub struct Plastic {
    /// 屈服应变阈（`|len − rest| / rest`；**非有限 = 关**）。
    pub yield_strain: f32,
    /// 塑性率（超阈部分的多少比例每子步并入 `rest`；`≤ 0` = 关流动）。
    pub rate: f32,
    /// 与 `cons` 同序的**累计塑性应变**（诊断/判据读；懒初始化 ⇒ 默认构造即可用）。
    pub(crate) strain: Vec<f32>,
}

impl Default for Plastic {
    /// **默认 = 关**（`∞`）+ 一个"打开时有意义"的率（`0.5`）。
    fn default() -> Self {
        Self {
            yield_strain: f32::INFINITY,
            rate: 0.5,
            strain: Vec::new(),
        }
    }
}

impl ClothSheet {
    /// **塑性流动**（每子步一次、投影之后、撕裂检查之前 —— 见 `cloth::damage::damage_step`）：
    /// 超阈应变按 `rate` 并入 `rest`。`yield_strain` 非有限 / `rate ≤ 0` ⇒ **显式短路**
    /// （与 `tear_check`、`bend_compliance = ∞` 同族先例）。
    pub(crate) fn plastic_flow(&mut self) {
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
            let rest = self.rest[k];
            if rest <= 0.0 {
                continue; // 退化边（注册期已滤，这里兜底 ⇒ 避免除零）
            }
            let [i, j] = self.cons[k];
            let len = (self.pos[j as usize] - self.pos[i as usize]).length();
            let strain = (len - rest) / rest;
            let excess = strain.abs() - gy;
            if excess.is_nan() || excess <= 0.0 {
                continue; // 亚阈（NaN 也走这条）
            }
            let step = rate * excess;
            self.rest[k] = rest * (1.0 + strain.signum() * step);
            self.damage.plastic.strain[k] += step;
        }
    }

    /// 第 `k` 条结构/剪切边的**当前注册长度** `rest`（判据仪器：塑性的永久变化就写在这里）。
    pub fn rest_of(&self, k: usize) -> f32 {
        self.rest[k]
    }

    /// 第 `k` 条边的**累计塑性应变**（诊断/判据读；未流动过 ⇒ `0`）。
    pub fn plastic_of(&self, k: usize) -> f32 {
        self.damage.plastic.strain.get(k).copied().unwrap_or(0.0)
    }
}
