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
//!
//! **本文件现在只留状态**：塑性域的**方法**（`plastic_flow` + 三个判据仪器）已搬进扩展
//! trait [`ClothPlasticExt`](crate::cloth::ClothPlasticExt)（`cloth/ext/plastic.rs`）——
//! god 门处置方案「约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引」的执行面
//! （见 `god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]`）。

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
    /// **弯曲对（二环对）是否也流动**（`false` = 只做结构/剪切 —— **默认**）。
    ///
    /// **为什么单独一个开关**：二环对是弯曲的**距离代理**（折痕的记忆）。它对"折痕"本身
    /// （对称折叠）几乎不敏感，但在拉伸/压缩下会跟着流动 ⇒ 关掉它时弯曲 `rest` 会**拉住**
    /// 结构侧的永久变形（实测：关档 Σ 边长比兑现值大 5%+）。开 = 折痕代理也记住永久变形。
    pub bend: bool,
    /// 与 `cons` 同序的**累计塑性应变**（诊断/判据读；懒初始化 ⇒ 默认构造即可用）。
    pub(crate) strain: Vec<f32>,
}

impl Default for Plastic {
    /// **默认 = 关**（`∞`）+ 一个"打开时有意义"的率（`0.5`）+ 弯曲子开关关。
    fn default() -> Self {
        Self {
            yield_strain: f32::INFINITY,
            rate: 0.5,
            bend: false,
            strain: Vec::new(),
        }
    }
}
