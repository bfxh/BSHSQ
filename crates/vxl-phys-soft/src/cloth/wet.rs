//! **湿质量**（布吸水后有效质量上升）：`cloth.rs` 的子模块，`ROUTE.md` §4「湿布（质量+阻力、双向）」
//! 里"质量"那半 —— 与 [`crate::cloth_medium`]（阻力 + 双向）同属一格。
//!
//! **口径**：逐三角取介质样本的 `occupied`（= ρ/ρ0 截到 `[0,1]`，自由表面判据），按面积无关的
//! 三角均分摊到三个顶点 ⇒ **顶点湿率** `wet_i ∈ [0,1]`；有效质量 `m_eff = m·(1 + κ·wet)`，
//! 只改 `inv_mass`（`1/m_eff`），**不动 `mass`**。
//!
//! **为什么改 `inv_mass` 而不是 `mass`**：XPBD 的约束投影与接触都用 `inv_mass` 加权，改它才真的
//! 让"吸水的那半边动得更慢"；而 `mass` 是**干质量**（薄壳均分的注册态），保留它才能每次从干质量
//! 重算 ⇒ **离开水（`wet = 0`）逐位还原 `1/m`**，不是单向漂移。
//!
//! **默认档逐位不变**：`medium` 空（无流体/未采样）⇒ 首行短路；采样全为真空（`occupied = 0`）⇒
//! 倍率精确为 `1.0` ⇒ `1.0/(m·1.0)` 与 `inv_mass_of` 逐位相同。
//!
//! **钉住/退化保持**：`inv_mass == 0`（`set_pinned` / `m ≤ 0`）⇒ 一字不动。
use crate::cloth::ClothSheet;

/// 吸水系数 κ：全湿时有效质量 = 干质量 × `(1 + κ)`。取 0.5 = "吸掉自身一半质量的水"，
/// 是**量级选择**（不是实测标定）——真标定要等带质量守恒的液面耦合片，届时改这一个常数即可。
const ABSORB: f32 = 0.5;

/// 重算逐顶点湿率与 `inv_mass`（`cloth.mass_scale` 兼作**湿率累加器** —— 零额外分配）。
pub(crate) fn apply(cloth: &mut ClothSheet) {
    if !cloth.medium.wet_mass
        || cloth.medium.samples.len() != cloth.tris.len()
        || cloth.medium.mass_scale.len() != cloth.pos.len()
    {
        return;
    }
    for s in cloth.medium.mass_scale.iter_mut() {
        *s = 0.0; // 兼作累加器：先清零，再逐面摊 `occupied/3`
    }
    for (k, tri) in cloth.tris.iter().enumerate() {
        let w = cloth.medium.samples[k].occupied * (1.0 / 3.0);
        for &v in tri {
            cloth.medium.mass_scale[v as usize] += w;
        }
    }
    for i in 0..cloth.inv_mass.len() {
        // 钉住/退化（`inv_mass == 0`）不参与；其余从**干质量**重算 ⇒ 干回去逐位还原。
        cloth.medium.mass_scale[i] = if cloth.inv_mass[i] == 0.0 {
            1.0
        } else {
            1.0 + ABSORB * cloth.medium.mass_scale[i]
        };
        if cloth.inv_mass[i] != 0.0 {
            cloth.inv_mass[i] = 1.0 / (cloth.mass[i] * cloth.medium.mass_scale[i]);
        }
    }
}
