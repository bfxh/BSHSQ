//! **ClothDamageExt**：`ClothSheet` 的**损伤域汇总步**——本族**不往 `ClothSheet` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]` 写明——36 个
//! 方法把 XPBD 各约束族（边/弯/塑/撕裂/自摩擦/自碰撞/气动/刚体接触）的求解全攥在同一片上，
//! 处置是「**约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引**」，判据是
//! 「新加一族约束**不必再往本类型添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `ClothSheet` 的方法数**不计入本类型**。
//!
//! **次序是物理决定**（判据④钉它）：每子步投影之后 **先塑性流动**（超阈应变并入 `rest`
//! ⇒ 弹性应变下降）**再撕裂检查**（判的正是弹性应变）⇒ 塑性**延迟**撕裂。
//!
//! **本族只留汇总步**：子步骤（`plastic_flow` / `tear_check`）分别在
//! [`ClothPlasticExt`] 与 [`ClothTearExt`] 里 ⇒ 汇总步是这两族的**编排**，
//! 放在损伤域的扩展点上最贴切。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
use crate::cloth::ext::{ClothPlasticExt, ClothTearExt};
use crate::cloth::ClothSheet;

/// **ClothDamageExt**：`ClothSheet` 的**损伤域汇总步**（塑性 → 撕裂 的次序编排）。
pub(crate) trait ClothDamageExt {
    /// **损伤步**（每子步一次、投影之后）：塑性流动**先**、撕裂检查**后** —— 见文件头注。
    fn damage_step(&mut self);
}

impl ClothDamageExt for ClothSheet {
    fn damage_step(&mut self) {
        self.plastic_flow();
        self.tear_check();
    }
}
