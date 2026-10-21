//! **损伤域汇总**（撕裂 + 塑性）：`ClothSheet.damage` 的**并组**结构 —— `ClothSheet` 受 god 门
//! **成员棘轮**（`max_type_members` 24；本域加状态前已 23/24 顶格）⇒ 按先例（`BodyCoupling` /
//! `SelfContacts` / `ContactTuning`）把两子域并成一个字段，**成员净零增长**。
//!
//! **次序是物理决定**（判据④钉它）：每子步投影之后 **先塑性流动**（超阈应变并入 `rest`
//! ⇒ 弹性应变下降）**再撕裂检查**（判的正是弹性应变）⇒ 塑性**延迟**撕裂。
//!
//! **本文件现在只留状态**：损伤域的**汇总步**（`damage_step`）已搬进扩展 trait
//! `ClothDamageExt`（`cloth/ext/damage.rs`）—— god 门处置方案
//! 「约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引」的执行面
//! （见 `god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]`）。
use crate::cloth::plastic::Plastic;
use crate::cloth_tear::Tearing;

/// **损伤域**（两子域各自默认关 ⇒ 默认档逐位不变）。
#[derive(Clone, Default)]
pub struct Damage {
    /// 撕裂（`eps = ∞` = 关 ⇒ `tear_check` 首行短路）。
    pub tear: Tearing,
    /// 塑性（`yield_strain = ∞` = 关 ⇒ `plastic_flow` 首行短路）。
    pub plastic: Plastic,
}
