//! **NarrowTierExt**：`World` 的**窄相卡上档面**——本域**不往 `World` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys::World"]` 把 `World` 登记为
//! **已登记债务（非豁免）**，处置写的是「**域侧 API 拆成扩展 trait**（`impl FluidExt for World`
//! 等），门面只留管线组装」，判据是「新加一域**不必再往 `World` 添方法**」。
//!
//! **先例**（同一判据的既有落法）：`world_step/destruction.rs` 的 `DestructionExt`、
//! `world_step/conversion.rs` 的 `VoxelConversionExt`、`vxl-phys-sense` 的 `SenseExt`、
//! `world_aero_wake.rs` 的 `AeroWakeExt`、`world_kinematic.rs` 的 `KinematicExt`。
//!
//! **门的口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! （`god_gate.py:216`）⇒ 搬进 trait 后 `World` 的方法数**不计入本类型**。
//!
//! **数值与顺序一字不改**（纯搬移：方法体逐字搬过来，未改一个算式、一个遍历序）。
use crate::World;

/// **NarrowTierExt**：`World` 的窄相卡上档面。
pub trait NarrowTierExt {
    fn set_narrow_tier(&mut self, tier: Box<dyn vxl_phys_core::narrow_tier::NarrowTierBackend>);
    fn has_narrow_tier(&self) -> bool;
    fn narrow_resident_active(&self) -> bool;
    fn pairs(&self) -> &[(u32, u32)];
    fn narrow_tier_stats(&self) -> crate::world_step::narrow_tier::NarrowTierStats;
}

impl NarrowTierExt for World {
    /// **注册卡上窄相档**：此后每个"检测趟"都先试卡上路径（`NarrowTierBackend`，如 GPU 档）；
    /// 未注册 / 后端报错 / 超容量 / `predict_dt > 0` 时**整趟回退 CPU**（不半途混用）。
    ///
    /// **逐位不变性**：未注册时一行都不走新路径 ⇒ 默认档（含金样门与三哈希）逐位不变。
    fn set_narrow_tier(&mut self, tier: Box<dyn vxl_phys_core::narrow_tier::NarrowTierBackend>) {
        self.narrow.tier = Some(tier);
    }
    /// 是否已注册卡上窄相档（诊断/探针用）。
    fn has_narrow_tier(&self) -> bool {
        self.narrow.tier.is_some()
    }
    /// 上一趟**是否真的走了常驻跑法**（诊断/探针用）。
    ///
    /// 为什么要一个专门的观察口：`716ed63` 的首版把进入条件写成 `resident && …` ⇒ 被 `&&`
    /// **短路**、常驻一次都没生效，而"耗时只比整表档低一点"这种**间接**信号没能把它顶出来。
    /// ⇒ 这类"优化到底在不在跑"必须能**直接读**，不能只靠数字反推。
    fn narrow_resident_active(&self) -> bool {
        self.narrow.resident
    }
    /// 本 tick 的对表（诊断用：卡上档那一趟的输入就是这个列表）。
    fn pairs(&self) -> &[(u32, u32)] {
        &self.pairs
    }
    /// 卡上档的**分项计时快照**（诊断/探针用；见 `NarrowTierStats`）。返回 `(累计, 趟数)`。
    fn narrow_tier_stats(&self) -> crate::world_step::narrow_tier::NarrowTierStats {
        self.narrow.stats
    }
}
