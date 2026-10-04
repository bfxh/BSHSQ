//! # vxl-phys-mech
//!
//! 机械（§1 vxl-phys-mech）：齿轮/皮带/活塞/马达约束组 —— ⚠️ **参数骨架**：只有 `MechJoint`/`JointParams`，**无消费方**（2026-10-05 更正：原写 "M2+ 落地"）。
//! 关节族本体在 `vxl-phys-solver`（`JointKind` 5 种）；**本 crate 与它尚未接线**（计划口径见 §2.5）。

#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MechJoint {
    /// 齿轮：传动比 + 相位。
    Gear,
    /// 皮带：等速 + 打滑阈值。
    Belt,
    /// 活塞：直线驱动 + 冲程限位。
    Piston,
    /// 马达：目标角速度 + 最大扭矩。
    Motor,
}

/// 可断裂/可限位/可阻尼的关节通用参数（§2.5 关节族）。
#[derive(Clone, Copy, Debug)]
pub struct JointParams {
    pub break_stress: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub damping: f32,
}

impl Default for JointParams {
    fn default() -> Self {
        Self {
            break_stress: f32::INFINITY,
            lower_limit: f32::NEG_INFINITY,
            upper_limit: f32::INFINITY,
            damping: 0.0,
        }
    }
}
