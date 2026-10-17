//! builders：`Joint` 的构造与链式设置（从 types.rs 按域拆出；语义未改）。
//!
//! 拆出去的直接原因：god 门的 file_lines 放行要求「最长函数也一起降」，而给 `JointSet` 加
//! 断裂状态字段属于纯加法 —— 把这块搬走之后 types.rs 反而更小。
use super::types::{Joint, JointKind};
use vxl_phys_core::Vec3;

impl Joint {
    pub fn new(kind: JointKind, a: u32, b: u32, anchor_a: Vec3, anchor_b: Vec3) -> Self {
        Self {
            kind,
            a,
            b,
            anchor_a,
            anchor_b,
            axis_a: Vec3::X,
            axis_b: Vec3::X,
            rest: 0.0,
            motor_target: 0.0,
            motor_max_force: 0.0,
            limit_lower: 1.0,
            limit_upper: -1.0,
        }
    }

    /// 加**转动限位**（rad，绕 `axis_a`；`lower >= upper` 视为未设）。
    pub fn with_limits(mut self, lower: f32, upper: f32) -> Self {
        self.limit_lower = lower;
        self.limit_upper = upper;
        self
    }

    /// 加**马达**（转动：目标角速度 rad/s；棱柱：目标线速度 m/s；`max_force <= 0` = 关）。
    /// 限位另由 [`Self::with_limits`] 设，**已实现**（2026-09-27 更正原"仍未实现"，见模块文档）。
    pub fn with_motor(mut self, target_velocity: f32, max_force: f32) -> Self {
        self.motor_target = target_velocity;
        self.motor_max_force = max_force;
        self
    }

    pub fn with_axis(mut self, axis: Vec3) -> Self {
        self.axis_a = axis;
        self.axis_b = axis;
        self
    }

    pub fn with_rest(mut self, rest: f32) -> Self {
        self.rest = rest;
        self
    }
}
