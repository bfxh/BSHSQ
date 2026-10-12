//! 选择性 CCD（§4.12 / §2.7）。
//!
//! M1 判据（三判据中的动体侧两条 + 目标侧一条）：
//! 1. 速度阈值：|v| ≥ `config.ccd_speed_threshold`；
//! 2. 尺寸比：单帧位移 ≥ 最小半长 × `config.ccd_extent_ratio`；
//! 3. 目标侧：**静态/沉睡 + 运动体**（运动目标按相对运动采样与相对速度判决，F-15 ③ 已修）。
//!
//! ⚠️ **默认档 = 关**（`PhysConfig::default().ccd_speed_threshold = INFINITY`）：§4.12 的应然是
//! "默认选择性"，但默认开被实测挡下 —— 扫掠钳位不看**关节约束**（`OPEN-PROBLEMS.md` P17：
//! R6 球链分离 0.00966 → 0.12382）；
//!
//! 方法：保守推进采样——把本 tick 位移分成 ≤ `ccd_max_steps` 段
//! （段长 ≤ 0.5×最小半长），沿段检测与目标的窄相接触，命中即钳位到
//! 上一安全采样点并清零法向速度（M1 非弹性停下；恢复系数路径随
//! TGS-Soft 重构接入）。确定性：段数由状态决定，采样按序。

#![forbid(unsafe_code)]

use vxl_phys_core::{BodySet, PhysConfig, Shape};

/// 形状最小半长（尺寸比判据的分母；高度场无意义 → INFINITY）。
pub fn min_half_extent(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half } => half.x.min(half.y).min(half.z),
        Shape::Sphere { radius } => radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => half_height.min(radius),
        Shape::Capsule {
            half_height,
            radius,
        } => half_height.min(radius),
        Shape::Cone {
            half_height,
            radius,
        } => half_height.min(radius),
        Shape::ConvexHull { half, .. } => half.x.min(half.y).min(half.z),
        Shape::Compound { half, .. } => half.x.min(half.y).min(half.z),
        Shape::TriMesh { half, .. } => half.x.min(half.y).min(half.z),
        Shape::HeightField(_) | Shape::Provider(_) => f32::INFINITY,
    }
}

/// 选择性 CCD 判据。
pub fn needs_ccd(bodies: &BodySet, i: usize, config: &PhysConfig, dt: f32) -> bool {
    if !bodies.is_dynamic_or_kinematic(i) || !bodies.awake[i] {
        return false;
    }
    if !config.ccd_speed_threshold.is_finite() {
        return false;
    }
    let v = bodies.linvel[i].length();
    if v < config.ccd_speed_threshold {
        return false;
    }
    let travel = v * dt;
    let ext = min_half_extent(&bodies.shape[i]);
    if !ext.is_finite() || ext <= 0.0 {
        return false;
    }
    travel >= ext * config.ccd_extent_ratio
}

/// 本 tick 扫描段数（≥1，确定性）。
pub fn step_count(bodies: &BodySet, i: usize, config: &PhysConfig, dt: f32) -> u32 {
    let v = bodies.linvel[i].length();
    let travel = v * dt;
    let ext = min_half_extent(&bodies.shape[i]).max(1e-4);
    let steps = (travel / (ext * 0.5)).ceil() as u32;
    steps.clamp(1, config.ccd_max_steps.max(1))
}
