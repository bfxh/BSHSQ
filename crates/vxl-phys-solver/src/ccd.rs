//! 选择性 CCD（§4.12 / §2.7）。
//!
//! **默认档 = 选择性**（2026-10-13 起）：`PhysConfig::default().ccd_speed_threshold = 10 m/s`
//! （与 §4.3 的 skin 0.02 成对）；显式 `INFINITY` = 关档（无穿透保证）。
//!
//! 判据（动体侧两条 + 目标侧一条）：
//! 1. 速度阈值：**最大点速** `|v| + |ω|·r_bound` ≥ `config.ccd_speed_threshold`；
//! 2. 尺寸比：单帧位移（同上口径 × dt）≥ 最小半长 × `config.ccd_extent_ratio`；
//! 3. 目标侧：**静态/沉睡 + 运动体**（运动目标按相对运动采样与相对速度判决，F-15 ③ 已修）。
//!
//! ⚠️ **旋转扫掠（2026-10-13）**：判据 1/2 从"只看线速度"扩成**最大点速**——细长体高速自旋时
//! 尖端速度 `ω×r` 可以远超体质心速度（实测：2 m 细杆 ω = 40 rad/s ⇒ 尖端 40 m/s，而体质心接近静止）
//! ⇒ 旧口径**根本不会标记它**，刀锋直接切过薄障碍（实测：4×4 cm 小柱，关档咬进 7.8 mm 并把杆弹飞
//! 47 m；开档埋深 **0.0000**、杆被停在柱面外，见 `tests/ccd_rotational_sweep.rs`）。
//! `ω = 0` 时新口径退化为旧式（**逐位不变**）。
//! ⚠️ 本片只到"被检出 + 采样到 + 姿态钳回安全采样点"：**转动响应有意不做**（两种点冲量写法都被
//! 实测反例挡下，见 `world_ccd/sweep.rs::resolve`）⇒ 自旋体会"空转顶住"。读数与换代账见
//! `OPEN-PROBLEMS.md` **P18**；转动耦合随 M2 的 TGS-Soft 接触接入。
//!
//! ⚠️ 落这个默认档曾被 **P17**（扫掠钳位不看关节约束 ⇒ R6 球链分离 13×）挡下；修法见
//! `joints/project.rs`（钳后按关节连通分量做位置级投影）⇒ 现已放开（`OPEN-PROBLEMS.md` P17）。
//! 方法：保守推进采样——把本 tick 位移分成 ≤ `ccd_max_steps` 段
//! （段长 ≤ 0.5×最小半长），沿段检测与目标的窄相接触，命中即钳位到
//! 上一安全采样点并清零法向速度（M1 非弹性停下；恢复系数路径随
//! TGS-Soft 重构接入）。确定性：段数由状态决定，采样按序。

#![forbid(unsafe_code)]

use vxl_phys_core::{BodySet, PhysConfig, Shape};

/// **包围半径**（质心到体上最远点的距离）—— 旋转扫掠的口径：任何一点的速度 ≤ `|v| + |ω|·r_bound`。
/// 高度场/提供者（无体积语义）⇒ `INFINITY`（调用方据此跳过）。
pub fn bounding_radius(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half }
        | Shape::ConvexHull { half, .. }
        | Shape::Compound { half, .. }
        | Shape::TriMesh { half, .. } => half.length(),
        Shape::Sphere { radius } => radius,
        Shape::Cylinder {
            half_height,
            radius,
        }
        | Shape::Capsule {
            half_height,
            radius,
        }
        | Shape::Cone {
            half_height,
            radius,
        } => (half_height * half_height + radius * radius).sqrt(),
        Shape::HeightField(_) | Shape::Provider(_) => f32::INFINITY,
    }
}

/// 最大点速（`|v| + |ω|·r_bound`）：CCD 的两条动体侧判据都用它。
/// `ω = 0` 或包围半径不可用 ⇒ 退化为 `|v|`（既有场景逐位不变）。
pub fn max_point_speed(bodies: &BodySet, i: usize) -> f32 {
    let v = bodies.linvel[i].length();
    let w = bodies.angvel(i).length();
    if w <= 0.0 {
        return v;
    }
    let r = bounding_radius(&bodies.shape[i]);
    if !r.is_finite() {
        return v;
    }
    v + w * r
}

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
    // 最大点速（含 `ω×r`）：细长体自旋时尖端才是会穿的那个点（见模块头注）。
    let v = max_point_speed(bodies, i);
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
    let v = max_point_speed(bodies, i);
    let travel = v * dt;
    let ext = min_half_extent(&bodies.shape[i]).max(1e-4);
    let steps = (travel / (ext * 0.5)).ceil() as u32;
    steps.clamp(1, config.ccd_max_steps.max(1))
}
