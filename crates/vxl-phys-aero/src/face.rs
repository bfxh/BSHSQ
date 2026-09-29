//! **面元气动力**（面元 = 三角形）：**阻力** / **阻力 + 升力** / **三角面助手**三件都在这里
//! —— 面积、法线、退化面的口径集中一处，布料与刚体侧共用同一句话。
//!
//! **升力**（2026-09-29）：`face_force_with_lift` —— `AeroConfig::lift_slope` 的落点，
//! 骨架里立了很久、本文件才给它的消费方。

use crate::AeroConfig;
use vxl_phys_core::Vec3;

/// **单面元的气动力**（Bridson 线化）：`F = ½·ρ·Cd·A·u·|u|`。
///
/// `u` = 面元处的**相对气流**（风 − 面元速度）；`u = 0 ⇒ F = 0`（**逐位精确**，金丝雀判据用）。
/// 面元 ⊥ 风时 `|F| = ½ρv²A·Cd`（平板阻力解析式）；含切向分量（全相对速度口径，
/// 与本 crate 文档头的 Bridson 公式一致）。
#[inline]
pub fn face_force(u: Vec3, area: f32, cfg: &AeroConfig) -> Vec3 {
    u * (0.5 * cfg.air_density * cfg.drag_coefficient * area * u.length())
}

/// **单面元的气动力（阻力 + 升力）**：[`face_force`] 再加**线性升力**项
/// `F_lift = ½·ρ·A·|u|²·lift_slope·sinα·cosα·d̂` —— 实现取**未归一化**形式
/// `= ½·ρ·A·|u|²·lift_slope·(n·û)·(n − û(n·û))`（`û = u/|u|`；`sinα = n·û` 带符号），
/// 两式恒等（`|n − û(n·û)| = cosα`）但后者**没有任何除法**。
///
/// **为什么幅值必须带 `sinα·cosα`**（2026-09-29 实测教训）：若只带 `sinα`（归一化 `d̂`），
/// 迎角 → 90° 时**幅值不归零**、只有方向退化 ⇒ 布片的微小数值倾斜会让升力以**满幅值突然介入**
/// （判据④实测：自由片末速 8.60 → 9.836、单调性破坏 —— 本仓 §11 的"跳变机制"现场）。
/// 带 `cosα` ⇒ 在 0°（风在面内）与 90°（⊥风）**两端都平滑归零**，小迎角区仍线性
/// （`≈ lift_slope·α`，薄翼斜率的口径）。
///
/// **退化即零**：`n·û = 0` ⇒ 项为零；`|n·û| = 1` ⇒ `np` 是浮点噪声 ⇒ **守卫精确返回纯阻力**
/// ⇒ 既有的"静板 ⊥ 风"与"面内风"判据**逐位不变**。**双面自洽**：`n → −n` ⇒ 两处变号相消
/// ⇒ `F_lift` 不变（薄壳无内外）。**`lift_slope = 0` ⇒ 逐位等于 [`face_force`]**
/// （首行短路）⇒ "关升力"是精确的。
#[inline]
pub fn face_force_with_lift(u: Vec3, n: Vec3, area: f32, cfg: &AeroConfig) -> Vec3 {
    let drag = face_force(u, area, cfg);
    if cfg.lift_slope == 0.0 || !cfg.lift_slope.is_finite() {
        return drag;
    }
    let speed = u.length();
    if speed <= 1e-9 {
        return drag; // 静止气流：与 `face_force` 同一条零金丝雀
    }
    let ndu = n.dot(u);
    let np = n - u * (ndu / (speed * speed)); // = sinα·d̂（未归一化 ⇒ 无除法、无跳变）
    let np2 = np.length_squared();
    if !(np2.is_finite()) || np2 <= 1e-12 {
        return drag; // ⊥风（|np| 是浮点噪声）⇒ 升力物理为零 ⇒ 精确短路
    }
    let lift = np * (0.5 * cfg.air_density * area * speed * speed * cfg.lift_slope * (ndu / speed));
    drag + lift
}

/// **单个三角面的气动力**（**全量 `F`**；要均分到顶点由调用方自己 `× 1/3`）：
/// 面积与单位法线从**环绕序**算出，退化面（`area ≤ 0`）⇒ `None`（法向无定义，
/// 与 `project_constraints` 的 `len < 1e-9` 同惯例）。
///
/// 法线 = 环绕序的单位法线；[`face_force_with_lift`] 是**双面**的（法线翻转 ⇒ 力不变）
/// ⇒ 布片两侧受同样的升力，与 `mesh_mesh` 的双面口径同一句话。
pub fn face_force_tri(
    pa: Vec3,
    pb: Vec3,
    pc: Vec3,
    v_c: Vec3,
    wind: Vec3,
    cfg: &AeroConfig,
) -> Option<Vec3> {
    let x = pb - pa;
    let y = pc - pa;
    let n_x = x.cross(y);
    let area = n_x.length() * 0.5;
    if area <= 0.0 {
        return None;
    }
    let n = n_x * (0.5 / area); // 单位法线（`|n_x| = 2·area`）
    Some(face_force_with_lift(wind - v_c, n, area, cfg))
}
