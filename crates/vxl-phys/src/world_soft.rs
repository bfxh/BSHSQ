//! **软体域（绳索）通道** + 2b 边界的**成组状态**。
//!
//! **为什么 `FluidBoundary` 定义在这里**：`World` 是 god 门棘轮下的**记录型**结构（成员数只准减），
//! 加一个域就得腾一个成员位 ⇒ 把 2b 那一族的 3 个散字段收成一个结构（成员 23 → 22，再 +1 给绳索）。
//! 定义随域走、`World` 只留一个字段。
//!
//! **绳索通道**（T1，`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8）：与液体域同款——每 tick **一次**
//! （体子步全部完成之后）推进；接触走**统一提供者通道**（`Providers` 实现 `ProviderColliders`，
//! 与窄相同一个 id 空间，见 `world_step.rs` 的 `fluid_pass` 同款段位）。**没有绳索的场景逐位不变**
//! （`ropes` 空 ⇒ `rope_pass` 首行短路）⇒ 默认档判据不受影响。
//!
//! **本片边界**：绳索的接触走**提供者**（地形）+ **刚体代理**（形状：Sphere/Box/Capsule）；
//! 自碰撞、`Compound`/`Cylinder`/`Cone`/`ConvexHull` 代理、体积/弯曲约束都属后续切片。
use super::*;

// **域反作用的自由函数**（角反作用 + 布→介质反作用）：走 `#[path]` 子模块——`lib.rs` 零行预算，
// 加不了 crate 级 `mod`（同 `vxl-phys-soft` 的 `cloth_medium` 先例）。
#[path = "world_soft_reaction.rs"]
pub(crate) mod soft_reaction;
pub(crate) use soft_reaction::{apply_rope_angular_reactions, cloth_medium_reaction};

// **软体域的扩展 trait**（`SoftExt`）——走 `#[path]` 子模块（同 `world_soft_reaction.rs` 先例）：
// `lib.rs` 零行预算，加不了 crate 级 `mod`（god 门的行数棘轮）。
#[path = "world_soft_ext.rs"]
mod soft_ext;
pub use soft_ext::SoftExt;

/// 2b（Akinci 边界粒子）那一族的成组状态（原为 `World` 的 3 个散字段：开关 / 暂存 / 覆盖集）。
#[derive(Default)]
pub struct FluidBoundary {
    /// 与 `fluids` 同序的 2b 开关：`true` = 该流体每 tick 重建边界粒子并回流反作用。
    pub two_b: Vec<bool>,
    /// 边界粒子生成的暂存 `(体 id, 形状, 位姿)`（复用免每 tick 分配）。
    pub scratch: Vec<(u32, Shape, vxl_phys_fluid::BodyPose)>,
    /// **覆盖集**（**按流体**各一份：`covered[fi]` 与 `bodies` 同序，每 tick 由该流体的
    /// `refresh_fluid_boundary` 重建）= 2a 对该流体的让位集。修 §2 C4：原先跨流体单数组
    /// ⇒ 只有最后一个 2b 流体的覆盖集有效 ⇒ 2a/2b 双重计账（判据 `fluid_covered_multi`：
    /// 修前体被 2a 浮力从 y 1.27 弹到 5.85）。
    pub covered: Vec<Vec<bool>>,
}

/// **软体域的成组状态**（原 `World` 的 `ropes`/`rope_proxies` 两个散字段收成一组，
/// 给布片腾成员位——`FluidBoundary` 同款先例；god 门成员棘轮只准减）。
#[derive(Default)]
pub struct SoftDomain {
    /// 绳索（`World::add_rope` 注册；`rope_pass` 每 tick 推进一次，空集零成本短路）。
    pub ropes: Vec<vxl_phys_soft::Rope>,
    /// 绳索的**刚体代理暂存**（每 tick 重建，复用免分配；`rope_pass` 里 `mem::take` 借出后归还）。
    pub rope_proxies: Vec<vxl_phys_soft::RigidProxy>,
    /// 布片（`World::add_cloth` 注册；`cloth_pass` 每 tick 推进一次，空集零成本短路）。
    pub cloths: Vec<vxl_phys_soft::ClothSheet>,
}

impl FluidBoundary {
    /// 该流体是否开了 2b（越界一律 `false`）。**写成方法而不是在调用点展开链**：
    /// 展开式在原地超 `chain_width` 会被 rustfmt 折成 5 行 —— 而 `world_body.rs` 受尺寸棘轮
    /// （只准减），折行会让它"变胖"。
    pub fn is_two_b(&self, fluid: usize) -> bool {
        self.two_b.get(fluid).copied().unwrap_or(false)
    }
}

// **反作用两腿**的施加本体已收口进 `world_step/coupling.rs`（`cpl::apply_two_leg_reactions`，
// C3 片 2 第 1 步纯搬移）——本文件不再直接写 `bodies.linvel` / `bodies.position`。
