//! **布片 ↔ 刚体耦合**（软体切片 2b-i / 2b-ii）——`ClothSheet` 的刚体接触**状态**
//! （`BodyHit` / `BodyCoupling`）。
//!
//! **本文件现在只留状态**：刚体耦合域的**方法**（七个）已搬进扩展 trait
//! [`ClothBodyExt`](crate::cloth::ClothBodyExt)（`cloth/ext/body.rs`）—— god 门处置方案
//! 「约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引」的执行面
//! （见 `god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]`）。
//!
//! **为什么当初独立成文件**：`cloth.rs` 受 god 门**文件行数棘轮**（只准减）——反作用两腿一上来就把
//! 它从 425 行推到 648 行（超出 10% 交换窗）⇒ 按先例（`world_mesh.rs`：`impl World` 方法数顶 god 门
//! 24 成员，故按域拆新文件）把"刚体接触"这**一个域**整块搬走。
//!
//! **两条路径**：
//! - **静态/睡眠代理 = 墙**（2b-i）：逐粒子解析穿透 + 库仑锥，**无反作用**（`inv_mass = 0`）。
//!   逐字保留（`tests/cloth_body.rs` 的两条读数**逐位不变**）。
//! - **动态代理 = 两体约束 + 反作用两腿**（2b-ii）：口径与 `rope::apply_body_hit` 同款，
//!   §8.4.20 三条使能条件一条不缺。**唯一偏离**见 `body_contacts_dynamic` 的注。
use vxl_phys_core::Vec3;

/// **一次体接触的收集项**（切片 2b-ii；与 `rope::BodyHit` 同款）：
/// 先收全部命中、按**几何序**排、再施加 ⇒ 施加顺序是**几何的函数**。
///
/// 为什么必须排（§8.4.24 的教训）：`body.dv` 是**就地推进**的顺序 Gauss-Seidel ⇒ "谁先被处理"
/// 会改变结果。实测（rope，§8.4.23）：把粒子循环反向，常规载重下的横向漂移**符号翻转**
/// （0.5 kg：`+0.098 → −0.124`）⇒ 那是**顺序偏差**、不是物理。
#[derive(Clone, Copy)]
pub(crate) struct BodyHit {
    /// 粒子号。
    pub(crate) i: u32,
    /// 排序主键：粒子自己的 `x`（世界横坐标，升序）——**物理量** ⇒ 与数组方向无关。
    pub(crate) ord: f32,
    /// 排序次键：`z`（同 `x` 不可能出现 ⇒ 仅作确定性兜底）。
    pub(crate) z: f32,
    /// 该次接触的法线 / 穿透 / 接触点（`shape_penetration` 的读数）。
    pub(crate) n: Vec3,
    pub(crate) depth: f32,
    pub(crate) q: Vec3,
}

/// **粒子↔刚体耦合状态**（与 `step` 传入的 `bodies` 同序；每 tick 开头重置）。
///
/// **为什么收成一个结构**：`ClothSheet` 受 god 门**成员棘轮**（21 成员顶格，阈值 24）——
/// 这四样一上来就超阈 ⇒ 按先例（`rope::VirtRot` / `world_soft::SoftDomain`）合并成一个
/// （`ClothSheet` 成员 21 → 22）。
///
/// **`disp` 是"虚拟位移"**：体在本 tick 内自己走的 + 被接触推开的（**两条腿都算**），
/// **逐子步增量累加**。
/// ⚠️ **不可写成 `(linvel + Δv)·(sub_idx·h)`**（rope 实测过的公式错误）：那会把"当前累计的
/// `Δv`"乘上**整段时间**（末子步乘 8h）⇒ 虚拟位姿被放大 ⇒ 接触几何算错。
#[derive(Clone, Default)]
pub struct BodyCoupling {
    /// **体的速度增量**（门面 `bodies.linvel[j] += dv[j]`）：引擎在体解算**之后**施加
    /// ⇒ **下一 tick 生效**。每 tick 开头清零。
    pub dv: Vec<Vec3>,
    /// **位置口径的补足量**（门面 `bodies.position[j] += dx[j]`，§8.4.9）：被速度钳位
    /// `λ = min(λ_geom, λ_vel)` 压掉的那一份**只补位置、不补速度**。
    ///
    /// **为什么必须有**：静载下 `approach ≈ 0 ⇒ λ ≈ 0` ⇒ 接触**只回速度、不回位置** ⇒
    /// 体每 tick 先按 `v·dt` 走过的 `g·dt² = 2.72 mm` **一去不回**（rope 实测下沉 = 整漏的 83%）。
    /// **为什么用"差值"而不是 `λ_geom` 本身**：`disp`（虚拟位姿）已含钳位后的那一份 ⇒
    /// 补足 `λ_geom − λ` 之后，体在本 tick 的**总位置效应恰好 = `λ_geom`**（不重复计账），
    /// 且**钳位生效时（运动/冲击工况）恒为 0 ⇒ 既有工况逐位不变**。
    pub dx: Vec<Vec3>,
    /// 子步内**虚拟位移**（体自己走的 + 被推开的，两条腿都算；增量累加，见类型注）。
    pub(crate) disp: Vec<Vec3>,
    /// 命中收集暂存（复用免分配）。
    pub(crate) hits: Vec<BodyHit>,
}
