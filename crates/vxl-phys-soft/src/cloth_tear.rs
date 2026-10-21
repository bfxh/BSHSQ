//! **布料撕裂**（`SPEC.md` §4.6 / 骨架 [`crate::params::TearStrain`] 的落点）——
//! `ClothSheet` 的撕裂**状态**（`Tearing`）。
//!
//! **本文件现在只留状态**：撕裂域的**方法**（`tear_check` / `project_edges` / 六个判据仪器）
//! 已搬进扩展 trait [`ClothTearExt`](crate::cloth::ClothTearExt)（`cloth/ext/tear.rs`）——
//! god 门处置方案「约束族求解挪进独立求解器，`ClothSheet` 只留表示与索引」的执行面
//! （见 `god.gate.json::god_pairs["crates/vxl-phys-soft::ClothSheet"]`）。
//!
//! **口径**：`eps` = **应变阈值**（`|len − rest| / rest`；骨架档位 `E03/E05/E10/None` =
//! 0.3 / 0.5 / 1.0 / **`∞`**）。超阈的**距离约束**（唯一边**与**弯曲对 —— 两者同一条规则）
//! 被标记为**已撕裂** ⇒ 该约束不再参与投影。
//!
//! **"标记"而不是"移除"**：索引稳定 ⇒ ① 确定性（不重排数组）② `rest`/`lambda` 等并行数组不必
//! 重分配 ③ 判据里的 warm id 不漂。
//!
//! **默认关闭**：`eps = ∞`（[`Tearing::default`]）⇒ `tear_check` 首行短路、跳过分支永不命中
//! ⇒ 既有场景**逐位不变**（判据实测：默认档与显式 `∞` 逐位相同）。

/// **撕裂状态**。
///
/// **关闭语义**（与 `bend_compliance = ∞` 同族先例）：`eps = f32::INFINITY`（**默认**）⇒
/// `tear_check` 首行短路 ⇒ 默认档零成本、逐位不变。
#[derive(Clone)]
pub struct Tearing {
    /// 撕裂应变阈值（`|len − rest| / rest`；**非有限 = 关**）。
    pub eps: f32,
    /// 与 `cons` 同序的**已撕裂**标记（长度不匹配时按"全未撕"处理 ⇒ 默认构造即可用）。
    pub(crate) torn: Vec<bool>,
    /// 与 `bend` 同序的**已撕裂**标记。
    ///
    /// **为什么弯曲对也要撕**（2026-09-29 实测教训）：只撕 `cons` 时，撕开处仍被**二环（弯曲）
    /// 距离约束**连着 ⇒ 布片"撕了但不松"（判据实测：单角钉住撕开 10 条边后只移动 1.53 m，
    /// 而完全自由的布片该飞走 40 m）。物理上撕口两侧的弯曲刚度也该消失 ⇒ **同一阈值也作用于
    /// `bend`**（两者都是距离约束 ⇒ 同一条"应变超阈即断"的规则）。
    pub(crate) torn_bend: Vec<bool>,
}

impl Default for Tearing {
    /// ⚠️ **默认必须是 `∞`（关）** —— 若用 `#[derive(Default)]` 会得到 `0.0`，
    /// 那等于"一有应变就撕"，会把所有既有场景撕碎。
    fn default() -> Self {
        Self {
            eps: f32::INFINITY,
            torn: Vec::new(),
            torn_bend: Vec::new(),
        }
    }
}
