//! **流体的状态桥**（`StateBridge`）—— `ROUTE.md` §5 ②表示层的实现之一。
//!
//! 两档语义（接口见 `vxl_phys_core::interop`，数据结构见 `interop::BridgeState`）：
//! ① **位置桥**：`export_positions` 按索引序只写**流体段**（`n_fluid` 之后的边界粒子每 tick
//!    由门面重建，不算流体状态）；`import_positions` 长度必须**恰好**等于流体段，否则**拒绝且
//!    一字不动**（返回 `false`）；成功后**同时清速度** —— 导入语义是"把粒子放到给定位置"，
//!    不是凭空注入动能（与既有两个实现同口径，逐位不变）。
//! ② **状态桥**：`export_state` 把位置 + 速度 + 逐粒质量（`pmass` 前缀）一并导出；`import_state`
//!    按**先校验、再写入**的顺序全量替换，**空段 = 不动**（`vel` 空 ⇒ 清速度，与①同口径；
//!    `mass` 空 ⇒ 保持原质量），任一段非空但长度不符 ⇒ **整体拒绝**（不许半写）。
//!    ⚠️ 质量是**真状态**（SPH 的 `pmass` 逐粒参与密度/压力），交接会改动力学口径 ——
//!    这正是"状态继承"要的效果，也是它必须显式登记（不许静默）的理由。
use crate::{FluidSystem, Vec3};
use vxl_phys_core::interop::{BridgeKind, BridgeState, StateBridge};

// 紧凑写法（god 门文件行数棘轮）：`#[rustfmt::skip]` 保持单行 fn，本仓既有先例。
#[rustfmt::skip]
impl StateBridge for FluidSystem {
    fn kind(&self) -> BridgeKind { BridgeKind::Particle }
    fn export_positions(&self, out: &mut Vec<Vec3>) { out.extend_from_slice(&self.pos[..self.n_fluid]); }
    fn import_positions(&mut self, src: &[Vec3]) -> bool {
        if src.len() != self.n_fluid { return false; }
        self.pos[..self.n_fluid].copy_from_slice(src);
        self.vel[..self.n_fluid].fill(Vec3::ZERO);
        true
    }
    fn export_state(&self, out: &mut BridgeState) {
        out.pos.clear();
        out.vel.clear();
        out.mass.clear();
        let n = self.n_fluid;
        out.pos.extend_from_slice(&self.pos[..n]);
        out.vel.extend_from_slice(&self.vel[..n]);
        out.mass.extend_from_slice(&self.pmass[..n]);
    }
    fn import_state(&mut self, src: &BridgeState) -> bool {
        let n = self.n_fluid;
        if src.pos.len() != n { return false; }
        if !src.vel.is_empty() && src.vel.len() != n { return false; }
        if !src.mass.is_empty() && src.mass.len() != n { return false; }
        self.pos[..n].copy_from_slice(&src.pos);
        if src.vel.is_empty() { self.vel[..n].fill(Vec3::ZERO); } else { self.vel[..n].copy_from_slice(&src.vel); }
        if !src.mass.is_empty() { self.pmass[..n].copy_from_slice(&src.mass); }
        true
    }
}
