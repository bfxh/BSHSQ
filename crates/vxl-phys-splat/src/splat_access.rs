//! **高斯喷溅的状态桥**（`StateBridge`）—— `ROUTE.md` §3.1 三层里的①（渲染/状态桥）与
//! ⑤`MediumField` 之间的**状态**面：此前只有单向的 [`crate::export_splats`]（渲染参数拷贝），
//! 本文件补上接口层的导出/导入与反向导入。
//!
//! 语义：
//! ① `kind()` = `BridgeKind::Splat`；② `export_positions` 按注册序写**核中心**；
//! ③ `import_positions` 长度不符 ⇒ **拒绝且一字不动**；长度相符 ⇒ 逐核换中心，**尺度/姿态/
//!    不透明度/颜色不动**（换的是同一批核的位置，不是换一批核 —— 换核属表示转换侧，不在本仓）；
//! ④ `export_state` 的质量口径 = [`Splat::mass`]（**介质密度场的积分质量**，P11 定案；与
//!    `flow.rs` 的动量分摊**同一本账**）；`medium_density <= 0`（没声明介质质量）⇒ `mass` **留空
//!    = 未登记**（`momentum()` 返回 `None`，不是 0）。速度 = 逐核速度场 `kern_vel`，只在
//!    **双向耦合开着且长度相符**时登记，否则空 = 未登记；
//! ⑤ `import_state`：`vel` 非空要求**已开双向耦合**（没开就拒绝：该场没有速度槽，静默丢掉就是
//!    丢动量）；`mass` 是**派生量**（`opacity`+尺度+`ρ_medium` 的函数）⇒ 非空时必须与当前派生值
//!    逐位一致，否则拒绝（"要改质量请改 opacity/尺度"，不许把派生量当独立状态写进去）。
use crate::GaussianSplatField;
use vxl_phys_core::interop::{BridgeKind, BridgeState, StateBridge};
use vxl_phys_core::Vec3;

// 紧凑写法（god 门文件行数棘轮）：`#[rustfmt::skip]` 保持单行 fn，本仓既有先例。
#[rustfmt::skip]
impl StateBridge for GaussianSplatField {
    fn kind(&self) -> BridgeKind { BridgeKind::Splat }
    fn export_positions(&self, out: &mut Vec<Vec3>) {
        for s in &self.splats { out.push(s.center); }
    }
    fn import_positions(&mut self, src: &[Vec3]) -> bool {
        if src.len() != self.splats.len() { return false; }
        for (s, p) in self.splats.iter_mut().zip(src) { s.center = *p; }
        self.grid = None; // 中心变了 ⇒ 均匀网格登记失效（查询退回全扫，与网格逐位一致）
        true
    }
    fn export_state(&self, out: &mut BridgeState) {
        out.pos.clear();
        out.vel.clear();
        out.mass.clear();
        let rho = self.medium_density;
        for s in &self.splats {
            out.pos.push(s.center);
            if rho > 0.0 { out.mass.push(s.mass(rho)); }
        }
        if self.two_way && self.kern_vel.len() == self.splats.len() { out.vel.extend_from_slice(&self.kern_vel); }
    }
    fn import_state(&mut self, src: &BridgeState) -> bool {
        let n = self.splats.len();
        if src.pos.len() != n { return false; }
        if !src.mass.is_empty() && src.mass.len() != n { return false; }
        if !src.vel.is_empty() && (!self.two_way || src.vel.len() != n) { return false; }
        // `mass` 是**派生量**：只接受与当前派生值逐位一致的那份（否则拒绝 —— 不许把派生量当状态写）
        if !src.mass.is_empty() {
            for (k, s) in self.splats.iter().enumerate() {
                if src.mass[k] != s.mass(self.medium_density) { return false; }
            }
        }
        for (k, s) in self.splats.iter_mut().enumerate() { s.center = src.pos[k]; }
        if !src.vel.is_empty() { self.kern_vel.clear(); self.kern_vel.extend_from_slice(&src.vel); }
        self.grid = None;
        true
    }
}
