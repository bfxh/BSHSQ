//! **布片的 `StateBridge`**——`ROUTE.md` §5 表示层的**第二个**真实现（第一个是流体的
//! `crates/vxl-phys-fluid/src/fluid_access.rs`；此前该 trait 全仓只有一个实现）。
//!
//! 语义与流体侧**逐条对齐**（同一 trait、同一取舍），只换表示种类与状态槽：
//! ① `kind()` = `Mesh`（`BridgeKind` 的定义里那条就是"三角网（**布料**/软体表面/静态网格）"）；
//! ② `export_positions` 按**粒子索引序**写 `pos`（顺序稳定 ⇒ 可哈希、可回放）；
//! ③ `import_positions` 长度必须**恰好**等于粒子数，否则**拒绝且一字不动**（返回 `false`）；
//! ④ 成功后**同时清速度**，并把 XPBD 的"上一位置"`prev` 也对齐——导入的语义是"把粒子放到
//!    给定位置"，不是凭空注入动能；**不清 `prev` 会让下一次 `step` 立刻读出巨大隐式速度**
//!    （`ClothSheet::step` 只在 `prev` 长度不符时才用它兜底，长度相符就照用）。
//!
//! ⚠️ 约束长度 `rest` / XPBD 乘子 `lambda` / 弯曲族**一律不动**：本桥导的是**同一张布**的
//! 状态（扫描/回放语义），不是换一张网——换网属 `PLAN-CONVERSION` 的交接策略，不在本片。
//!
//! **状态桥（2026-10-08）**：`export_state` / `import_state` 把速度与逐顶点质量一起搬。
//! 语义与流体侧逐条对齐（空段 = 不动；非空段长度必须相等，否则**先校验、整体拒绝**，不许半写）：
//! - `vel` 空 ⇒ 清速度（同 `import_positions`）；非空 ⇒ 逐点写入。XPBD 的状态其实是
//!   `(pos, prev)`，`vel` 是**下一步预测的输入**（`predict` 用它推进、`write_back` 再用
//!   `(pos−prev)/h` 重建）⇒ 导入后 `prev` 必须对齐到 `pos`，否则第一步就带出巨大隐式速度。
//! - `mass` 空 ⇒ 保持原质量；非空 ⇒ 逐点写 `mass` 并按 `1/m` 重算 `inv_mass`，**钉住位保持不变**
//!   （钉住 = `inv_mass == 0`，与 `set_pinned` 同一记法）。

use crate::cloth::ClothSheet;
use vxl_phys_core::interop::{BridgeKind, BridgeState, StateBridge};
use vxl_phys_core::Vec3;

// 紧凑写法（god 门文件行数棘轮）：`#[rustfmt::skip]` 保持单行 fn，同流体侧先例。
#[rustfmt::skip]
impl StateBridge for ClothSheet {
    fn kind(&self) -> BridgeKind { BridgeKind::Mesh }
    fn export_positions(&self, out: &mut Vec<Vec3>) { out.extend_from_slice(&self.pos); }
    fn import_positions(&mut self, src: &[Vec3]) -> bool {
        if src.len() != self.pos.len() { return false; }
        self.pos.copy_from_slice(src);
        self.prev.clear();
        self.prev.extend_from_slice(src);
        self.vel.fill(Vec3::ZERO);
        true
    }
    fn export_state(&self, out: &mut BridgeState) {
        out.pos.clear();
        out.vel.clear();
        out.mass.clear();
        out.pos.extend_from_slice(&self.pos);
        out.vel.extend_from_slice(&self.vel);
        out.mass.extend_from_slice(&self.mass);
    }
    fn import_state(&mut self, src: &BridgeState) -> bool {
        let n = self.pos.len();
        if src.pos.len() != n { return false; }
        if !src.vel.is_empty() && src.vel.len() != n { return false; }
        if !src.mass.is_empty() && src.mass.len() != n { return false; }
        self.pos.copy_from_slice(&src.pos);
        if src.vel.is_empty() { self.vel.fill(Vec3::ZERO); } else { self.vel.copy_from_slice(&src.vel); }
        self.prev.clear();
        self.prev.extend_from_slice(&self.pos);
        if !src.mass.is_empty() {
            for i in 0..n {
                let pinned = self.inv_mass[i] == 0.0;
                self.mass[i] = src.mass[i];
                self.inv_mass[i] = if pinned || self.mass[i] <= 0.0 { 0.0 } else { 1.0 / self.mass[i] };
            }
        }
        true
    }
}
