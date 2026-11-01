//! **GPU 后端统一入口**（`PhysGpuBackend`）与 CPU 回退占位—— `vxl-phys-gpu` 的 `lib.rs` 拆出。
//!
//! **为什么拆出来**：`lib.rs` 顶在 god 门**文件行数棘轮**（只准减），而"新增一个卡上档就要在
//! `lib.rs` 添一行 `pub mod`"是常态（本片 = `solve_tier`）⇒ 清单文件必须能腾得开。
//! 内容逐字搬移，语义零改动。

/// GPU 后端统一入口。
pub trait PhysGpuBackend: Send + Sync {
    fn name(&self) -> &'static str;
    /// 是否可用（设备枚举 + 特性检测）。
    fn available(&self) -> bool;
    /// 批量任务提交入口（粒子/流体/布料/批量碰撞/批量 memfind）。
    /// M4 实现；M0 返回 false 表示未就绪。
    fn submit(&self, _tag: &'static str) -> bool {
        false
    }
}

/// CPU 回退占位：任何 GPU 路径都必须能在无 GPU 环境退化到与 CPU 一致的路径。
pub struct CpuFallback;

impl PhysGpuBackend for CpuFallback {
    fn name(&self) -> &'static str {
        "cpu-fallback"
    }
    fn available(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::{CpuFallback, PhysGpuBackend};

    #[test]
    fn fallback_always_available() {
        assert!(CpuFallback.available());
    }
}
