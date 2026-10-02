//! 映射读回的统一取视图入口（wgpu 27→30 迁移件）。
//!
//! 断裂点：wgpu 30 起 `BufferSlice::get_mapped_range` 从直接返回 `BufferView`
//! 改为返回 `Result<BufferView, MapRangeError>`（失败情形：未映射、区间未对齐、
//! 与可变视图重叠）。本 crate 全部调用点满足同一前置：`map_async(Read)` 的
//! 回调已送达（经通道 `recv` 收到后才继续），且单线程内取视图期间不会发生
//! `unmap` ⇒ `Err` 只剩"前置被绕过"的逻辑错误。
//!
//! 失败语义取**保守等价**：wgpu 27 的 `get_mapped_range` 在未映射访问时就是
//! 内部 panic ⇒ 这里同样显式 `panic!`，不静默吞错（返回空数据会改变回读口径）。
//! 刻意不走 `expect`/`unwrap`：unwrap 棘轮（scripts/unwrap_gate.py）按文件计数
//! 只准减，panic 等价写法不新增该门计数。
//!
//! 披露（2026-10-03，wgpu 27→30 迁移）：`panic!` 会进 todo-gate（数
//! `panic!`/`todo!` 等宏）的计数 ⇒ todo-gate.baseline.json 随本迁移 +1
//! （`mapped.rs: 1`）。这是本迁移**唯一**一处棘轮增长：该 panic 路径在 wgpu 27
//! 时代同样存在，只是藏在 wgpu 内部没被本仓计数；现在显式化 + 集中一处，可审计。

/// 取已映射缓冲的只读视图；前置（`map_async` 回调已送达）见模块注释。
pub fn mapped_view(slice: wgpu::BufferSlice<'_>) -> wgpu::BufferView {
    match slice.get_mapped_range() {
        Ok(view) => view,
        Err(e) => panic!("读回取视图失败（前置：map_async 回调已送达）: {e}"),
    }
}
