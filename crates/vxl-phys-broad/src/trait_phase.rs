//! trait_phase：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub trait BroadPhase {
    /// 输出确定性有序对（a < b，字典序升序，去重）。
    fn compute_pairs(
        &mut self,
        bodies: &BodySet,
        hf_bounds: &[Aabb],
        provider_bounds: &[Aabb],
        jobs: &dyn JobSystem,
    ) -> &[(u32, u32)];

    /// 步长告知（速度自适应边距用；每子步调用一次，dt ≤ 0 表示未知）。
    /// 默认空实现（不需要该信息的宽相可直接忽略）。
    fn set_step(&mut self, _dt: f32) {}

    /// 任意 AABB 命中查询（CCD 扫掠区域用，§4.12）；输出体 id 升序去重。
    /// 默认空实现（不需要该能力的宽相可直接忽略）。
    fn query_aabb(&mut self, _aabb: &Aabb, _out: &mut Vec<u32>) {}

    /// 诊断：上一帧子阶段耗时（µs）=(AABB, 树更新, 查询, 排序)；默认全 0。
    fn breakdown_us(&self) -> (u64, u64, u64, u64) {
        (0, 0, 0, 0)
    }

    /// 诊断：树高（链路审计；默认 0 = 不适用）。
    fn tree_height(&self) -> u32 {
        0
    }

    /// 诊断：上一帧候选总数（查询返回的候选条目数之和）——候选粒度审计；
    /// 默认 0 = 不适用。
    fn cand_total(&self) -> usize {
        0
    }

    /// 诊断：上一帧**候选来源拆分** `(复用体, 新遍历体, 复用候选, 新遍历候选)`；默认 `(0,0,0,0)`。
    ///
    /// 用途（issue #4 的剩余项 + `OPEN-PROBLEMS.md`「共同真因」）：生产查询相位里绝大多数体
    /// **不遍历**，只对上一拍的候选表做精确过滤，而该过滤实测 ≈**32ns/条**（`aabbs` 是 4.8MB
    /// 冷数组）⇒ **"过滤成本落在哪一类候选上"此前没有任何数**，于是无法判断"给清醒体建紧凑
    /// AABB 副本"这条布局杠杆打在多数还是少数候选上。**纯诊断：只计数，不改判定、不进哈希。**
    fn cand_split(&self) -> (usize, usize, usize, usize) {
        (0, 0, 0, 0)
    }
}
