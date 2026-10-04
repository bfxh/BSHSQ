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

    /// 诊断：被过滤的候选按**体域**计数 `(静态, 睡眠, 清醒)`；默认 `(0,0,0)`。
    ///
    /// 用途（issue #4「候选过滤的访存局部性」）：过滤要为每条候选读按**全局体号**索引的 `aabbs[j]`；
    /// 谁是候选决定"能否给位置不变的体建只读紧凑副本"。**仅 `CAND_KIND_DIAG` 开时非零**（默认零开销）。
    fn cand_kind(&self) -> (usize, usize, usize) {
        (0, 0, 0)
    }

    /// 诊断：**查询相位的两段拆分** `(逃逸重查 µs, 精确过滤 µs)`；默认 `(0,0)`。
    ///
    /// 用途（issue #4）：`breakdown_us` 的"查询"把 refresh 与 filter 混在一起，算不出"每次逃逸花多少"。
    /// 配上同场景的逃逸次数（`cand_split().1`）即可判断大头在**次数**还是**每次的成本**。
    fn query_split_us(&self) -> (u64, u64) {
        (0, 0)
    }

    /// 每 tick 清零上面的两段拆分（默认无操作；实现方按"每子步 +=、每 tick 清"口径维护）。
    fn reset_query_split(&mut self) {}

    /// 诊断：重查次数按**来源**拆分 `(翻转帧全清, 非翻转帧)`；默认 `(0,0)`。
    ///
    /// 用途（issue #4）：翻转帧那部分是**物理驱动的尖峰**、与速度项边距 K 无关；
    /// 只有非翻转帧那部分才是 K 真正影响的量（代理盒变化 + 真逃出自缓存盒）。
    fn escape_split(&self) -> (usize, usize) {
        (0, 0)
    }
}
