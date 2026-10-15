//! **接触点选点**（2026-10-16 从 `prims.rs` 搬出：`prims.rs` 是形状原语（球/胶囊/凸体），
//! 选点是独立关注点；搬出同时让 `prims.rs` 只准减的棘轮（`scripts/god_gate.py`）保持绿）。
//!
//! 口径：按深度降序 + 空间去重（`min_sep` 内视为同一点）+ 截断 ≤4。**本口径未变**——
//! 2026-10-16 试过「并列带内铺开（变体 B）」与「并列带内取最靠面心（变体 C）」两种
//! 对称化选点，都修好了静置/门槛档（塔上 383 个超 slop 点 → 0、门槛档末态活跃 12→0、
//! 可入睡），但都退化 1 万密堆压力档与两处宽堆聚合指标（默认档 Σv² 1.8124→3.01~3.09、
//! `arena_bench pyramid` Σv² 0.7051→1.10~1.91）；且并列带规模/跨度分布在「沉降」与
//! 「密堆坍塌」两场景族间**统计上无差别**（无可分辨的几何量）⇒ 无「限定规则」可分离。
//! 结论：**回退到本口径**，P20（静置叠体维持 > `linear_slop` 埋深 ⇒ 永不入睡）
//! 记为未解，取数见 `docs/OPEN-PROBLEMS.md` §P20 与 `docs/EXPERIMENTS.md`。
use crate::DefaultNarrowPhase;

pub(crate) fn select_contacts(np: &mut DefaultNarrowPhase) -> bool {
    let min_sep = np.min_point_sep;
    if np.ws.cand.is_empty() {
        return false;
    }
    np.ws.cand.sort_by(|x, y| {
        y.depth
            .total_cmp(&x.depth)
            .then(x.point.x.total_cmp(&y.point.x))
            .then(x.point.y.total_cmp(&y.point.y))
            .then(x.point.z.total_cmp(&y.point.z))
    });
    let min2 = min_sep * min_sep;
    // 复用 scratch（T3：旧实现每调用一次 Vec::with_capacity(4) —— 8B 场景
    // ~22 万次/帧的堆分配）。语义逐位不变：按深度序取 ≤4 个非重复点。
    let mut kept = std::mem::take(&mut np.ws.kept_buf);
    kept.clear();
    for &c in np.ws.cand.iter() {
        if kept.len() >= 4 {
            break;
        }
        let dup = kept
            .iter()
            .any(|k| (k.point - c.point).length_squared() < min2);
        if !dup {
            kept.push(c);
        }
    }
    np.ws.cand.clear();
    np.ws.cand.extend_from_slice(&kept);
    np.ws.kept_buf = kept;
    !np.ws.cand.is_empty()
}
