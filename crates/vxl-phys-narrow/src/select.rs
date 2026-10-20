//! **接触点选点**（2026-10-16 从 `prims.rs` 搬出：`prims.rs` 是形状原语（球/胶囊/凸体），
//! 选点是独立关注点；搬出同时让 `prims.rs` 只准减的棘轮（`scripts/god_gate.py`）保持绿）。
//!
//! 口径（2026-10-19 换代）：**深度降序 → 并列带内铺开 → 空间去重 → 截断 ≤4**。
//!
//! **为什么要有「并列带」**（P20 根因，实测）：两个**平放**盒叠放时两接触面平行，裁剪点深度
//! **全等**（只有浮点噪声级差异）⇒ 纯「深度降序 + 坐标字典序」退化成**按 x 升序**取前 4 个，
//! 选中集全在**单侧**（实测：偏航 0.2° 起，x 跨度 0.8 → 0.379、形心偏移 −0.295 m；埋深
//! −20…+5 mm 扫描逐档不变 ⇒ 结构性而非噪声）。单侧流形让体停在 slop 之上（稳态埋深 6 mm >
//! `linear_slop` 5 mm）⇒ 去穿透偏置恒不为 0 ⇒ `|ω|` 卡在阈值的 3–4 倍 ⇒ **永不入睡**。
//!
//! **铺开规则**：把「深度在首成员 `TIE_TOL` 内」的连续前缀当并列带；**带内 > 4 个点时才铺开**
//! （首个取带首，其后每次取「到已选集最近距离最大」者，且与已选集距离 ≥ `min_sep`）⇒ 流形
//! 覆盖整个接触面而不是单侧。带内 ≤ 4 个点时逐字走原路径（多数流形不受影响）。
//!
//! ⚠️ `min_sep` 去重在铺开时**必须保留**：2026-10-16 那版没有去重，铺开的 4 点可能彼此极近
//! ⇒ 流形退化成单点 ⇒ 1 万密堆压力档最大深度 +30%。本轮带上去重后压力档反而更好
//! （深穿透 119 → 104），说明当时判"两变体各修一侧"是**那个 bug** 而不是方向错。
//!
//! 取数见 `docs/OPEN-PROBLEMS.md` §P20 与 `docs/EXPERIMENTS.md`。
mod band;

use crate::DefaultNarrowPhase;

/// `spread` = 是否允许并列带内铺开。**只对凸体-凸体（SAT 裁剪）开**：那里并列点来自
/// 同一接触面的四角；mesh / 高度场 / 球胶囊的候选是「顶点×三角」交点，铺开会选到
/// 分布很远的点 ⇒ 薄壳接触被持续外推（实测 `mesh_vs_mesh` 末间距 22.3×skin）。
pub(crate) fn select_contacts(np: &mut DefaultNarrowPhase, spread: bool) -> bool {
    let ws = &mut np.ws;
    if ws.cand.is_empty() {
        return false;
    }
    ws.cand.sort_by(|x, y| {
        y.depth
            .total_cmp(&x.depth)
            .then(x.point.x.total_cmp(&y.point.x))
            .then(x.point.y.total_cmp(&y.point.y))
            .then(x.point.z.total_cmp(&y.point.z))
    });
    let ok = band::pick(&ws.cand, &mut ws.kept_buf, np.min_point_sep, spread);
    ws.cand.clear();
    ws.cand.extend_from_slice(&ws.kept_buf);
    ok
}
