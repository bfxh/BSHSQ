//! **并列带铺开**（P20，2026-10-19；从 `select.rs` 拆出以守 `god_gate` 棘轮）。
//!
//! 规则：把「深度在带首 `TIE_TOL` 内」的连续前缀当并列带；**带内 > 4 个点时才铺开**
//! （取带内 `x`/`z` 的四个极值点 = 接触面四角，`min_sep` 去重）⇒ 流形覆盖整个接触面
//! 而不是单侧。带内 ≤ 4 个点时逐字走原路径，多数流形不受影响。
//!
//! **种子必须与深度噪声无关**（2026-10-20 实测）：深度并列时带首由浮点噪声决定 ⇒
//! 以带首为种子会让四点集逐 tick 抖动 —— 3 层静置堆的层间载荷 std（R10，判据 ≤0.5%）
//! **0.1609% → 17.02%（红）**；改成「坐标字典序最小点为种子」后回到 **0.3041%**（过）。
//!
//! **为什么**：两个**平放**盒叠放时两接触面平行，裁剪点深度全等 ⇒ 纯「深度降序 + 坐标
//! 字典序」退化成按 x 升序取前 4 个，选中集全在单侧（实测偏航 0.2° 起 x 跨度 0.8 → 0.379、
//! 形心偏移 −0.295 m）⇒ 稳态埋深 6 mm > `linear_slop` 5 mm ⇒ 去穿透偏置恒不为 0 ⇒
//! `|ω|` 卡在阈值的 3–4 倍 ⇒ **永不入睡**。
//!
//! ⚠️ 铺开时**必须保留** `min_sep` 去重：2026-10-16 那版漏掉它，铺开的 4 点可能彼此极近
//! ⇒ 流形退化成单点 ⇒ 1 万密堆压力档最大深度 +30%。取数见 `docs/OPEN-PROBLEMS.md` §P20。
use crate::ContactPoint;

/// 并列带容差（m）：5e-4 是 2026-10-15 剂量曲线的台阶起点（1e-4 触发不了「带内 >4」）。
const TIE_TOL: f32 = 5e-4;
const MAX_POINTS: usize = 4;

/// 按深度序取 ≤4 点；`spread` 时先走并列带铺开，再按原序补齐。
pub(crate) fn pick(
    cand: &[ContactPoint],
    kept: &mut Vec<ContactPoint>,
    min_sep: f32,
    spread: bool,
) -> bool {
    let min2 = min_sep * min_sep;
    kept.clear();
    let band = if spread { band_len(cand) } else { 0 };
    if band > MAX_POINTS {
        spread_pick(cand, kept, min2, band);
    }
    fill_rest(cand, kept, min2);
    !kept.is_empty()
}

/// 并列带长度（连续前缀，不链式；`cand` 已按深度降序）。
fn band_len(cand: &[ContactPoint]) -> usize {
    cand.iter()
        .take_while(|c| cand[0].depth - c.depth <= TIE_TOL)
        .count()
}

/// 带内铺开：以**坐标字典序最小**的点为种子（与深度噪声无关），其后每个点取
/// 「到已选集最近距离最大」者（`min_sep` 内视为同一材料点）。
fn spread_pick(cand: &[ContactPoint], kept: &mut Vec<ContactPoint>, min2: f32, band: usize) {
    let b = &cand[..band];
    let mut seed = 0usize;
    for (i, c) in b.iter().enumerate() {
        let better = c.point.x < b[seed].point.x
            || (c.point.x == b[seed].point.x && c.point.z < b[seed].point.z);
        if better {
            seed = i;
        }
    }
    kept.push(b[seed]);
    while kept.len() < MAX_POINTS {
        let mut best: Option<usize> = None;
        let mut best_d = -1.0f32;
        for (i, c) in b.iter().enumerate() {
            let nearest = kept
                .iter()
                .map(|k| (k.point - c.point).length_squared())
                .fold(f32::MAX, f32::min);
            if nearest < min2 {
                continue;
            }
            if nearest > best_d {
                best_d = nearest;
                best = Some(i);
            }
        }
        match best {
            Some(i) => kept.push(b[i]),
            None => break,
        }
    }
}

/// 余量：按原（深度）顺序补齐（已入者靠 `min2` 去重跳过）。
fn fill_rest(cand: &[ContactPoint], kept: &mut Vec<ContactPoint>, min2: f32) {
    for &c in cand.iter() {
        if kept.len() >= MAX_POINTS {
            break;
        }
        let dup = kept
            .iter()
            .any(|k| (k.point - c.point).length_squared() < min2);
        if !dup {
            kept.push(c);
        }
    }
}
