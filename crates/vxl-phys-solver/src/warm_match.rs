//! **warm 匹配**：精确特征 / 近邻回退 / 未匹配的归因（自 `constraint.rs` 纯搬移，语义未改；
//! 搬出来是为了给"完全未匹配"的拒因诊断腾位置——两个文件都卡在 god 门基线上）。
//!
//! 未匹配拒因（互斥，按回退分支同一顺序判）：
//! 1. **深度门** `cp.depth <= FB_DEPTH_MIN`：本帧已分离/投机接触 ⇒ 不继承暖冲量；
//! 2. **距离门**：1×skin 内没有上一帧锚点 ⇒ 材料点真的换了位置/换了一组点；
//! 3. **深度跳变门**：锚点就在旁边，但 `|cp.depth - depth0| > FB_DEPTH_JUMP`。
//!
//! 自校验：三类之和 == `WARM_MISS`（`miss_gate_take()` 与 `warm_match_stats_take()` 对账）。
use std::sync::atomic::{AtomicU64, Ordering};

use vxl_phys_core::Vec3;
use vxl_phys_narrow::{ContactPoint, Manifold};

use crate::stats::{
    FB_DEPTH_MIN, WARM_EXACT, WARM_FALLBACK, WARM_MISS, WB_CLIP, WB_HASH, WB_SAME, WB_SIDE,
    WN_FLIP, WN_SAME,
};
use crate::warm::WarmPoint;
use crate::warm_data::{WarmManifold, FB_DEPTH_JUMP};

static MS_DEPTH: AtomicU64 = AtomicU64::new(0);
static MS_DIST: AtomicU64 = AtomicU64::new(0);
static MS_JUMP: AtomicU64 = AtomicU64::new(0);

/// 读取并清零：(深度门拒, 距离门拒, 深度跳变门拒)。
pub fn miss_gate_take() -> (u64, u64, u64) {
    (
        MS_DEPTH.swap(0, Ordering::Relaxed),
        MS_DIST.swap(0, Ordering::Relaxed),
        MS_JUMP.swap(0, Ordering::Relaxed),
    )
}

/// 一个**完全未匹配**的点按三道门做互斥归因（调用点已确认：有流形、法向对齐、两路都没命中）。
fn miss_reason(
    cp: &ContactPoint,
    wm: &WarmManifold,
    warm_world: &[Vec3; 4],
    warm_n: usize,
    match_dist: f32,
) {
    if cp.depth <= FB_DEPTH_MIN {
        MS_DEPTH.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let fb = match_dist * 0.25; // 与回退分支同口径（1×skin）
    let mut near = 0usize;
    let mut jump_ok = 0usize;
    for (k, w) in warm_world.iter().enumerate().take(warm_n) {
        if (*w - cp.point).length_squared() < fb * fb {
            near += 1;
            if (cp.depth - wm.points[k].depth0).abs() <= FB_DEPTH_JUMP {
                jump_ok += 1;
            }
        }
    }
    if near == 0 {
        MS_DIST.fetch_add(1, Ordering::Relaxed);
    } else if jump_ok == 0 {
        MS_JUMP.fetch_add(1, Ordering::Relaxed);
    }
}

/// warm starting 匹配：① 特征 ID 精确匹配（带距离护栏，锚点当前世界位置量距）；
/// ② 近邻回退（无特征或 ID 未命中）。命中/回退/未命中都进诊断计数。
pub(crate) fn match_warm_point(
    cp: &ContactPoint,
    m: &Manifold,
    warmm: Option<&WarmManifold>,
    warm_world: &[Vec3; 4],
    warm_n: usize,
    match_dist: f32,
) -> Option<WarmPoint> {
    let wm = warmm?;
    if warm_n == 0 || wm.normal.dot(m.normal) <= 0.95 {
        return None;
    }
    if let Some(w) = exact_match(cp, wm, warm_world, warm_n, match_dist) {
        WARM_EXACT.fetch_add(1, Ordering::Relaxed);
        return Some(w);
    }
    if let Some(w) = fallback_match(cp, wm, warm_world, warm_n, match_dist) {
        count_fallback(m, wm, cp, w);
        return Some(w);
    }
    miss_reason(cp, wm, warm_world, warm_n, match_dist);
    WARM_MISS.fetch_add(1, Ordering::Relaxed);
    None
}

/// ① 精确匹配：特征号相同、且锚点当前世界位置在 `match_dist` 内。
fn exact_match(
    cp: &ContactPoint,
    wm: &WarmManifold,
    warm_world: &[Vec3; 4],
    warm_n: usize,
    match_dist: f32,
) -> Option<WarmPoint> {
    if cp.feature == 0 {
        return None;
    }
    (0..warm_n)
        .find(|&k| {
            wm.points[k].feature == cp.feature
                && (warm_world[k] - cp.point).length_squared() < match_dist * match_dist
        })
        .map(|k| wm.points[k])
}

/// ② 近邻回退。
///
/// **回退分支单独收紧接受半径**（上一轮实测结论的直接产物）：回退（特征未命中）按
/// 4×skin ≈4cm 接受时会把**错配锚点**也认下来 ⇒ 帧间「漂移」= 两点真实错位 ⇒ 暖启动冲量
/// 过驱动速度 ⇒ 堆族永不入睡（实测：整个关掉回退使 col45 45/45、pile5 1985/2000 全睡且
/// 位置更准，但塔崩）。⇒ 收到 **1×skin**：只认「确实还是同一个材料点」的候选；
/// 拒配 ⇒ 按新接触处理、不暖启动。
fn fallback_match(
    cp: &ContactPoint,
    wm: &WarmManifold,
    warm_world: &[Vec3; 4],
    warm_n: usize,
    match_dist: f32,
) -> Option<WarmPoint> {
    // 接触状态门（非距离信号）：分离/预期接触不继承上一帧的暖冲量
    // （错配冲量过驱动间歇角点接触）。
    if cp.depth <= FB_DEPTH_MIN {
        return None;
    }
    let fb = match_dist * 0.25;
    (0..warm_n)
        .filter(|&k| {
            if (warm_world[k] - cp.point).length_squared() >= fb * fb {
                return false;
            }
            // 深度跳变门（非距离信号）：|本帧裁剪深度 − 锚点烘焙深度| 超限 ⇒ 拒配。
            // 见 FB_DEPTH_JUMP 注（含「恒等式陷阱」教训）。
            (cp.depth - wm.points[k].depth0).abs() <= FB_DEPTH_JUMP
        })
        .min_by(|&x, &y| {
            (warm_world[x] - cp.point)
                .length_squared()
                .total_cmp(&(warm_world[y] - cp.point).length_squared())
        })
        .map(|k| wm.points[k])
}

/// 回退命中的诊断：参考面代理 + 位域成因分解（只计数）。
fn count_fallback(m: &Manifold, wm: &WarmManifold, cp: &ContactPoint, w: WarmPoint) {
    WARM_FALLBACK.fetch_add(1, Ordering::Relaxed);
    // 参考面代理：法向是否变（见 `warm_normal_flip_take` 注）。
    if wm.normal.dot(m.normal) < 0.99999 {
        WN_FLIP.fetch_add(1, Ordering::Relaxed);
    } else {
        WN_SAME.fetch_add(1, Ordering::Relaxed);
    }
    // 成因分解（诊断，只计数）：见 `vxl_phys_narrow::feature_kind` 注。
    let (s_new, c_new) = vxl_phys_narrow::feature_kind(cp.feature);
    let (s_old, c_old) = vxl_phys_narrow::feature_kind(w.feature);
    if s_new != s_old {
        WB_SIDE.fetch_add(1, Ordering::Relaxed);
    } else if c_new != c_old {
        WB_CLIP.fetch_add(1, Ordering::Relaxed);
    } else if vxl_phys_narrow::feature_hash_part(cp.feature)
        != vxl_phys_narrow::feature_hash_part(w.feature)
    {
        WB_HASH.fetch_add(1, Ordering::Relaxed);
    } else {
        WB_SAME.fetch_add(1, Ordering::Relaxed);
    }
}
