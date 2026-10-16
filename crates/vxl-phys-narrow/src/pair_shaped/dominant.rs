//! **提供者通道的候选面判定与取点** —— `pair_shaped` 的**子模块**（`#[path]` 指向
//! `pair_shaped/dominant.rs`）。
//!
//! **为什么拆出来**：`provider_pair`（父模块）从提供者候选里挑**主导接触面**、再选
//! **四角优先**的流形点；这两段判定与「提供者 × 某形状」的几何采样无关（后者在
//! `crate::provider`），是 provider 对的**共用判定**。独立成族后，`pair_shaped.rs` 里
//! 「哪一类形状对由哪一段管」一眼可见。
//!
//! ⚠️ 两处与父模块不同的写法（都只为过门禁，语义零变化）：本文件**不用 glob 导入**
//! （`glob-gate` 对**新文件**零基线）⇒ 显式 `use` 等价的那几个名字；两个函数由私有
//! `fn` 改成 `pub(super) fn`（子模块的私有项父模块看不见，而调用方 `provider_pair`
//! 就在父模块里）。另：`InteropContact` 原文即全路径书写，故无需导入。
use crate::{ContactPoint, CLOSING_MIN};
use vxl_phys_core::Vec3;

/// 法向量化键（0.1 精度）：同面法线落进同一小组（确定性、组数 ≤ 10）。
fn quant_normal(v: Vec3) -> (i32, i32, i32) {
    (
        (v.x * 10.0).round() as i32,
        (v.y * 10.0).round() as i32,
        (v.z * 10.0).round() as i32,
    )
}

/// 从提供者候选里选**主导接触面**并给出带符号法线；`None` = 无可用面（整对结束）。
///
/// 选择次序（2026-09-15 修复，取代旧的"点数最多、并列取最深"单一规则）：
/// ① 有**闭合速度**的面优先（取最大者）；② 否则只考虑**带面心样本**的组；
/// ③ 组内仍按点数最多、并列取最深。理由与实测见 `provider_pair` 处注。
pub(super) fn pick_dominant_normal(
    buf: &[vxl_phys_core::interop::InteropContact],
    vrel: Vec3,
    sgn: f32,
) -> Option<((i32, i32, i32), Vec3)> {
    // 小组统计（确定性；组数 ≤ 10）
    pub(crate) struct Group {
        pub(crate) key: (i32, i32, i32),
        pub(crate) count: usize,
        pub(crate) deepest: f32,
        pub(crate) closing: f32,
        pub(crate) has_center: bool,
    }
    let mut groups: Vec<Group> = Vec::with_capacity(8);
    for c in buf {
        let key = quant_normal(c.normal);
        let idx = match groups.iter().position(|g| g.key == key) {
            Some(i) => i,
            None => {
                groups.push(Group {
                    key,
                    count: 0,
                    deepest: f32::NEG_INFINITY,
                    closing: f32::NEG_INFINITY,
                    has_center: false,
                });
                groups.len() - 1
            }
        };
        let g = &mut groups[idx];
        g.count += 1;
        g.deepest = g.deepest.max(c.depth);
        g.closing = g.closing.max(-(c.normal * sgn).dot(vrel));
        if c.feature % 16 == 0 {
            g.has_center = true;
        }
    }
    let closing_group = groups
        .iter()
        .filter(|g| g.closing > CLOSING_MIN)
        .max_by(|x, y| x.closing.total_cmp(&y.closing));
    let pick = closing_group.or_else(|| {
        let with_center: Vec<&Group> = groups.iter().filter(|g| g.has_center).collect();
        let pool: Vec<&Group> = if with_center.is_empty() {
            groups.iter().collect()
        } else {
            with_center
        };
        pool.into_iter()
            .max_by(|x, y| x.count.cmp(&y.count).then(x.deepest.total_cmp(&y.deepest)))
    });
    pick.map(|g| {
        (
            g.key,
            buf.iter()
                .find(|c| quant_normal(c.normal) == g.key)
                .map(|c| c.normal * sgn)
                .unwrap_or(Vec3::Y),
        )
    })
}

/// **选点：四角优先，面心补位**（2026-09-22，P10 修复）。
/// 提供者每面发 5 点：`feature % 16 == 0` = **面心**，1..4 = **四角**；而流形只有
/// 4 槽。旧的 `take(4)` 按**生成序**取 ⇒ 恰好丢掉**第 4 个角** ⇒ 接触力偶不对称
/// ⇒ 每 tick 注入净力矩 ⇒ **静置单盒持续自旋**（实测 `|ω| ≈ 1.6 rad/s`、永不如入睡；
/// 见 `OPEN-PROBLEMS.md` P10 的逐 tick 轨迹：补丁恰为「中心 + 三角」、缺 (+x,+z)）。
/// ⇒ 角点定义力偶、面心只是冗余：**先把角点取满**，面心仅在角点不足时补位。
/// 稳定性：`sort_by_key` 是稳定排序 ⇒ 同类内保持生成序（确定性不变）。
pub(super) fn four_corner_points(
    buf: &[vxl_phys_core::interop::InteropContact],
    best_key: (i32, i32, i32),
) -> Vec<ContactPoint> {
    let mut group_pts: Vec<&vxl_phys_core::interop::InteropContact> = buf
        .iter()
        .filter(|c| quant_normal(c.normal) == best_key)
        .collect();
    if group_pts.len() > 4 {
        group_pts.sort_by_key(|c| u32::from(c.feature % 16 == 0));
    }
    group_pts
        .iter()
        .take(4)
        .map(|c| ContactPoint {
            point: c.point,
            depth: c.depth,
            feature: c.feature,
        })
        .collect()
}
