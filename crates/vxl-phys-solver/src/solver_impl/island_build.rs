//! **分岛构建**（并查集 + 岛桶）—— `solver_impl` 的**子模块**。
//!
//! **为什么拆出来**：`solver_impl.rs` 顶在 god 门**文件行数棘轮**（只准减），而「1) 并查集分岛 +
//! 2) 岛桶」是独立关注点（被 `solve_phase` 与 `tier_phase::solve_phase_tier` **两条**编排链共用，
//! 卡上求解档正是第二个调用方）。选子模块而非新文件是因为 `solver/lib.rs` 加 `mod` 就顶棘轮，
//! 且**子模块能直接看见父模块私有项**；两个方法写成 `pub(crate)`（父模块看不见子模块私有项）。
use crate::{find_small_root, union_small_root};
use crate::{ImpulseSolver, Island};
use std::collections::HashMap;
use vxl_phys_core::BodySet;
use vxl_phys_narrow::Manifold;

impl ImpulseSolver {
    /// 1) 并查集分岛（直接对流形；双静态对不入岛）。固定规则：小索引为根（确定性）。
    ///    ⚠️ **运动学体进岛**（A 档，`PLAN-kinematic.md` §2）：判据是"**非静态 × 非静态**"。
    ///    代价已实测并被接受：一部平台会把它托着的所有塔并成**一个**岛
    ///    （探针⑦：静态支撑 7 个岛 → 动态/运动学支撑 1 个岛），而本仓并行是岛级。
    ///    缓冲跨帧复用（`self.parent`），避免每帧 20 万级 alloc/fill。
    pub(crate) fn build_union_find(
        &mut self,
        bodies: &BodySet,
        manifolds: &[Manifold],
    ) -> Vec<u32> {
        let n = bodies.len();
        let mut parent = std::mem::take(&mut self.parent);
        parent.clear();
        parent.extend(0..n as u32);
        for m in manifolds {
            let (a, b) = (m.a as usize, m.b as usize);
            if bodies.is_dynamic_or_kinematic(a) && bodies.is_dynamic_or_kinematic(b) {
                union_small_root(&mut parent, m.a, m.b);
            }
        }
        parent
    }

    /// 2) 岛桶（拆成三遍，见下面三个自由函数；`cyc` 门 soft=15 的上限正卡在这条链的分支数上）。
    ///    只有「与清醒体连通」的体参与建岛：
    ///    - 第一遍：清醒动体建岛（睡眠体不建岛 → 全睡眠帧岛构建 ≈ O(查是否为空)）；
    ///    - 第二遍：睡眠动体若其连通分量已被激活（root 已在槽位表）则并入——
    ///      保证「被撞唤醒」的接触对里有沉睡侧的体（求解冲量要施加到它并唤醒）。
    ///
    ///    岛池跨帧复用（Vec 容量保留），全清醒场景（10 万岛）不再逐帧分配。
    ///    ⚠️ 三遍的**遍历序与判定序逐条保持**（§4.14 确定性模式）：拆的是函数，不是算法。
    pub(crate) fn fill_island_buckets(
        &mut self,
        bodies: &BodySet,
        manifolds: &[Manifold],
        parent: &mut [u32],
    ) -> (Vec<Island>, usize) {
        let mut root_slot: HashMap<u32, usize> = HashMap::new();
        let mut pool = std::mem::take(&mut self.island_pool);
        let mut islands_used = 0usize;
        bucket_awake(bodies, parent, &mut pool, &mut root_slot, &mut islands_used);
        if !root_slot.is_empty() {
            absorb_sleeping(bodies, parent, &mut pool, &root_slot);
            assign_manifolds(bodies, manifolds, parent, &mut pool, &root_slot);
        }
        (pool, islands_used)
    }
}

/// 第一遍：清醒动/运动学体建岛（体号升序 ⇒ 岛内体序确定）。
fn bucket_awake(
    bodies: &BodySet,
    parent: &mut [u32],
    pool: &mut Vec<Island>,
    root_slot: &mut HashMap<u32, usize>,
    used: &mut usize,
) {
    for i in 0..bodies.len() {
        if !(bodies.is_dynamic_or_kinematic(i) && bodies.awake[i]) {
            continue;
        }
        let r = find_small_root(parent, i as u32);
        let slot = *root_slot.entry(r).or_insert_with(|| {
            if *used == pool.len() {
                pool.push(Island {
                    bodies: Vec::new(),
                    manifs: Vec::new(),
                });
            }
            let s = *used;
            pool[s].bodies.clear();
            pool[s].manifs.clear();
            *used += 1;
            s
        });
        pool[slot].bodies.push(i as u32);
    }
}

/// 第二遍：睡眠侧并入（其根已被清醒体激活的连通分量）。
fn absorb_sleeping(
    bodies: &BodySet,
    parent: &mut [u32],
    pool: &mut [Island],
    root_slot: &HashMap<u32, usize>,
) {
    for i in 0..bodies.len() {
        if !bodies.is_dynamic(i) || bodies.awake[i] {
            continue;
        }
        let r = find_small_root(parent, i as u32);
        if let Some(&slot) = root_slot.get(&r) {
            pool[slot].bodies.push(i as u32);
        }
    }
}

/// 第三遍：流形按全局序进岛（双静态对不入岛）。
fn assign_manifolds(
    bodies: &BodySet,
    manifolds: &[Manifold],
    parent: &mut [u32],
    pool: &mut [Island],
    root_slot: &HashMap<u32, usize>,
) {
    for (mi, m) in manifolds.iter().enumerate() {
        let (a, b) = (m.a as usize, m.b as usize);
        if !bodies.is_dynamic(a) && !bodies.is_dynamic(b) {
            continue;
        }
        let root = if bodies.is_dynamic(a) {
            find_small_root(parent, m.a)
        } else {
            find_small_root(parent, m.b)
        };
        if let Some(&slot) = root_slot.get(&root) {
            pool[slot].manifs.push(mi);
        }
    }
}
