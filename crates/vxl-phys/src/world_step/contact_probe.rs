//! **接触点 churn 探针**（M1 只读仪表）：记录每个子步的接触点**身份集合**，回答
//! 「一 tick 之内有多少接触点是**新出现**的」——这是"复用接触集"唯一丢掉的量
//! （深度/锚点每子步都会按当前位姿重算，见 `build_constraint`）。
//!
//! 关闭时每子步只多一次原子读，零分配。
use std::cell::RefCell;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};

use vxl_phys_narrow::{ContactPoints, Manifold};

/// 一次采样：`(tick, substep, 本子步的点身份集合)`。
pub type Sample = (u64, u32, Vec<u64>);

thread_local! {
    #[allow(clippy::missing_const_for_thread_local)]
    static ON: AtomicBool = const { AtomicBool::new(false) };
    #[allow(clippy::missing_const_for_thread_local)]
    static BUF: RefCell<Vec<Sample>> = const { RefCell::new(Vec::new()) };
}

/// 打开/关闭探针（关闭会清空缓冲）。
pub fn enable(on: bool) {
    ON.with(|c| c.store(on, Ordering::Relaxed));
    if !on {
        BUF.with(|b| b.borrow_mut().clear());
    }
}

/// 取出并清空采样。
pub fn take() -> Vec<Sample> {
    BUF.with(|b| std::mem::take(&mut *b.borrow_mut()))
}

/// 每个子步末由 `World::step` 调用（探针关时只多一次原子读）。
pub(crate) fn on_substep(tick: u64, substep: u32, manifolds: &[Manifold]) {
    if !ON.with(|c| c.load(Ordering::Relaxed)) {
        return;
    }
    let mut ids: BTreeSet<u64> = BTreeSet::new();
    for m in manifolds {
        let pts: &ContactPoints = &m.points;
        for p in pts.iter() {
            // 点的身份 = (体对, 特征 ID 低 20 位)——刻意**不含几何**：
            // "新点"问的是"这个特征是否第一次出现在本 tick"，不是"它挪了多少"。
            let (a, b) = (m.a.min(m.b) as u64, m.a.max(m.b) as u64);
            ids.insert((a << 44) | (b << 24) | (p.feature & 0x000F_FFFF) as u64);
        }
    }
    let v: Vec<u64> = ids.into_iter().collect();
    BUF.with(|buf| {
        let mut b = buf.borrow_mut();
        if b.len() < 100_000 {
            b.push((tick, substep, v));
        }
    });
}
