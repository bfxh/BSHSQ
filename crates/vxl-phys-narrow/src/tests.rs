//! tests：从 `lib.rs` 拆出的单元测试（纯搬移 + 去一层缩进）。
//!
//! 2026-10-05 拆：本文件原 836 行（全仓仅剩的两个 >800 行文件之一）⇒ 20 个用例按域分到
//! `tests/` 三个子模块（`shapes` / `heightfield` / `sat`），共享夹具 `manifolds_for` 留在本文件。

use super::*;
use vxl_phys_core::{BodySet, SerialJobSystem};

fn manifolds_for(b: &BodySet, hf: &[HeightField]) -> Vec<Manifold> {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 全对暴力（测试用）。
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        b,
        &pairs,
        hf,
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    out
}

mod heightfield;
mod sat;
mod shapes;
