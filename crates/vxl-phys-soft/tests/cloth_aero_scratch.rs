//! **气动暂存复用判据**（切片 T4 收尾的**欠账已还**）：`apply_aero` 的速度快照走 `aero.v0`
//! **复用缓冲**，而不是每子步一次局部 `vel.clone()`。
//!
//! **为什么单独一个文件**：① 它测的是**实现细节**（缓冲是不是复用），与 `cloth_aero.rs` 里
//! "气动力学对不对"是两件事；② god 门对**既有**测试文件管"只准减"（把这条塞进
//! `cloth_aero.rs` 会让它 198 → 235 行且最长函数不降 ⇒ 双红）。
//!
//! **判据 = 地址稳定**（§12：**"优化在不在跑"必须能直读**，别用耗时反推 —— 判例是
//! `&&` 短路让常驻从未生效，而症状只是"快得少"）：连跑多 tick 后 `v0` 的底层指针**不变**
//! ⇒ 复用（而非重新分配）。
use vxl_phys_aero::AeroConfig;
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::cloth_aero::ClothAero;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const WIND: f32 = 10.0;

/// 平铺网格（`2×2` 格 ⇒ 9 粒 / 8 三角）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// 自由平铺布片（不钉、单子步）+ 风沿 `+Y`。
fn free_sheet() -> ClothSheet {
    let (pts, tris) = plate(2, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.substeps = 1;
    s.aero = ClothAero {
        enabled: true,
        cfg: AeroConfig {
            wind: [0.0, WIND, 0.0],
            ..AeroConfig::default()
        },
        ..ClothAero::default()
    };
    s
}

#[test]
fn aero_scratch_is_reused_not_reallocated() {
    let mut s = free_sheet();
    s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let n = s.particle_count();
    assert_eq!(
        s.aero.v0.len(),
        n,
        "快照暂存该被填成粒子数（= 复用路径真的走了）"
    );
    let p0 = s.aero.v0.as_ptr();
    for _ in 0..16 {
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
    println!(
        "[欠账已还·暂存复用] 17 tick 后 len={} cap={} 指针{}",
        s.aero.v0.len(),
        s.aero.v0.capacity(),
        if s.aero.v0.as_ptr() == p0 {
            "不变 ✅（复用）"
        } else {
            "**变了 ❌（重分配）**"
        }
    );
    assert_eq!(
        s.aero.v0.as_ptr(),
        p0,
        "快照暂存的底层地址变了 ⇒ 每子步又在重新分配（欠账没还上）"
    );
}
