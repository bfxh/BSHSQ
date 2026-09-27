//! **碎块质量 = 密度 × 体积**（**尺寸扫描**回归门，2026-09-27 立）。
//!
//! 为什么必须扫尺寸、而不是只测一档：缺陷形态是「把按体积算好的**质量**当**密度**传」给
//! `BodySet::push_dynamic`——该入口第 4 参是**密度**，内部 `mass_props` 会**再乘一次体积**
//! ⇒ 实际质量 = `ρ·V²`（**尺寸相关**）。而 **V = 1 的单元立方体恰好掩盖它**：
//! `src/tests.rs` 的 `carve_top_spawns_debris_resting_on_column` 用的正是 1 m 立方体
//! （V = 1 ⇒ `ρ·V² = ρ·V`），且它只断言几何、不断言质量。
//!
//! 实测（修复前，按 `1/inv_mass` 读回质量）：h=0.5（V=1）→ 1000 = 应有的 1000（**巧合**）；
//! h=0.1（V=0.008）→ **0.064** vs 应有的 8.0（**轻 125×**）；h=1.0（V=8）→ **64000**
//! vs 应有的 8000（**重 8×**）。
//!
//! 本门刻意跨过 V=1：V=0.25 / **V=0.5** / **V=1（掩盖档，也必须过）** / **V=8**——
//! 修复前 V=0.25 差 4×、V=0.5 差 2×、V=8 差 8× ⇒ 门会红。
//!
//! 放在**集成测试**（而非 `src/tests.rs` 的内联模块）是有意的：`src/tests.rs` 受
//! `god.gate.json` 的**行数棘轮**管（只准减），给既有文件加 40 行会让 god 门红；
//! 新文件只判阈值（≤800 行 / 最长函数 ≤120 行）——这是仓内加测试的既定通道。
//!
//! 对应修复：`crates/vxl-phys/src/world_body.rs` 的三处 `spawn_box_debris_vel` /
//! `carve_sphere` / `fracture_voronoi` 反解密度（`d = (1e-3 / V).max(density).max(1e-3)`）。

use vxl_phys::{PhysConfig, Shape, Vec3, World};

/// 建「恰好 ext × hgt × ext」的体素柱 → 全部提取为碎块 → 逐碎块核对
/// `质量 == density × 体积`（碎块都是盒 ⇒ `V = 8·hx·hy·hz`）。
fn assert_debris_mass(ext: f32, hgt: f32, density: f32) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 8, 8);
    vol.fill_box(Vec3::ZERO, Vec3::new(ext, hgt, ext));
    w.add_voxel(vol);
    let n = w.spawn_box_debris(0, Vec3::ZERO, Vec3::new(ext, hgt, ext), density);
    assert!(n >= 1, "应产出碎块（ext={ext} hgt={hgt}）");
    let mut checked = 0usize;
    for i in 0..w.bodies.len() {
        if !w.bodies.is_dynamic(i) {
            continue;
        }
        let Shape::Box { half } = w.bodies.shape[i] else {
            panic!("碎块应为盒");
        };
        let v = 8.0 * half.x * half.y * half.z;
        let m_expect = density * v;
        let m_got = 1.0 / w.bodies.inv_mass[i];
        let rel = (m_got - m_expect).abs() / m_expect.max(1e-9);
        assert!(
            rel < 1e-4,
            "碎块质量应为 density×V = {m_expect}（V={v}），实得 {m_got}（相对差 {rel}）\
             —— 偏差随体积放大说明「把质量当密度传了」（实际 ρ·V²）"
        );
        checked += 1;
    }
    assert_eq!(checked, n, "核对的碎块数应与返回值一致");
}

/// V = 0.25（细柱：1 格 × 2 格 × 1 格）——修复前差 **4×**。
#[test]
fn debris_mass_thin_column() {
    assert_debris_mass(0.5, 1.0, 1000.0);
}

/// V = 0.5（薄板：2 格 × 1 格 × 2 格）——修复前差 **2×**。
#[test]
fn debris_mass_thin_slab() {
    assert_debris_mass(1.0, 0.5, 1000.0);
}

/// **V = 1 的掩盖档**：单元立方体上两条口径重合（`ρ·V² = ρ·V`）⇒ 它必须也过，
/// 用它钉住"修法没有把本来正确的那一档弄坏"。
#[test]
fn debris_mass_unit_cube_is_the_masked_case() {
    assert_debris_mass(1.0, 1.0, 1000.0);
}

/// V = 8（大柱：4 格 × 4 格 × 4 格）——修复前差 **8×**。
#[test]
fn debris_mass_large_column() {
    assert_debris_mass(2.0, 2.0, 1000.0);
}

/// 密度也是自由量：同一几何换密度，质量必须**线性**跟着走（0.5× 与 4×）。
#[test]
fn debris_mass_scales_linearly_with_density() {
    assert_debris_mass(1.0, 0.5, 500.0);
    assert_debris_mass(1.0, 0.5, 4000.0);
}
