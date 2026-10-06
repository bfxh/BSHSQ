//! **布片 `StateBridge` 往返判据**（表示层②的**第二个**真实现；第一个是流体侧
//! `vxl_phys_fluid::FluidSystem`）。语义逐条对齐流体那条判据
//! （`crates/vxl-phys-fluid/tests/state_bridge_roundtrip.rs`）：
//! ① `kind()` = `Mesh`（`BridgeKind` 那条的定义就是"三角网（布料/软体表面/静态网格）"）；
//! ② 导出按粒子索引序、长度 = 3 × 粒子数；
//! ③ 长度不符 ⇒ **拒绝且一字不动**；④ 长度相符 ⇒ 位置逐位变成给定值、速度归零；
//! ⑤ **往返幂等**：导出 → 扰动 → 导回 ⇒ 与导出时**逐位相同**；
//! ⑥ 导入后 `prev` 也对齐（否则下一次 `step` 会读出巨大隐式速度）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys_core::interop::{BridgeKind, NoProviders, StateBridge};
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 3×3 格的平铺布（16 顶点、18 三角；y = 0 平面、格距 0.2）。
fn sheet() -> ClothSheet {
    let n = 3usize;
    let mut pts = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            pts.push(Vec3::new(ix as f32 * 0.2, 0.0, iz as f32 * 0.2));
        }
    }
    let mut tris = Vec::new();
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard)
}

/// 导出位置的**位图**（逐位比较用）。
fn bits(s: &ClothSheet) -> Vec<u32> {
    let mut out = Vec::new();
    s.export_positions(&mut out);
    out.iter()
        .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
        .collect()
}

#[test]
fn cloth_state_bridge_roundtrip_is_exact() {
    let mut s = sheet();
    // 先让它不是注册初态（重力下落 30 tick）。
    for _ in 0..30 {
        s.step(1.0 / 60.0, Vec3::new(0.0, -9.8, 0.0), &NoProviders, 0, &[]);
    }
    assert_eq!(s.kind(), BridgeKind::Mesh);
    let saved = bits(&s);
    assert_eq!(saved.len(), 3 * s.pos.len(), "导出长度 = 3 × 粒子数");

    // ③ 长度不符 ⇒ 拒绝且一字不动
    let mut wrong = Vec::new();
    s.export_positions(&mut wrong);
    wrong.pop();
    assert!(!s.import_positions(&wrong), "长度不符必须拒绝");
    assert_eq!(bits(&s), saved, "拒绝的导入不该改动任何一位");

    // ④ 导入平移后的位置 ⇒ 位置逐位变成它、速度归零
    let mut src = Vec::new();
    s.export_positions(&mut src);
    let moved: Vec<Vec3> = src.iter().map(|p| *p + Vec3::new(0.1, 0.2, 0.3)).collect();
    assert!(s.import_positions(&moved), "长度相符必须接受");
    let mut got = Vec::new();
    s.export_positions(&mut got);
    assert_eq!(got, moved, "导入后位置应逐值等于给定值");
    assert!(s.vel.iter().all(|v| *v == Vec3::ZERO), "导入后速度必须归零");

    // ⑤ 往返幂等：导回原位置 ⇒ 与导出时逐位相同
    assert!(s.import_positions(&src), "长度相符必须接受");
    assert_eq!(bits(&s), saved, "往返后必须逐位还原");

    // ⑥ `prev` 也对齐：零重力 + 零速度 ⇒ 下一 tick 位移应≈0（`prev` 没对齐就会弹开）
    let before = s.pos[0];
    s.step(1.0 / 60.0, Vec3::ZERO, &NoProviders, 0, &[]);
    let jump = (s.pos[0] - before).length();
    assert!(
        jump < 1e-3,
        "导入后一 tick 位移应≈0（prev 对齐），实测 {jump}"
    );
}
