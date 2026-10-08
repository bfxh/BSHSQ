//! **布片状态继承判据**（`StateBridge` 的②档，2026-10-08）：位置 + 速度 + 逐顶点质量。
//!
//! 判据链（逐条钉）：
//! ① `export_state` 登记位置/速度/质量三段（薄壳均分质量 > 0）；
//! ② 导入速度后**真的生效**：零重力下一 tick 位移 ≈ `v·dt`（`prev` 没对齐会立刻偏离 ——
//!    XPBD 的状态是 `(pos, prev)`，`vel` 只是下一步预测的输入）；
//! ③ 质量导入按 `1/m` 重算 `inv_mass`，而**钉住位保持钉住**（钉住 = `inv_mass == 0`）；
//! ④ 长度不符 ⇒ 整体拒绝且**一字不动**。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{BridgeKind, BridgeState, NoProviders, StateBridge};
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 3×3 格的平铺布（16 顶点、18 三角；y = 0 平面、格距 0.2）—— 与 `cloth_state_bridge.rs` 同夹具。
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

#[test]
fn cloth_state_inheritance_carries_velocity_and_mass() {
    let mut s = sheet();
    let n = s.particle_count();
    assert_eq!(s.kind(), BridgeKind::Mesh);
    let mut st = BridgeState::default();
    s.export_state(&mut st);
    assert_eq!(st.pos.len(), n);
    assert_eq!(st.vel.len(), n, "速度段必须登记（与位置同长）");
    assert_eq!(st.mass.len(), n, "质量段必须登记（薄壳均分）");
    assert!(st.mass.iter().all(|m| *m > 0.0), "质量必须为正");

    let v = Vec3::new(0.3, 0.0, -0.2);
    st.vel.fill(v);
    assert!(s.import_state(&st), "长度相符必须接受");
    let mut got = BridgeState::default();
    s.export_state(&mut got);
    assert_eq!(got.vel, st.vel, "速度必须逐位继承");
    assert_eq!(got.mass, st.mass);

    // ② 速度真的生效：零重力一 tick 位移 ≈ v·dt（`prev` 没对齐就会读出巨大隐式速度）
    let p0 = s.pos[0];
    s.step(1.0 / 60.0, Vec3::ZERO, &NoProviders, 0, &[]);
    let moved = s.pos[0] - p0;
    let want = v * (1.0 / 60.0);
    let err = (moved - want).length() / want.length();
    assert!(
        err < 1e-3,
        "一 tick 位移应 ≈ v·dt：err={err:e}（moved={moved:?}）"
    );
}

#[test]
fn cloth_mass_import_recomputes_inv_mass_and_keeps_pins() {
    let mut s = sheet();
    s.set_pinned(0, true);
    assert_eq!(s.inv_mass[0], 0.0, "钉住 = inv_mass 0");

    let mut st = BridgeState::default();
    s.export_state(&mut st);
    for m in st.mass.iter_mut() {
        *m *= 2.0;
    }
    let before_inv = s.inv_mass[1];
    assert!(s.import_state(&st));
    assert_eq!(s.inv_mass[0], 0.0, "钉住位不许被质量导入解锁");
    assert_eq!(s.mass[1], st.mass[1]);
    let want = before_inv * 0.5; // 质量翻倍 ⇒ 1/m 减半
    let rel = ((s.inv_mass[1] - want) / want).abs();
    assert!(
        rel < 1e-6,
        "inv_mass 应 = 1/m：rel={rel:e}（得 {}）",
        s.inv_mass[1]
    );

    // ④ 长度不符 ⇒ 整体拒绝且一字不动
    let mut bad = BridgeState::default();
    s.export_state(&mut bad);
    bad.vel.pop();
    let mut before = BridgeState::default();
    s.export_state(&mut before);
    assert!(!s.import_state(&bad), "长度不符必须拒绝");
    let mut after = BridgeState::default();
    s.export_state(&mut after);
    assert_eq!(before, after, "拒绝的导入不许改动任何一位");
}
