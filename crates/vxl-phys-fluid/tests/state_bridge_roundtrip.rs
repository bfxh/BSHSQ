//! **`StateBridge` 往返判据**（表示层 ②的**第一个真实现**，2026-10-05）—— `ROUTE.md` §5 把
//! "表示转换/导出"列为独立一层，但该 trait 此前全仓**零实现**；本片给流体的**流体粒子段**接上。
//!
//! 语义（判据逐条钉）：
//! ① `kind()` = `Particle`；
//! ② `export_positions` 按索引序写出流体段，**不含** 2b 边界粒子；
//! ③ `import_positions` 长度不符 ⇒ **拒绝且一字不动**（返回 `false`）；
//! ④ 长度相符 ⇒ 位置**逐位**变成给定值，且**速度归零**（导入不是凭空注入动能）；
//! ⑤ **往返幂等**：导出 → 扰动 → 导回 ⇒ 与导出时**逐位相同**（扫描/回放起步的判据）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{BridgeKind, NoProviders, StateBridge};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// 4³ 水块（间距 0.05、默认核半径）。
fn sys() -> FluidSystem {
    FluidSystem::new(FluidConfig::default(), Vec3::splat(-0.2), [4, 4, 4], 0.05)
}

/// 导出位置的**位图**（逐位比较用）。
fn bits(f: &FluidSystem) -> Vec<u32> {
    let mut out = Vec::new();
    f.export_positions(&mut out);
    out.iter()
        .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
        .collect()
}

#[test]
fn fluid_state_bridge_roundtrip_is_exact() {
    let mut f = sys();
    f.step(1.0 / 60.0, &NoProviders); // 让它不是静止初态
    assert_eq!(f.kind(), BridgeKind::Particle);
    let saved = bits(&f);
    assert_eq!(saved.len(), 3 * f.len(), "导出长度 = 3 × 流体粒子数");

    // ③ 长度不符 ⇒ 拒绝且一字不动
    let mut wrong = Vec::new();
    f.export_positions(&mut wrong);
    wrong.pop();
    assert!(!f.import_positions(&wrong), "长度不符必须拒绝");
    assert_eq!(bits(&f), saved, "拒绝的导入不该改动任何一位");

    // ④ 导入平移后的位置 ⇒ 位置逐位变成它、速度归零
    let mut src = Vec::new();
    f.export_positions(&mut src);
    let moved: Vec<Vec3> = src.iter().map(|p| *p + Vec3::new(0.1, 0.2, 0.3)).collect();
    assert!(f.import_positions(&moved), "长度相符必须接受");
    let mut got = Vec::new();
    f.export_positions(&mut got);
    assert_eq!(got, moved, "导入后位置应逐值等于给定值");
    assert!(
        f.velocities().iter().all(|v| *v == Vec3::ZERO),
        "导入后速度必须归零"
    );

    // ⑤ 往返幂等：导回原位置 ⇒ 与导出时逐位相同
    assert!(f.import_positions(&src), "长度相符必须接受");
    assert_eq!(bits(&f), saved, "往返后必须逐位还原");
}
