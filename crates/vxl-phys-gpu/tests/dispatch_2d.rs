//! 二维分派的**字面量锁步**金丝雀（独立集成测试文件：god 门对既有文件"只准减"）。
//! 判据：核里写死的展平步长必须等于 `probe::WG_X_CAP * 64`——两边漂移会让
//! "一维 ≡ 二维（逐粒索引不变）"的承诺悄悄失效，而且**只在大档上才暴露**。

/// 含展平步长字面量的全部核文件（§28.1 同族：管线四相位 + bbox + broad + narrow）。
const STRIDE_FILES: &[(&str, &str)] = &[
    ("grid", include_str!("../src/grid.wgsl")),
    ("density", include_str!("../src/density.wgsl")),
    ("eos", include_str!("../src/eos.wgsl")),
    ("force", include_str!("../src/force.wgsl")),
    ("integrate", include_str!("../src/integrate.wgsl")),
    ("bbox", include_str!("../src/bbox.wgsl")),
    ("broad", include_str!("../src/broad.wgsl")),
    ("narrow", include_str!("../src/narrow.wgsl")),
];

/// 各核含展平步长字面量；`split_2d` 的形状是"装得下就一维、装不下才二维"。
#[test]
fn dispatch_2d_lockstep() {
    let stride = format!("{}u * 64u", vxl_phys_gpu::probe::WG_X_CAP);
    for (name, src) in STRIDE_FILES {
        assert!(
            src.contains(&stride),
            "{name} 缺二维分派展平步长 `{stride}`（与 probe::WG_X_CAP 漂移了）"
        );
    }
    assert_eq!(vxl_phys_gpu::probe::split_2d(1), (1, 1));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65535), (65535, 1));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65536), (65535, 2));
    assert_eq!(vxl_phys_gpu::probe::split_2d(65535 * 3), (65535, 3));
}
