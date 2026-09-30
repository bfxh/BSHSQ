//! **判据：包围盒归约在 `n > 4.19M` 时仍与主机规则逐位相同**（GPU；需适配器）。
//!
//! **为什么需要它**：`bbox` 的归约分派原本是**一维** `⌈n/64⌉` 个工作组 ⇒ `n > 65 535 × 64 = 4 194 240`
//! 时直接越 wgpu 的单维上限（**校验报错**：实测 `gpu_tick_probe -- 180 6 --tank --box=follow` 就炸在
//! `dispatch group size dimension ([102215, 1, 1]) must be less or equal to 65535`）。
//! 修法：分派走 `probe::split_2d`、核里按同一步长展平（`rp.y` = **总组数**）——覆盖仍是"每粒恰好一次"
//! ⇒ 小档逐位不变。
//!
//! **本判据 = 逐位对拍**（不是"不炸"）：展平写错（例如 `rp.y` 仍写一维组数）会得到**偏小的盒**
//! （每个线程的步进跨过了大半数组）⇒ 只有与 `host_box` 逐位对拍才抓得住。
//! **金丝雀**：断言 `⌈n/64⌉ > 65 535` ⇒ 场景**真的**踩到二维展开（否则这条判据空过）。
use vxl_phys_gpu::bbox::{box_on_adapter, host_box};

/// 刚过阈值的规模（`⌈4.30M/64⌉ = 67 188 > 65 535` ⇒ 二维分派）。
const N: usize = 4_300_000;

/// 确定性位置表（自写 LCG，避免引入依赖）：**非对称分布**——若归约少看一段，盒必然变小。
fn positions(n: usize) -> Vec<f32> {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((s >> 33) as f32) / (u32::MAX as f32) // [0,1)
    };
    let mut v: Vec<f32> = Vec::with_capacity(n * 3);
    for _ in 0..n {
        // 三个轴用**不同**的区间与偏置 ⇒ 少看一段会立刻反映到角点上。
        v.push(-10.0 + 20.0 * next());
        v.push(-5.0 + 9.0 * next());
        v.push(-10.0 + 21.0 * next());
    }
    v
}

#[test]
fn bbox_reduction_matches_host_rule_above_the_dispatch_cap() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    // **金丝雀**：必须真的越单维上限，否则这条判据空过（一维档由 `bbox::tests` 那三条守）。
    assert!(
        (N as u32).div_ceil(64) > 65_535,
        "n = {N} 没越单维分派上限 ⇒ 判据空过"
    );
    let pos = positions(N);
    let h = 0.05f32;
    let max_bins = 1usize << 20;
    let (hmin, hmax, hbin, hdims) = host_box(&pos, h, max_bins);
    let out = box_on_adapter(0, &pos, h, max_bins);
    if let Some(e) = out.error {
        println!("（本机无可用适配器 ⇒ 跳过）：{e}");
        return;
    }
    println!(
        "n = {N}（⌈n/64⌉ = {}）箱 dims {:?} / total {}",
        (N as u32).div_ceil(64),
        out.dims,
        out.total
    );
    assert_eq!(
        out.min, hmin,
        "min 角必须逐位相同（适配器：{}）",
        out.adapter
    );
    assert_eq!(out.max, hmax, "max 角必须逐位相同");
    assert_eq!(out.bin, hbin, "bin 必须逐位相同");
    assert_eq!(out.dims, hdims, "dims 必须逐位相同");
    assert_eq!(
        out.total,
        hdims[0] * hdims[1] * hdims[2],
        "total = dims 之积"
    );
    assert_eq!(out.inv, 1.0 / hbin, "inv 与主机同式 ⇒ 逐位相同");
    assert_eq!(out.gpu_dims, hdims, "卡上 setup 的 dims 必须逐位相同");
    assert_eq!(
        out.gpu_total,
        hdims[0] * hdims[1] * hdims[2],
        "卡上 setup 的 total"
    );
}
