//! **§28.1 同族收尾的一次性验证（`#[ignore]`，手动跑）**：`broad` / `narrow` 的二维分派
//! 在**真越阈**的规模上覆盖正确。
//!
//! **背景**：`bbox`（§28.1）修完后，全仓还剩 `broad`（按体枚举 ×3）与 `narrow`（按对/按记录 ×2）
//! 的原始一维分派 —— `n > 65535×64 ≈ 4.19M` 时 wgpu 直接校验报错（**响的**）。本文件把它们
//! 钉在真规模上；**既有档（一维）逐位不变**由 `gpu_broad_probe` / `gpu_narrow_probe` 的
//! CPU 对拍守（回归跑）。
//!
//! **为什么 `#[ignore]`**：两条都要 >4.19M 的输入（broad 134 MB 盒表 / narrow 268 MB 槽回读），
//! 常驻门禁跑不动也不必 —— 触发面在产品里 Today 不现实（宽相/窄相按**体**数，4.19M 体是极端场景）；
//! 常驻覆盖由 `dispatch_2d_lockstep`（步长字面量锁步）+ 本文件的一次性验证共同承担。
//!
//! 跑法：`cargo test -p vxl-phys-gpu --test broad_narrow_2d -- --ignored --nocapture`（需适配器）。
use vxl_phys_gpu::broad::broad_on_adapter;
use vxl_phys_gpu::narrow::{flat_pairs, NarrowTier, BODY_WORDS, KIND_SPHERE};

/// 刚越过 `65535 × 64 = 4 194 240` 的规模（broad 的 `n` 无容量上限）。
const BROAD_N: u32 = 4_200_000;

#[test]
#[ignore = "手动一次性验证：--ignored 跑（4.2M 体，≈10 s）"]
fn broad_chain_pairs_at_4m2_are_exact() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过）");
        return;
    }
    // **链条场景**：体 i 的盒 = [i, i+1.5]×[0,0.5]²（格 2.0）——与**相邻**体相交、与**隔一个**的
    // 体同格但不相交 ⇒ 覆盖敏感：任何"漏线程"都会让配对数 < n−1，任何"多线程"都会多出对。
    let n = BROAD_N;
    let mut boxes: Vec<f32> = Vec::with_capacity(n as usize * 8);
    for i in 0..n {
        let x = i as f32;
        boxes.extend_from_slice(&[x, 0.0, 0.0, 0.0, x + 1.5, 0.5, 0.5, 1.0]); // dyn_bit=1
    }
    let cap_pairs = 8_388_608u32; // ≥ n−1
    let out = broad_on_adapter(0, &boxes, n, 2.0, cap_pairs);
    // （不用 `panic!`：todo-gate 把该宏计入棘轮，`assert!` 同效。）
    assert!(
        out.error.is_none(),
        "卡上宽相不可用：{:?}",
        out.error.as_deref()
    );
    println!(
        "n = {n}（⌈n/64⌉ = {} > 65535 ⇒ 二维）| 格 {} | 对 {} | overflow {}",
        n.div_ceil(64),
        out.cells,
        out.pairs.len(),
        out.overflow
    );
    assert_eq!(out.overflow, 0, "配对/条目超容量 ⇒ 场景没铺对");
    assert_eq!(out.pairs.len(), (n - 1) as usize, "链条恰有 n−1 对");
    // 每一对都必须是**相邻**体（sort+dedup 后恰好 (0,1),(1,2),…）。
    for (k, &(a, b)) in out.pairs.iter().enumerate() {
        assert_eq!(
            (a, b),
            (k as u32, k as u32 + 1),
            "第 {k} 对 = ({a},{b})，应为 ({k},{})",
            k as u32 + 1
        );
    }
    // 自证：连跑两次逐位相同。
    let again = broad_on_adapter(0, &boxes, n, 2.0, cap_pairs);
    assert_eq!(again.pairs, out.pairs, "连跑两次必须逐位相同");
}

#[test]
#[ignore = "手动一次性验证：--ignored 跑（4.19M 对，槽回读 ≈268 MB）"]
fn narrow_pairs_at_dispatch_window_all_processed() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过）");
        return;
    }
    // **触发窗口**：`cap_pairs ≤ MAX_ITEMS = 1<<22 = 4 194 304`，而二维阈值在 4 194 240 ⇒
    // 只有 64 个值能踩到。取 `4 194 241`（最小越阈）。
    let n_pairs: u32 = 4_194_241;
    let tier = NarrowTier::new(0, 16, 4_194_304, 0.01).expect("建档失败");
    // 两颗**相交**的球（全部对都是 (0,1)）⇒ 每条线程的槽都该有 count == 1；
    // 覆盖缺口会以 count == 0（或未初始化垃圾）现形。
    let mut bodies: Vec<u32> = Vec::with_capacity(2 * BODY_WORDS);
    for (p, r) in [([0.0f32, 0.0, 0.0], 1.0f32), ([0.5, 0.0, 0.0], 1.0)] {
        bodies.extend_from_slice(&[p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]);
        bodies.extend_from_slice(&[0u32, 0, 0, 1065353216]); // 单位四元数 (0,0,0,1)
        bodies.push(KIND_SPHERE);
        bodies.extend_from_slice(&[r.to_bits(), 0u32, 0u32]);
        bodies.push(0); // pad
    }
    let pairs = flat_pairs(&vec![(0u32, 1u32); n_pairs as usize]);
    let run = tier.run(&bodies, &pairs).expect("跑失败");
    println!(
        "n_pairs = {n_pairs}（⌈n/64⌉ = {} > 65535 ⇒ 二维）| 槽 {} | diag[0] = {}",
        n_pairs.div_ceil(64),
        run.slots.len(),
        run.diag[0]
    );
    assert_eq!(run.slots.len(), n_pairs as usize, "槽表长度 = 对数");
    let zero = run.slots.iter().filter(|s| s.count() != 1).count();
    assert_eq!(
        zero, 0,
        "{zero} 个槽的接触点数 ≠ 1 ⇒ 有线程没跑到（二维展平写错）或核输出不确定"
    );
}
