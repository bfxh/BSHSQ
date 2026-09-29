//! **`PLAN-gpu.md` §26.1 第 1 步的真场景判据（GPU；需适配器）**：把新落地的**两类格表**
//! 放到**引擎自己的 2b 场景**上、在**真适配器**上跑，与 CPU oracle **逐位对拍**。
//!
//! **为什么另立一条**（现有两条覆盖不到的地方）：
//! - `two_block_order.rs` / `two_block_tables.rs`（纯 CPU、CI 可跑）：证的是**平铺序 == 两块序**
//!   与**表布局语义** —— 但用的是 CPU 现算的表；
//! - `examples/gpu_two_class_probe.rs`（GPU）：证的是**卡上两遍分派**与 CPU 同表 —— 但场景是
//!   **合成晶格**（箱参数是我手填的常数）；
//! - **本条补的正是缝合处**：**真场景的箱参数**（`neighbor_grid()` 的 `min/inv/dims`）+
//!   **引擎的真实粒子分布**下，卡上两张表仍与 CPU 逐位相同、且拼出的每格序列与平铺序逐条相同。
//!   ⇒ 第 2–3 步（核里"每格两段"）动核前，先把"表在真场景上是对的"钉死。
//!
//! ⚠️ **无适配器 ⇒ 跳过**（与既有四条 GPU 判据同款：CI 上不跑）。
//! ⚠️ **绿 ≠ SKIP**：跳过时打印明确标记；跑起来时必须断言"两遍真在跑"（`overflow == 0`、
//! 边界表**非平凡**），否则"空转"也会绿（§26.2 的坑 ①）。
mod support;

use support::{count_sort_range, params_of, scene_2b};
use vxl_phys_gpu::grid::{grid_two_class_on_adapter, GridInputs};

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

#[test]
fn two_class_tables_match_cpu_on_the_real_2b_scene() {
    if !have_adapter() {
        println!("[SKIP] 无可用适配器 ⇒ 本判据不跑（CI 常态）");
        return;
    }
    let f = scene_2b();
    let (apos, _, _, n_fluid) = f.raw_particles();
    let np = apos.len();
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for p in apos {
        pos_flat.extend_from_slice(&[p.x, p.y, p.z]);
    }
    let params = params_of(&f);
    let total = params.total as usize;
    println!(
        "== 真场景两类表判据：{np} 粒（流体 {n_fluid} + 边界 {}）| 箱 {}×{}×{} = {total} 格 ==",
        np - n_fluid,
        params.nx,
        params.ny,
        params.nz
    );

    let out = grid_two_class_on_adapter(
        0,
        &GridInputs {
            pos_flat: &pos_flat,
        },
        params,
        n_fluid as u32,
    );
    assert!(out.error.is_none(), "适配器报错：{:?}", out.error);
    println!(
        "  适配器 {} | 一轮（两遍）{:.2} ms",
        out.adapter, out.per_run_ms
    );

    // —— CPU oracle（同源同刻：同一份位置 + 同一组箱参数）——
    let (fstart_w, perm_f) = count_sort_range(&f, 0, n_fluid);
    let (bstart_w, perm_b) = count_sort_range(&f, n_fluid, np);
    let mut items_w = vec![0u32; np];
    items_w[..n_fluid].copy_from_slice(&perm_f);
    items_w[n_fluid..].copy_from_slice(&perm_b);

    let d_f = out
        .start
        .iter()
        .zip(&fstart_w)
        .filter(|(a, b)| a != b)
        .count();
    let d_b = out
        .start2
        .iter()
        .zip(&bstart_w)
        .filter(|(a, b)| a != b)
        .count();
    let d_i = out
        .items
        .iter()
        .zip(&items_w)
        .filter(|(a, b)| a != b)
        .count();
    // "两遍真在跑"的证据（防空转假绿）：边界表非平凡 + 段里确有边界粒子
    let b_tbl_nontrivial = out.start2.iter().any(|v| *v > 0);
    let seg_ok = (0..total).all(|g| {
        let (a, z) = (out.start[g] as usize, out.start[g + 1] as usize);
        let (c, d) = (out.start2[g] as usize, out.start2[g + 1] as usize);
        a <= z && c <= d && z <= n_fluid && d <= np
    });
    println!(
        "  类 0 表：差 {d_f} 项 | 类 1 表：差 {d_b} 项 | items：差 {d_i} 项 | 边界表非平凡 {} | 段界合法 {} | overflow {}",
        if b_tbl_nontrivial { "✅" } else { "**空**" },
        if seg_ok { "✅" } else { "**越界 ❌**" },
        out.overflow
    );
    assert!(b_tbl_nontrivial, "类 1 表全 0 ⇒ 第二遍空转（§26.2 坑①）");
    assert!(seg_ok, "段界越界：`items` 槽位与类别基址对不上");
    assert_eq!(out.overflow, 0, "有格超 cap ⇒ 表不再与 CPU 同表");
    assert_eq!(d_f, 0, "**真场景**类 0 表与 CPU 不一致（{d_f} 项）");
    assert_eq!(d_b, 0, "**真场景**类 1 表与 CPU 不一致（{d_b} 项）");
    assert_eq!(d_i, 0, "**真场景** items 与 CPU 不一致（{d_i} 项）");
    println!("  ⇒ ✅ 真场景（引擎箱参数 + 真实分布）：卡上两张表 + items 与 CPU 逐位相同。");
}
