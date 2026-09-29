//! **两类格表探针（GPU；`PLAN-gpu.md` §26.1 第 1 步）**：在**真适配器**上跑两遍四入口
//! （类 0 = 流体 `[0, n_fluid)`、类 1 = 边界 `[n_fluid, n)`），产出**两张表 + 共享 `items`**。
//!
//! **判据 = 与 CPU 计数排序逐位同表**（两张表 + `items` + `overflow == 0`）：CPU 侧用与
//! `grid.wgsl` 逐字同式的分箱（一次减、一次乘、一次 floor、同钳位顺序）现算，两侧同源同刻。
//!
//! 场景：**晶格 + 边界层**（两族在 y 上交错 ⇒ 必然出现**混装格** —— 正是"标签判断会失效"
//! 的那些格；本探针的价值就在这些格上：两块方案必须与平铺逐条一致）。
//!
//! 用法：`cargo run -p vxl-phys-gpu --release --example gpu_two_class_probe`
//! （可选参数：适配器序号，默认 0）。适配器缺席 ⇒ 打印 SKIP 后正常退出（CI 上不跑 GPU）。
use vxl_phys_gpu::grid::{grid_two_class_on_adapter, GridInputs, GridParams};

/// **场景**：流体 10×10×10 + 边界 10×10×4（边界层压在流体底部两层之间 ⇒ 有混装格）。
/// 返回 `(位置扁平数组, 流体粒数, 箱参数)`。
fn scene() -> (Vec<f32>, u32, GridParams) {
    let (nx, ny, nz) = (10usize, 10usize, 10usize);
    let sp = 0.1f32;
    let mut pos_flat: Vec<f32> = Vec::new();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                pos_flat.extend_from_slice(&[ix as f32 * sp, iy as f32 * sp, iz as f32 * sp]);
            }
        }
    }
    let n_fluid = (nx * ny * nz) as u32;
    for iz in 0..nz {
        for iy in 0..4usize {
            for ix in 0..nx {
                // y = −0.05, −0.15, −0.25, −0.35（与流体底两层交错）
                let p = [ix as f32 * sp, -(iy as f32) * sp - 0.05, iz as f32 * sp];
                pos_flat.extend_from_slice(&p);
            }
        }
    }
    let np = (pos_flat.len() / 3) as u32;
    let params = GridParams {
        gmin: [-0.2, -0.5, -0.2],
        inv: 16.0 / 1.4, // 箱边长 1.4 ⇒ 格边 1.4/16
        nx: 16,
        ny: 16,
        nz: 16,
        n: np,
        total: 16 * 16 * 16,
        cap: 512,
        n_fluid,
        class_lo: 0,
    };
    (pos_flat, n_fluid, params)
}

/// 分箱（与 `grid.wgsl::axis_bin` + 线性化**逐字同式**）。
fn bin_of(p: [f32; 3], params: &GridParams) -> u32 {
    let ax = |o: f32, v: f32, n: u32| -> u32 {
        (((v - o) * params.inv).floor().max(0.0) as u32).min(n - 1)
    };
    let idx = (ax(params.gmin[0], p[0], params.nx) * params.ny
        + ax(params.gmin[1], p[1], params.ny))
        * params.nz
        + ax(params.gmin[2], p[2], params.nz);
    idx.min(params.total - 1)
}

/// CPU 计数排序（对 `[lo, hi)`）：返回 `(每格起点, 按格分组且格内按粒子索引升序的置换)`。
fn count_sort(pos_flat: &[f32], lo: usize, hi: usize, params: &GridParams) -> (Vec<u32>, Vec<u32>) {
    let total = params.total as usize;
    let pt = |i: usize| [pos_flat[i * 3], pos_flat[i * 3 + 1], pos_flat[i * 3 + 2]];
    let mut counts = vec![0u32; total + 1];
    for i in lo..hi {
        counts[bin_of(pt(i), params) as usize + 1] += 1;
    }
    for c in 0..total {
        counts[c + 1] += counts[c];
    }
    let start = counts.clone();
    let mut cur = counts;
    let mut perm = vec![0u32; hi - lo];
    for i in lo..hi {
        let c = bin_of(pt(i), params) as usize;
        perm[cur[c] as usize] = i as u32;
        cur[c] += 1;
    }
    (start, perm)
}

/// 逐项比较两条表/序列，返回 `(是否相同, 差项数)`。
fn diff_of(a: &[u32], b: &[u32]) -> (bool, usize) {
    let d = a.iter().zip(b).filter(|(x, y)| x != y).count();
    (d == 0 && a.len() == b.len(), d)
}

/// 三件对拍 + 读数打印（`main` 的行数棘轮 ⇒ 抽出来）。
fn report(out: &vxl_phys_gpu::grid::GridOut, pos_flat: &[f32], params: &GridParams, n_fluid: u32) {
    let np = params.n as usize;
    let (fstart_w, perm_f) = count_sort(pos_flat, 0, n_fluid as usize, params);
    let (bstart_w, perm_b) = count_sort(pos_flat, n_fluid as usize, np, params);
    let mut items_w = vec![0u32; np];
    items_w[..n_fluid as usize].copy_from_slice(&perm_f);
    items_w[n_fluid as usize..].copy_from_slice(&perm_b);
    let (t_f, d_f) = diff_of(&out.start, &fstart_w);
    let (t_b, d_b) = diff_of(&out.start2, &bstart_w);
    let (t_i, d_i) = diff_of(&out.items, &items_w);
    let mut mixed = 0usize;
    for c in 0..params.total as usize {
        if fstart_w[c + 1] > fstart_w[c] && bstart_w[c + 1] > bstart_w[c] {
            mixed += 1; // 同时含流体与边界的格
        }
    }
    println!(
        "  类 0 表：{}（差 {d_f} 项）| 类 1 表：{}（差 {d_b} 项）| items：{}（差 {d_i} 项）| 混装格 {mixed}",
        if t_f { "逐位相同 ✅" } else { "**不同 ❌**" },
        if t_b { "逐位相同 ✅" } else { "**不同 ❌**" },
        if t_i { "逐位相同 ✅" } else { "**不同 ❌**" },
    );
    assert_eq!(
        out.overflow, 0,
        "overflow 非 0 ⇒ 有格超 cap（表不再与 CPU 同表）"
    );
    assert!(t_f, "类 0（流体）表与 CPU 不一致（{d_f} 项）");
    assert!(t_b, "类 1（边界）表与 CPU 不一致（{d_b} 项）");
    assert!(t_i, "items（全局槽位）与 CPU 不一致（{d_i} 项）");
    println!("  ⇒ ✅ 两类档三件（两张表 + items）与 CPU 计数排序**逐位同表**。");
}

fn main() {
    let adapter_index: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let (pos_flat, n_fluid, params) = scene();
    println!(
        "== 两类格表探针：{} 粒（流体 {n_fluid} + 边界 {}）| 箱 dims 16³ = {} 格 ==",
        params.n,
        params.n - n_fluid,
        params.total
    );
    let out = grid_two_class_on_adapter(
        adapter_index,
        &GridInputs {
            pos_flat: &pos_flat,
        },
        params,
        n_fluid,
    );
    if let Some(e) = &out.error {
        println!("  适配器 #{adapter_index} 不可用 ⇒ SKIP（{e}）");
        return;
    }
    println!(
        "  适配器 {} | setup {:.1} ms | 一轮（两遍）{:.2} ms | overflow {}",
        out.adapter, out.setup_ms, out.per_run_ms, out.overflow
    );
    report(&out, &pos_flat, &params, n_fluid);
}
