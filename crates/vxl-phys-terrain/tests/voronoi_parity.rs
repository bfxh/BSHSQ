//! **快速销毁：Voronoi 归属图 vs 朴素双重扫描**（2026-10-09）。
//!
//! 判据：
//! ① **逐位一致**：新实现（一遍归属图 + 按归属贪心）与旧算法（逐种子 × 逐格 × 再逐种子）
//!    在同一输入上给出**完全相同**的 `(种子序号, 盒列表)`；陪测把旧算法**照抄在测试里**
//!    （它已从生产代码移除，这里当**陪测**用）。端到端证据另有一条：`m3_collapse` 金样哈希
//!    （覆盖整条破坏管线）必须不变。
//! ② **成本读数**（只报数不判）：按种子数扫，打印两版耗时与加速比。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::Vec3;
use vxl_phys_terrain::voxel::VoxelVolume;

/// 旧算法（**照抄自 2026-10-09 之前的 `fracture_voronoi`**）：逐种子 × 逐格 × 再逐种子。
fn naive_fracture(
    v: &mut VoxelVolume,
    min: Vec3,
    max: Vec3,
    seeds: &[Vec3],
) -> Vec<(usize, Vec<(Vec3, Vec3)>)> {
    let (origin, step) = (v.origin(), v.step());
    let mut out = Vec::new();
    for (si, &seed) in seeds.iter().enumerate() {
        let boxes = v.extract_where(min, max, |ix, iy, iz| {
            let c = origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * step;
            let d_me = (c - seed).length_squared();
            // 并列取序号小者：只有「更近」或「并列且序号更小」才归我
            for (sj, &other) in seeds.iter().enumerate() {
                if sj == si {
                    continue;
                }
                let d_o = (c - other).length_squared();
                if d_o < d_me || (d_o == d_me && sj < si) {
                    return false;
                }
            }
            true
        });
        if !boxes.is_empty() {
            out.push((si, boxes));
        }
    }
    out
}

/// 场景：`side³` 实心块 + 若干"挖空"格（让归属图必须跳过空格）+ `seeds` 个抖动种子。
fn scene(side: u32, step: f32, n_seeds: usize) -> (VoxelVolume, Vec3, Vec3, Vec<Vec3>) {
    let ext = side as f32 * step;
    let mut v = VoxelVolume::new(Vec3::ZERO, step, side, side, side);
    v.fill_box(Vec3::ZERO, Vec3::splat(ext));
    // 每隔 5 格挖一个洞（确定性；不吃掉边界）
    for i in 1..side - 1 {
        if i % 5 == 0 {
            v.set(i, i, i, false);
            v.set(i, side - 1 - i, i, false);
        }
    }
    let (min, max) = (Vec3::ZERO, Vec3::splat(ext));
    let seeds = VoxelVolume::seeds_jittered(min, max, n_seeds, 0.7);
    (v, min, max, seeds)
}

fn fresh(side: u32, step: f32, n_seeds: usize) -> (VoxelVolume, Vec3, Vec3, Vec<Vec3>) {
    scene(side, step, n_seeds)
}

#[test]
fn owner_map_matches_the_naive_double_scan_bitwise() {
    for (side, step, n_seeds) in [(10u32, 0.5f32, 8usize), (16, 0.25, 64), (12, 0.5, 200)] {
        let (mut a, min, max, seeds) = scene(side, step, n_seeds);
        let (mut b, min_b, max_b, seeds_b) = fresh(side, step, n_seeds);
        assert_eq!((min, max), (min_b, max_b));
        assert_eq!(seeds, seeds_b);
        let got = a.fracture_voronoi(min, max, &seeds);
        let want = naive_fracture(&mut b, min_b, max_b, &seeds_b);
        assert_eq!(got.len(), want.len(), "非空种子数应相同（side={side}）");
        assert_eq!(
            got, want,
            "划分必须逐位一致（side={side}, seeds={n_seeds}）"
        );
        assert_eq!(a.filled_count(), b.filled_count(), "消费的格数应相同");
    }
}

#[test]
fn voronoi_cost_sweep_report_only() {
    // 只报数不判（时间类断言在 CI 上不稳）；**扫种子数**看两版的复杂度差异。
    let (side, step) = (24u32, 0.25f32);
    for n_seeds in [16usize, 32, 64, 128] {
        let (mut a, min, max, seeds) = fresh(side, step, n_seeds);
        let t0 = std::time::Instant::now();
        let ra = a.fracture_voronoi(min, max, &seeds);
        let new_ms = t0.elapsed().as_secs_f64() * 1e3;
        let (mut b, min_b, max_b, seeds_b) = fresh(side, step, n_seeds);
        let t1 = std::time::Instant::now();
        let rb = naive_fracture(&mut b, min_b, max_b, &seeds_b);
        let old_ms = t1.elapsed().as_secs_f64() * 1e3;
        assert_eq!(ra, rb, "读数场景也必须逐位一致");
        println!(
            "seeds={n_seeds:4}（{side}³/{step}）：归属图 {new_ms:8.2} ms | 朴素 {old_ms:8.2} ms | 加速 {:.1}×",
            old_ms / new_ms
        );
    }
}
