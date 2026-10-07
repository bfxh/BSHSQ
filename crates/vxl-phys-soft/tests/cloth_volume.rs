//! **体积 / 气压约束判据**（`SPEC.md` §4.6「体积守恒：Müller 2007 压力约束（气压系数 k）」）。
//!
//! 三条（公式与口径见 `src/cloth/volume.rs` 头注）：
//! ① **气压回弹**：封闭网格被压到 `0.9³` 体积后，开 `k` 应把它拉回 `V0`（±5%）；
//! ② **关断即零影响**：`k ≤ 0`（默认）时位置演化与"没有这个域"**逐位一致**；
//! ③ **k 单调**：同一初始压缩下，`k` 越大末态体积越贴近 `V0`。
//!
//! 扰动取**静态压缩**（顶点整体乘 `s < 1`，再跑 300 tick）：气压腿的恢复量与"无气压"
//! 对照组的差就是它的贡献。
//!
//! ⚠️ 两条实测坑（本轮踩到，别重走）：
//! ⑴ **不要用初速度**扰动：XPBD 是位置式、`write_back` 按"实际位移 / h"重算速度 ⇒
//!   初速度一个子步就被约束吃掉，量不出差异；
//! ⑵ **时间窗要够长**：`damping = 1.0`（无阻尼）⇒ 短期体积在振荡，60 tick 时末态取决于
//!   相位（实测 `k=100` 会**比 `k=0` 更差**：0.257 vs 0.043）；300 tick 才收敛成单调序
//!   （0.206 → 0.108 → 0.013 → 0.000，见 ③ 的读数）。判据取**收敛后**的量。
//!
//! ⚠️ 不要用 `Stiffness::Custom(f32::INFINITY)` 去"关"距离约束：`project_edges` 里
//! `a_tilde * lambda` 在 `λ = 0` 时是 `∞·0 = NaN` ⇒ 整场中毒（本轮实测踩到）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::cloth::volume::mesh_volume;
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 半径 `r` 的正八面体，面绕序**自动定向向外**（法线 · 面重心 ≥ 0 ⇒ 有向体积为正）。
fn octahedron(r: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let pos = vec![
        Vec3::new(r, 0.0, 0.0),
        Vec3::new(-r, 0.0, 0.0),
        Vec3::new(0.0, r, 0.0),
        Vec3::new(0.0, -r, 0.0),
        Vec3::new(0.0, 0.0, r),
        Vec3::new(0.0, 0.0, -r),
    ];
    let raw = [
        [0u32, 2, 4],
        [0, 4, 3],
        [0, 3, 5],
        [0, 5, 2],
        [1, 4, 2],
        [1, 3, 4],
        [1, 5, 3],
        [1, 2, 5],
    ];
    let tris = raw
        .iter()
        .map(|&[a, b, c]| {
            let (pa, pb, pc) = (pos[a as usize], pos[b as usize], pos[c as usize]);
            let n = (pb - pa).cross(pc - pa);
            if n.dot((pa + pb + pc) * (1.0 / 3.0)) < 0.0 {
                [a, c, b]
            } else {
                [a, b, c]
            }
        })
        .collect();
    (pos, tris)
}

/// 封闭气球：顶点整体压到 `squeeze` 倍后交还给求解器。**距离与弯曲都取软档**
/// （`Custom(1.0)` / `bend_compliance = 1.0`）⇒ 网格保持住压缩态；气压腿负责把体积
/// 拉回 `V0`。⚠️ 两条实测坑（本轮踩到）：⑴ 硬距离约束会把正八面体锁成刚体、压缩立刻
/// 抹平；⑵ **弯曲约束默认是 `Soft`（1e-4），比距离约束硬得多** —— 只调距离那一档，
/// 压缩仍会被弯曲腿整个撑回 `V0`（60 tick 就把 0.729 拉回 1.000）。
fn balloon(r: f32, squeeze: f32) -> (ClothSheet, f32) {
    let (pos, tris) = octahedron(r);
    let v0 = mesh_volume(&pos, &tris);
    let mut cs = ClothSheet::new(pos, tris, 1000.0, 0.01, Stiffness::Custom(1.0));
    cs.bend_compliance = 1.0;
    for p in cs.pos.iter_mut() {
        *p *= squeeze;
    }
    (cs, v0)
}

fn run(cs: &mut ClothSheet, ticks: usize) {
    for _ in 0..ticks {
        cs.step(1.0 / 60.0, Vec3::ZERO, &NoProviders, 0, &[]);
    }
}

/// ① 同一压缩下，开大 `k` 的末态体积**比关断更贴近** `V0`（气压腿真的在顶）。
#[test]
fn pressure_reduces_collapse() {
    let (mut off, v0) = balloon(0.5, 0.9);
    let (mut on, _) = balloon(0.5, 0.9);
    println!(
        "压缩后立即：{} ｜ V0 {} ｜ 比值 {:.4}",
        mesh_volume(&off.pos, &off.tris),
        v0,
        mesh_volume(&off.pos, &off.tris) / v0
    );
    on.volume.k = 1e6;
    on.volume.target = v0;
    run(&mut off, 300);
    run(&mut on, 300);
    let e_off = (mesh_volume(&off.pos, &off.tris) - v0).abs() / v0;
    let e_on = (mesh_volume(&on.pos, &on.tris) - v0).abs() / v0;
    println!("压缩后 300 tick：k=0 相对误差 {e_off:.4} ｜ k=1e6 {e_on:.4}");
    assert!(e_on < e_off, "开气压未改善：{e_on} !< {e_off}");
}

/// ② 默认（`k = 0`）与"显式关"逐位一致；且不产生 NaN。
#[test]
fn disabled_is_bitwise_identical_to_explicit_off() {
    let (mut a, _) = balloon(0.5, 0.9);
    let (mut b, _) = balloon(0.5, 0.9);
    b.volume.k = 0.0;
    run(&mut a, 300);
    run(&mut b, 300);
    for i in 0..a.pos.len() {
        assert_eq!(a.pos[i], b.pos[i], "粒子 {i} 不同：默认档被本域碰到了");
        assert!(a.pos[i].is_finite(), "粒子 {i} 出现非有限值");
    }
}

/// ③ 同一压缩下 `k` 越大，末态体积越贴近 `V0`（单调）。
#[test]
fn stiffness_is_monotone() {
    let mut prev = f32::INFINITY;
    for k in [0.0f32, 1e2, 1e4, 1e6] {
        let (mut cs, v0) = balloon(0.5, 0.9);
        cs.volume.k = k;
        cs.volume.target = v0;
        run(&mut cs, 300);
        let err = (mesh_volume(&cs.pos, &cs.tris) - v0).abs() / v0;
        println!("k={k:>8}：末态相对误差 {err:.4}");
        assert!(err.is_finite(), "k={k} 出现非有限残差");
        assert!(err <= prev + 1e-6, "k={k} 的残差 {err} 大于上一档 {prev}");
        prev = err;
    }
}
