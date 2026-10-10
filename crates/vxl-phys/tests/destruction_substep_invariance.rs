//! **冲击破坏必须与子步数无关**（2026-10-10 立的回归判据）。
//!
//! 为什么单列一条：`impacts` 旧实现**每子步 `clear` 重记** ⇒ `step()` 之后拿到的是
//! **最后一个子步**的快照；而一记快撞**通常在第 1 个子步就被求解器解掉**（法向速度
//! 归零）⇒ 末子步记录的接近速度只剩 ~0.1–0.2 m/s ⇒ **按"每次 `step` 之后调用"设计的
//! 破坏管线（`apply_impact_destruction`）在默认 2 子步下对快撞一块碎块都挖不出**。
//! 实测：同一记 12 m/s 撞击，`substeps=1` 挖出 108 块、默认 `substeps=2` 挖出 **0** 块
//! ——破坏结果居然取决于子步数（而 `substeps` 只是积分细分，不该改变"撞上了没有"）。
//!
//! 修法：`record_impacts` 按 **tick 合并**（首子步清空、其余子步取峰值，同
//! `(provider, body)` 只留接近速度最大的一条）。
//!
//! 本文件钉两条：① **两档子步结果相同**（触发 tick 与碎块数逐值相等）；② 都要真的挖出洞。
//! 场景刻意选"撞击落在**第 1 个子步**"的那种相位——旧实现在这里挖不出、`substeps=1` 却能挖出。
//!
//! 放**集成测试**而非 `src/tests/mesh_carve.rs` 内联：后者受 god 门行数棘轮管，新文件只判阈值。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 关重力，弹体从 `GAP` 处以 12 m/s 沿 +x 撞一块厚体素块；返回 `(首次触发 tick, 碎块总数)`。
fn run(substeps: u32, threshold: f32) -> (usize, usize) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        substeps,
        ..PhysConfig::default()
    });
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(0.0, 0.0, -1.0), 0.1, 24, 20, 20);
    vol.fill_box(Vec3::new(0.0, 0.0, -1.0), Vec3::new(2.4, 2.0, 1.0));
    let vid = w.add_voxel(vol);
    let bullet = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(-1.0, 1.0, 0.0),
        Quat::IDENTITY,
        2000.0,
    );
    w.bodies.linvel[bullet as usize] = Vec3::new(12.0, 0.0, 0.0);
    let mut first = 0usize;
    let mut total = 0usize;
    for t in 1..=120 {
        w.step();
        let n = w.apply_impact_destruction(vid, threshold, 1000.0);
        if n > 0 && first == 0 {
            first = t;
        }
        total += n;
    }
    (first, total)
}

#[test]
fn impact_destruction_is_substep_invariant() {
    let (t1, n1) = run(1, 5.0);
    let (t2, n2) = run(2, 5.0);
    // ① 非空洞：两档都必须挖出洞（旧实现在 2 子步档挖不出、这里会红）
    assert!(n1 > 0, "1 子步档没挖出碎块（场景失效）");
    assert!(
        n2 > 0,
        "2 子步档一块都没挖出——快撞的记录又被最后一个子步覆盖了"
    );
    // ② 子步不变性：触发时刻与碎块数逐值相同
    assert_eq!(
        t2, t1,
        "触发 tick 取决于子步数：{t1}（1 子步） vs {t2}（2 子步）"
    );
    assert_eq!(
        n2, n1,
        "碎块数取决于子步数：{n1}（1 子步） vs {n2}（2 子步）"
    );
    println!("[破坏] 1 子步 {t1} tick/{n1} 块 = 2 子步 {t2} tick/{n2} 块");
}

#[test]
fn sub_threshold_impact_never_carves() {
    // 反向精度：阈值远高于撞击速度 ⇒ 一次都不许触发（不许"沾上就算"）。
    let (_, n) = run(2, 60.0);
    assert_eq!(n, 0, "亚阈冲击挖出了 {n} 块碎块");
}
