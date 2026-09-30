//! **多流体 2a 让位判据（`PLAN-COUPLING.md` §2 C4）**：`covered`（2a 对 2b 的让位集）原是
//! **跨流体单数组**——每个 2b 流体每 tick 的 `refresh_fluid_boundary` 都 clear+重写它
//! ⇒ 多流体时**只有最后一个 2b 流体的覆盖集有效**：浸在流体 0 里的体会被 2a **误施加**
//! 浮力/阻力（与 2b 反作用双重计账）。
//!
//! 判据：在远处多放一个 2b 流体（与体、与流体 0 的粒子 AABB 都不重叠），体的末态必须
//! 与"只有流体 0"**逐位一致**——2a 让位正确时，远处流体的存在不改变体的任何受力
//! （它自己的 2a 循环因 AABB 不相交而空过，2b 反作用各流体独立）。
//! **修前必红**（体多收 ≈ρVg 量级的 2a 浮力）：这条同时是 C4 缺陷的取证。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 水槽：5×5 格地板 + 中心围堰 ⇒ 0.5 m 内腔（与 `fluid_boundary.rs` 门同款）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

fn water(origin: Vec3) -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        origin,
        [8, 8, 8],
        0.05,
    )
}

/// 轻盒（300 kg/m³）浸在流体 0 中被 2b 托住；`with_far_fluid` ⇒ 远处多一个 2b 流体。
/// 每 10 tick 记一次体 y（诊断用：定位两场景的第一个分歧 tick）。
fn run_trace(with_far_fluid: bool, ticks: usize) -> Vec<f32> {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    w.add_fluid_with_boundary_coupling(water(Vec3::new(-0.2, 1.05, -0.2)), &[v]);
    if with_far_fluid {
        // 远处（+3.3, +3.3）第二个 2b 流体：与体、与流体 0 的粒子 AABB 都不重叠。
        w.add_fluid_with_boundary_coupling(water(Vec3::new(3.1, 1.05, 3.1)), &[v]);
    }
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.06),
        },
        Vec3::new(0.0, 1.12, 0.0),
        Quat::IDENTITY,
        300.0,
    ) as usize;
    let mut trace = Vec::new();
    for t in 1..=ticks {
        w.step();
        if t % 10 == 0 {
            trace.push(w.bodies.position[b].y);
        }
    }
    trace
}

#[test]
fn far_fluid_must_not_change_a_covered_body() {
    let (ta, tb) = (run_trace(false, 120), run_trace(true, 120));
    for (i, (a, b)) in ta.iter().zip(tb.iter()).enumerate() {
        if a.to_bits() != b.to_bits() {
            println!(
                "第一个分歧 tick {}：单流体 {a:.9} vs 双流体 {b:.9}（Δ={:+.3e}）",
                (i + 1) * 10,
                b - a
            );
        }
    }
    let (ya, yb) = (ta[ta.len() - 1], tb[tb.len() - 1]);
    println!(
        "体末态 y：单流体 {ya:.6} vs 远处多一个 2b 流体 {yb:.6}（差 {:+.3e}）",
        yb - ya
    );
    // 用例非平凡的自证：体确实被 2b 托在水面附近（沉底 ≈0.x / 被 2a 误施加弹飞 ≈5.9 都
    // 会让本判据退化——修前取证实测后者 y = 5.852）。
    assert!(
        ya > 0.9 && ya < 1.6,
        "单流体场景体应被 2b 托在水面附近（末 y = {ya:.4}）——用例退化则判据无效"
    );
    // 让位正确 ⇒ 两个场景逐位一致（位级断言，不给浮点噪声留门缝）。
    assert_eq!(
        ya.to_bits(),
        yb.to_bits(),
        "远处第二个 2b 流体不该改变覆盖体的末态：{ya:.6} vs {yb:.6}\n\
         差了说明 2a 让位集（covered）被最后一个 2b 流体的 refresh 清掉 ⇒ 2a/2b 双重计账（§2 C4）"
    );
}
