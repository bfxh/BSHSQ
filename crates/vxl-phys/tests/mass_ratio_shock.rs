//! **高质量比支撑的契约判据**（2026-10-09）：重盒压轻盒时不得静默互穿。
//!
//! 背景（R5）：默认档 2 子步 × 3 迭代下，质量比 r = 30 的两盒叠放会收敛失败——
//! 轻盒被按穿地面、与重盒同位（位置误差恰约一个盒高），而 `nan = false`：错得安静。
//! 修法 = 按岛给 shock 附加扫掠预算（`shock_budget.rs`），只在"质量比大且未收敛"时追加。
//!
//! 判据两条：
//! ① r = 30 必须**站住**（位置误差 < 0.08 m，末态无深穿透）；
//! ② 均匀场景（质量比 1）**逐位不变**——这是"零换代"的可执行形式：把质量比门关掉
//!    （`mass_ratio_shock_min` 抬到无穷）时本测试的第二条会红，说明它确实在守东西。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 两盒叠放：下盒 1000 kg，上盒 1000·r kg，跑 `ticks`。返回（末态位置误差, 最大深度）。
fn stack(r: f32, ticks: usize) -> (f32, f32) {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let lo = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let hi = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 1.5, 0.0),
        Quat::IDENTITY,
        1000.0 * r,
    ) as usize;
    let mut max_depth = 0.0f32;
    for _ in 0..ticks {
        w.step();
        for mf in w.manifolds() {
            for cp in &mf.points {
                max_depth = max_depth.max(cp.depth);
            }
        }
    }
    let err = ((w.bodies.position[lo].y - 0.5).abs()).max((w.bodies.position[hi].y - 1.5).abs());
    (err, max_depth)
}

/// ① 30:1 的"重压轻"必须站住（旧行为：pos_err ≈ 1.000 m、深度 0.20 m）。
#[test]
fn heavy_on_light_ratio_30_does_not_interpenetrate() {
    let (err, depth) = stack(30.0, 600);
    println!("[30:1] 末态位置误差 {err:.4} m，最大深度 {depth:.4} m");
    assert!(
        err < 0.08,
        "上盒该站在下盒上（实测位置误差 {err:.4} m）——静默互穿又回来了"
    );
    assert!(
        depth < 0.08,
        "接触深度该在皮肤带量级（实测 {depth:.4} m）——支撑没收敛"
    );
}

/// ② 均匀场景（质量比 1）逐位不变：自适应预算不得动"本来没事"的岛。
///
/// 判据是**钉死的常量哈希**，不是"两次同构造相等"（后者在确定性引擎里恒成立、
/// 守不住东西）。常量取自 2026-10-09 修法落地时的实测值，且已用"把收敛门短路回
/// `base`"复核过：关掉自适应预算时本哈希不变 ⇒ 均匀岛走的确实是旧路径。
/// 若将来谁让自适应预算在均匀岛上也触发，本测试会红。
#[test]
fn uniform_stack_is_bit_identical_to_frozen_hash() {
    const FROZEN_UNIFORM_HASH: u128 = 0xd017_fe49_21aa_70a1_03fe_3926_9ab7_c56e;
    let hash = uniform_hash();
    println!("[均匀 1:1] 末态哈希 {hash:#034x}");
    assert_eq!(
        hash, FROZEN_UNIFORM_HASH,
        "均匀岛（质量比 1）的轨迹被动了——自适应预算只该在高比未收敛时触发"
    );
}

/// 均匀 3 层盒堆（质量比 1），返回末态哈希。
fn uniform_hash() -> u128 {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    for k in 0..3u32 {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.5 + k as f32, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
    }
    for _ in 0..300 {
        w.step();
    }
    w.state_hash()
}
