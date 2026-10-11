//! **运行期 NaN 隔离的判据**（安全审计 F-08 第 3 点）。
//!
//! 审计读数：一具 NaN 速度的体放进 5 箱堆 ⇒ **5 具全污染**；`dt`/`gravity` 为 NaN ⇒ 全世界
//! 7/7 NaN（那两条已在门面构造时收口，见 `tests/config_entry_guard.rs`）。本文件守**运行期**
//! 那一半：状态在跑动中被写脏（`set_linvel(NaN)`、直接改 `bodies` 数组）时只冻结该体、
//! **不扩散、不改 `inv_mass`**（不把它变成隐形墙），且跑完 step 后 `health().nan_bodies == 0`。
//!
//! 口径说明（判据只钉**稳定可观测**的那几条）：隔离发生在**子步开头**，所以同一 tick 的物理
//! 会在之后照常推进该体（速度/`awake` 不是稳定读数）；稳定读数 = 全世界有限 + NaN 计数为 0 +
//! `inv_mass` 未变 + 脏姿态被有限化。
//!
//! 四条：① 一具脏体不污染邻居；② `inv_mass` 不变（不是隐形墙）、`nan_bodies` 归零；
//! ③ 给回有限速度 ⇒ 该体照常参与（不是永久隔离）；④ 位置/姿态脏同样被有限化。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 5 箱（每箱 0.25 m、间距 0.5 m，互不接触——污染只能经由世界状态传播）。
fn stack() -> (World, Vec<usize>) {
    let mut w = World::new(PhysConfig::default());
    let mut ids = Vec::new();
    for k in 0..5 {
        ids.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.25),
            },
            Vec3::new(0.0, 0.25 + 0.5 * k as f32, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize);
    }
    (w, ids)
}

fn all_finite(w: &World) -> bool {
    (0..w.bodies.len()).all(|i| {
        let p = w.bodies.position[i];
        let v = w.bodies.linvel[i];
        let a = w.bodies.angvel(i);
        let q = w.bodies.rot(i);
        p.is_finite()
            && v.is_finite()
            && a.is_finite()
            && q.x.is_finite()
            && q.y.is_finite()
            && q.z.is_finite()
            && q.w.is_finite()
    })
}

/// ① + ②：一具脏体不扩散；它自己也不被变成"隐形墙"；跑完 step 后 NaN 计数归零。
#[test]
fn one_dirty_body_does_not_pollute_the_stack() {
    let (mut w, ids) = stack();
    w.bodies.set_linvel(ids[2], Vec3::new(f32::NAN, 0.0, 0.0));
    w.bodies.set_angvel(ids[2], Vec3::new(0.0, f32::NAN, 0.0));
    for _ in 0..60 {
        w.step();
    }
    assert!(all_finite(&w), "隔离没挡住污染：世界里出现了非有限状态");
    assert_eq!(
        w.health().nan_bodies,
        0,
        "跑过 step 之后 nan_bodies 应当恒为 0（隔离在子步开头执行）"
    );
    assert!(
        w.bodies.inv_mass[ids[2]] > 0.0,
        "不得把它变成隐形墙（inv_mass 不动）"
    );
    assert!(
        (0..5).any(|k| k != 2 && w.bodies.linvel[ids[k]].y < -0.5),
        "邻居应当照常下落（隔离不能把全世界冻住）"
    );
}

/// ③ **不是永久隔离**：脏体被有限化之后照常参与（给回有限速度即可看到）。
#[test]
fn dirty_body_resumes_after_finitize() {
    let (mut w, ids) = stack();
    w.bodies.set_linvel(ids[2], Vec3::new(f32::NAN, 0.0, 0.0));
    w.step();
    assert!(all_finite(&w));
    w.bodies.set_linvel(ids[2], Vec3::new(1.0, 0.0, 0.0));
    w.step();
    let v = w.bodies.linvel[ids[2]];
    assert!(
        v.x.is_finite() && v.x > 0.5,
        "恢复后应带着给的横向速度继续跑（实际 {v:?}）"
    );
}

/// ④ 位置/姿态脏：被有限化（不飞到 Inf），且不扩散。
#[test]
fn dirty_pose_is_finitized_without_spreading() {
    let (mut w, ids) = stack();
    w.bodies.position[ids[1]] = Vec3::new(f32::NAN, 1.0, f32::INFINITY);
    w.bodies.set_rot(ids[1], Quat::new(f32::NAN, 0.0, 0.0, 1.0));
    w.step();
    assert!(all_finite(&w), "脏姿态/位置不得扩散");
    let p = w.bodies.position[ids[1]];
    assert!(
        p.x.abs() < 0.5 && p.z.abs() < 0.5,
        "非有限分量应被钳成 0（有限化）后至多被物理推一点点（实际 {p:?}）"
    );
    let q = w.bodies.rot(ids[1]);
    let n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    assert!(
        (n - 1.0).abs() < 1e-3,
        "脏四元数应变成单位四元数（实际范数 {n}）"
    );
}
