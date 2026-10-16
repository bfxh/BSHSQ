//! 反作用采样：src/tests.rs 拆出的用例（纯搬移，判据/容差/字面量未动）。

use super::still_pose;
use crate::{FluidConfig, FluidSystem};
use vxl_phys_core::{Shape, Vec3};

/// **C2 判据（`PLAN-COUPLING.md` §5 判据④）**：反作用采样是 **dt_sub 时间加权的 tick 平均**。
/// 合成场景：不走物理（物理给不出可解析的线性力），直接喂四个子步的 `bforce`
/// （力线性增长 1→2→3→4，全可精确表示），走与 `step` 相同的"累加 + ÷ 子步数"路径
/// ⇒ `F̄` 必须等于解析平均，且 ≠ 末子步值（**有分辨力**：旧口径会读到末值）。
#[test]
fn reaction_sampling_is_time_weighted() {
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.1, 0.2, -0.1),
        [2, 2, 2],
        0.05,
    );
    let body = (
        0u32,
        Shape::Box {
            half: Vec3::splat(0.15),
        },
        still_pose(Vec3::new(0.0, 0.55, 0.0)),
    );
    assert!(f.set_boundary_particles(&[body]) > 0, "应当造出边界粒子");
    let nb = f.boundary_count() as f32; // 聚合是**段内逐粒求和** ⇒ 账 = nb × 每粒力
                                        // 与 `FluidSystem::step` 同一条账路：清零 → 逐子步 `aggregate_reactions` → ÷ 子步数。
    f.breact.clear();
    for s in 0..4u32 {
        for b in f.bforce[f.n_fluid..].iter_mut() {
            *b = Vec3::new((s + 1) as f32, 0.0, 0.0);
        }
        f.aggregate_reactions();
    }
    let inv = 1.0 / 4.0f32;
    for e in f.breact.iter_mut() {
        e.1 *= inv;
        e.2 *= inv;
    }
    assert_eq!(f.boundary_reactions().len(), 1, "单体场景账上应恰一段");
    let r = &f.boundary_reactions()[0];
    assert_eq!(r.0, 0, "该段属体 0");
    assert_eq!(
        r.1.x,
        nb * 2.5,
        "tick 平均 = nb·(1+2+3+4)/4 = nb·2.5（全整数 × 0.25 ⇒ f32 精确）"
    );
    assert!(r.1.x != nb * 4.0, "不得是末子步快照（旧口径会读到 nb·4）");
    // 力沿 x 且各粒 y/z 各异 ⇒ τ_y/τ_z 非零（有内容）；τ_x = Σ(d_y·f_z − d_z·f_y) ≡ 0 精确。
    assert_eq!(r.2.x, 0.0, "对 x 向对心力矩的 x 分量应精确为零");
    assert!(
        r.2.y != 0.0 || r.2.z != 0.0,
        "力矩账不应为空（用例须非平凡）"
    );
}
