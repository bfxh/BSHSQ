//! **深穿透恢复判据**（`SURVEY-RIGID-HARD-CASES §6.3①`：把球**埋进体素/外壳**半米 ⇒
//! 允许顶出、**能量不增**、**不抖**；§6.2① 说"体素/SDF 都当**表面**用，深穿透明确不保证"）。
//!
//! 2026-10-14 一轮实测（`examples/deep_penetration_probe.rs`，四条通道 × 三个埋深）把这句话量成了台账：
//!
//! | 通道 | 埋 0.2 m | 埋 0.5 m | 埋 1.0 m |
//! |---|---|---|---|
//! | 体素（1 m 厚地板） | ✅ 顶出 | ✅ 顶出 | ✅ 顶**下**（几何正确：1 m 厚的地板埋 1 m ⇒ 最近面是底面） |
//! | 三角网（无厚薄壳） | ✅ | ❌（平面 provider 的 AABB 零厚 ⇒ **配对都没有**） | ❌ |
//! | 喷溅 SDF 场 | ❌（**法线错**：密度场在材料内部梯度反向） | ❌ | ❌ |
//! | 凸壳 / 盒 | ✅ / ✅ | ❌ 凸壳（**法线错**） | ❌ 凸壳 |
//!
//! **本轮修的**：盒那档的**深度符号**——`prims::sphere_convex_ab` 的"球心在凸体内"分支原来写
//! `depth = radius + max_d`（`max_d` 是最近面的**负**距离）⇒ 只有"埋深 < 半径"时才为正、且**越深报得越浅**
//! ⇒ 埋过半径直接报"无接触" ⇒ **球穿过实心盒**（实测埋 1.0 m 掉到 y = −219）。改成 `radius − max_d`
//! ⇒ 埋 1.0 m 顶出并静置在 **y = +0.4450**（KE → 0）。⚠️ 只改深度，法线口径（`-in_n` = 把球推出去）不动。
//!
//! ⚠️ **仍未做（如实登记，两条都是本轮新量出来的）**：
//! 1. **凸壳/盒的"顶出后落回"会侧滑**：顶出阶段给球带上 `vx ≈ −1.7 m/s` + 自旋（`ω_z ≈ 0.12`）——
//!    一次**纯法向**的顶出不该产生横向速度+自旋（**修前修后都有**，实测同一 tick 的 `vx` 修前 −1.75 /
//!    修后 −1.65 ⇒ **不是本刀引入的**）⇒ 球随后从有限平台边缘滚出去（`x` 单调漂 1.8 m/120 tick）。
//!    ⇒ 另立一条查（接触点在平面上"随体移动"时的切向/转动口径）。
//! 2. **凸壳的深埋（≥0.5 m）法线错**：`(球, 凸壳)` 走 GJK/EPA 那条路，深嵌时报 `n = (−1,0,0)`、
//!    深度 2.85（正确的最小平移 0.95）⇒ 球被横向推走。修法候选：把 `(球, 凸壳)` 也走
//!    `sphere_convex_ab` 的解析路（外壳本就是多面体）。
//! 3. **三角网/喷溅的深埋**：前者是"薄壳无内外判定 + 平面 AABB 零厚"（语义限制）；后者是密度场
//!    在材料内部梯度反向（模型限制）。两条都登记，不在本条判据的范围内（§6.3① 只点"体素/外壳"）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

const R: f32 = 0.45;
const TICKS: usize = 400;

/// 平体素地板：8×2×8 格、边长 0.5、**顶面 y = 1.0**（与 `rest_height_contract.rs` 同配方）。
fn flat_voxel() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

/// 埋球一档：返回 `(末态 y, 末态 KE, 末窗 |v| 均值)`。
fn bury(surface_y: f32, depth: f32, build: impl Fn(&mut World)) -> (f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    build(&mut w);
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu: 0.9 },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.4, surface_y - depth, -0.4),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    let (mut v_tail, mut n) = (0.0f32, 0.0f32);
    for k in 0..TICKS {
        w.step();
        if k + 60 >= TICKS {
            v_tail += w.bodies.linvel[i].length();
            n += 1.0;
        }
    }
    let m_inv = 1.0 / w.bodies.inv_mass[i].max(1e-12);
    let ke = 0.5 * m_inv * w.bodies.linvel[i].length_squared();
    (w.bodies.position[i].y, ke, v_tail / n)
}

/// **① 体素**：埋 0.5 m ⇒ 必须被**顶出**到球面贴住地表（球心 ≈ 顶面 + r）、末态动能≈0、不抖。
/// （体素是本仓四条体积通道里**唯一**深埋本来就正确的一条 ⇒ 这条是它的回归护栏。）
#[test]
fn voxel_buried_sphere_is_ejected() {
    let (y, ke, v) = bury(1.0, 0.5, |w| {
        w.add_voxel(flat_voxel());
    });
    println!(
        "① 体素埋球 0.5 m：末态 y = {y:+.4}（期望 ≈ {:.2}）、KE = {ke:.3e}、末窗 |v| = {v:.5}",
        1.0 + R
    );
    assert!(
        (y - (1.0 + R)).abs() < 0.05,
        "埋进球必须被顶出到地表上（期望 y ≈ {:.2}）：实测 {y:+.4}",
        1.0 + R
    );
    assert!(ke < 1.0, "顶出后应当静止（能量不增）：KE = {ke:.3e}");
    assert!(v < 0.01, "末窗不许抖：|v| = {v}");
}

/// **② 盒（本轮修的那一档）**：埋 1.0 m（= 2.2 倍半径）⇒ 必须顶出并**静置**在盒顶。
/// 修前 `depth = radius + max_d` ⇒ 埋过半径就报"无接触" ⇒ 球**穿过实心盒**掉到 y = −219 ✗；
/// 修后（`radius − max_d`）⇒ `y = +0.4450`、`KE → 0`。
#[test]
fn box_deep_buried_sphere_is_ejected() {
    let (y, ke, v) = bury(0.0, 1.0, |w| {
        w.add_static(
            Shape::Box {
                half: Vec3::new(2.0, 2.0, 2.0),
            },
            Vec3::new(0.0, -2.0, 0.0),
            Quat::IDENTITY,
        );
    });
    println!("② 盒埋球 1.0 m：末态 y = {y:+.4}（期望 ≈ {R:.2}）、KE = {ke:.3e}、末窗 |v| = {v:.5}");
    assert!(
        y > 0.0,
        "球必须被顶出盒顶（穿过实心盒 = 修前的读数 y = −219）：实测 {y:+.4}"
    );
    assert!(ke < 1.0, "顶出后应当静止（能量不增）：KE = {ke:.3e}");
}

/// **③ 外壳（浅埋档的护栏）**：埋 0.2 m ⇒ 顶出并贴住外壳顶面。
/// ⚠️ 深埋档（≥0.5 m）**仍不达标**（GJK/EPA 那条路给错法线）—— 见文件头注的"仍未做 2"，别把这条当全覆盖。
#[test]
fn hull_buried_sphere_is_ejected() {
    let (y, ke, _v) = bury(0.0, 0.2, |w| {
        let mut pts = Vec::new();
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    pts.push(Vec3::new(sx * 2.0, sy * 2.0, sz * 2.0));
                }
            }
        }
        let hull = w.add_hull(pts);
        w.add_static(
            Shape::ConvexHull {
                hull,
                half: Vec3::new(2.0, 2.0, 2.0),
            },
            Vec3::new(0.0, -2.0, 0.0),
            Quat::IDENTITY,
        );
    });
    println!("③ 外壳埋球 0.2 m：末态 y = {y:+.4}（期望 ≈ {R:.2}）、KE = {ke:.3e}");
    assert!(y > 0.4, "浅埋必须被顶出：实测 {y:+.4}");
    assert!(ke < 1.0, "顶出后应当静止：KE = {ke:.3e}");
}

/// **④ 对照：正常静置**（球心 = 顶面 + r，不埋）⇒ 400 tick 内不许漂（`|Δy| < 1 mm`、末窗 |v| ≈ 0）。
/// 这条把"深埋恢复"与"静置"分开钉：深埋那两条的"侧滑"现象**只出现在顶出之后**（见文件头注"仍未做 1"）。
#[test]
fn sphere_rests_on_box_without_drift() {
    let (y, ke, v) = bury(0.0, -R, |w| {
        w.add_static(
            Shape::Box {
                half: Vec3::new(2.0, 2.0, 2.0),
            },
            Vec3::new(0.0, -2.0, 0.0),
            Quat::IDENTITY,
        );
    });
    println!(
        "④ 盒顶静置：末态 y = {y:+.4}（期望 ≈ {:.3}）、KE = {ke:.3e}、末窗 |v| = {v:.5}",
        R
    );
    assert!(
        (y - R).abs() < 1e-3,
        "静置不许漂：实测 y = {y:+.4}（期望 {R:.3}）"
    );
    assert!(v < 1e-3, "静置末窗 |v| 应当 ≈ 0：{v}");
}
