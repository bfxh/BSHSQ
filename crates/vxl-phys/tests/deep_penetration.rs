//! **深穿透恢复判据**（`SURVEY-RIGID-HARD-CASES §6.3①`：把球**埋进体素/外壳**半米 ⇒
//! 允许顶出、**能量不增**、**不抖**；§6.2① 说"体素/SDF 都当**表面**用，深穿透明确不保证"）。
//!
//! 2026-10-14 一轮实测（`examples/deep_penetration_probe.rs`，四条通道 × 三个埋深）把这句话量成了台账：
//!
//! | 通道 | 埋 0.2 m | 埋 0.5 m | 埋 1.0 m |
//! |---|---|---|---|
//! | 体素（1 m 厚地板） | ✅ 顶出 | ✅ 顶出 | ⚠️ 顶**下**后落走（`y = −237`：埋 1 m ⇒ 球心恰在**底面**平面上、最近面是底面 —— 是"体素当地表"的**语义**，不是缺陷） |
//! | 三角网（无厚薄壳） | ✅ | ❌（平面 provider 的 AABB 零厚 ⇒ **配对都没有**） | ❌ |
//! | 喷溅 SDF 场 | ❌（**法线错**：密度场在材料内部梯度反向） | ❌ | ❌ |
//! | 凸壳 / 盒 | ✅ / ✅ | ✅ / ✅ | ✅ / ✅ |
//!
//! **2026-10-14 修的（P22，三刀）**：
//!
//! 1. 盒那档的**深度符号**——`prims::sphere_convex_ab` 的"球心在凸体内"分支原来写
//!    `depth = radius + max_d`（`max_d` 是最近面的**负**距离）⇒ 只有"埋深 < 半径"时才为正、且**越深报得越浅**
//!    ⇒ 埋过半径直接报"无接触" ⇒ **球穿过实心盒**（实测埋 1.0 m 掉到 y = −219）。改成 `radius − max_d`
//!    ⇒ 埋 1.0 m 顶出并静置在 **y = +0.4450**（KE → 0）。⚠️ 只改深度，法线口径（`-in_n` = 把球推出去）不动。
//! 2. 凸壳深埋的**最近面解析路**（`hull_sphere.rs`）：`(球, 凸壳)` 原来一律走 GJK/EPA，而 EPA 对光滑支撑
//!    在深嵌时给**最远面**的法线（埋 0.5 m 报 `n = (−1,0,0)`、深度 2.85）⇒ 球被横向推走。修后三档全 ✅。
//! 3. **「球心落在体表面平面（外侧一个浮点噪声量）」这一档的接触口径**（本轮）：`closest − center` 的长度
//!    只剩 `~1e-8` 时，方向被最近点重算的舍入噪声主导 ⇒ 报**斜法线**（盒档 `n ≈ (−0.943,+0.106,−0.314)`，
//!    实测就是"顶出**恰好经过**顶面平面那一 tick"）⇒ 纯法向顶出被塞进 `vx ≈ −2.6 m/s` + 自旋 ⇒ 球随后从
//!    平台边缘滚走（埋 0.2 m 落 `y = −94`、埋 0.5 m 落 `y = −26`）。三处同修（`surface.rs`）：
//!    `prims::sphere_convex_ab` 加"退化小距离 ⇒ 与体内同用最近面口径"（判据 ⑤ + 窄相单测）；
//!    `hull_sphere::deep_exit` 的"体内"判据加**尺寸量级**容差；`hull_pair` 的 EPA 腿对面是**球**时
//!    收敛成**单点**（否则接触点是**壳顶点** ⇒ 力臂错 2.8 m ⇒ 同样的自旋）。修后盒/壳两档在
//!    **μ = 0**（摩擦这条腿关掉）下三档全静置、`max|ω| = 0`。
//!
//! ⚠️ **仍未做（如实登记）**：
//!
//! 1. **三角网/喷溅的深埋**：前者是"薄壳无内外判定 + 平面 AABB 零厚"（语义限制）；后者是密度场
//!    在材料内部梯度反向（模型限制）。两条都登记，不在本条判据的范围内（§6.3① 只点"体素/外壳"）。
//! 2. **浅接触的接触点口径不一致**（本轮顺带量到，未修）：`(球, 凸壳)` 的**浅**接触走 EPA + 顶点带
//!    （接触点取壳顶点），而**深**接触走最近面投影（接触点 = 球心投影）—— 同一对形状两套口径。
//!    浅档当前的读数（`hull_on_cylinder_cap` 等）稳定，故只登记。
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

/// **③ 外壳（三档全判）**：0.2 / 0.5 / 1.0 m 都必须**顶出并贴住外壳顶面**。
/// 修前只有 0.2 m 过：`(球, 凸壳)` 一律走 GJK/EPA，而 EPA 对**光滑支撑**在深嵌时给**最远面**的
/// 法线（埋 0.5 m 报 `n = (−1,0,0)`、深度 2.85 —— 正确的最小平移是 1.45）⇒ 球被横向推走。
/// 修法 = `hull_sphere.rs`：球心在壳内时改用**最近面**的解析口径（与 `prims::sphere_convex_ab`
/// 同把尺；凸壳只存点云 ⇒ 暴力枚举面 + 按 (法线, 偏移) 去重）。
#[test]
fn hull_buried_sphere_is_ejected() {
    for depth in [0.2f32, 0.5, 1.0] {
        let (y, ke, _v) = bury(0.0, depth, |w| {
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
        println!("③ 外壳埋球 {depth} m：末态 y = {y:+.4}（期望 ≈ {R:.2}）、KE = {ke:.3e}");
        assert!(
            y > 0.4,
            "埋 {depth} m 必须被顶出到壳顶（修前深埋档报远面法线 ⇒ 横向推走）：实测 {y:+.4}"
        );
        assert!(ke < 1.0, "顶出后应当静止：KE = {ke:.3e}");
    }
}

/// **④ 对照：正常静置**（球心 = 顶面 + r，不埋）⇒ 400 tick 内不许漂（`|Δy| < 1 mm`、末窗 |v| ≈ 0）。
/// 这条把"深埋恢复"与"静置"分开钉。
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

/// 埋球一档（μ 可调）：返回 `[末态 y, 末态 x, 末态 z, max|vx|, max|vz|, max|ω|]`（全程取最大）。
fn bury_lat(mu: f32, surface_y: f32, depth: f32, build: impl Fn(&mut World)) -> [f32; 6] {
    let mut w = World::new(PhysConfig::default());
    build(&mut w);
    let m = w.add_material(vxl_phys_core::Material {
        friction: vxl_phys_core::FrictionModel::Coulomb { mu },
        restitution: 0.02,
    });
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.4, surface_y - depth, -0.4),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.set_material(i, m);
    let (mut mx, mut mz, mut mw) = (0.0f32, 0.0f32, 0.0f32);
    for _ in 0..TICKS {
        w.step();
        let v = w.bodies.linvel[i];
        mx = mx.max(v.x.abs());
        mz = mz.max(v.z.abs());
        mw = mw.max(w.bodies.angvel(i).length());
    }
    let p = w.bodies.position[i];
    [p.y, p.x, p.z, mx, mz, mw]
}

/// 埋 0.2 m 的**盒**（顶面 y = 0）。
fn build_buried_box(w: &mut World) {
    w.add_static(
        Shape::Box {
            half: Vec3::new(2.0, 2.0, 2.0),
        },
        Vec3::new(0.0, -2.0, 0.0),
        Quat::IDENTITY,
    );
}

/// 埋 0.2 m 的**凸壳**（8 点盒壳、顶面 y = 0；与判据 ③ 同配方）。
fn build_buried_hull(w: &mut World) {
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
}

/// 判据 ⑤ 的建场景函数指针（`fn` 指针 ⇒ 两个通道能放进同一个数组；①~④ 用内联闭包）。
type BuildFn = fn(&mut World);

/// **⑤ 顶出是「纯法向」的**：埋 0.2 m ⇒ 球心会被顶出**恰好经过**顶面平面那一 tick（本缺口的触发条件）。
/// 把 μ 设成 **0**（摩擦这条腿关掉）⇒ 任何横向速度/自旋都只能来自**接触口径**（法线方向 / 接触点位置）。
///
/// 判据：末态停在顶面上（不许滚出平台）、`|Δx|,|Δz| < 1e-2`、`max|vx|,max|vz| < 1e-2`、`max|ω| < 1e-2`。
/// 修前：盒档在那一 tick 报**斜法线** ⇒ `vx ≈ −2.6 m/s` ⇒ 滚出 4 m 平台（末态 `y = −94`）；
/// 壳档接触点落到**壳角**上（力臂 2.8 m）⇒ 自旋 ⇒ 滚走（末态 `y = +0.30`）。
#[test]
fn ejection_is_purely_normal() {
    let channels: [(&str, BuildFn); 2] = [("盒  ", build_buried_box), ("外壳", build_buried_hull)];
    for (name, build) in channels {
        let [y, x, z, mvx, mvz, mw] = bury_lat(0.0, 0.0, 0.2, build);
        println!(
            "⑤ {name}埋球 0.2 m（μ=0）：末态 (x,z,y) = ({x:+.4},{z:+.4},{y:+.4})、\
             max|vx| = {mvx:.5}、max|vz| = {mvz:.5}、max|ω| = {mw:.5}"
        );
        assert!(
            y > 0.4,
            "{name}：必须顶出并停在顶面（修前横向速度把它带出平台）：实测 y = {y:+.4}"
        );
        assert!(
            (x - 0.4).abs() < 1e-2 && (z + 0.4).abs() < 1e-2,
            "{name}：纯法向顶出不许横向漂：x = {x:+.4}、z = {z:+.4}"
        );
        assert!(
            mvx < 1e-2 && mvz < 1e-2,
            "{name}：不许注入横向速度：max|vx| = {mvx:.5}、max|vz| = {mvz:.5}"
        );
        assert!(
            mw < 1e-2,
            "{name}：不许注入自旋（接触点必须是球心投影 ⇒ 力臂 ∥ 法线）：max|ω| = {mw:.5}"
        );
    }
}
