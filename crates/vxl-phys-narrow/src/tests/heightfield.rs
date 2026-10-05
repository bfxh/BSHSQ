//! **高度场腿的判据**（盒/球/胶囊/外壳/复合体 × 地形）—— `tests` 的子模块。
//!
//! 2026-10-05 从 836 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use super::manifolds_for;
use crate::{CompoundChild, DefaultNarrowPhase, HeightField, NarrowPhase, Shape};
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Vec3};

/// 胶囊 × 地形：此前高度场分支**显式拒绝**本组合 ⇒ 静默无接触。
/// 现沿中心线取 5 个样本、每个按球处理。判据：有接触、法线竖直、压入 ≈1 cm。
#[test]
fn capsule_on_heightfield() {
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    // 竖直胶囊：下端点 y0 − 0.4、帽面再 −0.3 ⇒ 压入 1 cm 时 y0 = 0.69。
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.69, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert!(!m.is_empty(), "竖直胶囊 × 地形应有接触（此前为静默无接触）");
    assert!(
        m[0].normal.y.abs() > 0.99,
        "法线应竖直，实得 {:?}",
        m[0].normal
    );
    let dmax = m[0].points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!((dmax - 0.01).abs() < 5e-3, "压入应 ≈1 cm，实得 {dmax}");
}

/// 外壳 × 地形：此前 `hull_pair` 明确不受理（"列裁剪对任意凸壳未实现"）⇒ 静默无接触。
/// 现走**逐顶点采样**（与 `poly_heightfield` 同款）。判据：有接触、法线竖直、正压入。
#[test]
fn hull_on_heightfield() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 外壳：3×3×3 立方点云（半 0.3）。
    let mut pts: Vec<Vec3> = Vec::with_capacity(27);
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
            }
        }
    }
    let hid = np.add_hull(pts);
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    // 底面压入地面（y=0）1 cm ⇒ 中心 y = 0.29。
    b.push_dynamic(
        Shape::ConvexHull {
            hull: hid,
            half: Vec3::splat(0.3),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[hf],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(!out.is_empty(), "外壳 × 地形应有接触（此前为静默无接触）");
    let m = &out[0];
    assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
}

#[test]
fn sphere_on_heightfield() {
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 0.45, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    // a=球(0) 在上，b=marker(1) → 法线 a→b = -Y（推向地面）。
    assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
    assert!((m[0].points[0].depth - 0.05).abs() < 0.02);
}

/// 复合体 × 地形：子形状各自走地形路径（此前该组合在高度场分支被**显式拒绝**）。
/// 判据：两个球子形状各给出一条流形、法线竖直（含接触）。
#[test]
fn compound_on_heightfield() {
    let mut np = DefaultNarrowPhase::new(0.01);
    let cid = np.add_compound(vec![
        CompoundChild {
            shape: Shape::Sphere { radius: 0.3 },
            offset: Vec3::new(-0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Sphere { radius: 0.3 },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
    ]);
    let mut b = BodySet::new();
    let hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    b.push_dynamic(
        Shape::Compound {
            compound: cid,
            half: np.compound_half_extents(cid),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[hf],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(
        out.len() >= 2,
        "两个球子形状应各出一条地形流形，实得 {} 条",
        out.len()
    );
    for m in &out {
        assert!(
            m.normal.y.abs() > 0.99,
            "地形法线应竖直，实得 {:?}",
            m.normal
        );
        // 球子形状的特征号恒为 0 ⇒ 子序号标记必须对**全部**点位生效，否则两个子形状
        // 的接触点在同一个体对（暖启动缓存键）上无法区分。
        for p in m.points.iter() {
            assert!(
                p.feature >> 16 != 0,
                "地形接触也应带子序号标记，实得 {}",
                p.feature
            );
        }
    }
}

#[test]
fn box_on_heightfield_slope() {
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11 {
        for ix in 0..11 {
            // 以 x=0 为零点、沿 x 抬升的斜坡（h(0)=0）。
            hf.set_height(ix, iz, (ix as f32 - 5.0) * 0.2);
        }
    }
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 0.35, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _marker = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    // a=盒(0) 在上，b=marker(1) → 流形法线 a→b 指向地面（-y 分量为主），
    // 且因地面沿 +x 抬升而偏向 +x；求解器给盒子的推力 = -n = 朝上偏 -x。
    assert!(m[0].normal.y < -0.9, "normal {:?}", m[0].normal);
    assert!(m[0].normal.x > 0.1, "normal {:?}", m[0].normal);
}

/// 内边（折痕）幽灵接触判据：盒正中骑在「平地面 / 斜坡」的折痕上时，
/// 采样法线**不许是两块面法线的混合**——混合即内边假接触（幽灵推力）。
/// 高度场路径取「最深点采样法线」作整条流形法线，折痕处的双线性采样
/// 天然会把两块面混在一起，故这里是该缺陷的天然复现位。
#[test]
fn heightfield_crease_normal_is_not_blended() {
    // ix≤5 平（h=0），ix>5 沿 +x 抬升 0.5/格 ⇒ 折痕在 x=0（spacing=1，x0=-5）。
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11 {
        for ix in 0..11 {
            hf.set_height(ix, iz, (ix as f32 - 5.0).max(0.0) * 0.5);
        }
    }
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 0.35, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let _ = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let m = manifolds_for(&b, &[hf]);
    assert_eq!(m.len(), 1);
    let n = m[0].normal; // 约定：a=盒 → b=地面（指向地面者为主）
    let floor = Vec3::new(0.0, -1.0, 0.0);
    let ramp = -Vec3::new(-0.5, 1.0, 0.0).normalize();
    let d_floor = n.dot(floor);
    let d_ramp = n.dot(ramp);
    assert!(n.z.abs() < 1e-3, "折痕法线出现 z 分量（邻格串扰）：{n:?}");
    assert!(
        d_floor.max(d_ramp) > 0.995,
        "折痕法线是两块面法线的混合 ⇒ 内边幽灵接触：{n:?}（floor 对齐 {d_floor}，ramp 对齐 {d_ramp}）"
    );
}
