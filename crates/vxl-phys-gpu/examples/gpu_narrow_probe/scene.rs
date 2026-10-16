//! 窄相探针的**场景构造/夹具**：地板格 / 球阵 / 盒族（对齐 + 倾斜）/ 盒阈三连 / 圆柱簇 /
//! 边角簇 / 复合体 —— 全是**纯几何摆放**（放置先后即体号次序，金丝雀的靶体号由它决定）。
//! （主文件 `main.rs` 只留编排与打印；本文件被 `mod scene;` 收进来 ⇒ 条目对父模块可见即可。）
//!
//! **判据与诊断**在 `diag.rs`；本文件不参与判据，只负责「摆场景 + 把边角对号带回」。
//! ⚠️ 改这里的任何字面量 = 改判据的输入（容差按场景尺度放宽、覆盖面自检按族计数）。

use super::*;

/// 场景里的**边角对**（给覆盖面断言用；索引即体号）。
pub(super) struct Edge {
    pub(super) concentric: (u32, u32),
    pub(super) touching: (u32, u32),
    pub(super) near: (u32, u32),
}

/// 盒族的 skin 阈三连。
pub(super) struct BoxEdge {
    pub(super) touch: (u32, u32),
    pub(super) near: (u32, u32),
    pub(super) sep: (u32, u32),
}

pub(super) struct Scene {
    pub(super) w: World,
    pub(super) edge: Edge,
    pub(super) box_edge: BoxEdge,
    /// 复合体体号（同一体对上有多条流形 ⇒ 只有主机回填才装得下）。
    pub(super) compound: u32,
    /// 复合体子形状：**必须同时注册进探针自建的那份窄相**。
    pub(super) kids: Vec<CompoundChild>,
    /// 格点里一颗**普通球**（金丝雀挪它 ⇒ 只改几何、不改配对）。
    pub(super) ball: u32,
}

pub(super) fn floor_of(w: &mut World, side: usize) {
    for k in 0..side * side {
        let x = (k % side) as f32 - side as f32 * 0.5;
        let z = (k / side) as f32 - side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, 0.0, z),
            Quat::IDENTITY,
        );
    }
}

/// 三层咬合球阵：间距 0.75 < 半径和 0.8 ⇒ 正交邻居**重叠**、对角邻居**分离**
/// ⇒ 同一批对里既有接触也有"接手但无接触"（两条臂都跑得到）。返回**首颗球的体号**。
pub(super) fn balls_of(w: &mut World, side: usize, layers: usize) -> u32 {
    let first = w.bodies.len() as u32;
    for ly in 0..layers {
        for iz in 0..side {
            for ix in 0..side {
                let k = (ly * side * side + iz * side + ix) as f32;
                let jx = ((k * 37.0) % 17.0) * 0.01 - 0.08;
                let jz = ((k * 53.0) % 19.0) * 0.01 - 0.09;
                let r = 0.40 + ((k * 29.0) % 5.0) * 0.01;
                w.add_dynamic(
                    Shape::Sphere { radius: r },
                    Vec3::new(
                        ix as f32 * 0.75 - 4.5 + jx,
                        0.88 + ly as f32 * 0.72,
                        iz as f32 * 0.75 - 4.5 + jz,
                    ),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    first
}

/// 一排动态盒：盒×盒（卡上 SAT+裁剪）与盒×地板（**仍是主机回填**）—— 两侧都覆盖到。
/// ⚠️ x/z 间距**故意不相等**（0.781 / 0.783）：相等间距会让对角邻居在 x/z 两根轴上**精确平局**
/// （同渗透量）⇒ SAT 选哪根轴由末位比特决定，判据就变成"比谁的舍入更巧"（实测踩过，见 §17.7）。
pub(super) fn boxes_of(w: &mut World, n: usize) {
    for k in 0..n {
        let x = -3.0 + (k % 12) as f32 * 0.781;
        let z = 6.0 + (k / 12) as f32 * 0.783;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, 0.9, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
}

/// 倾斜盒簇（搬到地板格之外、悬在 y≈30）：**彼此深叠 + 姿态各异** ⇒ 同时压到面轴与 9 条棱轴，
/// 并且能裁出 **4 点**流形。对齐盒只走面轴 ⇒ 单靠它们覆盖不到棱轴与多点裁剪这两条路。
pub(super) fn tilted_boxes_of(w: &mut World, n: usize) -> u32 {
    let first = w.bodies.len() as u32;
    let axis = Vec3::new(0.3, 1.0, 0.2).normalize();
    for k in 0..n {
        let rot = Quat::from_axis_angle(axis, 0.25 + k as f32 * 0.07);
        let x = (k % 2) as f32 * 0.95;
        let z = ((k / 2) % 2) as f32 * 0.95;
        let y = 30.0 + (k / 4) as f32 * 0.9;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(x, y, z),
            rot,
            1000.0,
        );
    }
    first
}

/// 盒对的 skin 阈三连（体在半空、**三对分开放**（z 隔开）⇒ 只与各自那一对成对），
/// 覆盖 `sep > skin` 与**负深度（投机接触）**两条边：`gap = 0`（恰好接触）/`0.01`（带内 ⇒ 预期接触）/
/// `0.03`（AABB 仍相交 ⇒ 在配对表里，但 `sep > skin` ⇒ **无流形**）。
pub(super) fn box_skin_cases(w: &mut World) -> BoxEdge {
    let mk = |w: &mut World, y: f32, z: f32| {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(40.0, y, z),
            Quat::IDENTITY,
            1000.0,
        )
    };
    BoxEdge {
        touch: (mk(w, 40.0, 0.0), mk(w, 41.0, 0.0)),
        near: (mk(w, 40.0, 4.0), mk(w, 41.01, 4.0)),
        sep: (mk(w, 40.0, 8.0), mk(w, 41.03, 8.0)),
    }
}

/// 圆柱簇（**卡上未接的族** ⇒ 全部走主机回填）。它的用处是：球×盒上卡之后，场景里若只剩复合体
/// 需要回填，回填路径就近乎空转 ⇒ 用一簇"真接触的未接族"把它**钉在覆盖面上**。
pub(super) fn cyls_of(w: &mut World, n: usize) -> u32 {
    let first = w.bodies.len() as u32;
    for k in 0..n {
        let x = 10.0 + (k % 4) as f32 * 0.7;
        let z = -8.0 + (k / 4) as f32 * 0.7;
        w.add_dynamic(
            Shape::Cylinder {
                half_height: 0.4,
                radius: 0.35,
            },
            Vec3::new(x, 0.7, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    first
}

/// 边角簇（搬到地板格之外 ⇒ 只与彼此成对）：同心（`dist < 1e-9` 特殊分支）/
/// 恰好接触（`dist == rr` ⇒ **无流形**）/ 擦边（深度 1e-3）。
pub(super) fn edge_of(w: &mut World) -> Edge {
    let s = |w: &mut World, p: Vec3| {
        w.add_dynamic(Shape::Sphere { radius: 0.5 }, p, Quat::IDENTITY, 1000.0)
    };
    let a0 = s(w, Vec3::new(60.0, 0.0, 0.0));
    let a1 = s(w, Vec3::new(60.0, 0.0, 0.0));
    let b0 = s(w, Vec3::new(62.0, 0.0, 0.0));
    let b1 = s(w, Vec3::new(63.0, 0.0, 0.0));
    let c0 = s(w, Vec3::new(64.0, 0.0, 0.0));
    let c1 = s(w, Vec3::new(64.999, 0.0, 0.0));
    Edge {
        concentric: (a0, a1),
        touching: (b0, b1),
        near: (c0, c1),
    }
}

pub(super) fn scene() -> Scene {
    let mut w = World::new(PhysConfig::default());
    floor_of(&mut w, 20);
    let first_ball = balls_of(&mut w, 12, 3);
    boxes_of(&mut w, 48);
    tilted_boxes_of(&mut w, 24);
    cyls_of(&mut w, 24);
    let box_edge = box_skin_cases(&mut w);
    let edge = edge_of(&mut w);
    let kids = vec![
        CompoundChild {
            shape: Shape::Sphere { radius: 0.28 },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Sphere { radius: 0.28 },
            offset: Vec3::new(0.5, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
    ];
    let cid = w.add_compound(kids.clone());
    let compound = w.spawn_compound_body(cid, Vec3::new(-2.0, 0.72, -6.0), Quat::IDENTITY, 1000.0);
    Scene {
        w,
        edge,
        box_edge,
        compound,
        kids,
        // 金丝雀靶：球阵里第四排的那颗（**四周都有邻居** ⇒ 挪它必然改动若干条流形的几何）。
        ball: first_ball + 40,
    }
}
