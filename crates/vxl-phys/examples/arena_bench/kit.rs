//! kit：复刻 PhysArena（`D:\KF\physarena`）场景的共享工具。
//!
//! 复刻口径沿用 `probes_a/b`：**同体数 / 同尺寸 / 同出生位姿 / 同材质**
//! （出生序也保持一致，体索引一一对应）。与 arena 的差异只有两类，且逐场景
//! 在注册表注记里显式标注：
//!   ① 本仓**无逐体阻尼**（arena 的 `linearDamping`/`angularDamping` 一律略过）；
//!   ② 本仓**无 kinematic 体、无 sensor**（对应场景给等效物或省略，见注记）。
use super::{mat, CompoundChild, Shape, Vec3, World};

/// arena `kit.ts::make` 的默认材质（未给 opts 时）：friction 0.5 / restitution 0.05。
pub(crate) const ARENA_DEFAULT: Surf = Surf {
    friction: 0.5,
    restitution: 0.05,
};

/// 一对面参数（摩擦/恢复）——避免工具函数超过 7 参（clippy）。
#[derive(Clone, Copy)]
pub(crate) struct Surf {
    pub friction: f32,
    pub restitution: f32,
}

pub(crate) const fn s2(friction: f32, restitution: f32) -> Surf {
    Surf {
        friction,
        restitution,
    }
}

/// PhysArena `kit.ts::rng(seed)`（mulberry32）的**逐位等价**实现：
/// 同 seed ⇒ 同 f32 序列（JS 侧 f64 除法 + 本仓 f32 舍入，见 `EXPERIMENTS` 注）。
pub(crate) fn rng32(seed: u32) -> impl FnMut() -> f32 {
    let mut a = if seed == 0 { 1 } else { seed };
    move || {
        a = a.wrapping_add(0x6d2b79f5);
        let mut t = (a ^ (a >> 15)).wrapping_mul(1 | a);
        t = t.wrapping_add((t ^ (t >> 7)).wrapping_mul(61 | t)) ^ t;
        ((t ^ (t >> 14)) as f32) / 4294967296.0
    }
}

/// 绕 Y 轴偏航（arena 的四元数写法 `[0, sin(a/2), 0, cos(a/2)]`）。
pub(crate) fn rot_y(yaw: f32) -> vxl_phys_core::Quat {
    let (s, c) = ((yaw / 2.0).sin(), (yaw / 2.0).cos());
    vxl_phys_core::Quat {
        x: 0.0,
        y: s,
        z: 0.0,
        w: c,
    }
}

/// 绕单位轴转角（弧度）——逐字照 arena 的 `[axis*sin(a/2), cos(a/2)]` 写法，
/// 不依赖 `Quat::from_axis_angle` 的归一化/符号约定。
pub(crate) fn rot_axis(axis: Vec3, angle: f32) -> vxl_phys_core::Quat {
    let (s, c) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    vxl_phys_core::Quat {
        x: axis.x * s,
        y: axis.y * s,
        z: axis.z * s,
        w: c,
    }
}

/// 地面（顶面 y=0），指定摩擦（arena `ground(size, 1, 0, {friction})` 的对应物）。
pub(crate) fn ground_mu(w: &mut World, size: f32, friction: f32) {
    ground_ex(w, size, s2(friction, 0.05));
}

/// 地面（顶面 y=0），摩擦与恢复都指定（arena `ground(size, 1, 0, {friction, restitution})`）。
pub(crate) fn ground_ex(w: &mut World, size: f32, surf: Surf) {
    let m = mat(w, surf.friction, surf.restitution);
    let i = w.bodies.len();
    w.add_static(
        Shape::Box {
            half: Vec3::new(size / 2.0, 1.0, size / 2.0),
        },
        Vec3::new(0.0, -1.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
    );
    w.bodies.set_material(i, m);
}

/// 动态盒（带旋转与逐体材质）→ 体索引。
pub(crate) fn add_box_r(
    w: &mut World,
    pos: Vec3,
    half: Vec3,
    rot: vxl_phys_core::Quat,
    surf: Surf,
    density: f32,
) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let i = w.bodies.len();
    w.add_dynamic(Shape::Box { half }, pos, rot, density);
    w.bodies.set_material(i, m);
    i
}

/// 动态球。
pub(crate) fn add_ball(w: &mut World, pos: Vec3, radius: f32, surf: Surf, density: f32) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Sphere { radius },
        pos,
        vxl_phys_core::Quat::IDENTITY,
        density,
    );
    w.bodies.set_material(i, m);
    i
}

/// 动态体（任意形状）的**通用入口**：尺寸在 `Shape` 里，参数 6 个（args 门 ≤7）。
/// 圆柱/胶囊/圆锥都走这里（各自的包装会多出 1 个形参，超 args 门的软线）。
pub(crate) fn add_dyn_shape(
    w: &mut World,
    shape: Shape,
    pos: Vec3,
    rot: vxl_phys_core::Quat,
    surf: Surf,
    density: f32,
) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let i = w.bodies.len();
    w.add_dynamic(shape, pos, rot, density);
    w.bodies.set_material(i, m);
    i
}

/// 静态盒（arena `b.box(..., {type: 'static'})` 的对应物）→ 体索引。
pub(crate) fn add_static_box(
    w: &mut World,
    pos: Vec3,
    half: Vec3,
    rot: vxl_phys_core::Quat,
    surf: Surf,
) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let i = w.bodies.len();
    w.add_static(Shape::Box { half }, pos, rot);
    w.bodies.set_material(i, m);
    i
}

/// 凸包体（arena `b.convex`）：注册点云 → `spawn_hull_body`。
pub(crate) fn spawn_hull(
    w: &mut World,
    points: Vec<Vec3>,
    pos: Vec3,
    rot: vxl_phys_core::Quat,
    surf: Surf,
    density: f32,
) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let hull = w.add_hull(points);
    let bi = w.spawn_hull_body(hull, pos, rot, density) as usize;
    w.bodies.set_material(bi, m);
    bi
}

/// 复合体（arena `b.compound`）。
pub(crate) fn spawn_compound(
    w: &mut World,
    children: Vec<CompoundChild>,
    pos: Vec3,
    surf: Surf,
    density: f32,
) -> usize {
    let m = mat(w, surf.friction, surf.restitution);
    let id = w.add_compound(children);
    let bi = w.spawn_compound_body(id, pos, vxl_phys_core::Quat::IDENTITY, density) as usize;
    w.bodies.set_material(bi, m);
    bi
}

/// arena `rockPoints(radius, r, vertices)` 的复刻（Fibonacci 球 + 抖动 ⇒ 不规则凸包）。
pub(crate) fn rock_points(radius: f32, r: &mut impl FnMut() -> f32, vertices: usize) -> Vec<Vec3> {
    let d = (vertices.saturating_sub(1)).max(1) as f32;
    let mut pts = Vec::with_capacity(vertices);
    for i in 0..vertices {
        let y = 1.0 - (i as f32 / d) * 2.0;
        let rad = (1.0 - y * y).max(0.0).sqrt();
        let theta = i as f32 * std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
        let jitter = 0.72 + r() * 0.42;
        pts.push(Vec3::new(
            theta.cos() * rad * radius * jitter,
            y * radius * jitter,
            theta.sin() * rad * radius * jitter,
        ));
    }
    pts
}

/// arena `icosaPoints(radius)` 的复刻（单位二十面体 × radius）。
pub(crate) fn icosa_points(radius: f32) -> Vec<Vec3> {
    let t = (1.0 + 5.0f32.sqrt()) / 2.0;
    let raw: [[f32; 3]; 12] = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let norm = (1.0 + t * t).sqrt();
    raw.iter()
        .map(|p| {
            Vec3::new(
                p[0] / norm * radius,
                p[1] / norm * radius,
                p[2] / norm * radius,
            )
        })
        .collect()
}

/// 网格地形（arena `heightfieldMesh` 的对应物；顶点/索引序一致）。
pub(crate) fn heightfield_mesh(
    size: f32,
    segments: usize,
    height_at: impl Fn(f32, f32) -> f32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let step = size / segments as f32;
    for iz in 0..=segments {
        for ix in 0..=segments {
            let x = -size / 2.0 + ix as f32 * step;
            let z = -size / 2.0 + iz as f32 * step;
            verts.push(Vec3::new(x, height_at(x, z), z));
        }
    }
    let row = (segments + 1) as u32;
    for iz in 0..segments as u32 {
        for ix in 0..segments as u32 {
            let a = iz * row + ix;
            let b = a + 1;
            let c = a + row;
            let d = c + 1;
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    (verts, tris)
}
