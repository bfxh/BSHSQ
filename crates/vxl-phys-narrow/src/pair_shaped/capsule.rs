//! **胶囊族（两个方向）** —— `pair_shaped` 的**子模块**（`#[path]` 指向 `pair_shaped/capsule.rs`）。
//!
//! **为什么拆出来**：`pair_shaped.rs` 受 god 门**文件行数棘轮**（阈值 800，它已经 883 行），而这一族
//! （`capsule_ab` + `capsule_ba`，~190 行）是里面最大的一块内聚代码。选**子模块**而不是 crate 级新文件，
//! 理由与 `mesh_mesh.rs`/`mesh_grid.rs` 相同：`narrow/lib.rs` 只剩 1 行预算，加 `mod` 声明就顶棘轮；
//! **子模块还能直接看见父模块的私有项**。
//!
//! ⚠️ 两处与父模块不同的写法（都只为过门禁，语义零变化）：本文件**不用 glob 导入**（`glob-gate` 对
//! **新文件**零基线）⇒ 显式 `use` 等价的那几个名字；两个方法由 `fn` 改成 **`pub(crate) fn`**
//! （子模块的私有项父模块看不见，而调用方 `pair_non_heightfield` 就在父模块里）。
use crate::gjk;
use crate::{ContactPoint, ContactPoints, DefaultNarrowPhase, Manifold};
use vxl_phys_core::{Mat3, Quat, Shape, Vec3};

/// —— 胶囊体 × {盒|球|外壳|胶囊|圆柱}：**GJK 距离路径** ——
///
/// 光滑外形不能走 SAT/裁剪；接触判据 = **线段到对方的距离 < radius**
/// （此时线段本身可能并未碰到对方，故 EPA 也不适用）。流形点 = 线段两端
/// 各自的表面点（贴平躺给 2 点支撑，竖直时只有一端落进 skin 带 ⇒ 退化为 1 点）。
/// 胶囊在 a 侧（b 侧见 `capsule_ba`）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn capsule_ab(
    np: &mut DefaultNarrowPhase,
    a: u32,
    b: u32,
    pa: Vec3,
    ra: Quat,
    pb: Vec3,
    rb: Quat,
    sb: &Shape,
    half_height: f32,
    radius: f32,
    out: &mut Vec<Manifold>,
) {
    let axis = Mat3::from_quat(ra).mul_vec3(Vec3::Y);
    // 凸体对方走**解析最近点**（`EXPERIMENTS.md` R.2）；对方不是多面体（胶囊/球）
    // 时退回 GJK 距离老路。两条路都给出对方**表面点** `p_other`。
    let (n_raw, p_other) = if np.poly_for(sb).is_some() {
        let Some((p_sample, surf)) = np.capsule_axis_reach(
            pa - axis * half_height,
            pa + axis * half_height,
            radius,
            sb,
            pb,
            rb,
        ) else {
            return;
        };
        (p_sample - surf, surf)
    } else {
        let cap = gjk::CapsuleSupport {
            half_height,
            radius,
            pos: pa,
            rot: Mat3::from_quat(ra),
        };
        // 借用作用域：`support_of` 借 `np.hulls` ⇒ 先把结论算成局部值。
        let reach = match np.support_of(sb, pb, rb) {
            Some(other) => DefaultNarrowPhase::capsule_reach(&cap, &other),
            None => None,
        };
        let Some((_n, _dist, p_cap, p_other)) = reach else {
            return;
        };
        (p_other - p_cap, p_other)
    };
    // 法线 a→b：初始符号不重要——**一律用两体中心定号**（GJK 穿透时见证点会换序，
    // 这正是要抹平的不确定性）。
    let mut n_ab = n_raw;
    if n_ab.length_squared() < 1e-18 {
        n_ab = pb - pa;
    }
    if n_ab.length_squared() < 1e-18 {
        return;
    }
    let mut n_ab = n_ab.normalize();
    if n_ab.dot(pb - pa) < 0.0 {
        n_ab = -n_ab;
    }
    // 压入量用**见证点平面**（对方表面点沿 n 的投影），不是对方的支撑平面：大盒配
    // 微倾法线时支撑平面会给出数十米偏移（R.1 实测的 42 m）。两条路的 `p_other`
    // 都是真表面点，此式统一适用。
    let plane = n_ab.dot(p_other);
    let mut local: Vec<ContactPoint> = Vec::with_capacity(2);
    for (i, s) in [pa - axis * half_height, pa + axis * half_height]
        .into_iter()
        .enumerate()
    {
        // 该端帽压入量（n 由胶囊指向对方）：`n·端点 + radius − plane`；
        // 平面取对方朝胶囊那侧，故压入为正。
        let depth = n_ab.dot(s) + radius - plane;
        if depth > -np.skin {
            local.push(ContactPoint {
                // 接触点 = 朝向对方那侧的帽面（+n 侧）。
                point: s + n_ab * radius,
                depth,
                feature: i as u32 + 1,
            });
        }
    }
    if local.is_empty() {
        return;
    }
    np.ws.cand.clear();
    np.ws.cand.extend_from_slice(&local);
    if !np.select_contacts(np.min_point_sep) {
        return;
    }
    out.push(Manifold {
        a,
        b,
        normal: n_ab,
        points: ContactPoints::from_slice(&np.ws.cand),
    });
}

/// 胶囊在 b 侧（同 `capsule_ab`，两体角色互换）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn capsule_ba(
    np: &mut DefaultNarrowPhase,
    a: u32,
    b: u32,
    pa: Vec3,
    ra: Quat,
    pb: Vec3,
    rb: Quat,
    sa: &Shape,
    half_height: f32,
    radius: f32,
    out: &mut Vec<Manifold>,
) {
    let axis = Mat3::from_quat(rb).mul_vec3(Vec3::Y);
    // 同臂 1：凸体对方走解析最近点，非多面体（胶囊/球）退回 GJK 老路。
    let (n_raw, p_other) = if np.poly_for(sa).is_some() {
        let Some((p_sample, surf)) = np.capsule_axis_reach(
            pb - axis * half_height,
            pb + axis * half_height,
            radius,
            sa,
            pa,
            ra,
        ) else {
            return;
        };
        (surf - p_sample, surf)
    } else {
        let cap = gjk::CapsuleSupport {
            half_height,
            radius,
            pos: pb,
            rot: Mat3::from_quat(rb),
        };
        let reach = match np.support_of(sa, pa, ra) {
            Some(other) => DefaultNarrowPhase::capsule_reach(&cap, &other),
            None => None,
        };
        let Some((_n, _dist, p_cap, p_other)) = reach else {
            return;
        };
        (p_cap - p_other, p_other)
    };
    // 法线 a→b：初始符号不重要（同上：中心定号，别信见证点序）。
    let mut n_ab = n_raw;
    if n_ab.length_squared() < 1e-18 {
        n_ab = pb - pa;
    }
    if n_ab.length_squared() < 1e-18 {
        return;
    }
    let mut n_ab = n_ab.normalize();
    if n_ab.dot(pb - pa) < 0.0 {
        n_ab = -n_ab;
    }
    // 压入量用见证点平面（理由同臂 1）。
    let plane = n_ab.dot(p_other);
    let mut local: Vec<ContactPoint> = Vec::with_capacity(2);
    for (i, s) in [pb - axis * half_height, pb + axis * half_height]
        .into_iter()
        .enumerate()
    {
        // 该端帽压入量（n 由对方指向胶囊）：`plane − (n·端点 − radius)`。
        let depth = plane - (n_ab.dot(s) - radius);
        if depth > -np.skin {
            local.push(ContactPoint {
                // 接触点 = 朝向对方那侧的帽面（−n 侧）。
                point: s - n_ab * radius,
                depth,
                feature: i as u32 + 1,
            });
        }
    }
    if local.is_empty() {
        return;
    }
    np.ws.cand.clear();
    np.ws.cand.extend_from_slice(&local);
    if !np.select_contacts(np.min_point_sep) {
        return;
    }
    out.push(Manifold {
        a,
        b,
        normal: n_ab,
        points: ContactPoints::from_slice(&np.ws.cand),
    });
}
