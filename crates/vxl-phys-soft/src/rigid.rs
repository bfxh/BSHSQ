//! **粒子 ↔ 刚体耦合**（`SPEC.md` §4.6「粒子-刚体距离约束（Akinci 式边界处理）」的最小实现）：
//! 刚体**代理视图** + 穿透查询 + **反作用回填**。
//!
//! **为什么自带查询而不是用窄相**：软体 crate 在依赖图上只挂 `core`（`soft <- 门面`）——
//! 引窄相会多一条边，而这里只需要"**球 vs 形状**"这一种查询（粒子 = 半径 `radius` 的球）。
//! 本片只做 **Sphere / Box / Capsule**（解析、共 ~60 行）；`Cylinder`/`Cone`/`ConvexHull`/`Compound`
//! 返回 `None`（写清边界，属后续切片——与窄相那条"六形状 × 四提供者"是两码事）。
//!
//! **反作用口径**（与 2b 流体反作用同段位）：软体侧累加**冲量**（粒子的动量变化取反），
//! 门面 `÷dt` 后按**力**加到体上（引擎每子步施加一次 ⇒ 一个 tick 的冲量 = `F·dt`，账平）。
use vxl_phys_core::{Mat3, Quat, Shape, Vec3};

/// 刚体代理：门面每 tick 填一次，软体侧只读（含接触所需的最小字段）。
pub struct RigidProxy {
    /// 门面的体索引（反作用回填用）。
    pub body: u32,
    pub shape: Shape,
    pub pos: Vec3,
    pub rot: Quat,
    /// 体心线速度（摩擦用：粒子滑移量取**相对体**的，体在动时绳才不会被"粘"在原地）。
    pub linvel: Vec3,
    /// **体心角速度**（§8.4.29 / 计划 2c-1：接触几何要跟着**转动**走 —— 虚拟位姿的朝向由它推进）。
    /// 静态体与睡眠体一律填 `0`（睡眠体对软体域呈现静态，§8.4.27）。
    pub angvel: Vec3,
    /// `0` = 静态（仍参与接触，但不接收反作用）。
    pub inv_mass: f32,
    /// **本体系逆惯量**（主轴）：开角反作用（`Rope::angular_reaction`）时用来推进"虚拟角速度"
    /// （与 `body_dv` 对平移的作用对称）。静态/睡眠体填 0。
    pub local_inv_inertia: Vec3,
}

/// 反作用（**冲量**口径，每 tick 累加；门面 `÷dt` 后作为力施加）。
#[derive(Clone, Copy)]
pub struct RigidReaction {
    pub body: u32,
    /// 作用在体上的冲量（= 粒子所受冲量取反）。
    pub impulse: Vec3,
    /// 绕**体原点**的角冲量（接触点 × 冲量）。
    pub torque: Vec3,
}

/// **粒子这一子步"穿过"盒的哪个面**（§8.4.5 的"穿过面法线"）：返回
/// `(该面的世界外法线, 沿该法线的穿透量, 面上交点（世界）, 面号)`；没有"由外入内"的穿越 ⇒ `None`。
/// **面号编码**：`k*2 + (s>0)`（`k` = 0/1/2 → x/y/z；`s` = ±1）⇒ 0=x−、1=x+、2=y−、3=y+、4=z−、5=z+。
///
/// **⚠️ 必须给两个位姿**（`pos_prev` = 上一子步的体位姿、`pos_now` = 当前）：穿越可能来自**体在动**
/// 而粒子不动（盒落在绳上正是这种）—— 只用一个位姿算 `prev`/`now` ⇒ **永远检测不到穿越**
/// （实测：整场"缓存命中 = 0"、盒子直接穿过绳线）。
///
/// **判据阈值取 `radius`（不是 0）**：接触是**球面**相碰 —— 球心停在面外 `radius` 处就已接触；
/// 只按"球心过面"写 ⇒ **永不成穿越**。即"由 `radius` 外侧进入带内"：`d_a ≥ radius && d_b < radius`、
/// `depth = radius − d_b`。
///
/// **调用方必须缓存这个面号**：穿越只在一子步成立 ⇒ 接触是**一段状态**（见 `Rope::entry`）。
/// **为什么不用"最近面"**：体相对绳线**下沉**时最近面会在底面↔侧面↔顶面之间翻转 ⇒ 推力方向突变
/// ⇒ 一次踢击把体送走（§8.4.1/§8.4.3 实测）；而"它穿过的那个面"只要还在从下面顶就一直是底面 ✓。
#[allow(clippy::too_many_arguments)] // 两个位姿（各带朝向）+ 两个位置 + 半径
pub fn crossed_face(
    rot_prev: Quat,
    rot_now: Quat,
    half: Vec3,
    pos_prev: Vec3,
    pos_now: Vec3,
    p_prev: Vec3,
    p_now: Vec3,
    radius: f32,
) -> Option<(Vec3, f32, Vec3, u8)> {
    fn ax(v: Vec3, k: usize) -> f32 {
        match k {
            0 => v.x,
            1 => v.y,
            _ => v.z,
        }
    }
    // **两个位姿各带自己的朝向**（§8.4.29）：体在转时"上一子步的局部系"与"当前的局部系"不同，
    // 只用一个朝向就是把转动漏掉（`ω = 0` 时两者相同 ⇒ 与既有行为逐位一致）。
    let m_a = Mat3::from_quat(rot_prev);
    let m_b = Mat3::from_quat(rot_now);
    let a = m_a.transpose_mul_vec3(p_prev - pos_prev);
    let b = m_b.transpose_mul_vec3(p_now - pos_now);
    let mut best: Option<(f32, usize, f32, f32)> = None; // (深度, 轴, 符号, 带边交点参数)
    for k in 0..3 {
        for s in [-1.0f32, 1.0] {
            let h = ax(half, k);
            let (da, db) = (s * ax(a, k) - h, s * ax(b, k) - h);
            if da < radius || db >= radius {
                continue; // 只收"由 radius 外侧进入带内"
            }
            // 线段跨"带边 `d = radius`"的参数（`d_a ≥ radius > d_b` ⇒ t ∈ [0,1]）
            let t = (da - radius) / (da - db);
            let p = a + (b - a) * t;
            let (j1, j2) = ((k + 1) % 3, (k + 2) % 3);
            if ax(p, j1).abs() > ax(half, j1) || ax(p, j2).abs() > ax(half, j2) {
                continue; // 穿过的是该面的**延长平面**，不是这张面
            }
            let depth = radius - db;
            if best.is_none_or(|(d, _, _, _)| depth > d) {
                best = Some((depth, k, s, t));
            }
        }
    }
    let (depth, k, s, t) = best?;
    let face = (k as u8) * 2 + u8::from(s > 0.0);
    let pl = a + (b - a) * t;
    let n_local = match k {
        0 => Vec3::new(s, 0.0, 0.0),
        1 => Vec3::new(0.0, s, 0.0),
        _ => Vec3::new(0.0, 0.0, s),
    };
    let q_local = match k {
        0 => Vec3::new(s * half.x, pl.y, pl.z),
        1 => Vec3::new(pl.x, s * half.y, pl.z),
        _ => Vec3::new(pl.x, pl.y, s * half.z),
    };
    Some((
        // 法线与接触点取**当前**朝向的局部系（`rot_now`）⇒ 与"现在这张面在哪"一致。
        m_b.mul_vec3(n_local),
        depth,
        pos_now + m_b.mul_vec3(q_local),
        face,
    ))
}

/// 球（心 `p`、半径 `radius`）对 `shape`（位姿 `pos`/`rot`）的**穿透**：
/// 返回 `(外向法线, 穿透深度, 体表接触点)`；不接触 / 形状不支持 ⇒ `None`。
///
/// 法线约定与提供者通道一致：**从体表面指向粒子**（`pos += n·depth` 即推出）。
pub fn shape_penetration(
    shape: &Shape,
    pos: Vec3,
    rot: Quat,
    p: Vec3,
    radius: f32,
) -> Option<(Vec3, f32, Vec3)> {
    match *shape {
        Shape::Sphere { radius: r } => {
            let d = p - pos;
            let dist = d.length();
            if dist < 1e-9 {
                return Some((Vec3::Y, r + radius, pos)); // 同心退化：任取方向
            }
            if dist > r + radius {
                return None;
            }
            let n = d * (1.0 / dist);
            Some((n, r + radius - dist, p - n * radius))
        }
        Shape::Box { half } => {
            let m = Mat3::from_quat(rot);
            // 局部系里做"点 vs 盒"（夹取求最近点），再变换回去。
            let local = m.transpose_mul_vec3(p - pos);
            let c = Vec3::new(
                local.x.clamp(-half.x, half.x),
                local.y.clamp(-half.y, half.y),
                local.z.clamp(-half.z, half.z),
            );
            let d = local - c;
            let dist = d.length();
            if dist > 1e-6 {
                if dist > radius {
                    return None; // 外侧且够远
                }
                let n_local = d * (1.0 / dist);
                return Some((m.mul_vec3(n_local), radius - dist, pos + m.mul_vec3(c)));
            }
            // 点在盒内：法线取**最近面**的外向，深度 = 到该面 + 半径。
            let dx = half.x - local.x.abs();
            let dy = half.y - local.y.abs();
            let dz = half.z - local.z.abs();
            let (deep, axis) = if dx <= dy && dx <= dz {
                (dx, Vec3::new(local.x.signum(), 0.0, 0.0))
            } else if dy <= dz {
                (dy, Vec3::new(0.0, local.y.signum(), 0.0))
            } else {
                (dz, Vec3::new(0.0, 0.0, local.z.signum()))
            };
            let n = m.mul_vec3(axis);
            Some((n, deep + radius, p - n * radius))
        }
        Shape::Capsule {
            half_height,
            radius: r,
        } => {
            let m = Mat3::from_quat(rot);
            let axis = m.mul_vec3(Vec3::Y);
            // 点到轴段的最近点（夹取投影参数）。
            let t = (p - pos).dot(axis).clamp(-half_height, half_height);
            let q_axis = pos + axis * t;
            let d = p - q_axis;
            let dist = d.length();
            let total = r + radius;
            if dist < 1e-9 {
                return Some((Vec3::Y, total, q_axis));
            }
            if dist > total {
                return None;
            }
            let n = d * (1.0 / dist);
            Some((n, total - dist, p - n * radius))
        }
        _ => None, // Cylinder / Cone / ConvexHull / Compound / Provider / HeightField：待补
    }
}
