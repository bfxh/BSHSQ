//! **各向异性摩擦的"接触面内切向基与极限"**（K5，`SPEC §4.4` 的另一半）—— `constraint` 的子模块。
//!
//! 为什么单独一个模块：`constraint.rs` 受 god 门文件行数棘轮管；而"从材质对推出切向基/双轴极限"
//! 是与"建约束"相对独立的关注点（同 `solver_impl/sleep.rs` 的拆法）。

use vxl_phys_core::{BodySet, Material, Vec3};
use vxl_phys_narrow::Manifold;

/// 切向基 + 粘着判定用的标量 μ + 带速：各向异性 ⇒ **对齐基**（`t1` = 主轴投影）；
/// 各向同性 ⇒ `tangents(n)` 的原路径 + `belt = 0`（既有场景**逐位不变**）。
pub(crate) fn tangent_setup(
    frame: Option<FrictionFrame>,
    n: Vec3,
    mu: f32,
) -> (Vec3, Vec3, f32, f32) {
    match frame {
        Some(f) => (f.t1, f.t2, f.mu_dir, f.belt),
        None => {
            let (t1, t2) = crate::helpers::tangents(n);
            (t1, t2, mu, 0.0)
        }
    }
}

/// 体材质读取（缺表/越界 ⇒ 默认材质；与 M0 行为一致）。
pub(crate) fn material_of(bodies: &BodySet, i: usize) -> Material {
    bodies
        .material
        .get(i)
        .and_then(|&id| bodies.materials.get(id as usize))
        .copied()
        .unwrap_or_default()
}

/// 各向异性接触的切向基与极限（对齐基里的"盒"：沿 `t1` 的 μ1、沿 `t2` 的 μ2）。
#[derive(Clone, Copy)]
pub(crate) struct FrictionFrame {
    /// 主轴在接触面内的投影（单位向量）；`t2 = n × t1`（与 `tangents` 同一手性）。
    pub(crate) t1: Vec3,
    pub(crate) t2: Vec3,
    /// 沿 `t1` / `t2` 的**组合**摩擦（同 `√(μ·μ)` 组合律）。
    pub(crate) mu1: f32,
    pub(crate) mu2: f32,
    /// 带符号的切向目标速度（沿 `t1`）：该侧在 `a` 取 `+belt`、在 `b` 取 `−belt`。
    pub(crate) belt: f32,
    /// 沿**预解切向速度方向**的椭圆 μ（给粘着判定用；方向退化 ⇒ 两轴均值）。
    pub(crate) mu_dir: f32,
}

/// 这一对材质是否要按**各向异性**求解；要 ⇒ 给出切向基 / 双轴极限 / 带速。
///
/// - 只有一侧各向异性 ⇒ 用它那一侧的主轴与 `angle`（体局部 XZ 面），逐轴与另一侧组合；
/// - **两侧都各向异性** ⇒ 取 `a` 侧主轴（确定性），逐轴组合 `√(μ1a·μ1b)`、`√(μ2a·μ2b)`，
///   带速取 `a` 侧（`b` 侧带速按 0 记，文档化）；
/// - **各向同性等效**（`μ1 == μ2` 且 `belt == 0`）或主轴几乎垂直接触面（投影退化）⇒ `None`：
///   调用方走原路径（`tangents(n)` + 圆/盒锥）⇒ 既有场景**逐位不变**。
pub(crate) fn friction_frame(
    bodies: &BodySet,
    a: usize,
    b: usize,
    n: Vec3,
    m: &Manifold,
) -> Option<FrictionFrame> {
    use vxl_phys_core::FrictionModel as F;
    let (fa, fb) = (
        material_of(bodies, a).friction,
        material_of(bodies, b).friction,
    );
    let (side, aniso, other) = match (fa, fb) {
        // 两侧都各向异性 ⇒ 取 `a` 侧主轴（确定性），另一侧**逐轴**组合。
        (F::Anisotropic { .. }, F::Anisotropic { .. }) => (a, fa, fb),
        (F::Anisotropic { .. }, _) => (a, fa, fb),
        (_, F::Anisotropic { .. }) => (b, fb, fa),
        _ => return None,
    };
    let F::Anisotropic {
        mu1,
        mu2,
        angle,
        belt,
    } = aniso
    else {
        return None;
    };
    if mu1 == mu2 && belt == 0.0 {
        return None; // 各向同性等效：原路径（逐位不变的金丝雀）
    }
    // 主轴：体局部 XZ 面内按 `angle` 旋转 → 世界系 → 投影到接触面。
    let local = Vec3::new(angle.cos(), 0.0, angle.sin());
    let world = vxl_phys_core::Mat3::from_quat(bodies.rot(side)).mul_vec3(local);
    let proj = world - n * world.dot(n);
    let len = proj.length();
    if len < 1e-4 {
        return None; // 主轴几乎 ⊥ 接触面 ⇒ 退化为各向同性
    }
    let t1 = proj * (1.0 / len);
    let t2 = n.cross(t1);
    // 组合律：各向异性侧逐轴、另一侧逐轴（若它也各向异性）或取其等效标量。
    let (mu1c, mu2c) = match other {
        F::Anisotropic {
            mu1: o1, mu2: o2, ..
        } => ((mu1 * o1).sqrt(), (mu2 * o2).sqrt()),
        o => {
            let m = o.effective_mu();
            ((mu1 * m).sqrt(), (mu2 * m).sqrt())
        }
    };
    let belt = if side == a { belt } else { -belt };
    // 粘着判定用的方向 μ：取预解切向速度方向上的椭圆极限（方向退化 ⇒ 两轴均值）。
    let cp = &m.points[0];
    let vrel = bodies.velocity_at(b, cp.point - bodies.position[b])
        - bodies.velocity_at(a, cp.point - bodies.position[a]);
    let vt = vrel - n * vrel.dot(n);
    let mu_dir = if vt.length_squared() > 1e-12 {
        let u = vt.normalize();
        let (c1, c2) = (u.dot(t1) / mu1c, u.dot(t2) / mu2c);
        1.0 / (c1 * c1 + c2 * c2).sqrt().max(1e-6)
    } else {
        0.5 * (mu1c + mu2c)
    };
    Some(FrictionFrame {
        t1,
        t2,
        mu1: mu1c,
        mu2: mu2c,
        belt,
        mu_dir,
    })
}
