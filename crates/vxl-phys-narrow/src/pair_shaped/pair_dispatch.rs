//! **形状对的分派级联**（2026-10-06 从 `DefaultNarrowPhase::process_pair_shaped` 搬出来的
//! 模块级自由函数 + 一个上下文结构）。
//!
//! **为什么搬**（god 债务的**设计层**处置，见 `god.gate.json`）：这个级联是"**新加一个形状、
//! 或新加一个域时唯一必须改的地方**"——放在 `impl DefaultNarrowPhase` 里，等于每来一个新形状/
//! 新域都要去动那个已经 59 个方法的类型。搬到模块级之后**改这里不动类型**，验收写在那条债务里：
//! "新加一个形状或域**不必再动本类型**"。
//!
//! **[`PairArgs`] 为什么存在**（两条硬约束逼出来的）：① `args-gate` —— **新文件零基线**，
//! 函数形参 >7 直接红，12 个位置参数搬不过来；② 更重要的是：处理器将来要统一签名（分派表的前提），
//! 上下文必须是**一个可传递的对象**。复合体递归时**新建子上下文**而不是改父的字段：借用的生命
//! 周期绑在结构上，原地改字段会把 `&'a Shape` 收窄，compiler 直接拒。
//!
//! ⚠️ **纯搬移、语义未改**：段序（复合体 → 提供者 → 外壳 → 高度场 → 通用）、每段的 `return`
//! 语义、复合体递归与 `tag_child_features` 打标**一字未动**。判据 = 既有测试 +
//! `determinism` / `m0_gates` 两条冻结哈希逐位不变。

use crate::heightfield::HeightField;
use crate::{tag_child_features, DefaultNarrowPhase, Manifold};
use vxl_phys_core::interop::ProviderColliders;
use vxl_phys_core::{BodySet, Mat3, Quat, Shape, Vec3};

/// 一次形状对分派的**全部输入**（两侧体号 / 形状 / 世界位姿 + 环境 + 出参）。
/// `out` 是 `&mut` ⇒ 递归时用 `&mut *args.out` 重新借出（NLL 下与 `args` 不冲突）。
pub(crate) struct PairArgs<'a> {
    pub(crate) a: u32,
    pub(crate) b: u32,
    pub(crate) bodies: &'a BodySet,
    pub(crate) sa: &'a Shape,
    pub(crate) sb: &'a Shape,
    pub(crate) pa: Vec3,
    pub(crate) ra: Quat,
    pub(crate) pb: Vec3,
    pub(crate) rb: Quat,
    pub(crate) heightfields: &'a [HeightField],
    pub(crate) providers: &'a dyn ProviderColliders,
    pub(crate) out: &'a mut Vec<Manifold>,
}

/// **配对主入口**（只做分派；复合体展开见两个 `expand_compound_*`）。
pub(crate) fn process_pair_shaped(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    // 盒对专用路径开关：每对先复位（非盒对 / 圆柱对一律走通用路径）。
    np.ws.box_axes_a = None;
    np.ws.box_axes_b = None;

    // 复合体放**最前**（先于地形/提供者分支）⇒ 子形状各自走完整配对路径（含地形）。
    if let Shape::Compound { compound, .. } = *args.sa {
        expand_compound_a(np, args, compound);
        return;
    }
    if let Shape::Compound { compound, .. } = *args.sb {
        expand_compound_b(np, args, compound);
        return;
    }

    // 提供者参与的对（见 `provider_pair`）。
    if np.provider_pair(
        args.a,
        args.b,
        args.bodies,
        args.sa,
        args.sb,
        args.pa,
        args.ra,
        args.pb,
        args.rb,
        args.providers,
        args.out,
    ) {
        return;
    }

    // **凸体外壳参与的对**（多边形域）：外壳 × {盒|球|外壳} → GJK/EPA。
    // 与提供者的组合已在上面的 provider 分支处理；与高度场暂不受理。
    if matches!(*args.sa, Shape::ConvexHull { .. }) || matches!(*args.sb, Shape::ConvexHull { .. })
    {
        np.hull_pair(
            args.a,
            args.b,
            args.bodies,
            args.sa,
            args.sb,
            args.pa,
            args.ra,
            args.pb,
            args.rb,
            args.heightfields,
            args.out,
        );
        return;
    }

    // 高度场参与的对（见 `heightfield_pair`）。
    if np.heightfield_pair(
        args.a,
        args.b,
        args.bodies,
        args.sa,
        args.sb,
        args.pa,
        args.ra,
        args.pb,
        args.rb,
        args.heightfields,
        args.out,
    ) {
        return;
    }

    // 非 heightfield 对（见 `pair_non_heightfield` 的各臂 helper）。
    np.pair_non_heightfield(
        args.a,
        args.b,
        args.bodies,
        args.sa,
        args.sb,
        args.pa,
        args.ra,
        args.pb,
        args.rb,
        args.out,
    );
}

/// **a 侧复合体展开**：逐子形状建**子上下文**并递归；每个子形状的流形用 `tag_child_features`
/// 打上子序号（`feature` 是暖启动缓存键的一部分，不并会让不同子形状的接触点互相顶替）。
fn expand_compound_a(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>, compound: u32) {
    let Some(kids) = np.kids_take(compound) else {
        return;
    };
    for (ci, kid) in kids.iter().enumerate() {
        let before = args.out.len();
        let cpos = args.pa + Mat3::from_quat(args.ra).mul_vec3(kid.offset);
        let crot = args.ra * kid.rot;
        let mut child = PairArgs {
            a: args.a,
            b: args.b,
            bodies: args.bodies,
            sa: &kid.shape,
            sb: args.sb,
            pa: cpos,
            ra: crot,
            pb: args.pb,
            rb: args.rb,
            heightfields: args.heightfields,
            providers: args.providers,
            out: &mut *args.out,
        };
        process_pair_shaped(np, &mut child);
        tag_child_features(&mut args.out[before..], ci);
    }
    np.kids_put(kids);
}

/// **b 侧复合体展开**：与 [`expand_compound_a`] 镜像（理由见那份注）。
fn expand_compound_b(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>, compound: u32) {
    let Some(kids) = np.kids_take(compound) else {
        return;
    };
    for (ci, kid) in kids.iter().enumerate() {
        let before = args.out.len();
        let cpos = args.pb + Mat3::from_quat(args.rb).mul_vec3(kid.offset);
        let crot = args.rb * kid.rot;
        let mut child = PairArgs {
            a: args.a,
            b: args.b,
            bodies: args.bodies,
            sa: args.sa,
            sb: &kid.shape,
            pa: args.pa,
            ra: args.ra,
            pb: cpos,
            rb: crot,
            heightfields: args.heightfields,
            providers: args.providers,
            out: &mut *args.out,
        };
        process_pair_shaped(np, &mut child);
        tag_child_features(&mut args.out[before..], ci);
    }
    np.kids_put(kids);
}
