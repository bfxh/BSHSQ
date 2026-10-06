//! **形状对的分派**：一张**有序规则表** + 上下文结构（2026-10-06 从
//! `DefaultNarrowPhase::process_pair_shaped` 里搬出来并数据化）。
//!
//! **为什么**（god 债务的**设计层**处置，见 `god.gate.json`）：分派曾经是长在 `impl
//! DefaultNarrowPhase` 里的一段 `if` 链 —— 每加一个形状/域都要去编辑那个 59 方法的类型。
//! 现在：**顺序 = 规则表的数据**，加一个域 = 往 [`RULES`] **追加一条规则**（谓词 + 处理器），
//! `impl` 块与 `process_pair_shaped` 本体都**不用动**。验收句就是债务里那句
//! "新加一个形状或域**不必再动本类型**"。
//!
//! ⚠️ **顺序即语义**：规则**从上到下、先匹配先赢**。这张表的顺序与原 `if` 链逐条对应，
//! 每条的谓词 = 原那条 `if`/`return true` 的成立条件：
//! 1. `compound_a`（a 侧复合体）→ 展开子对并递归（`kids_take` 失败也**已处理**，与原码一致）；
//! 2. `compound_b` 同上（镜像）；
//! 3. `provider`（任一侧是 `Shape::Provider`）→ `provider_pair`（无提供者时它本会返回
//!    `false` 落空，故谓词与它**等价**）；
//! 4. `convex_hull`（任一侧是外壳）→ `hull_pair`（原码命中后无条件 `return`）；
//! 5. `heightfield`（任一侧是高度场）→ `heightfield_pair`（同 3 的等价性论证）；
//! 6. `generic`（**无条件**、必须排在最后）→ `pair_non_heightfield`。
//!
//! 改顺序 = 改语义；本文件的判据就是 `determinism` / `m0_gates` 两条冻结哈希逐位不变 +
//! `vxl-phys-narrow` 全部测试（含 `hf_provider_dispatch_parity` 三档对拍与
//! `vxl-phys` 的 `compound_warm`）。
//!
//! **[`PairArgs`] 为什么存在**：① `args-gate` 对**新文件零基线**，函数形参 >7 直接红
//! （12 个位置参数搬不过来）；② 处理器要**统一签名**，上下文必须是可传递的对象。
//! 复合体递归**新建子上下文**而不是改父的字段：借用生命周期绑在结构上，原地改字段会把
//! `&'a Shape` 收窄，compiler 直接拒。

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

/// **一条分派规则**。`name` 只用于诊断/文档；真正的语义是 [`RULES`] 的**顺序**。
pub(crate) struct Rule {
    pub(crate) name: &'static str,
    pub(crate) matches: fn(&PairArgs<'_>) -> bool,
    pub(crate) handle: fn(&mut DefaultNarrowPhase, &mut PairArgs<'_>),
}

/// **分派规则表**（从上到下、先匹配先赢；顺序与旧 `if` 链逐条对应，见文件头）。
/// **加一个域 = 在 `generic` 之前追加一条**（谓词 + 处理器），别动别的。
pub(crate) static RULES: &[Rule] = &[
    Rule {
        name: "compound_a",
        matches: is_compound_a,
        handle: handle_compound_a,
    },
    Rule {
        name: "compound_b",
        matches: is_compound_b,
        handle: handle_compound_b,
    },
    Rule {
        name: "provider",
        matches: is_provider_pair,
        handle: handle_provider_pair,
    },
    Rule {
        name: "convex_hull",
        matches: is_hull_pair,
        handle: handle_hull_pair,
    },
    Rule {
        name: "heightfield",
        matches: is_heightfield_pair,
        handle: handle_heightfield_pair,
    },
    Rule {
        name: "generic",
        matches: is_generic_pair,
        handle: handle_generic_pair,
    },
];

/// **配对主入口**：复位每对 scratch → 按 [`RULES`] 找第一条命中的规则 → 交给它。
pub(crate) fn process_pair_shaped(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    // 盒对专用路径开关：每对先复位（非盒对 / 圆柱对一律走通用路径）。
    np.ws.box_axes_a = None;
    np.ws.box_axes_b = None;
    // **兜底必须在最后**：它是无条件的 ⇒ 任何追加在它**之后**的规则都会被静默吞掉
    // （"加形状改哪"这一刀最容易踩的坑）。debug 档（CI 的 test-debug job）每对检一次。
    debug_assert!(
        RULES.last().is_some_and(|r| r.name == "generic"),
        "分派规则表的最后一条必须是无条件兜底 generic；新规则请追加在它之前"
    );
    for rule in RULES {
        if (rule.matches)(args) {
            (rule.handle)(np, args);
            return;
        }
    }
}

fn is_compound_a(a: &PairArgs<'_>) -> bool {
    matches!(*a.sa, Shape::Compound { .. })
}

fn is_compound_b(a: &PairArgs<'_>) -> bool {
    matches!(*a.sb, Shape::Compound { .. })
}

fn is_provider_pair(a: &PairArgs<'_>) -> bool {
    matches!(*a.sa, Shape::Provider(_)) || matches!(*a.sb, Shape::Provider(_))
}

fn is_hull_pair(a: &PairArgs<'_>) -> bool {
    matches!(*a.sa, Shape::ConvexHull { .. }) || matches!(*a.sb, Shape::ConvexHull { .. })
}

fn is_heightfield_pair(a: &PairArgs<'_>) -> bool {
    matches!(*a.sa, Shape::HeightField(_)) || matches!(*a.sb, Shape::HeightField(_))
}

/// 兜底：**无条件**（必须排在 [`RULES`] 最后）。
fn is_generic_pair(_: &PairArgs<'_>) -> bool {
    true
}

fn handle_compound_a(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    let Shape::Compound { compound, .. } = *args.sa else {
        return;
    };
    expand_compound_a(np, args, compound);
}

fn handle_compound_b(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    let Shape::Compound { compound, .. } = *args.sb else {
        return;
    };
    expand_compound_b(np, args, compound);
}

fn handle_provider_pair(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    super::provider_pair(
        np,
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
    );
}

fn handle_hull_pair(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    crate::support::hull_pair(
        np,
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
}

fn handle_heightfield_pair(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    super::heightfield_pair(
        np,
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
}

fn handle_generic_pair(np: &mut DefaultNarrowPhase, args: &mut PairArgs<'_>) {
    super::pair_non_heightfield(
        np,
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
