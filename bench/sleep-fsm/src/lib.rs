//! **睡眠/唤醒判据的三值审计**（`vxl-phys-solver/src/solver_impl/sleep.rs` 的判据侧）。
//!
//! 为什么是"三值"而不是"二值枚举"：判据的输入是 **f32 与阈值的比较**，而本仓自己量过
//! 跨平台位姿差有界而**不逐位**（`docs/PHYSICS-INVARIANTS.md` §2.4，最大差 1.2e-4）。
//! 落在阈值 ±ε 内的比较，在另一个平台/另一条浮点路径上是**可能翻转**的 ⇒ 它既不是真也不是假，
//! 而是 `X`。Kleene K3 正好是"不知道"的代数：`X ∧ F = F`、`X ∧ T = X`。
//! 判决口径：
//! - **Proved**：全空间（含 `X` 两种落法）都成立；
//! - **Undetermined**：只有把某些 `X` 当 T 才成立 ⇒ 结论**依赖浮点舍入**；
//! - **Refuted**：存在二值反例。
//!
//! 模型是**判据的镜像**，不是引擎本身；它与引擎的对应关系由
//! `tests/model_matches_measured.rs` 用**实测读数**钉住（四个测点）。

use trit_core::Algebra;
pub use trit_core::Ternary;

/// 本仓统一用 Kleene 强三值逻辑（`X` = 不知道，非毒值）。
pub const K3: Algebra = Algebra::Kleene;

/// 判据模型版本。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// 修复前：绕过接触数门的只有 `kin_deep` / `wake_gate_k == 0` / 强撞。
    Before,
    /// 修复后：另加"运动学支撑在动"（与 `kin_deep` 同一理由：宿主确定输入，不是抖动）。
    After,
}

/// 判据里的物理常数（与 `PhysConfig` / `wake_signal.rs` 同值）。
#[derive(Clone, Copy, Debug)]
pub struct Consts {
    pub sleep_linear: f32,
    pub sleep_angular: f32,
    pub contact_skin: f32,
    /// `wake_signal.rs` 的 `KIN_MOTION_EPS`：运动学体接触点的"非零运动"死区。
    pub kin_eps: f32,
}

impl Default for Consts {
    fn default() -> Self {
        Self {
            sleep_linear: 0.04,
            sleep_angular: 0.05,
            contact_skin: 0.02,
            kin_eps: 1e-6,
        }
    }
}

/// 一条流形的抽象输入（每个谓词都可以是 `X`）。
#[derive(Clone, Copy, Debug)]
pub struct Preds {
    pub kin: Ternary,
    pub deep: Ternary,
    pub rel_eps: Ternary,
    pub rel_2s: Ternary,
    pub ang_2a: Ternary,
    pub rel_8s: Ternary,
}

impl Preds {
    /// `hot`：`wake_signal()` 的中间量。`kin` 侧只看 `deep ∨ rel > KIN_MOTION_EPS`，
    /// 动态侧看 `rel > 2×sleep_linear ∨ |ω| > 2×sleep_angular`。
    pub fn hot(&self) -> Ternary {
        let hot_kin = K3.or(self.deep, self.rel_eps);
        let hot_dyn = K3.or(self.rel_2s, self.ang_2a);
        ite(self.kin, hot_kin, hot_dyn)
    }

    /// 绕过接触数门的条件（`wake_gate_k > 0` 时才有意义）。
    pub fn gate_pass(&self, model: Model) -> Ternary {
        let base = K3.or(self.deep, self.rel_8s);
        match model {
            Model::Before => base,
            Model::After => K3.or(base, K3.and(self.kin, self.hot())),
        }
    }

    /// 本调用内这个睡体是否被唤醒：`hot ∧ (gate_pass ∨ k == 0)`。
    pub fn wake(&self, wake_gate_k: u32, model: Model) -> Ternary {
        let gate = if wake_gate_k == 0 {
            Ternary::True
        } else {
            self.gate_pass(model)
        };
        K3.and(self.hot(), gate)
    }

    /// 已醒的体本调用内是否入睡：`¬wake ∧ ¬carried ∧ slow ∧ timer_ready`。
    ///
    /// `wake` 进来是因为被唤醒时 `sleep_timer` 刚被清零 ⇒ 同一调用内不该再入睡。
    pub fn sleep(
        &self,
        wake: Ternary,
        carried: Ternary,
        slow: Ternary,
        timer_ready: Ternary,
    ) -> Ternary {
        K3.and(
            K3.not(wake),
            K3.and(K3.not(carried), K3.and(slow, timer_ready)),
        )
    }
}

/// K3 的选择：条件为 `X` 时，两支相同才取值，否则仍为 `X`。
pub fn ite(c: Ternary, t: Ternary, e: Ternary) -> Ternary {
    match c {
        Ternary::True => t,
        Ternary::False => e,
        Ternary::Unknown => {
            if t == e {
                t
            } else {
                Ternary::Unknown
            }
        }
    }
}

/// 比较 `v > t` 的三值化：`|v - t| < eps` ⇒ `X`（`eps = 0` 时退化为二值）。
pub fn tri_gt(v: f32, t: f32, eps: f32) -> Ternary {
    if (v - t).abs() < eps {
        Ternary::Unknown
    } else if v > t {
        Ternary::True
    } else {
        Ternary::False
    }
}

/// 一个具体场景（物理量，不是谓词）。
#[derive(Clone, Copy, Debug)]
pub struct Scenario {
    pub kin: bool,
    /// 接触点相对速度（m/s）——对睡体一侧它等于对面那个体的接触点速度。
    pub rel: f32,
    /// 角速度（rad/s，动态侧才用）。
    pub ang: f32,
    /// 穿透深度（m）。
    pub depth: f32,
    pub wake_gate_k: u32,
}

impl Scenario {
    /// 把物理量三值化成谓词；`eps` = 跨平台分歧带。
    pub fn preds(&self, c: &Consts, eps: f32) -> Preds {
        Preds {
            kin: Ternary::from_bool(self.kin),
            deep: tri_gt(self.depth, 0.5 * c.contact_skin, eps),
            rel_eps: tri_gt(self.rel, c.kin_eps, eps),
            rel_2s: tri_gt(self.rel, 2.0 * c.sleep_linear, eps),
            ang_2a: tri_gt(self.ang, 2.0 * c.sleep_angular, eps),
            rel_8s: tri_gt(self.rel, 8.0 * c.sleep_linear, eps),
        }
    }
}

/// 判决。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Proved,
    Undetermined,
    Refuted,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Verdict::Proved => "Proved",
            Verdict::Undetermined => "Undetermined",
            Verdict::Refuted => "Refuted",
        }
    }
}

/// 一行审计结果。
#[derive(Clone, Copy, Debug)]
pub struct Row {
    pub s: Scenario,
    pub before: Ternary,
    pub after: Ternary,
    pub x_inputs: usize,
}

/// 审计报告：闸门只看 `p_wake`。
#[derive(Clone, Debug)]
pub struct Audit {
    pub rows: Vec<Row>,
    pub p_wake_before: Verdict,
    pub p_wake_after: Verdict,
    pub x_rows: usize,
    pub refuted_before: usize,
}

/// 性质：**运动学支撑在动、且不深、不到强撞、接触数门开着（K>1）⇒ 本调用内必须唤醒**。
///
/// 这正是"缓慢移动的平台托着睡着的载荷"那一支；`k == 0` 与强撞另有门看着，
/// `deep` 也另有门（`kin_deep`），故只审这一格。
pub fn is_p_wake_cell(s: &Scenario, c: &Consts) -> bool {
    s.kin
        && s.rel > c.kin_eps * 1000.0
        && s.depth < 0.5 * c.contact_skin
        && s.rel < 8.0 * c.sleep_linear
        && s.wake_gate_k > 1
}

/// 扫描网格：相对速度 × 角速度 × 深度 × `wake_gate_k` × 静态/运动学。
const RELS: [f32; 16] = [
    0.0, 1e-3, 0.02, 0.039, 0.04, 0.041, 0.06, 0.08, 0.081, 0.15, 0.30, 0.319, 0.32, 0.321, 0.5,
    3.0,
];
const ANGS: [f32; 3] = [0.0, 0.099, 0.101];
const DEPTHS: [f32; 4] = [0.0, 0.004, 0.011, 0.02];
const KS: [u32; 2] = [0, 8];

fn scenario_count() -> usize {
    RELS.len() * ANGS.len() * DEPTHS.len() * KS.len() * 2
}

/// 把线性下标散回 `Scenario`（避免五层嵌套循环把 nest 门顶红）。
fn scenario_at(idx: usize) -> Scenario {
    let (n_rel, n_ang, n_depth, n_k) = (RELS.len(), ANGS.len(), DEPTHS.len(), KS.len());
    let rel = RELS[idx % n_rel];
    let ang = ANGS[(idx / n_rel) % n_ang];
    let depth = DEPTHS[(idx / (n_rel * n_ang)) % n_depth];
    let wake_gate_k = KS[(idx / (n_rel * n_ang * n_depth)) % n_k];
    let kin = (idx / (n_rel * n_ang * n_depth * n_k)) % 2 == 0;
    Scenario {
        kin,
        rel,
        ang,
        depth,
        wake_gate_k,
    }
}

/// 扫一遍阈值空间，返回逐行读数与两个判决。
pub fn audit(c: &Consts, eps: f32) -> Audit {
    let rows: Vec<Row> = (0..scenario_count())
        .map(|idx| {
            let s = scenario_at(idx);
            let p = s.preds(c, eps);
            let preds = [p.deep, p.rel_eps, p.rel_2s, p.ang_2a, p.rel_8s];
            let x_inputs = preds.iter().filter(|v| **v == Ternary::Unknown).count();
            Row {
                s,
                before: p.wake(s.wake_gate_k, Model::Before),
                after: p.wake(s.wake_gate_k, Model::After),
                x_inputs,
            }
        })
        .collect();
    let p_wake_before = verdict_of(&rows, Model::Before, c);
    let p_wake_after = verdict_of(&rows, Model::After, c);
    let x_rows = rows.iter().filter(|r| r.x_inputs > 0).count();
    let refuted_before = rows
        .iter()
        .filter(|r| is_p_wake_cell(&r.s, c) && r.before == Ternary::False)
        .count();
    Audit {
        rows,
        p_wake_before,
        p_wake_after,
        x_rows,
        refuted_before,
    }
}

fn verdict_of(rows: &[Row], model: Model, c: &Consts) -> Verdict {
    let mut any_x = false;
    for r in rows.iter().filter(|r| is_p_wake_cell(&r.s, c)) {
        let v = match model {
            Model::Before => r.before,
            Model::After => r.after,
        };
        match v {
            Ternary::False => return Verdict::Refuted,
            Ternary::Unknown => any_x = true,
            Ternary::True => {}
        }
    }
    if any_x {
        Verdict::Undetermined
    } else {
        Verdict::Proved
    }
}

#[cfg(test)]
mod tests {
    use super::{audit, Consts, Preds, Ternary, Verdict};

    #[test]
    fn before_fix_refutes_after_fix_proves() {
        let a = audit(&Consts::default(), 0.0);
        assert_eq!(a.p_wake_before, Verdict::Refuted);
        assert_eq!(a.p_wake_after, Verdict::Proved);
        assert!(a.refuted_before > 0);
    }

    #[test]
    fn zero_eps_has_no_unknown_inputs() {
        let a = audit(&Consts::default(), 0.0);
        assert_eq!(a.x_rows, 0, "eps=0 时不应有 X 谓词");
    }

    #[test]
    fn epsilon_creates_unknowns_only_near_thresholds() {
        let a = audit(&Consts::default(), 1e-3);
        assert!(a.x_rows > 0, "eps>0 时应出现 X 谓词");
        // 远离任何阈值的格子仍必须是二值
        for r in &a.rows {
            let far = (r.s.rel - 0.15).abs() < 1e-9 && r.s.ang == 0.0 && r.s.depth == 0.0;
            if far {
                assert_eq!(r.x_inputs, 0);
            }
        }
    }

    #[test]
    fn carried_veto_is_proved_in_all_algebras_of_k3() {
        // ¬carried ∧ ... ⇒ 只要 carried = True，入睡必为 False（含 X 的其他项）
        for slow in Ternary::ALL {
            for timer in Ternary::ALL {
                for wake in Ternary::ALL {
                    let p = Preds {
                        kin: Ternary::True,
                        deep: Ternary::False,
                        rel_eps: Ternary::True,
                        rel_2s: Ternary::False,
                        ang_2a: Ternary::False,
                        rel_8s: Ternary::False,
                    };
                    let s = p.sleep(wake, Ternary::True, slow, timer);
                    assert_eq!(s, Ternary::False, "carried=True 时不该入睡");
                }
            }
        }
    }
}
