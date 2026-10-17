//! breakage：关节**可断裂**的状态与判定（从 types.rs / solve.rs 按域拆出）。
//!
//! 判据用**力**（本子步最大冲量 / `dt`）而不是几何拉伸：这个求解器的位置约束极硬
//! （实测给 5 m/s 初速，锚点间隙也只涨不到 0.01 m）⇒ 拉伸量没有可用量级。
//! 冲量取自 `solve_joint` 的返回值（它就是 `lambda.abs()`），因此不需要改任何签名。
//!
//! 状态放 `Breakage` 里、而不是 `Joint` 里：`Joint` 是按 `Copy` 传的 IR，状态混进去会被
//! 复制走，也让 IR 不再是纯输入。断开是**永久**的（不自动恢复）。
use super::types::{Joint, JointSet};

/// 与 `JointSet.joints` **同索引**的断裂状态（每子步复用，不分配）。
#[derive(Default)]
pub(crate) struct Breakage {
    pub(crate) broken: Vec<bool>,
    /// 本子步最大冲量（每子步清零）。
    pub(crate) impulse: Vec<f32>,
    /// 各关节的断裂力（N；`<= 0` = 不断）。放这里而不是 `Joint` 里：
    /// `max_type_members` 是棘轮、只准降，`Joint` 不能多字段。
    pub(crate) forces: Vec<f32>,
}

impl Breakage {
    /// 与关节数对齐（`joints` 是 pub 字段，可能被直接 push 过），并把冲量清零。
    pub(crate) fn prepare(&mut self, n: usize) {
        if self.broken.len() != n {
            self.broken.resize(n, false);
        }
        if self.impulse.len() != n {
            self.impulse.resize(n, 0.0);
        }
        if self.forces.len() != n {
            self.forces.resize(n, 0.0);
        }
        self.impulse.iter_mut().for_each(|v| *v = 0.0);
    }
}

impl JointSet {
    /// 加一条**可断裂**关节：等同 [`Self::add`] + 登记本索引的断裂力 `force`（N；`<= 0` = 不断）。
    ///
    /// 断裂力记在 `Breakage` 的并行数组里而不是 `Joint` 上 —— 见 `Breakage::forces` 的注释。
    pub fn add_breakable(&mut self, j: Joint, force: f32) -> u32 {
        let id = self.add(j);
        self.set_break(id, force);
        id
    }

    /// 改某个已有关节的断裂力（`<= 0` = 不断）。越界静默忽略（与 `is_broken` 同风格）。
    pub fn set_break(&mut self, id: u32, force: f32) {
        if let Some(slot) = self.breakage.forces.get_mut(id as usize) {
            *slot = force;
        }
    }

    /// 该关节是否**已断**（越界或尚未同步一律 `false`）。断开是永久的（不自动恢复）。
    pub fn is_broken(&self, id: u32) -> bool {
        self.breakage
            .broken
            .get(id as usize)
            .copied()
            .unwrap_or(false)
    }

    /// 已断关节数（金样与诊断用）。
    pub fn broken_count(&self) -> usize {
        self.breakage.broken.iter().filter(|b| **b).count()
    }

    /// **可断裂判定**：本子步该关节的最大约束力超过 `break_force` ⇒ 永久断开。
    ///
    /// 力 = 冲量 / `dt`；`break_force <= 0` 的关节不判（默认值 ⇒ 等价于没有这个功能）。
    /// 判定在**解完之后**统一做一次 ⇒ 与迭代数无关；断开后 `solve` 直接跳过该关节。
    pub(crate) fn check_break(&mut self, dt: f32) {
        let inv_dt = 1.0 / dt.max(1e-6);
        for i in 0..self.joints.len() {
            let force = self.breakage.forces[i];
            if self.breakage.broken[i] || force <= 0.0 {
                continue;
            }
            if self.breakage.impulse[i] * inv_dt > force {
                self.breakage.broken[i] = true;
            }
        }
    }
}
