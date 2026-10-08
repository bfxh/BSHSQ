//! `BridgeState`：`StateBridge` 的**物理状态**形态（位置 + 速度 + 每点质量）。
//!
//! 接口面在 `interop.rs`（该文件受 god 门 file_lines 棘轮 ⇒ 只准减），状态数据与守恒判据
//! 放这里 —— 两者同属"跨域唯一通道"的接口层。
//!
//! 语义（三条，与 `StateBridge` 的两个默认方法逐条对应）：
//! ① **空段 = 未登记**：`vel` 空 = 该桥不搬速度（即旧"位置桥"语义），`mass` 空 = 质量未知；
//! ② 动量/角动量/动能只在 `vel` 与 `mass` **都登记且长度自洽**时可算 —— `None` 表示"不知道"，
//!    与"等于 0"是两件事，审计里必须分开；
//! ③ [`handoff`] 是**域间**交接入口：源导出 → 目标导入；目标拒绝 ⇒ `false`，且目标一字不动。

use crate::interop::StateBridge;
use crate::Vec3;

/// 物理状态快照：位置 + 速度 + 每点质量（按各桥自己的索引序）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BridgeState {
    /// 位置（长度 = 点数）。
    pub pos: Vec<Vec3>,
    /// 速度（同序同长；**空 = 未登记**）。
    pub vel: Vec<Vec3>,
    /// 每点质量（同序同长；**空 = 未登记** ⇒ 动量/动能不可算）。
    pub mass: Vec<f32>,
}

impl BridgeState {
    /// 从位置构造 = 旧"位置桥"的快照（速度/质量未登记）。
    pub fn from_positions(pos: Vec<Vec3>) -> Self {
        Self {
            pos,
            vel: Vec::new(),
            mass: Vec::new(),
        }
    }

    /// 三段长度自洽（空段 = 未登记，不参与判定）。
    pub fn is_consistent(&self) -> bool {
        let n = self.pos.len();
        (self.vel.is_empty() || self.vel.len() == n)
            && (self.mass.is_empty() || self.mass.len() == n)
    }

    /// 速度与质量**都已登记**且长度 = 点数（动量类判据的前提）。
    pub fn is_full(&self) -> bool {
        let n = self.pos.len();
        self.is_consistent() && self.vel.len() == n && self.mass.len() == n
    }

    /// 总质量 Σ mᵢ（质量未登记/不自洽/非有限 ⇒ `None`）。
    pub fn total_mass(&self) -> Option<f32> {
        if self.mass.is_empty() || !self.is_consistent() {
            return None;
        }
        let mut m = 0.0f32;
        for x in &self.mass {
            m += *x;
        }
        m.is_finite().then_some(m)
    }

    /// 总动量 Σ mᵢvᵢ（速度或质量未登记 ⇒ `None`）。
    pub fn momentum(&self) -> Option<Vec3> {
        if !self.is_full() {
            return None;
        }
        let mut p = Vec3::ZERO;
        for i in 0..self.pos.len() {
            p += self.vel[i] * self.mass[i];
        }
        finite(p).then_some(p)
    }

    /// 绕 `origin` 的角动量 Σ mᵢ·(posᵢ − origin) × vᵢ（登记要求同 [`Self::momentum`]）。
    pub fn angular_momentum_about(&self, origin: Vec3) -> Option<Vec3> {
        if !self.is_full() {
            return None;
        }
        let mut l = Vec3::ZERO;
        for i in 0..self.pos.len() {
            l += (self.pos[i] - origin).cross(self.vel[i]) * self.mass[i];
        }
        finite(l).then_some(l)
    }

    /// 动能 Σ ½mᵢ|vᵢ|²（**不含势能** —— 势能取决于环境，不在桥的账上）。
    pub fn kinetic_energy(&self) -> Option<f32> {
        if !self.is_full() {
            return None;
        }
        let mut e = 0.0f32;
        for i in 0..self.pos.len() {
            e += 0.5 * self.mass[i] * self.vel[i].length_squared();
        }
        e.is_finite().then_some(e)
    }
}

/// 三分量都有限（NaN/∞ ⇒ 判据量不可用，返回 `None` 而不是放行）。
fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// **域间状态交接**：`src` 导出状态 → `dst` 导入；返回 `dst` 是否接受。
/// 拒绝语义由各域的 `import_state` 负责（长度/口径不符 ⇒ `false`，且目标**一字不动**）。
pub fn handoff(src: &dyn StateBridge, dst: &mut dyn StateBridge) -> bool {
    let mut st = BridgeState::default();
    src.export_state(&mut st);
    dst.import_state(&st)
}
