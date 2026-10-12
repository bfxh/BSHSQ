//! world_ccd：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) mod sweep;

impl World {
    /// 选择性 CCD（§4.12）：保守推进采样。
    ///
    /// 位置积分已把体推到 p1；这里从 p0 = p1 − v·dt 回扫到 p1，找到首个与
    /// 静态/沉睡目标接触的采样段，把体钳位到上一安全采样点，并清零指向
    /// 表面的法向速度（M1 非弹性停下）。
    ///
    /// **审计 F-15 的两处修复在 `sweep.rs`**：② 采样预算不够覆盖位移时**限速**而不是静默截段；
    /// ③ 目标集**含运动体**并按相对运动采样与判定（响应仍是非弹性的单向近似，双向留给 M2）。
    /// 返回**本子步被标记要扫掠的体**（供 CCD 之后的关节位置投影用；关档/无标记 ⇒ 空）。
    pub(crate) fn ccd_pass(&mut self, dt: f32) -> Vec<usize> {
        if !self.config.ccd_speed_threshold.is_finite() {
            return Vec::new();
        }
        let n = self.bodies.len();
        let flagged: Vec<usize> = (0..n)
            .filter(|&i| ccd::needs_ccd(&self.bodies, i, &self.config, dt))
            .collect();
        for &i in &flagged {
            sweep::sweep(self, i, dt);
        }
        flagged
    }
}
