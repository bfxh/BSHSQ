//! **空间尾流**（`AeroState::wakes`）—— 把均匀风场改成"障碍物下游有速度亏损"。
//!
//! 为什么单开文件：`lib.rs` / `medium.rs` 都在 god 门 file_lines 基线上（只准减），而尾流是一个
//! 自洽的关注点（"空间某点的风是多少"）。
//!
//! **模型（唯象、确定性、可解析对拍）**：每个尾流源是一个**圆柱形亏损区**：
//! - 沿风向 `dir` 从源点向下游延伸 `length`，横向半径 `radius`；
//! - 亏损按"离源点沿风向的距离 `s`"线性衰减、按"横向距离 `r`"余弦平方衰减：
//!   `deficit(x) = strength · (1 − s/length) · cos²(π/2 · r/radius)`（在 `s ∈ [0,length]`、
//!   `r ≤ radius` 内；域外为 0）。
//! - `strength` 是**速度亏损比**（0–1）：`wind_at = wind · (1 − Σ deficit)`，钳到 `[0, wind]`。
//!
//! 为什么不用 Navier–Stokes 尾流解：本仓的气动域是**面元 + 均匀风场**的工程档，没有压力场；
//! 引入一个可调、可对拍、逐位确定的唯象亏损区，比"假装解了 N–S"诚实（口径同 `SURVEY` 里
//! 对"空间尾流"的登记）。多源叠加取**逐源亏损之和**（按 `wakes` 注册序 ⇒ 确定）。
//!
//! **默认零换代**：`wakes` 为空 ⇒ `wind_at` 直接返回 `base`（与旧 `sample` 逐位一致）。
use super::AeroState;
use vxl_phys_core::Vec3;

/// 一个尾流源（圆柱形亏损区，沿风向）。
#[derive(Clone, Copy, Debug)]
pub struct WakeSource {
    /// 源点（障碍物中心）。
    pub origin: Vec3,
    /// 沿风向的下游长度（m）。
    pub length: f32,
    /// 横向半径（m）。
    pub radius: f32,
    /// 速度亏损比（0–1；1 = 完全遮蔽）。
    pub strength: f32,
}

#[rustfmt::skip]
impl WakeSource {
    pub fn new(origin: Vec3, length: f32, radius: f32, strength: f32) -> Self {
        Self { origin, length, radius, strength }
    }
}

/// 单个源在 `x` 处的亏损比（0–1）。域外返回 0。
pub(crate) fn deficit_of(w: &WakeSource, x: Vec3, dir: Vec3) -> f32 {
    if w.length <= 0.0 || w.radius <= 0.0 || w.strength <= 0.0 {
        return 0.0;
    }
    let d = x - w.origin;
    let s = d.dot(dir);
    if !(0.0..=w.length).contains(&s) {
        return 0.0;
    }
    let lateral = d - dir * s;
    let r = lateral.length();
    if r > w.radius {
        return 0.0;
    }
    let along = 1.0 - s / w.length;
    let across = (core::f32::consts::FRAC_PI_2 * (r / w.radius)).cos();
    (w.strength * along * across * across).clamp(0.0, 1.0)
}

impl AeroState {
    /// **空间风场**：基准风（含动量槽修正）减去所有尾流源的亏损。
    ///
    /// `wakes` 为空 ⇒ 与旧口径逐位一致（多源按注册序求和，钳到 `[0, 1]`）。
    pub fn wind_at(&self, x: Vec3) -> Vec3 {
        let cfg = self.cfg;
        let mut base = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
        if cfg.air_mass > 0.0 {
            base += self.absorbed * (1.0 / cfg.air_mass);
        }
        if self.wakes.is_empty() {
            return base;
        }
        let len = base.length();
        if len <= 1e-12 {
            return base; // 无风 ⇒ 没有可亏损的量（方向也未定义）
        }
        let dir = base * (1.0 / len);
        let mut deficit = 0.0f32;
        for w in &self.wakes {
            deficit += deficit_of(w, x, dir);
        }
        base * (1.0 - deficit.clamp(0.0, 1.0))
    }
}
