//! **介质体积场（A3）**：声线沿途的**声速**与**吸收**。
//!
//! 设计与调研见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md`（§4 原则 ③：介质 = 体积参数场 ——
//! 用户口径里"可以按体积密度等等"落在这里）。
//!
//! 两条约定：
//!
//! - **只读、由调用方交进来**：本层**不读** `World` 的介质通道。`vxl_phys_core::interop`
//!   那个 `MediumField` 是**动态双向耦合**（带 `deposit(&mut …)`，采样给的是密度/动量），
//!   声学要的是声速与吸收 ⇒ 由调用方把那个场**适配**成本 crate 的 [`AcousticMedium`]
//!   （例如 `c = sqrt(γ·p/ρ)`）；[`LayeredMedium`] 就是这种适配的样板。
//! - **确定性**：能用[解析积分](AcousticMedium::segment_exact)就用（[`LayeredMedium`] 提供）；
//!   不能用就退回**固定步长中点法**（步长由 [`AcousticConfig::medium_step`] 给）——无随机、
//!   无自适应、无时间片 ⇒ 同输入逐字一致。
//!
//! [`AcousticConfig::medium_step`]: crate::AcousticConfig::medium_step

use crate::BANDS;
use vxl_phys::Vec3;

/// 介质在一点的参数（A3）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Medium {
    /// 声速（m/s）。
    pub speed_of_sound: f32,
    /// 沿程**吸收系数**（1/m，**逐频带**）：能量按 `exp(-α·len)` 衰减 ⇒ 高频带更大就是"更闷"。
    pub attenuation: [f32; BANDS],
}

impl Medium {
    /// 空气（常温、无吸收）——最常用的基准档。
    pub const AIR: Medium = Medium {
        speed_of_sound: 343.0,
        attenuation: [0.0; BANDS],
    };
}

/// 逐带能量衰减因子 `exp(-τ)`。
fn exp_bands(tau: &[f32; BANDS]) -> [f32; BANDS] {
    let mut out = [1.0f32; BANDS];
    for (o, t) in out.iter_mut().zip(tau.iter()) {
        *o = (-*t).exp();
    }
    out
}

/// **介质体积场**（A3）：只读采样。
pub trait AcousticMedium {
    /// `p` 处的介质。
    fn at(&self, p: Vec3) -> Medium;

    /// **可选的解析积分**：线段 `a→b` 的 `(时延秒, 逐带能量衰减因子)`；`None` = 本实现不给，
    /// 由调用方退回固定步长中点法。分层介质拿它当**精确解**（判据的"解析式对拍"就靠它）。
    fn segment_exact(&self, _a: Vec3, _b: Vec3) -> Option<(f32, [f32; BANDS])> {
        None
    }
}

/// 分层介质的一层：`y ∈ [lo, hi)` 用 `medium`。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slab {
    pub lo: f32,
    pub hi: f32,
    pub medium: Medium,
}

/// **沿 y 分层**的介质场：最常见的"穿层"形状（空气/水、地面/房间）。
///
/// `at` 取**第一层**命中的（`lo <= y < hi`，按 `slabs` 顺序 ⇒ 确定性）；层外一律 `outside`。
/// [`segment_exact`](AcousticMedium::segment_exact) 把线段按层边界切开逐层积分 ⇒
/// 时延与衰减都是**解析值**（判据拿它跟手算对拍）。
#[derive(Clone, Debug, PartialEq)]
pub struct LayeredMedium {
    pub slabs: Vec<Slab>,
    pub outside: Medium,
}

impl LayeredMedium {
    /// 两层的常用写法：`y < split` 用 `below`、其余用 `above`（覆盖全空间）。
    pub fn split(split: f32, below: Medium, above: Medium) -> Self {
        let lo = f32::NEG_INFINITY;
        let hi = f32::INFINITY;
        Self {
            slabs: vec![
                Slab {
                    lo,
                    hi: split,
                    medium: below,
                },
                Slab {
                    lo: split,
                    hi,
                    medium: above,
                },
            ],
            outside: above,
        }
    }

    /// `y` 处的介质（第一层命中；层外 = `outside`）。
    pub fn medium_at_y(&self, y: f32) -> Medium {
        for s in &self.slabs {
            if y >= s.lo && y < s.hi {
                return s.medium;
            }
        }
        self.outside
    }

    /// 线段与各层边界的交点参数（只收 `t ∈ (0,1)` 的），升序 —— 逐段积分的切点。
    fn cuts(&self, a: Vec3, b: Vec3) -> Vec<f32> {
        let mut ts: Vec<f32> = Vec::new();
        let dy = b.y - a.y;
        if dy.abs() > 1e-9 {
            for s in &self.slabs {
                for edge in [s.lo, s.hi] {
                    if edge.is_finite() {
                        let t = (edge - a.y) / dy;
                        if t > 1e-6 && t < 1.0 - 1e-6 {
                            ts.push(t);
                        }
                    }
                }
            }
        }
        // 同一条边界会被相邻两层各报一次 ⇒ 重复的 `t` 只会产生零长度区间（无影响）。
        ts.sort_by(|p, q| p.partial_cmp(q).unwrap_or(std::cmp::Ordering::Equal));
        ts
    }

    /// 逐段（层内）积分：时延 = Σ len/c、逐带能量衰减 = `exp(-Σ α_b·len)`。
    fn integrate(&self, a: Vec3, b: Vec3, c0: f32) -> (f32, [f32; BANDS]) {
        let mut delay = 0.0f32;
        let mut tau = [0.0f32; BANDS];
        let mut prev = 0.0f32;
        let mut cuts = self.cuts(a, b);
        cuts.push(1.0);
        for t in cuts {
            let p0 = a + (b - a) * prev;
            let p1 = a + (b - a) * t;
            let len = (p1 - p0).length();
            let m = self.medium_at_y(0.5 * (p0.y + p1.y));
            let c = if m.speed_of_sound > 0.0 {
                m.speed_of_sound
            } else {
                c0
            };
            delay += len / c;
            for (t, a) in tau.iter_mut().zip(m.attenuation.iter()) {
                *t += a.max(0.0) * len;
            }
            prev = t;
        }
        (delay, exp_bands(&tau))
    }
}

impl AcousticMedium for LayeredMedium {
    fn at(&self, p: Vec3) -> Medium {
        self.medium_at_y(p.y)
    }

    fn segment_exact(&self, a: Vec3, b: Vec3) -> Option<(f32, [f32; BANDS])> {
        Some(self.integrate(a, b, Medium::AIR.speed_of_sound))
    }
}

/// 一段直线 `a→b` 在介质里的 **`(时延, 逐带能量衰减因子)`**。
///
/// `None`（= 没有声明介质场）⇒ 均匀空气（`c0`、无吸收），**数值与 A1 / A2 逐位相同**
/// （衰减因子恒为 `1.0`，`1.0` 的乘法是精确的）。
///
/// `Some(f)`：先用 [`AcousticMedium::segment_exact`]（分层介质给解析值）；拿不到就固定步长
/// 中点法，`n = ceil(len / step)`、`step` 下限 `1e-3` ⇒ 显式采样、可复现。
pub(crate) fn seg_effect(
    m: Option<&dyn AcousticMedium>,
    a: Vec3,
    b: Vec3,
    c0: f32,
    step: f32,
) -> (f32, [f32; BANDS]) {
    let d = b - a;
    let len = d.length();
    if len <= 1e-6 {
        return (0.0, [1.0; BANDS]);
    }
    let Some(f) = m else {
        return (len / c0, [1.0; BANDS]);
    };
    if let Some(e) = f.segment_exact(a, b) {
        return e;
    }
    let step = step.max(1e-3);
    let n = ((len / step).ceil() as usize).max(1);
    let dt = len / n as f32;
    let dir = d * (1.0 / len);
    let mut delay = 0.0f32;
    let mut tau = [0.0f32; BANDS];
    for i in 0..n {
        let p = a + dir * ((i as f32 + 0.5) * dt);
        let md = f.at(p);
        let c = if md.speed_of_sound > 0.0 {
            md.speed_of_sound
        } else {
            c0
        };
        delay += dt / c;
        for (t, a) in tau.iter_mut().zip(md.attenuation.iter()) {
            *t += a.max(0.0) * dt;
        }
    }
    (delay, exp_bands(&tau))
}
