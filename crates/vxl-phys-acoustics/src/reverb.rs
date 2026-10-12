//! **A7「统计混响尾巴」**：早期声由显式路径给（A1–A6），**晚段**（密集反射）用统计模型补上。
//!
//! 三条口径：
//!
//! 1. **只有 RT60 是物理的**：`RT60_b = 0.161 · V / (S · ᾱ_b)`（Sabine）—— `V` = 声学几何**围出的
//!    体积**（散度定理逐三角求和）、`S` = 总表面积、`ᾱ_b` = **面积加权**平均吸声（逐带）。三个量
//!    全部来自 [`AcousticGeometry`] 自己的表（不看碰撞体、不看 `World`）；
//! 2. **干湿比是工程常数**（[`MIX`]）：尾巴相对早期声的电平 —— **不是物理量**，宿主可自行缩放；
//!    本层只保证单调性（越吸声越短越弱、全反射最响）；
//! 3. **尾巴是确定性的**：合成在 [`crate::audio::mix_reverb_into`]（定种 LCG），无随机种子、不看时钟。
//!
//! - 想要尾巴但三角网不闭合（只建了"参与显式路径的那几面"）：用 [`reverb_params_from`] **声明**
//!   包络（`V` / `S`）—— 与 [`crate::Portal`]/[`crate::Edge`] 同一条"显式拓扑"思路。
//!
//! ⚠️ **边界（如实登记）**：
//!
//! - 体积只在**闭合**几何里才有意义 ⇒ 几何不闭合（或体积 / 面积退化）时**关掉尾巴**（`mix = 0`）；
//! - Sabine 口径里**只有吸声**：portal 漏出去的能量与墙的透射都不参与（v1）；
//! - 用声明包络时，**没建出来的面按零吸声**算（保守：`ᾱ` 被摊薄 ⇒ 尾巴偏长）；
//! - 这是**统计**尾巴，不是"第 2..N 次镜像反射逐个展开"（那要指数爆炸 + 可见性剪枝，见调研文档 §9）。

use crate::{tri_of, AcousticConfig, AcousticGeometry, BANDS};

/// 尾巴的**干湿比**（工程常数）：`mix_b = MIX · (1 − ᾱ_b)` —— 全反射 ≈ `MIX`、全吸收 ⇒ 0。
///
/// ⚠️ 这是**工程口径**（不是从物理量导出的）：尾巴的幅度包络 ≈ `MIX · 干信号的包络`
/// ⇒ `0.2` ≈ 比干信号低 **14 dB**（听得见、但不盖过早期声）——这个量级与"直达/混响能量比"\r\n/// `16πd²/(c·S·ᾱ)` 在 `d ≈ 3 m`、`S ≈ 120 m²`、`ᾱ ≈ 0.25` 下的估计（≈ −13 dB）对得上。想更"湿"就调它（或让宿主按自己的
/// 混音比例缩放）。
///
/// ⚠️ 它**不是**"多声源时的总电平"：多个事件在时间轴上会**不相干地**叠加（各自的噪声相位不同，
/// 见 [`crate::audio::mix_reverb_into`] 的种子）⇒ 密集场景里尾巴自然更厚，这是对的。
pub const MIX: f32 = 0.2;

/// Sabine 常数（米制：`RT60 = 0.161 · V / (S · α)`）。
const SABINE: f32 = 0.161;

/// **统计混响参数**（逐带）：尾巴多长（RT60）、多响（干湿比）、从哪一刻接上（平均自由程）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReverbParams {
    /// 逐带混响时间（秒）。
    pub rt60: [f32; BANDS],
    /// 逐带干湿比（0 = 没尾巴）。
    pub mix: [f32; BANDS],
    /// 尾巴起点（秒）＝ 平均自由程 / 声速（`4V/(S·c)`）—— 晚段不可能比这更早。
    pub onset: f32,
}

impl ReverbParams {
    /// 有没有尾巴（全带 `mix = 0` ⇒ 关）。
    pub fn is_on(&self) -> bool {
        self.mix.iter().any(|m| *m > 0.0)
    }
}

/// 从**声学几何自己的表**算统计混响（纯函数：同输入逐位同输出）。
///
/// 体积与表面积**从三角网推**（散度定理 + 面积和）⇒ 只有**闭合**几何才算得出来；几何不闭合
/// （开场景 / 只建了参与显式路径的那几面）时返回 `mix = 0`（关掉尾巴）—— 想要尾巴就显式声明
/// 包络，见 [`reverb_params_from`]。
pub fn reverb_params(geo: &AcousticGeometry<'_>, cfg: &AcousticConfig) -> ReverbParams {
    let (area, weighted, six_v) = sweep(geo);
    let v = (six_v / 6.0).abs();
    if !is_closed(geo) || area <= 1e-6 || v <= 1e-6 {
        return ReverbParams::default(); // 开场景 / 退化几何 ⇒ 没有"房间"可言
    }
    finish(cfg, v, area, &weighted)
}

/// 用**作者声明的包络**（`volume` / `surface`）算统计混响：吸声仍**面积加权**地取自几何表。
///
/// 什么时候用它：三角网只建了"参与显式路径的那几面"（比如 `vxl-phys-cli` 的演示房间是地板 +
/// 带门洞的墙 + 天花板，四面敞开）—— 这时"体积"是**作者声明**的，跟 [`crate::Portal`]/[`crate::Edge`] 一样是
/// **显式拓扑**（我们不装作能从开着的三角网里量出房间体积）。`volume` / `surface` 非正 ⇒ 关尾巴。
pub fn reverb_params_from(
    geo: &AcousticGeometry<'_>,
    cfg: &AcousticConfig,
    volume: f32,
    surface: f32,
) -> ReverbParams {
    // ⚠️ 写成"合取取反"而不是 `volume <= 0.0 || surface <= 0.0` 是刻意的：NaN 也走"关掉"这条路
    // （`<=` 对 NaN 是 false ⇒ 会把 NaN 当合法参数一路算下去）。
    if !(volume > 0.0 && surface > 0.0) {
        return ReverbParams::default();
    }
    let (_, weighted, _) = sweep(geo);
    finish(cfg, volume, surface, &weighted)
}

/// 公共尾巴：`RT60_b = 0.161·V/(S·ᾱ_b)`、`onset = 4V/(S·c)`、`mix_b = MIX·(1−ᾱ_b)`。
fn finish(
    cfg: &AcousticConfig,
    volume: f32,
    surface: f32,
    weighted: &[f32; BANDS],
) -> ReverbParams {
    let mut rv = ReverbParams::default();
    let speed = cfg.speed_of_sound.max(1e-3);
    rv.onset = 4.0 * volume / (surface * speed);
    for (b, slot) in rv.rt60.iter_mut().enumerate() {
        // ⚠️ 吸声的下限：`ᾱ → 0` 时 Sabine 的 RT60 → ∞ ⇒ 截在 1e-3（工程上"几乎不吸声"的房间
        // 也该有个头）。这也保证下面不会除出 inf。
        let alpha = (weighted[b] / surface).clamp(1e-3, 1.0);
        *slot = SABINE * volume / (surface * alpha);
        rv.mix[b] = MIX * (1.0 - alpha);
    }
    rv
}

/// 逐三角累加：总表面积、逐带吸声的**面积加权**和、以及带符号体积的六倍（散度定理）。
fn sweep(geo: &AcousticGeometry<'_>) -> (f32, [f32; BANDS], f32) {
    let (mut area, mut weighted, mut six_v) = (0.0f32, [0.0f32; BANDS], 0.0f32);
    for (i, t) in geo.tris.iter().enumerate() {
        let Some((a, b, c, _)) = tri_of(geo, i, *t) else {
            continue;
        };
        let cross = (b - a).cross(c - a);
        area += 0.5 * cross.length();
        let absorb = geo.absorption.get(i).copied().unwrap_or([0.0; BANDS]);
        for (w, al) in weighted.iter_mut().zip(absorb.iter()) {
            *w += 0.5 * cross.length() * al.clamp(0.0, 1.0);
        }
        six_v += a.dot(b.cross(c));
    }
    (area, weighted, six_v)
}

/// 几何是否**闭合**（每条无向边恰好出现两次、方向相反）—— 体积只对闭合面才有意义。
///
/// 用排序而不是哈希：判据只看"是不是闭合"这一个布尔量，排序的确定性更好说清楚。
fn is_closed(geo: &AcousticGeometry<'_>) -> bool {
    let mut edges: Vec<(u32, u32, i8)> = Vec::with_capacity(geo.tris.len() * 3);
    for (i, t) in geo.tris.iter().enumerate() {
        if tri_of(geo, i, *t).is_none() {
            return false; // 索引越界 ⇒ 当不闭合
        }
        for k in 0..3 {
            let (p, q) = (t[k], t[(k + 1) % 3]);
            edges.push((p.min(q), p.max(q), if p < q { 1 } else { -1 }));
        }
    }
    if !edges.len().is_multiple_of(2) {
        return false;
    }
    edges.sort_unstable();
    edges
        .chunks(2)
        .all(|w| w[0].0 == w[1].0 && w[0].1 == w[1].1 && w[0].2 != w[1].2)
}
