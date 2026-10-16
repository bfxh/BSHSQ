//! FLIP **仿射传输层**：APIC 的 `C` 从**粒子场**现算（`flip.rs` 的传输半）。
//!
//! 口径与推导写在 `flip.rs` 模块头"APIC"节；这里只放三件纯函数级的事：`C·d`、
//! 一阶矩累加、`C = M·D⁻¹`，以及在粒子场上跑一遍估出逐粒子 `C`。
//!
//! **为什么不去读网格速度场**（上一版的写法）：PIC 的 `v ← v_网格` 重置把粒子场在同一格的
//! 8 个节点上拉平，从那个场估出的梯度是"被磨过一次"的 ⇒ 自洽退化回 PIC（实测 `l_keep`
//! 0.145 → 0.155，见 `docs/OPEN-PROBLEMS.md` P23）。粒子场的**跨格**差分别没有这道自洽环。
//!
//! **契约**：`estimate_from_field` 是 `(pos, vel)` 的纯函数，每子步现算、不落任何跨 tick 缓冲
//! ⇒ 逐位复现/保存恢复/状态继承三份契约原样成立（`C` **不进粒子状态**）。
use crate::grid::UniformGrid;
use crate::{FluidSystem, Vec3};

/// APIC 仿射项 `C·d`（`C` 行主序 3×3，**世界单位** 1/s）；返回值是速度增量（不是位移）。
#[inline]
pub(super) fn affine_apply(c: &[f32; 9], d: Vec3) -> Vec3 {
    Vec3::new(
        c[0] * d.x + c[1] * d.y + c[2] * d.z,
        c[3] * d.x + c[4] * d.y + c[5] * d.z,
        c[6] * d.x + c[7] * d.y + c[8] * d.z,
    )
}

/// 把一个样本累加进两个一阶矩（`M = Σ w·v⊗d`、`D = Σ w·d⊗d`，行主序 3×3；`D` 对称），
/// 以及与偏移无关的**速度散布**标量 `T = Σ w·|v|²`（`|v|` 是相对 `v_i` 的差 ⇒ `T` 是局部总散布）。
#[inline]
fn apic_accumulate(m: &mut [f32; 9], d: &mut [f32; 9], t: &mut f32, w: f32, v: Vec3, off: Vec3) {
    *t += w * (v.x * v.x + v.y * v.y + v.z * v.z);
    let (vs, os) = ([v.x, v.y, v.z], [off.x, off.y, off.z]);
    for a in 0..3 {
        for b in 0..3 {
            m[a * 3 + b] += w * vs[a] * os[b];
            d[a * 3 + b] += w * os[a] * os[b];
        }
    }
}

/// 仿射矩阵 `C = M·D⁻¹`（`D` 对称 3×3，余子式求逆），带**岭正则**与**拟合质量收缩**。
///
/// **退化判据是相对的**（`D` 是世界单位 ⇒ 绝对值随 `h` 缩放，没有可写死的绝对阈值）：
/// `D` 是 `Σ w·d⊗d`（`w ≥ 0`）⇒ 半正定 ⇒ `tr D ≥ 0`、`det D ≥ 0`；用
/// `det D ≤ (tr D / 3)³ · 1e-6` 判"塌成薄片/一点"（三方均布时两边相等，比值为 1）。
/// 退化或非有限 ⇒ `None` ⇒ 该粒子这一子步退回 FLIP/PIC 传输（位置级退化，不是错误路径）。
///
/// **岭正则**：求逆前给对角加 `λ·(tr D / 3)`（`λ = 1e-2`）。表面粒子的邻域是单侧的 ⇒ `D` 在
/// 缺失方向欠定，不压住就会把那侧**偏置拟合成假梯度**（实测静水槽 `ke` 17.2 → 80）。
///
/// **拟合质量收缩**：`C ← s·C`，`s = ⟨C, M⟩ / T`（`T = Σ w·|v_j − v_i|²` 是局部速度的总散布，
/// `⟨C, M⟩` 是被线性模型解释掉的那部分——LS 的性质保证 `⟨C,M⟩ ≤ T`）⇒ 近静场里"信号"几乎
/// 全是噪声时 `s → 0`、`C` 自动哑火；`T = 0`（局部均匀）⇒ `C = 0`。
fn affine_from_moments(m: &[f32; 9], d: &[f32; 9], t: f32) -> Option<[f32; 9]> {
    let (a, b, c) = (d[0], d[1], d[2]);
    let (dd, e) = (d[4], d[5]);
    let f = d[8];
    let tr = a + dd + f;
    let det = a * (dd * f - e * e) - b * (b * f - e * c) + c * (b * e - dd * c);
    // 非有限（坐标 NaN/inf 会同时污染 `tr` 与 `det`）一并走 fallback：显式查有限性，
    // 不用 `!(tr > 0.0)` 那种靠 NaN 比较语义的写法（clippy 也点名过这种写法难读）。
    if !tr.is_finite() || tr <= 0.0 || !det.is_finite() || det <= tr * tr * tr * (1e-6 / 27.0) {
        return None;
    }
    // 岭：只动对角，量级按 `D` 自身的平均特征值（`tr / 3`）缩放 ⇒ 与 `h`、与权重归一无关。
    let lam = tr * (1e-2 / 3.0);
    let (a, dd, f) = (a + lam, dd + lam, f + lam);
    let det = a * (dd * f - e * e) - b * (b * f - e * c) + c * (b * e - dd * c);
    let inv = [
        dd * f - e * e,
        c * e - b * f,
        b * e - c * dd,
        c * e - b * f,
        a * f - c * c,
        b * c - a * e,
        b * e - c * dd,
        b * c - a * e,
        a * dd - b * b,
    ];
    let mut out = [0.0f32; 9];
    for r in 0..3 {
        for col in 0..3 {
            let mut s = 0.0f32;
            for q in 0..3 {
                s += m[r * 3 + q] * inv[q * 3 + col];
            }
            out[r * 3 + col] = s / det;
        }
    }
    // 拟合质量收缩：`⟨C, M⟩` = 被线性模型解释掉的方差（LS 的正交分解），`T` = 总散布。
    let explained = (0..9).map(|q| out[q] * m[q]).sum::<f32>();
    let shrink = if t > 0.0 {
        (explained / t).clamp(0.0, 1.0)
    } else {
        0.0
    };
    for c in out.iter_mut() {
        *c *= shrink;
    }
    Some(out)
}

/// 逐粒子仿射矩阵 `C`：在**粒子场**上做一遍加权最小二乘拟合局部速度梯度。
///
/// `C = M·D⁻¹`，`M = Σ_j w·(v_j − v_i)⊗(p_j − p_i)`、`D = Σ_j w·(p_j − p_i)⊗(p_j − p_i)`，
/// 邻居取 `|p_j − p_i| ≤ h`（`src/grid.rs` 的确定性 27 邻域网格；格内序 = 索引序 ⇒ 求和序
/// 是位置的确定函数），权重 = 线性衰减 `w = 1 − r/h`。
///
/// 对**线性速度场**（含刚体旋转 `v = ω×d`）该拟合是精确的 ⇒ P2G 散射 `v + C·(x_角点 − x_p)`
/// 能把这层亚格结构带到网格上（这正是 APIC 治旋转耗散的那一刀）；`v_j − v_i` 取相对量、
/// 不用 `v_i` 本身，避免分布不对称时的常数项污染。
pub(super) fn estimate_from_field(
    sys: &FluidSystem,
    ngrid: &mut UniformGrid,
    out: &mut Vec<[f32; 9]>,
) {
    let n = sys.n_fluid;
    let pos = &sys.pos[..n];
    ngrid.rebuild(pos, sys.h);
    out.clear();
    out.resize(n, [0.0; 9]);
    let h2 = sys.h * sys.h;
    let inv_h = 1.0 / sys.h;
    for (i, out_i) in out.iter_mut().enumerate() {
        let vi = sys.vel[i];
        let (mut m, mut dd, mut t) = ([0.0f32; 9], [0.0f32; 9], 0.0f32);
        ngrid.for_neighbors_in(pos, h2, i, |j, dij, r2| {
            // `dij = p_i − p_j`（网格给的是这个方向）⇒ 样本偏移取反。
            let w = 1.0 - r2.sqrt() * inv_h;
            apic_accumulate(&mut m, &mut dd, &mut t, w, sys.vel[j] - vi, -dij);
        });
        *out_i = affine_from_moments(&m, &dd, t).unwrap_or([0.0; 9]);
    }
}
