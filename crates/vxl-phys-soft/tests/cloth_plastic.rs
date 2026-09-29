//! **判据：布片塑性**（`docs/PLAN-plasticity.md` §四 —— `yield_strain`/`rate` 的落点）：
//! 应变 `|len − rest| / rest` 超**屈服阈**的部分按**塑性率**每子步永久并入 `rest`
//! ⇒ 卸载后留**永久变形**；默认 `∞` = 关（零换代）。
//!
//! **判据**（机器无关；装置 = **全钉住**布片 ⇒ 应变**运动学给定**、投影不干扰，
//! 除行为腿外）：
//! ① **亚阈 / 默认 = 纯弹性（逐位）**：亚阈（ε = 0.02 < yield）与默认档（∞，ε = 0.6）
//!    各跑 60 tick ⇒ `rest` **逐位不变**、累计塑性应变恒 `0`；
//! ② **逐边闭式对拍（主判据）**：ε = 0.6 / yield 0.05 / rate 0.5 跑 1 tick（8 子步）
//!    ⇒ 每条边与测试侧递推 `rest ← rest·(1 + sign·rate·(|ε| − yield))`（**同 f32 运算序**）
//!    逐位一致；
//! ③ **塑性率标度**：`rate` ×2 ⇒ 单子步 `Δrest` **≈2 倍**；
//! ④ **撕裂次序**：塑性**先**算 ⇒ 超阈弹性应变被吃掉 ⇒ 与撕裂（`eps` 0.5）同开时**不撕**；
//!    关塑性 / `rate = 0` 两个对照都**该撕**（次序 + 机制各一条证据）；
//! ⑤ **默认 = 显式关（逐位）** + 动态场景 `rest` 逐位不变（金丝雀）；行为腿：拉过阈再
//!    卸载 ⇒ **永久变长**，关塑性则弹回原长。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{cloth::plastic::Plastic, ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const N: usize = 2;
const SIZE: f32 = 0.5;
/// 横向拉伸比 ⇒ 水平边 ε = 0.6、对角边 ≈ 0.333、竖边 0（一网三档，同装置测多态）。
const STRETCH: f32 = 1.6;
const YIELD: f32 = 0.05;
const RATE: f32 = 0.5;
/// 撕裂阈（判据④：`0.5 < 0.6` ⇒ 不流动时首子步即撕）。
const TEAR_EPS: f32 = 0.5;
/// 行为腿（动态金丝雀）：`4×4` 格、列宽 `W`、左列钉住、速度驱动（`1 m/s`；撕裂判据同款手法）。
const W: usize = 5;
const PULL: f32 = 1.0;

fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// **全钉住** + 横向拉伸 `stretch`（注册长度取自原始点 ⇒ 应变为运动学常数，投影不动位置）。
fn pinned_stretched(stretch: f32) -> ClothSheet {
    let (pts, tris) = plate(N, SIZE);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    for i in 0..s.pos.len() {
        s.set_pinned(i, true);
    }
    for i in 0..s.pos.len() {
        s.pos[i].x *= stretch;
    }
    s
}

/// 测试侧递推（与引擎**同一 f32 运算序**：`excess → step = rate·excess → rest·(1 + sign·step)`）。
fn flow_recursion(r0: f32, len: f32, yield_strain: f32, rate: f32, passes: usize) -> f32 {
    let mut r = r0;
    for _ in 0..passes {
        let strain = (len - r) / r;
        let excess = strain.abs() - yield_strain;
        if excess > 0.0 {
            r *= 1.0 + strain.signum() * (rate * excess);
        }
    }
    r
}

fn rest_snapshot(s: &ClothSheet) -> Vec<f32> {
    (0..s.edge_count()).map(|k| s.rest_of(k)).collect()
}

/// 判据①②：亚阈与默认档逐位不变；超阈与递推逐位一致。
#[test]
fn plastic_flow_is_bitwise_below_threshold_and_matches_recursion_above() {
    // ①a 亚阈：ε = 0.02 < yield 0.05 ⇒ 不流动
    let mut a = pinned_stretched(1.02);
    a.damage.plastic.yield_strain = YIELD;
    a.damage.plastic.rate = RATE;
    let before = rest_snapshot(&a);
    for _ in 0..60 {
        a.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
    for (k, &was) in before.iter().enumerate() {
        assert!(a.rest_of(k) == was, "亚阈该逐位不变（边 {k}）");
        assert!(a.plastic_of(k) == 0.0, "亚阈不该累计塑性应变（边 {k}）");
    }
    // ①b 默认档（∞）：ε = 0.6 也不动
    let mut b = pinned_stretched(STRETCH);
    let before_b = rest_snapshot(&b);
    for _ in 0..60 {
        b.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
    for (k, &was) in before_b.iter().enumerate() {
        assert!(b.rest_of(k) == was, "默认档（∞）该逐位不变（边 {k}）");
        assert!(b.plastic_of(k) == 0.0);
    }
    println!("[判据①] 亚阈 + 默认档：全边 rest 逐位不变 ✅");
    // ② 超阈：逐边与测试侧递推对拍（1 tick = 8 子步 = 8 遍流动）
    let mut s = pinned_stretched(STRETCH);
    s.damage.plastic.yield_strain = YIELD;
    s.damage.plastic.rate = RATE;
    let r0 = rest_snapshot(&s);
    s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let (mut flowing, mut max_rel) = (0usize, 0.0f32);
    for (k, &r0k) in r0.iter().enumerate() {
        let [i, j] = s.cons[k];
        let len = (s.pos[j as usize] - s.pos[i as usize]).length();
        let want = flow_recursion(r0k, len, YIELD, RATE, 8);
        let got = s.rest_of(k);
        max_rel = max_rel.max((got - want).abs() / want);
        if got > r0k {
            flowing += 1;
        }
    }
    println!(
        "[判据②] 1 tick 后 {flowing}/{} 条边流动；与递推最大相对差 {max_rel:.3e}",
        s.edge_count()
    );
    assert!(flowing > 0, "ε = 0.6 > yield 的边该流动（实测 0 条）");
    assert!(
        max_rel < 1e-6,
        "逐边该与递推一致（最大相对差 {max_rel:.3e}）"
    );
}

/// 判据③：`rate` ×2 ⇒ 单子步增量 ≈2 倍（单子步 ⇒ 首步闭式 `Δ = rest·rate·excess`）。
#[test]
fn rate_scales_the_plastic_increment() {
    let delta = |rate: f32| -> f32 {
        let mut s = pinned_stretched(STRETCH);
        s.damage.plastic.yield_strain = YIELD;
        s.damage.plastic.rate = rate;
        s.substeps = 1;
        let before = rest_snapshot(&s);
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        let mut d = 0.0f32;
        for (k, &was) in before.iter().enumerate() {
            d = d.max(s.rest_of(k) - was);
        }
        d
    };
    let (d1, d2) = (delta(0.25), delta(0.5));
    let ratio = d2 / d1;
    println!("[判据③] rate 0.25 ⇒ Δ = {d1:.6}；rate 0.5 ⇒ Δ = {d2:.6}（比值 {ratio:.6}）");
    assert!(
        (ratio - 2.0).abs() < 1e-4,
        "rate ×2 ⇒ 增量该 ×2（实测比值 {ratio}）"
    );
}

/// 判据④：**塑性先于撕裂检查**（次序）+ 机制证据（关塑性 / `rate = 0` 都该撕）。
#[test]
fn plastic_flow_precedes_tear_check() {
    let mut on = pinned_stretched(STRETCH);
    on.damage.tear.eps = TEAR_EPS;
    on.damage.plastic.yield_strain = YIELD;
    on.damage.plastic.rate = 1.0; // 一遍把 ε 从 0.6 压到 ≈0.03 < eps
    on.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let residual = (0..on.edge_count()).fold(0.0f32, |m, k| m.max(on.edge_strain(k)));
    let mut off = pinned_stretched(STRETCH);
    off.damage.tear.eps = TEAR_EPS;
    off.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    let mut zero = pinned_stretched(STRETCH);
    zero.damage.tear.eps = TEAR_EPS;
    zero.damage.plastic.yield_strain = YIELD;
    zero.damage.plastic.rate = 0.0;
    zero.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    println!(
        "[判据④] 塑性 ON：torn = {}（最大残余应变 {residual:.4}）| 关塑性：torn = {} | \
         rate = 0：torn = {}",
        on.torn_count(),
        off.torn_count(),
        zero.torn_count()
    );
    assert_eq!(on.torn_count(), 0, "塑性先算 ⇒ 弹性应变被吃掉 ⇒ 不该撕");
    assert!(
        off.torn_count() > 0,
        "关塑性 ⇒ 首子步 ε 0.6 > eps 0.5 ⇒ 该撕"
    );
    assert!(zero.torn_count() > 0, "rate = 0（不流动）⇒ 该撕");
}

/// 判据⑤：默认 = 显式关（**逐位**）+ 动态场景 `rest` 逐位不变（金丝雀）。
#[test]
fn default_plastic_is_off_bitwise_in_a_dynamic_scene() {
    assert!(
        Plastic::default().yield_strain.is_infinite(),
        "默认必须是关（∞）"
    );
    let run = |explicit: bool| -> Vec<[u32; 3]> {
        let (pts, tris) = plate(4, SIZE);
        let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Soft);
        if explicit {
            s.damage.plastic.yield_strain = f32::INFINITY;
            s.damage.plastic.rate = RATE;
        }
        let before = rest_snapshot(&s);
        for iz in 0..W {
            s.set_pinned(iz * W, true);
        }
        for _ in 0..60 {
            for i in 0..s.pos.len() {
                if s.inv_mass[i] > 0.0 {
                    s.vel[i].x = PULL;
                }
            }
            s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        }
        assert!(
            (0..s.edge_count()).all(|k| s.rest_of(k) == before[k]),
            "默认档动态场景 rest 该逐位不变"
        );
        assert!(
            (0..s.edge_count()).all(|k| s.plastic_of(k) == 0.0),
            "默认档不该累计塑性应变"
        );
        s.pos
            .iter()
            .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect()
    };
    let (a, b) = (run(false), run(true));
    assert!(a == b, "默认档该与显式 ∞ **逐位相同**");
    println!("[判据⑤] 动态 60 tick：默认 = 显式关（逐位）✅");
}

/// **行为腿**：全钉住拉出 ε = 0.6 → 塑性流动 60 tick → **冻结**（`yield = ∞`）→ **全部解钉**
/// → 静置松弛 ⇒ 布片**永久变长**（Σ 边长 ≈ 生长的 Σ rest ≫ 原 Σ rest）；关塑性 ⇒
/// 松弛回原长（对照）。
///
/// ⚠️ **度量必须与形状无关**：解钉后布片可自由剪切/转动 ⇒ `x` 向跨度会被形状污染
/// （实测：关塑性也可能给出 1.37 的跨度）。**Σ 边长**才是 `rest` 的直接读出。
/// ⚠️ **不用速度驱动**（诊断实测：写回把驱动抹平 ⇒ 应变只在 `yield` 附近徘徊、永久部分仅 0.4 cm）。
/// ⚠️ **拉伸必须面内均匀**（`x` 与 `z` 同比例）：只拉一个方向会让生长后的 rest 集
/// **几何不自洽**（横边 ×1.53 / 对角 ×1.27 / 竖边 ×1.0，没有平面构型能同时兑现）
/// ⇒ 解钉松弛只能折中（实测 Σ 边长 9.42 vs Σ rest 11.17）。均匀拉伸 ⇒ 自洽 ⇒ 读数兑现。
/// ⚠️ **拉完必须冻结再解钉**（实测留档）：不冻结时，解钉松弛会把部分边压过 `−yield`
/// ⇒ **反向流动**把 Σ rest 从 **11.17 拉回 9.00**（对称 `|ε|` 设计的自然后果）⇒
/// "成形读数"要在**停止驱动的同时冻结**，否则测到的是"成形 + 松弛侵蚀"的合成量。
#[test]
fn released_sheet_stays_permanently_longer() {
    let run = |plastic: bool| -> (f32, f32) {
        let (pts, tris) = plate(N, SIZE);
        let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Soft);
        if plastic {
            s.damage.plastic.yield_strain = YIELD;
            s.damage.plastic.rate = RATE;
        }
        s.damping = 0.999;
        s.bend_compliance = f32::INFINITY; // 关弯曲：本档只做 cons 的塑性（PLAN §五-1），
                                           // 弯曲 rest 未长 ⇒ 留着它会把永久变形拉回去（同 T3/撕裂的先例）
        for i in 0..s.pos.len() {
            s.set_pinned(i, true);
        }
        for i in 0..s.pos.len() {
            s.pos[i].x *= STRETCH; // 注册长度取自原始点 ⇒ 应变为运动学常数
            s.pos[i].z *= STRETCH; // **面内均匀拉伸**（x 与 z 同比例 ⇒ 三类边同应变 0.6）
        }
        for _ in 0..60 {
            s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]); // 成形（钉住 ⇒ 位置不动）
        }
        s.damage.plastic.yield_strain = f32::INFINITY; // 冻结（卸载后的读数不受反向流动侵蚀）
        for i in 0..s.pos.len() {
            s.set_pinned(i, false); // 解钉 ⇒ 松弛到**新的** rest 长度
        }
        for _ in 0..600 {
            s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        }
        let lens: f32 = (0..s.edge_count())
            .map(|k| {
                let [i, j] = s.cons[k];
                (s.pos[j as usize] - s.pos[i as usize]).length()
            })
            .sum();
        let rests: f32 = (0..s.edge_count()).map(|k| s.rest_of(k)).sum();
        (lens, rests)
    };
    let ((on, on_rest), (off, off_rest)) = (run(true), run(false));
    println!(
        "[行为腿] 解钉静置后：Σ 边长 = {on:.4}（rest {on_rest:.4}）| 关塑性 {off:.4}（rest {off_rest:.4}）"
    );
    assert!(on.is_finite() && off.is_finite(), "出现非有限值");
    assert!(
        (off - off_rest).abs() < 0.02 * off_rest,
        "关塑性该松弛回注册长度（Σ 边长 {off:.4} vs Σ rest {off_rest:.4}）⇒ 0 = 无永久变形"
    );
    assert!(
        on > off * 1.3,
        "**行为**：解钉静置后仍该更长（{on:.4} vs 对照 {off:.4}）——永久变形要看得见"
    );
}
