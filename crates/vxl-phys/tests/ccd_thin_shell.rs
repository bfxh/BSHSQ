//! **薄壳 TOI 判据**（2026-10-13）：薄壁/薄壳这一类目标在扫掠 CCD 下**不该穿**，而且**不该**
//! 靠"退到上一安全采样点"凑合——那条退法在薄壁上是**两处实测缺陷**（`SURVEY-RIGID-HARD-CASES §7.2③`）：
//!
//! 1. **宽相漏配**：扫掠 AABB 只覆盖**质心路径 ± 皮肤**（旧写法）⇒ 大体积体撞薄壁时，接触发生在
//!    质心还差半个体长的地方、**压根不在质心路径上** ⇒ 配对进不来、窄相一片空白、扫掠报"无命中"
//!    ⇒ 体**径直穿过**（实测：60 m/s 的 1 m 盒穿过 5 mm 立板后继续飞、`|v|` 不减，见
//!    `examples/ccd_thin_shell_probe.rs` 的 `单页三角网` 那格至今仍是这条洞）。
//! 2. **停位精度 = 一整段采样长**：命中后一律退到上一样本 ⇒ 1 m 盒停在墙前 **0.25 m** 且法向速度
//!    已清零 ⇒ **悬停**（间距远大于皮肤带 0.02 ⇒ 离散管线接不上手、不落位也不入睡）。
//!
//! 修法：① 扫掠 AABB 改成**体在两端各自 AABB 之并**（`shape_aabb`）；② 命中后**二分细化 TOI**：
//! 在"带内接触之前那个样本"与"**表面真的贴上**（`depth ≥ 0`）那个样本"之间二分 ⇒ 停在**贴合
//! 那一瞬**（穿透 ≤ 二分分辨率 `|v|·sub/2⁸`）、接触实打实成立 ⇒ 离散接触接手落位与入睡。
//!
//! | 用例 | 旧写法 | 现在 |
//! |---|---|---|
//! | ① 5 mm 立板 × 1 m 盒 @60 m/s | **穿过**（继续飞） | 停在**贴合面**（0.5 mm 穿透）、**入睡** |
//! | ② 同档 @3000 m/s（预算闸咬住） | 穿 | 同上：0.5 mm 穿透、入睡 |
//! | ③ 起始偏移扫 16 相位（覆盖一整段） | — | 间距 ≥ **−0.0005 m**、最大嵌入 **≤ 0.0005**（**无相位敏感**） |
//!
//! ⚠️ 停位精度**就是二分分辨率**（`|v|·sub/2⁸`；60 m/s × 1/120 s ÷ 256 ≈ 2 mm，实测 0.5 mm），
//! 所以判据把"零嵌入"放宽到**亚毫米**。⚠️ 另一条**实测教训**（写在 `scan`/`refine_touch`）：
//! 大体积体高速撞**薄**壁时采样可能**整段跳过皮肤带**、第一个接触样本就已深嵌 ⇒ 二分的下界
//! 必须取"带内接触**之前**那个样本"，否则两端谓词同真、收敛到深嵌侧（实测最坏相位嵌 **0.1964 m**）。
//!
//! ⚠️ **已知仍存的洞**（不在本片范围，登记在 `OPEN-PROBLEMS.md` P19）：**小盒 × 大三角网薄壳**——
//! 网格侧只采样**网格顶点/边 vs 体**，1 m 盒骑在 4×4 m 的三角形面上时盒的 8 个角离面 0.5 m、
//! 网格顶点全在盒外 ⇒ 零接触 ⇒ 直接穿过（本文件不钉这条，`examples/ccd_thin_shell_probe.rs`
//! 把它作为**留档用例**打印出来）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 动态盒半长（1 m 立方；最小半长 0.5 ⇒ 段长上限 0.25 ⇒ 旧停位精度就是 0.25）。
const HALF: f32 = 0.5;
/// 立板半厚（总厚 5 mm，**比皮肤带 0.02 还薄**）。
const TH: f32 = 0.0025;
/// 跑多少 tick。
const TICKS: u32 = 400;

/// 薄板 + 侧向飞来的 1 m 盒（`v` = 初速、`d0` = 盒心起始 x）。
fn scene(v: f32, d0: f32) -> (World, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0, // 放开上限，让 CCD 预算闸自己咬
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(TH, 2.0, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(d0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(v, 0.0, 0.0);
    (w, b)
}

/// 跑一遍：`(末态前缘到壁近面的间距, 全程最深嵌入)`。间距为负 = 前缘越过壁近面。
fn run(v: f32, d0: f32) -> (f32, f32, f32, bool) {
    let (mut w, b) = scene(v, d0);
    let mut deepest = 0.0f32;
    for _ in 0..TICKS {
        w.step();
        let gap = -(w.bodies.position[b].x + HALF + TH);
        deepest = deepest.max(-gap);
    }
    let gap = -(w.bodies.position[b].x + HALF + TH);
    (gap, deepest, w.bodies.linvel[b].length(), w.bodies.awake[b])
}

/// **① 薄壁（5 mm < 皮肤带）**：60 m/s 的 1 m 盒不许穿，且要停在**皮肤带内**（旧写法悬停在 0.25 m 外）。
#[test]
fn thin_wall_stops_the_box_at_the_skin_band() {
    let (gap, deepest, speed, awake) = run(60.0, -5.0);
    assert!(
        deepest < 0.005,
        "薄壁不许被嵌入（旧写法直接穿过；实测全程最深嵌入 {deepest}，上限取二分分辨率 0.005）"
    );
    assert!(
        (-0.005..=0.03).contains(&gap),
        "停位应贴在表面（±二分分辨率）：实测间距 {gap}（旧写法 0.25 = 一整段采样长、还是悬停）"
    );
    assert!(
        speed < 1e-3 && !awake,
        "停在带内 ⇒ 离散接触接手 ⇒ 应当停住并入睡（实测 |v| = {speed}、清醒 = {awake}）"
    );
}

/// **② 预算闸 + 薄壁**：3000 m/s 由预算闸限速到可行域后，同样不许穿、要停在带内。
#[test]
fn thin_wall_holds_the_speed_limited_box() {
    let (gap, deepest, speed, awake) = run(3000.0, -5.0);
    assert!(
        deepest < 0.005,
        "限速后同样不许嵌入（实测 {deepest}，上限取二分分辨率）"
    );
    assert!(
        (-0.005..=0.03).contains(&gap),
        "限速后停位也应贴在表面：实测 {gap}"
    );
    assert!(
        speed < 1e-3 && !awake,
        "应停住并入睡（实测 {speed} / {awake}）"
    );
}

/// **③ 相位扫描**：起始偏移扫过**一整段采样长**（0.25 m，16 档）——薄壁的判据不许有相位敏感：
/// 每一档都不许嵌入、都要停在带内。
#[test]
fn thin_wall_is_phase_insensitive() {
    let (mut min_gap, mut max_deep) = (f32::INFINITY, f32::MIN);
    for k in 0..16 {
        let d0 = -5.0 + (k as f32) * 0.25 / 16.0;
        let (gap, deepest, _sp, _aw) = run(60.0, d0);
        min_gap = min_gap.min(gap);
        max_deep = max_deep.max(deepest);
    }
    assert!(
        max_deep < 0.005,
        "16 个相位里最坏那档也不许嵌入（实测最深 {max_deep}，上限取二分分辨率）"
    );
    assert!(
        min_gap > -0.005,
        "16 个相位里最坏那档也要贴在表面（实测最小 {min_gap}）"
    );
}
