//! **XPBD 的 dt 尺度与 lambda 生命周期**（《LSSMJ-CICD…v5》§15.4「XPBD 耦合合同」里，
//! 本仓此前**只有代码事实、没有判据**的那两条）。
//!
//! **判据**：同一物理时间、**不同 `substeps`**（⇒ 子步长 `h = dt/substeps` 不同）⇒ 末态**收敛**。
//! 一条判据同时钉两件事：
//! ① **顺应度随 `h²` 缩放**（`a_tilde = α/h²`，见 `cloth.rs::project_constraints`）——
//!    若写成 `α/h` 或漏除，有效刚度会随 `h` 变 ⇒ 两档结果**系统性偏离**；
//! ② **`lambda` 每子步清零**（同一个函数开头清零）—— 若跨子步累积，多子步档会**过冲** ⇒ 同样偏离。
//!
//! ⚠️ **容差取自实测**：XPBD 本身有 `O(h²)` 的离散误差 ⇒ 两档**不会**逐位相同（这点与金样不同）；
//! 但"尺度写对"时差异很小，而"尺度写错"会大一个量级 —— 具体读数是 `println` 那两行。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const G: f32 = 9.8;
const TICKS: usize = 600;

/// 平铺网格（与 `cloth_minimal.rs` 同款：`n×n` 格、跨度 `±size`、落在 `y = 0` 平面）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
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

/// 两角钉住（粒子 0 与 2）、重力悬垂；返回 `(最大应变, 后半段**平均**中心 y)`。
///
/// ⚠️ **取时间平均而不是末态读数**：`damping = 1.0`（无阻尼）⇒ 布**持续摆动**，末态读数取决于
/// 相位 —— 实测同一场景 4/8/16/32/64 substeps 的末态 `y` = −0.488 / −0.303 / −0.150 / −0.142 /
/// −0.205，**非单调**（那是相位，不是收敛）。后半段平均才代表"这个配置下的平均下垂深度"。
fn hang(substeps: u32, ticks: usize) -> (f32, f32) {
    let (pts, tris) = plate(2, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.substeps = substeps;
    s.set_pinned(0, true);
    s.set_pinned(2, true);
    let (mut sum_y, mut n) = (0.0f32, 0u32);
    for t in 0..ticks {
        s.step(DT, Vec3::new(0.0, -G, 0.0), &NoProviders, 0, &[]);
        if t + 1 > ticks / 2 {
            sum_y += s.pos[4].y;
            n += 1;
        }
    }
    (s.max_strain(), sum_y / n as f32)
}

/// **判据**：子步细化呈**收敛趋势** —— ① 平均下垂随 `substeps` **单调不增**；
/// ② **相邻档差随 `substeps` 递减**（细档之间的差小于粗档之间的差）。
///
/// ⚠️ **为什么不去断言"末态相等"**（本节最有价值的实测结论，别重走）：
/// - `damping = 1.0`（无阻尼）⇒ 布持续摆动，末态读数是**相位**（实测 4/8/16/32/64 档的末态 `y`
///   = −0.488 / −0.303 / −0.150 / −0.142 / −0.205，**非单调**）⇒ 必须先取**后半段时间平均**；
/// - 即便取了平均，收敛也**很慢**（每翻倍只降 7–14%，不是 `O(h²)`）⇒ "跨档相等"没有可用容差；
/// - 更关键：若把 `a_tilde = α/h²` 误写成 `α/h`，有效刚度仍**单调**地随 `h` 变化 ⇒ **趋势判据抓不到**
///   （它只会让收敛"更快"）。**精确钉住尺度律需要"只投影一步"的入口**（当前 `step` 把
///   predict/project/contacts/write_back 打包了）—— 那是一个后续切片。
///
/// 所以本判据钉的是**收敛趋势**（弱而真实），并在 `docs/CAPABILITIES.yaml` 里把这两条合同项
/// 标成 `partially`，而不是假装"已验证"。
#[test]
fn substep_refinement_shows_convergence_trend() {
    let grades = [4u32, 8, 16, 32, 64];
    let mut ys: Vec<f32> = Vec::new();
    for s in grades {
        let (e, y) = hang(s, TICKS);
        println!("substeps={s:3}：最大应变 {e:.6}｜后半段平均 y {y:.6}");
        assert!(e.is_finite() && y.is_finite(), "substeps={s} 出现非有限值");
        ys.push(y);
    }
    // ① 单调不增（子步越细、平均下垂越浅）。常数级容差吸收末位舍入。
    for (k, w) in ys.windows(2).enumerate() {
        assert!(
            w[1] >= w[0] - 1e-6,
            "第 {k}→{k}+1 档非单调：{:.6} → {:.6}",
            w[0],
            w[1]
        );
    }
    // ② 收敛趋势：最细两档之差 < 最粗两档之差。
    let coarse = (ys[1] - ys[0]).abs();
    let fine = (ys[4] - ys[3]).abs();
    println!("相邻档差：最粗 {coarse:.6}｜最细 {fine:.6}（细档应更小）");
    assert!(
        fine < coarse,
        "收敛趋势不成立：最细档差 {fine:.6} ≥ 最粗档差 {coarse:.6}"
    );
}
