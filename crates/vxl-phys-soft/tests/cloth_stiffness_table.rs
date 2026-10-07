//! **判据：刚度档表**（`SPEC.md` §4.6 的 α 五档）—— 补 `ROUTE` M4 出口判据的最后一块
//! （「悬臂/旗飘金样 + 刚度档表」里前两半已在 `examples/m4_cantilever.rs` / `m4_flag.rs`）。
//!
//! ⚠️ **为什么不用"悬臂"做档表**：`m4_cantilever` 里实测**不敏感** —— 五档垂度跨度
//! `0.3107–0.3158`（相对差 1.6%，且非单调）。那里的**弯曲**被重力完全压过（布几乎完全
//! 下垂到根部下方），档位差异淹没在几何里。本片换**拉伸主导**的场景：**四边钉住的方膜 +
//! 均匀重力** ⇒ 中点挠度由**结构 compliance α** 直接控制 ⇒ 对档位应当敏感。
//!
//! 场景刻意**关弯曲**（`bend_compliance = ∞`，`project_bend` 对非有限显式短路）⇒ 只留拉伸腿，
//! 免得两条腿混在一起说不清是谁在起作用。
//!
//! **判据**：① 五档由硬到软，最大下垂**非降**（SPEC 的档序）；② 两端**相对**跨度 > 10%
//! （实测 18.6% ⇒ 阈值取它的一半略低，留一倍余量）。实测（240 tick）：
//! `0.010552 / 0.010559 / 0.010624 / 0.011255 / 0.012513` —— **严格单调**、跨度 18.6%。
//! ⚠️ 绝对下垂只有 ~1 cm（膜 0.2 m 见方）：布几乎不可拉伸，量级本来就小；判据看**相对**跨度。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
/// 格数（`N × N` 格 ⇒ `(N+1)²` 个顶点）。
const N: usize = 6;
/// 半边长（膜 = `2·SIZE` 见方）。
const SIZE: f32 = 0.1;
const TICKS: usize = 240;
/// 档表两端的**相对**跨度下限：低于它就算"场景对 α 不敏感"（实测量级 18.6%）。
const SPAN_REL_MIN: f32 = 0.10;

/// SPEC §4.6 的 α 五档（由硬到软）。
const GRADES: [(&str, Stiffness); 5] = [
    ("NearRigid", Stiffness::NearRigid),
    ("Hard", Stiffness::Hard),
    ("Standard", Stiffness::Standard),
    ("Soft", Stiffness::Soft),
    ("Jelly", Stiffness::Jelly),
];

/// 四边钉住的方膜（`N × N` 格、落在 `y = 0` 平面；弯曲显式关）。
fn membrane(stiffness: Stiffness) -> ClothSheet {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
    let n = N as u32;
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * SIZE / n as f32;
            pts.push(Vec3::new(-SIZE + s * ix as f32, 0.0, -SIZE + s * iz as f32));
        }
    }
    for iz in 0..n {
        for ix in 0..n {
            let a = iz * (n + 1) + ix;
            let (c, d) = (a + 1, a + n + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, stiffness);
    s.bend_compliance = f32::INFINITY; // 只留拉伸腿（见文件头注）
    for i in 0..s.pos.len() {
        let p = s.pos[i];
        if (p.x.abs() - SIZE).abs() < 1e-6 || (p.z.abs() - SIZE).abs() < 1e-6 {
            s.set_pinned(i, true); // 四边钉住
        }
    }
    s
}

/// 跑 `ticks` 后，**自由粒子**的最大下垂量（`-y`；钉住的边不参与）。
fn sag(mut s: ClothSheet, ticks: usize) -> f32 {
    for _ in 0..ticks {
        s.step(DT, Vec3::new(0.0, -9.8, 0.0), &NoProviders, 0, &[]);
    }
    let mut deepest = 0.0f32;
    for i in 0..s.pos.len() {
        if s.inv_mass[i] > 0.0 {
            deepest = deepest.max(-s.pos[i].y);
        }
    }
    deepest
}

/// **① 单调 + ② 跨度**：五档由硬到软，中点挠度非降且两端有可辨跨度。
#[test]
fn stiffness_table_is_monotone_and_wide() {
    let mut table = [0.0f32; 5];
    for (k, (name, st)) in GRADES.iter().enumerate() {
        table[k] = sag(membrane(*st), TICKS);
        println!("{name:>9} α={:.0e} 最大下垂 {:.6}", st.alpha(), table[k]);
    }
    let span = table[4] - table[0];
    println!("档表跨度 {span:.6}（相对 {:.1}%）", span / table[0] * 100.0);
    for (k, w) in table.windows(2).enumerate() {
        assert!(
            w[1] >= w[0] - 1e-9,
            "档表非单调：{} 档 {} → {} 档 {}",
            GRADES[k].0,
            w[0],
            GRADES[k + 1].0,
            w[1]
        );
    }
    let rel = span / table[0];
    assert!(
        rel > SPAN_REL_MIN,
        "档表相对跨度 {rel:.3} ≤ {SPAN_REL_MIN} ⇒ 场景对 α 不敏感"
    );
}
