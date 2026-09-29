//! **判据：布料撕裂**（`SPEC.md` §4.6 / 骨架 `TearStrain` 的落点）。
//!
//! **口径**：`ClothSheet.damage.tear.eps` = **应变阈值**（`|len − rest| / rest`；骨架档位
//! `E03/E05/E10/None` = 0.3 / 0.5 / 1.0 / **`∞`**）。超过阈值的**唯一边**被标记为**已撕裂**
//! ⇒ 该边**不再是约束**（`project_edges` 跳过、`max_strain` 也不再统计它）。
//!
//! **"标记"而不是"移除"**：索引稳定 ⇒ 确定性 + `rest`/`lambda` 等并行数组不必重分配。
//! **默认关闭**：`eps = ∞`（`Tearing::default()`）⇒ `tear_check` 首行短路 ⇒ 既有场景**逐位不变**。
//!
//! **场景**：5×5 布片、**左列钉住**、无重力；每 tick 把自由粒子的 `vel.x` 设为常数（**速度驱动**
//! ⇒ 布片被持续向右拉）⇒ 钉住边先被拉过阈值 ⇒ 撕裂 ⇒ 布片**脱离**钉住边飞走。
//! 于是有两个可直接读的量：**已撕边数** 与 **x 向跨度**（撕裂后持续增大）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const N: usize = 4;
const W: usize = N + 1;
/// 驱动速度（m/s，+X）。取"每 tick 再推一把"的**速度驱动** ⇒ 与求解器不打架（T3 的教训）。
const PULL: f32 = 4.0;
const TICKS: usize = 600;

/// 平铺网格（`n×n` 格、跨度 `±0.5`、落在 `y = 0` 平面）。
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

/// 5×5 布片：**左列（`ix = 0`）钉住**。`eps` 用 `Option`：`None` = **完全不碰 `damage` 字段**
/// （默认档）；`Some(v)` = 显式设阈值。
///
/// **为什么 `Soft` 档**：拉伸本来很软才拉得出应变；`Hard`（α = 1e-6）下应变被压在 1e-6 量级
/// ⇒ 阈值型判据没有意义（同族：T2 的"接触带要按相对速度自适应"也是"尺度要匹配"）。
fn sheet(eps: Option<f32>) -> ClothSheet {
    let (pts, tris) = plate(N, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Soft);
    if let Some(v) = eps {
        s.damage.tear.eps = v;
    }
    for iz in 0..W {
        s.set_pinned(iz * W, true);
    }
    s
}

/// 推进 `ticks`：每 tick 把自由粒子的 `vel.x` 设为 `PULL`（速度驱动 ⇒ 持续向右拉）。
fn pull(s: &mut ClothSheet, ticks: usize) {
    for _ in 0..ticks {
        for i in 0..s.pos.len() {
            if s.inv_mass[i] > 0.0 {
                s.vel[i].x = PULL;
            }
        }
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
}

/// **右端 x**（= 自由段离钉住列多远）。
///
/// ⚠️ **不能用"x 跨度"当"脱离"的判据**（首版就栽在这条上）：撕裂之后整片布**平移到一起**飞走，
/// 跨度几乎不变（实测 1.000 → 1.035）⇒ 跨度对"脱离"没有分辨力。右边界的**绝对 x** 才有：
/// 脱离后它一路增大，没脱离则被钉住列拉住（只弹性地涨一点）。
fn right_x(s: &ClothSheet) -> f32 {
    s.pos.iter().fold(f32::MIN, |m, p| m.max(p.x))
}

fn bits_eq(x: &[Vec3], y: &[Vec3]) -> bool {
    x.iter().zip(y).all(|(p, q)| {
        p.x.to_bits() == q.x.to_bits()
            && p.y.to_bits() == q.y.to_bits()
            && p.z.to_bits() == q.z.to_bits()
    })
}

/// ① **拉过阈值 ⇒ 撕裂，且不级联**：`eps = 0.05` ⇒ 已撕边 > 0，而且**撕到一定数量就稳住**
/// （剩下的对角/剪切边继续承载 ⇒ `torn` 在 60 与 600 tick 相同）。
///
/// ⚠️ **首版预期"整片飞走"是错的**（留档）：左列全钉住时，撕开一部分边之后**载荷路径仍然活着**
/// （剩余边接着拉）⇒ 布片既没飞走、也没继续撕。⇒ 判据要照**真实物理**写；"完全脱离"另立一场
/// （单角钉住，见判据⑤）。
#[test]
fn stretching_past_the_threshold_tears_and_then_settles() {
    let mut s = sheet(Some(0.05));
    pull(&mut s, 60);
    let torn_60 = s.torn_count();
    pull(&mut s, TICKS - 60);
    let torn_end = s.torn_count();
    println!(
        "[判据①撕裂] 已撕边：60 tick = {torn_60} / {}，600 tick = {torn_end}（差值 {}）",
        s.edge_count(),
        torn_end as i64 - torn_60 as i64
    );
    assert!(s.pos.iter().all(|p| p.is_finite()), "出现非有限值");
    assert!(torn_60 > 0, "拉过阈值该撕裂（实测 60 tick 时已撕边 = 0）");
    assert_eq!(
        torn_60, torn_end,
        "撕裂该**不级联**（60 tick 撕了 {torn_60} 条、600 tick 变成 {torn_end} 条）——\
         剩下的边该继续承载 ⇒ 长期不该继续撕"
    );
}

/// ⑤ **单角钉住 ⇒ 撕开就完全脱离**：只钉右上角那一颗 ⇒ 拉过阈值后布片**整体飞走**。
#[test]
fn corner_pinned_sheet_detaches_after_tearing() {
    let (pts, tris) = plate(N, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Soft);
    s.damage.tear.eps = 0.05;
    s.set_pinned(W - 1, true); // 右上角
    let x0 = right_x(&s);
    pull(&mut s, TICKS);
    let (torn, x) = (s.torn_count(), right_x(&s));
    println!(
        "[判据⑤脱离] {TICKS} tick：已撕边 = {torn} / {}；右端 x {x0:.3} → {x:.3}",
        s.edge_count()
    );
    assert!(s.pos.iter().all(|p| p.is_finite()), "出现非有限值");
    assert!(torn > 0, "单角钉住 + 持续拉 ⇒ 该把角上的边撕开（实测 0）");
    assert!(
        x > x0 + 5.0,
        "撕开角上的边之后布片该**整体飞走**（右端 {x0:.3} → {x:.3}，只涨了 {:.3} m）——\
         没涨说明角上的边没真松（撕了但约束还在）",
        x - x0
    );
}

/// ② **未撕的边仍在阈值内**：撕裂是"**逐边**放开"而不是"整场失效" ⇒
/// 没被撕的边必须仍然满足 `strain ≤ eps`（留一点容差：位置求解器在一子步内可以短暂越阈）。
#[test]
fn untorn_edges_stay_within_the_threshold() {
    let eps = 0.05f32;
    let mut s = sheet(Some(eps));
    pull(&mut s, 60); // 只跑一小段：这时已经撕了几条、大片还连着
    let mut worst = 0.0f32;
    for k in 0..s.edge_count() {
        if !s.is_torn(k) {
            worst = worst.max(s.edge_strain(k));
        }
    }
    println!(
        "[判据②守阈] 60 tick：已撕边 = {} / {}，未撕边的最大应变 = {:.4}（阈值 {eps}）",
        s.torn_count(),
        s.edge_count(),
        worst
    );
    assert!(s.torn_count() > 0, "这一小段该已经撕了几条边");
    assert!(
        worst <= eps * 1.5,
        "未撕边的最大应变 {worst:.4} 超出阈值 {eps} 太多 ⇒ 撕裂放开得太多（该逐边放开）"
    );
}

/// ③ **金丝雀：关掉撕裂（`eps = ∞`）⇒ 不撕、也不脱离**（右端被钉住列拉住）。
#[test]
fn no_tear_when_disabled() {
    let mut s = sheet(Some(f32::INFINITY));
    let x0 = right_x(&s);
    pull(&mut s, TICKS);
    let (torn, x) = (s.torn_count(), right_x(&s));
    println!("[金丝雀③关撕裂] {TICKS} tick：已撕边 = {torn}；右端 x {x0:.3} → {x:.3}",);
    assert_eq!(torn, 0, "关掉撕裂（eps = ∞）不该有边被撕");
    assert!(
        x < x0 + 1.0,
        "关掉撕裂时布片该被约束拉住（右端只该弹性地涨一点，实测 +{:.3} m）——\
         涨得多说明它还是松开了",
        x - x0
    );
}

/// ④ **默认档逐位不变**：`Tearing::default()`（`eps = ∞`）与显式设 `∞` **逐位相同**
/// ⇒ 新增的撕裂状态对既有场景是**纯零成本**（这正是 `gate_all` 的 `gold` 门守的那条）。
#[test]
fn default_tear_state_is_bit_identical_to_explicit_infinity() {
    let (mut a, mut b) = (sheet(Some(f32::INFINITY)), sheet(None));
    assert!(b.damage.tear.eps.is_infinite(), "默认必须是关（∞）");
    pull(&mut a, 120);
    pull(&mut b, 120);
    let same = bits_eq(&a.pos, &b.pos);
    println!(
        "[判据④默认档] 120 tick 逐位比较：pos {}",
        if same { "相同 ✅" } else { "**不同 ❌**" }
    );
    assert!(
        same,
        "`Tearing::default()` 该与显式设 ∞ **逐位相同** ⇒ 默认档零成本（`tear_check` 首行短路）"
    );
}
