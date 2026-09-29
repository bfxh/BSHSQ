//! **判据：点-边自碰撞**（T3 进阶档：补点-点对"从两粒子之间穿过"的盲区）。
//!
//! **场景 = 折平 + 上层横滑 + 过驱压制**（1.0 m 见方、8×8 格、间距 0.125 m）：
//! - **折平**（T3 同款：折线 `x=0`、折痕列钉住、瞬时符号自刹车）⇒ 两层先对齐地压到 `d_c`；
//! - **横滑**（tick 120–360，左=上层沿 `+x` 滑 0.025、`+z` 滑 0.06）⇒ 两层变成**真二维交错**：
//!   粒子到**边**的垂距 `b = 0.025`，到**最近粒子**的距离 `a = √(0.025²+0.06²) ≈ 0.065`
//!   —— 点-点与点-边的分辨带被几何拉开（交错区离折痕 ≥3 列 ⇒ 在弯曲禁止环之外，
//!   排除集不许与折痕弹簧对顶，T3 同款）；
//! - **过驱压制**（tick 360+，ω×4）：**转向按出生半区**（非 T3 的瞬时符号）—— 折平+滑移后
//!   整层都在折线另一侧，瞬时符号会把折过面的粒子抬回去（层永不相压）；出生半区 ⇒ 继续
//!   同向转 ⇒ 折叠几何强制交叠；
//! - **半径** = 0.028 ⇒ `d_c = 0.056`：交错态点-点够不到（0.065 > 0.056），点-边够得到
//!   （0.025 < 0.056）⇒ **接住交错的只能是点-边**。
//!
//! **判据**（机器无关）：
//! ① **层距保持（主判据）**：全程**跨层非网格邻居**的**粒子-边**最小距离 ≥ `0.6·d_c`
//!    —— ON 稳在 `d_c`（实测逐位 = 0.056）；这个量在压穿后**留得住痕**（粒子-粒子距离
//!    压平后会恢复到面内 `a ≈ 0.065`，只有到边的距离持续停在 `b = 0.025`）；
//! ② **金丝雀**：`set_point_edge(false)` ⇒ 同场景必须压穿（粒子-边最小距离 → `b = 0.025`
//!    `< 0.6·d_c`）；
//! ③ **点-边在传**：稳态窗内解算对数最大值 `> 0`；
//! ④ **对照打印**：粒子-粒子最小距离（全程/窗；分辨带 0.025–0.065 的直读，不判红）。
//!
//! **边界**：测的是"边约束接住了交错层"，不声称 CCS（离散投影，与 T3 同一句话）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const N: usize = 8;
const SIZE: f32 = 0.5;
/// 折线 = `x = 0`（网格线上，折痕列钉住，T3 同款）；层间交错由**上层横滑**制造。
const CREASE: f32 = 0.0;
/// 碰撞半径 ⇒ `d_c = 0.056`（0.025 < 0.056 < 0.065 ⇒ 交错态只有边够得到）。
const R: f32 = 0.028;
const D_C: f32 = 2.0 * R;
/// 判据阈：`0.6·d_c`（ON 稳在 ≈`d_c`、OFF 压穿到 ≈`b = 0.025` ⇒ 两边都有余量）。
const THRESH: f32 = 0.6 * D_C;
/// 折合角速度；**过驱段**（滑完之后）×[`OVERDRIVE`] ⇒ 折叠几何强制交叠。
const OMEGA: f32 = 1.0;
const OVERDRIVE: f32 = 4.0;
/// 左（上）层滑速：x 240 tick 走 0.025、z 走 0.06 ⇒ 与折叠镜像组合成真二维交错。
const SLIDE_X: f32 = 0.006_25;
const SLIDE_Z: f32 = 0.015;
const FOLD_TICKS: usize = 120;
const SLIDE_END: usize = 360;
const TICKS: usize = 420;
const WIN: usize = 60;

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

/// 粒子 `idx` 的**出生半区**（`false` = 左）：滑移方向与判据分层都按出生算
/// （折过去之后瞬时 `x` 会翻到对面，不能作分层依据）。
fn birth_left(idx: usize) -> bool {
    idx % (N + 1) <= N / 2
}

fn sheet() -> ClothSheet {
    let (pts, tris) = plate(N, SIZE);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.bend_compliance = f32::INFINITY; // 关弯曲（T3 同款：一次只测一个变量）
    s.damping = 0.999;
    s.self_contacts.cfg.enabled = true;
    s.self_contacts.cfg.particle_radius = R;
    for i in 0..s.pos.len() {
        if s.pos[i].x.abs() < 1e-6 {
            s.set_pinned(i, true); // 折痕列钉住（T3 同款：无漂移 + 折线固定）
        }
    }
    s
}

/// **驱动**：折合（绕折线反向角速度场）+ 左（上）层 x/z 横滑 + 滑完过驱。
///
/// ⚠️ **转向按出生半区取，不用 T3 的"瞬时符号"**：折平+滑移之后整层都在折线另一侧，
/// 瞬时符号的自刹车会把折过面的粒子**抬回去**（过驱 ×4 也只是把整层抬离，层永不相压；
/// 实测三种终相读数逐位相同 = 压力全被近刚性弹簧吸收的签名）。出生半区 ⇒ 折过面后
/// **继续同一转向**，折叠几何强制交叠（弹簧网络跟着转、拦不住）；退化区由边约束在
/// `b = 0.025 ≫ 1e-9` 处接住，T3 的"同心退化"陷阱（纯点-点、对齐几何）不适用。
fn drive(s: &mut ClothSheet, t: usize) {
    let omega = if t >= SLIDE_END {
        OMEGA * OVERDRIVE
    } else {
        OMEGA
    };
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        let p = s.pos[i];
        let sgn = if birth_left(i) { -1.0 } else { 1.0 };
        let r = Vec3::new(p.x - CREASE, p.y, 0.0);
        let mut v = Vec3::new(-r.y, r.x, 0.0) * (sgn * omega);
        if (FOLD_TICKS..SLIDE_END).contains(&t) && birth_left(i) {
            v.x += SLIDE_X;
            v.z += SLIDE_Z;
        }
        s.vel[i] = v;
    }
}

/// 点到线段距离（Ericson；与引擎的最近点口径一致）。
fn seg_dist(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let ab2 = ab.length_squared();
    if ab2 < 1e-12 {
        return (p - a).length();
    }
    let u = ((p - a).dot(ab) / ab2).clamp(0.0, 1.0);
    (p - (a + ab * u)).length()
}

/// **跨层、非网格邻居**的最小粒子-边距离（判据①/② 主量；与引擎同一份排除集 ⇒ 同口径）。
fn cross_edge_min(s: &ClothSheet, best: &mut f32) {
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        let left = birth_left(i);
        for e in 0..s.cons.len() {
            let [j, k] = s.cons[e];
            let (jl, kl) = (birth_left(j as usize), birth_left(k as usize));
            if jl != kl || jl == left {
                continue; // 只取"两端点都在对面层"的边（跨折线边归属不明 ⇒ 跳过）
            }
            if s.self_contacts.is_forbidden(i as u32, j)
                || s.self_contacts.is_forbidden(i as u32, k)
            {
                continue; // 端点与粒子成网格邻居 ⇒ 与引擎同口径
            }
            let d = seg_dist(s.pos[i], s.pos[j as usize], s.pos[k as usize]);
            *best = best.min(d);
        }
    }
}

/// **跨层、非网格邻居**的最小粒子-粒子距离（判据④对照；⚠️ 必须带 `is_forbidden` 排除：
/// 折痕处的弯曲邻居对在任何折叠里都合法地贴到 0 ⇒ 不滤会把"折痕环"误读成穿透）。
fn cross_point_min(s: &ClothSheet) -> f32 {
    let mut best = f32::MAX;
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 || !birth_left(i) {
            continue;
        }
        for j in 0..s.pos.len() {
            if s.inv_mass[j] == 0.0 || birth_left(j) {
                continue;
            }
            if s.self_contacts.is_forbidden(i as u32, j as u32) {
                continue;
            }
            best = best.min((s.pos[j] - s.pos[i]).length());
        }
    }
    best
}

struct Read {
    /// 全程跨层粒子-边最小距离（判据①/② 主量；压穿后留痕的量）。
    edge_min: f32,
    /// 全程 / 稳态窗内跨层粒子-粒子最小距离（判据④对照）。
    point_min: f32,
    win_point_min: f32,
    /// 稳态窗内点-边解算对数的最大值（判据③）与全程总数。
    win_pairs_hi: u32,
    total_pairs: u32,
    finite: bool,
}

fn run(point_edge: bool) -> Read {
    let mut s = sheet();
    s.self_contacts.set_point_edge(point_edge);
    let mut r = Read {
        edge_min: f32::MAX,
        point_min: f32::MAX,
        win_point_min: f32::MAX,
        win_pairs_hi: 0,
        total_pairs: 0,
        finite: true,
    };
    for t in 0..TICKS {
        drive(&mut s, t);
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        cross_edge_min(&s, &mut r.edge_min);
        r.point_min = r.point_min.min(cross_point_min(&s));
        if t >= TICKS - WIN {
            r.win_point_min = r.win_point_min.min(cross_point_min(&s));
            r.win_pairs_hi = r.win_pairs_hi.max(s.self_contacts.edge_pairs());
        }
        r.total_pairs += s.self_contacts.edge_pairs();
    }
    r.finite = s.pos.iter().all(|p| p.is_finite());
    r
}

fn report(tag: &str, r: &Read) {
    println!(
        "  {tag:<22} d_c={:.4} 阈={:.4} | 粒子-边 min {:.5} | 粒子-粒子 min {:.5}（窗 {:.5}）\
         | 点-边对 窗max {} / 全程 {} ⇒ {}",
        D_C,
        THRESH,
        r.edge_min,
        r.point_min,
        r.win_point_min,
        r.win_pairs_hi,
        r.total_pairs,
        if r.edge_min >= THRESH {
            "层距保持 ✅"
        } else {
            "**压穿 ❌**"
        }
    );
}

/// **判据本体**：点-边接住交错层（粒子-边距离全程 ≥ 0.6·d_c、点-边在传）。
#[test]
fn point_edge_holds_interleaved_layers() {
    println!(
        "[判据·点-边自碰撞] 8×8 / 折平后上层滑 (0.025, 0.06) + 出生半区转向过驱 / \
         d_c={D_C:.3} / 阈={THRESH:.3}"
    );
    let r = run(true);
    report("点-边 ON", &r);
    assert!(r.finite, "出现非有限值（NaN/inf）");
    assert!(r.total_pairs > 0, "全程一次点-边都没解算 ⇒ 机制没在跑");
    assert!(
        r.win_pairs_hi > 0,
        "稳态窗内没有点-边对（窗max {}）⇒ 层距不是它托的",
        r.win_pairs_hi
    );
    assert!(
        r.edge_min >= THRESH,
        "跨层粒子-边最小距离 {:.5} < 阈 {:.5} ⇒ 交错层被压穿（点-边没接住）",
        r.edge_min,
        THRESH
    );
    println!(
        "  [对照④] 粒子-粒子 min 全程 {:.5} / 窗 {:.5}（分辨带 0.025–0.065 的直读）",
        r.point_min, r.win_point_min
    );
}

/// **金丝雀**：关掉点-边（点-点仍开）⇒ 同场景必须压穿（否则判据没有分辨力）。
#[test]
fn canary_without_point_edge_the_layers_pass_through() {
    let r = run(false);
    report("点-边 OFF（金丝雀）", &r);
    assert!(r.finite, "出现非有限值（NaN/inf）");
    assert_eq!(r.total_pairs, 0, "点-边关了还有解算对 ⇒ 开关短路失效");
    assert!(
        r.edge_min < THRESH,
        "金丝雀本该压穿：粒子-边最小距离 {:.5} ≥ 阈 {:.5} ⇒ 层没穿 ⇒ 判据没有分辨力",
        r.edge_min,
        THRESH
    );
}
