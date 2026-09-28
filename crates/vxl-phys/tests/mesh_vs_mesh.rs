//! **判据：三角网 × 三角网**（T2 续 —— `PLAN-triangle-first-class.md` 的收尾片）。
//!
//! **口径 = 双面**（`SURVEY` §8.4.47②）：薄壳**无内外** ⇒ 法线取「**从网面最近点指向顶点**」
//! （顶点在哪侧就朝哪侧推），接触距离 = 接触带 `skin`（扮演两片半厚之和）、**双向采样**
//! （A 顶点 × B 三角 且 B 顶点 × A 三角）。
//!
//! ⚠️ **与 `hull_vs_mesh` 的"单面"口径不同**：那条固定用**环绕法线**，顶点从反面压入时会被
//! **推得更深**（它自己登记过这个边界）。判据 ②③ 就是**锁死双面口径**的。
//!
//! **判据（全部机器无关）**：
//! ① **准静态（无重力）**：两片初始**带内**微穿透 ⇒ 接触把它们**打开到 ≈ `skin`** 后停住
//!    （无重力 ⇒ 没有"再落回来"的相位 ⇒ 读数**不依赖取哪一 tick**）；
//! ② **翻转 A 的环绕序 ⇒ 读数逐位不变**（双面口径的直接推论：法线取自"顶点在哪侧"）；
//! ③ **背侧**（A 在 B 之下带内）⇒ 被推向**更远的一侧**、**不穿过去**；
//! ④ **有重力（动态）**：取**整段窗口的最小间距** ⇒ 两片**任何时刻都不互穿**。
//!    ⚠️ 单点读数在这里**没用**：无阻尼下会弹跳（实测 tick 600 读到 20×skin，只是极限环的相位
//!    —— 正是 `SURVEY` §8.4.46 记的那条"窗口会被极限环骗过"）。
use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Quat, Vec3};

/// 窄相接触带（`PhysConfig::default()` ⇒ 0.02，见 `vxl-phys-core/config.rs`）。
const SKIN: f32 = 0.02;
const TICKS: usize = 600;

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 `y = 0` 平面）。`flip` ⇒ **三角环绕序翻转**（法线朝 −Y）。
fn plate(n: usize, size: f32, flip: bool) -> (Vec<Vec3>, Vec<[u32; 3]>) {
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
            if flip {
                tris.push([a, c, d]);
                tris.push([c, d + 1, d]);
            } else {
                tris.push([a, d, c]);
                tris.push([c, d, d + 1]);
            }
        }
    }
    (pts, tris)
}

/// 一次场景的读数。**两片都保持水平、心线在 `y = 0`（局部）** ⇒ 体心 `y` 就是那一片的**心线高度**
/// ⇒ 差 = 层间距（与绝对高度无关 ⇒ 位置无关判据，§7）。
struct Read {
    /// 末态间距。
    dy_end: f32,
    /// 全程最小间距（有重力时 < 0 = 某时刻互穿）。
    dy_min: f32,
    /// 全程最大间距。
    dy_max: f32,
    finite: bool,
}

/// A（小片 0.5×0.5、0.5 kg）相对 B（大片 1×1、10 kg）以 `offset` 起步；`flip_a` = A 的环绕序翻转。
fn run(offset: f32, flip_a: bool, gravity: bool) -> Read {
    let cfg = PhysConfig {
        gravity: if gravity {
            Vec3::new(0.0, -9.81, 0.0)
        } else {
            Vec3::ZERO
        },
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let (pb, tb) = plate(2, 0.5, false);
    let mesh_b = w.add_trimesh(pb, tb);
    let b = w.spawn_trimesh_body(mesh_b, Vec3::ZERO, Quat::IDENTITY, 1000.0, 0.01) as usize;
    let (pa, ta) = plate(2, 0.25, flip_a);
    let mesh_a = w.add_trimesh(pa, ta);
    let a = w.spawn_trimesh_body(
        mesh_a,
        Vec3::new(0.0, offset, 0.0),
        Quat::IDENTITY,
        200.0,
        0.01,
    ) as usize;
    let mut r = Read {
        dy_end: 0.0,
        dy_min: f32::MAX,
        dy_max: f32::MIN,
        finite: true,
    };
    for _ in 0..TICKS {
        w.step();
        let dy = w.bodies.position[a].y - w.bodies.position[b].y;
        r.dy_min = r.dy_min.min(dy);
        r.dy_max = r.dy_max.max(dy);
        r.finite &= w.bodies.position[a].is_finite() && w.bodies.position[b].is_finite();
    }
    r.dy_end = w.bodies.position[a].y - w.bodies.position[b].y;
    r
}

/// ① **准静态（无重力）**：带内微穿透 ⇒ 接触把它们**打开到 ≈ `skin`** 后停住。
#[test]
fn contact_opens_the_gap_to_the_skin_band() {
    let r = run(0.5 * SKIN, false, false);
    println!(
        "[判据①准静态打开] 末间距 dy = {:+.5} = {:.2}×skin（全程 [{:+.5}, {:+.5}]）",
        r.dy_end,
        r.dy_end / SKIN,
        r.dy_min,
        r.dy_max
    );
    assert!(r.finite, "出现非有限值");
    assert!(
        r.dy_end >= 0.3 * SKIN,
        "间距该被打开到 ≈ skin（实测 {:.2}×skin）——太小说明接触没生效（两片叠在一起）",
        r.dy_end / SKIN
    );
    assert!(
        r.dy_end <= 1.5 * SKIN,
        "间距该**停在** skin 附近（实测 {:.2}×skin）——大了说明接触在持续外推",
        r.dy_end / SKIN
    );
}

/// ② **翻转环绕序 ⇒ 读数逐位不变**（**双面口径的直接推论**）。这是"口径锁定"判据：
/// 若有人把口径改回**单面**（固定环绕法线），翻转后 A 会被推向错误的一侧 ⇒ 这里会红。
#[test]
fn flipped_winding_reads_bit_identically() {
    let plain = run(0.5 * SKIN, false, false);
    let flipped = run(0.5 * SKIN, true, false);
    println!(
        "[判据②环绕序] 未翻转 dy = {:+.8}；**翻转后** dy = {:+.8}（差 {:.2e}）",
        plain.dy_end,
        flipped.dy_end,
        (plain.dy_end - flipped.dy_end).abs()
    );
    assert!(flipped.finite, "出现非有限值");
    assert!(
        flipped.dy_end >= 0.3 * SKIN,
        "翻转环绕序后 A 仍该被推离（实测 dy={:+.5}）——红了说明口径退回**单面**\
         （固定环绕法线会让反面的顶点被推得更深）",
        flipped.dy_end
    );
    assert_eq!(
        plain.dy_end.to_bits(),
        flipped.dy_end.to_bits(),
        "环绕序不该改变读数（未翻转 {:+.8} vs 翻转 {:+.8}）——双面口径下法线取自\
         「顶点在哪侧」，与环绕序无关",
        plain.dy_end,
        flipped.dy_end
    );
}

/// ③ **背侧**：A 在 B **之下**带内 ⇒ 被推向**更远的一侧**，**不穿过去**。
/// 单面口径下这里会被推向错误方向（穿过去）。
#[test]
fn sheet_below_is_pushed_away_not_through() {
    let r = run(-0.5 * SKIN, false, false);
    println!(
        "[判据③背侧] 末间距 dy = {:+.5} = {:.2}×skin（负 = A 在 B 之下）",
        r.dy_end,
        r.dy_end / SKIN
    );
    assert!(r.finite, "出现非有限值");
    assert!(
        r.dy_end <= -0.3 * SKIN,
        "A 起点在 B 之下 ⇒ 该被**推离**（留在下方，实测 {:.2}×skin）——反号就是它穿过去了",
        r.dy_end / SKIN
    );
}

/// ④ **有重力（动态）**：取**整段窗口的最小间距** —— 两片**任何时刻都不互穿**。
/// ⚠️ 必须用窗口最小值：无阻尼下会弹跳，单点读数只是相位（实测 tick 600 读到 20×skin）。
#[test]
fn sheets_never_interpenetrate_under_gravity() {
    let r = run(0.5 * SKIN, false, true);
    println!(
        "[判据④有重力窗口] 间距 ∈ [{:+.5}, {:+.5}]（末 {:+.5}，skin = {SKIN}）",
        r.dy_min, r.dy_max, r.dy_end
    );
    assert!(r.finite, "出现非有限值");
    assert!(
        r.dy_min >= 0.0,
        "全程最小间距 {:+.5} 该 ≥ 0（负 = 某一刻两片互穿了）——弹跳是物理的，穿模不是",
        r.dy_min
    );
}
