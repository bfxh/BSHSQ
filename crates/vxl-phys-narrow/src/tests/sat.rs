//! **SAT 快路径 vs 通用路径**（逐位对拍）—— `tests` 的子模块。
//!
//! 2026-10-05 从 836 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use crate::box_axes;
use crate::{DefaultNarrowPhase, Shape};
use vxl_phys_core::{Quat, Vec3};

/// T3 盒对 SAT extents 快路径 vs 通用逐顶点路径：300 对随机盒
/// （重叠/分离/极端姿态混合）同帧对拍，断言 None/Some 类别一致、
/// sep 差 ≤1e-4、法线对齐 >0.999、来源分类一致。守门对象：`sat()`
/// 内盒对分支（extents 公式）与通用顶点 min/max 的等价性。
#[test]
fn box_sat_fast_matches_vertex_reference() {
    let mut np = DefaultNarrowPhase::new(0.01);
    let ha = Vec3::new(0.5, 0.3, 0.7);
    let hb = Vec3::new(0.4, 0.6, 0.2);
    let ia = crate::support::poly_for(&mut np, &Shape::Box { half: ha }).unwrap_or(usize::MAX);
    let ib = crate::support::poly_for(&mut np, &Shape::Box { half: hb }).unwrap_or(usize::MAX);
    assert!(ia != usize::MAX && ib != usize::MAX, "盒多面体未注册");
    let mut rng: u32 = 0x1234_5678;
    let mut next = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (rng >> 8) as f32 / 16_777_216.0
    };
    let mut max_delta = 0.0f32;
    for k in 0..300 {
        let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
        let pb = pa + Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0);
        let ax = Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize();
        let bx = Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize();
        let qa = Quat::from_axis_angle(ax, next() * core::f32::consts::TAU);
        let qb = Quat::from_axis_angle(bx, next() * core::f32::consts::TAU);
        np.ws.poly_a.fill(&np.ws.polys[ia], pa, qa);
        np.ws.poly_b.fill(&np.ws.polys[ib], pb, qb);
        let d = pb - pa;
        // 快路径（盒对 extents 公式）。
        np.ws.box_a = Some((ha, pa));
        np.ws.box_b = Some((hb, pb));
        let fast = crate::sat::sat(&mut np, d);
        // 通用路径（逐顶点 min/max；轴序、取向、平局规则完全相同，
        // 唯一差异即投影计算方式）。
        np.ws.box_a = None;
        np.ws.box_b = None;
        let slow = crate::sat::sat(&mut np, d);
        match (fast, slow) {
            (None, None) => {}
            (Some((s1, n1, r1)), Some((s2, n2, r2))) => {
                let d = (s1 - s2).abs();
                if d > max_delta {
                    max_delta = d;
                }
                assert!(d <= 1e-6, "k{k}: sep {s1} vs {s2}（Δ {d}，非 ULP 级）");
                assert!(n1.dot(n2).abs() > 0.999, "k{k}: normal {n1:?} vs {n2:?}");
                assert_eq!(r1, r2, "k{k}: src {r1:?} vs {r2:?}");
            }
            (f, s) => assert_eq!(f.is_some(), s.is_some(), "k{k}: 类别不一致（fast/slow）"),
        }
    }
    eprintln!("快/通用路径 sep 最大 Δ = {max_delta:.3e}（f32 ULP 级；>0 表示快路径确被拉到）");
    assert!(max_delta > 0.0, "两路径逐位相同 ⇒ 快路径未被真正测到");
}

/// T3 盒对专用路径 vs 通用路径**全链对拍**：同一姿态下，唯一变量是
/// 「体轴直生（专用）还是多面体填充（通用）」，断言 SAT 的 sep/法线/来源
/// 三者一致，且 `clip` 的接触点集合逐点对应（点数相同、特征号逐位相同、
/// 位置差 ≤1e-5——两条路径的顶点算术序不同，只保证到 ULP 级）。
/// 守门对象：专用路径的轴序 / 面表环绕序 / 特征号编号。
#[test]
fn box_dedicated_matches_generic_full_chain() {
    let mut np = DefaultNarrowPhase::new(0.02);
    let ha = Vec3::new(0.5, 0.3, 0.7);
    let hb = Vec3::new(0.4, 0.6, 0.2);
    let ia = crate::support::poly_for(&mut np, &Shape::Box { half: ha }).unwrap_or(usize::MAX);
    let ib = crate::support::poly_for(&mut np, &Shape::Box { half: hb }).unwrap_or(usize::MAX);
    assert!(ia != usize::MAX && ib != usize::MAX, "盒多面体未注册");
    let mut rng: u32 = 0x51ED_2701;
    let mut next = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (rng >> 8) as f32 / 16_777_216.0
    };
    let mut contacts_seen = 0usize;
    for k in 0..300 {
        let qa = Quat::from_axis_angle(
            Vec3::new(next() + 0.2, next() + 0.5, 1.0).normalize(),
            next() * core::f32::consts::TAU,
        );
        let qb = Quat::from_axis_angle(
            Vec3::new(next() + 0.5, next() + 0.2, 1.0).normalize(),
            next() * core::f32::consts::TAU,
        );
        let pa = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
        // 近半样本深度重叠（保证 clip 真的跑出接触点），其余随机。
        let d = if k % 2 == 0 {
            Vec3::new(
                next() * 0.3 - 0.15,
                next() * 0.3 - 0.15,
                next() * 0.3 - 0.15,
            )
        } else {
            Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0)
        };
        let pb = pa + d;

        // 通用路径：多面体填充 + 盒对 extents 快路径。
        np.ws.poly_a.fill(&np.ws.polys[ia], pa, qa);
        np.ws.poly_b.fill(&np.ws.polys[ib], pb, qb);
        np.ws.box_axes_a = None;
        np.ws.box_axes_b = None;
        np.ws.box_a = Some((ha, pa));
        np.ws.box_b = Some((hb, pb));
        let generic = match crate::sat::sat(&mut np, d) {
            Some((sep, n, src)) if sep <= np.skin => {
                if crate::sat::clip(&mut np, n, src) {
                    Some((sep, n, src, np.ws.cand.clone()))
                } else {
                    Some((sep, n, src, Vec::new()))
                }
            }
            _ => None,
        };

        // 专用路径：体轴直生（不填多面体——与生产路径一致）。
        let aa = box_axes(qa);
        let ab = box_axes(qb);
        np.ws.box_axes_a = Some(aa);
        np.ws.box_axes_b = Some(ab);
        let dedicated = match crate::sat::sat(&mut np, d) {
            Some((sep, n, src)) if sep <= np.skin => {
                if crate::sat::clip(&mut np, n, src) {
                    Some((sep, n, src, np.ws.cand.clone()))
                } else {
                    Some((sep, n, src, Vec::new()))
                }
            }
            _ => None,
        };

        match (generic, dedicated) {
            (None, None) => {}
            (Some((s1, n1, r1, c1)), Some((s2, n2, r2, c2))) => {
                assert_eq!(s1.to_bits(), s2.to_bits(), "k{k}: sep 位不等");
                assert!(n1.dot(n2).abs() > 0.9999, "k{k}: 法线不一致");
                assert_eq!(r1, r2, "k{k}: 来源分类不一致");
                assert_eq!(c1.len(), c2.len(), "k{k}: 接触点数不一致");
                for p in &c1 {
                    let hit = c2
                        .iter()
                        .any(|q| q.feature == p.feature && (q.point - p.point).length() <= 1e-5);
                    assert!(hit, "k{k}: 通用点 {:?} 在专用路径无对应", p.point);
                }
                if !c1.is_empty() {
                    contacts_seen += 1;
                }
            }
            (g, s) => assert_eq!(g.is_some(), s.is_some(), "k{k}: 类别不一致（通用/专用）"),
        }
    }
    assert!(
        contacts_seen >= 100,
        "接触样本仅 {contacts_seen}，鉴别力不足"
    );
}
