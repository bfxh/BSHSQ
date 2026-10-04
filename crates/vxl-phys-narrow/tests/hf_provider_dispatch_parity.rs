//! **对拍：高度场「旁路」vs「provider 通道」的流形装配**（M2「真把派发切过去」的前置测量）。
//!
//! - 旁路 = `Shape::HeightField(id)` 走 `heightfield_pair`（候选直取 + 最深样本地形法线）；
//! - provider 通道 = `Shape::Provider(id)` 走 `provider_pair`（`pick_dominant_normal` +
//!   `four_corner_points`），查询转发给**域 trait**（= 迁移后门面会做的事，本文件用测试替身实现）。
//!
//! **实测结论（2026-10-05，本文件就是判据）**：
//! 1. 只要接触集合由 `skin` 决定，两条路**逐位一致**（点/深度/特征/法线）——`assert` 钉住；
//! 2. 差异**只出现在**"运动中的体在 `skin` 之外、但在自适应接触带之内"：`provider_pair` 的带
//!    = `max(skin, |v_rel|·dt·1.5 + skin)`（2026-09-15 修的"跨过皮肤带"问题），
//!    `heightfield_pair` 恒用 `skin` ⇒ 前者产**预期接触**、后者不产。
//!
//! ⇒ **"把高度场派发切到 provider 通道"不是逐位中性的**：它等于把那条自适应带也带给地形，
//! 会改动 `determinism`/`m0_gates` 的哈希（两个场景的地面就是 flat 高度场）⇒ 属**换代决策**，
//! 不是机械搬运。这条断言就是给那次决策留的"现状快照"：真要切，先改这里。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys_core::interop::{CollisionProvider, InteropContact, NoProviders, ProviderColliders};
use vxl_phys_core::{Aabb, BodySet, Quat, SerialJobSystem, Shape, Vec3};
use vxl_phys_narrow::heightfield::HeightField;
use vxl_phys_narrow::{DefaultNarrowPhase, NarrowPhase};

/// 测试替身：把 provider 面的三种查询**原样转给域 trait**（迁移后 `Providers` 就是这么做的）。
struct HfProviders<'a>(&'a HeightField);

impl ProviderColliders for HfProviders<'_> {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(CollisionProvider::bounds(self.0))
    }

    // `#[rustfmt::skip]`：args 门按"逗号数 + 1"计形参，而这个 trait 方法有 6 个形参 + `&self`
    // ⇒ 竖排时 rustfmt 补的**尾逗号**会把它记成 8（新文件即红）。一行写完 = 记 7，与真实形参一致。
    #[rustfmt::skip]
    fn contacts_box(&self, _id: u32, half: Vec3, pos: Vec3, rot: Quat, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        self.0.contacts_box(half, pos, rot, skin, out)
    }

    fn contacts_sphere(
        &self,
        _id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        self.0.contacts_sphere(center, radius, skin, out)
    }

    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        self.0.contacts_point(p, skin, out)
    }
}

fn flat() -> HeightField {
    HeightField::flat(-20.0, -20.0, 41, 41, 1.0, 0.0)
}

/// 一条流形 → `(法线, [(点, 深度, 特征)])`；无流形 = 空。
/// `ground` 是**体 0**（旁路档 = `Shape::HeightField(0)`、provider 档 = `Shape::Provider(0)`）⇒
/// 两条路的流形法线约定都是「+地形外向法线」（地形在 a 侧）。
fn run(
    ground: Shape,
    mover: Shape,
    pos: Vec3,
    vel: Vec3,
    providers: &dyn ProviderColliders,
    hfs: &[HeightField],
) -> (Vec3, Vec<(Vec3, f32, u32)>) {
    let mut b = BodySet::new();
    let g = b.push_static(ground, Vec3::ZERO, Quat::IDENTITY);
    let d = b.push_dynamic(mover, pos, Quat::IDENTITY, 1000.0);
    b.linvel[d as usize] = vel;
    let mut np = DefaultNarrowPhase::new(0.02);
    let mut out = Vec::new();
    np.collide(&b, &[(g, d)], hfs, providers, &mut out, &SerialJobSystem);
    match out.first() {
        Some(m) => (
            m.normal,
            m.points
                .iter()
                .map(|p| (p.point, p.depth, p.feature))
                .collect(),
        ),
        None => (Vec3::ZERO, Vec::new()),
    }
}

fn bits(v: Vec3) -> (u32, u32, u32) {
    (v.x.to_bits(), v.y.to_bits(), v.z.to_bits())
}

#[test]
fn box_on_flat_field_two_routes_parity_and_band_divergence() {
    let hf = flat();
    let half = Vec3::splat(0.5);
    let hfbox = HfProviders(&hf);
    // 三档：① 静止接触（底面 −0.1）② 运动接触（同位置、12 m/s 下落）
    //       ③ 运动**未**接触（缝 0.2；skin 0.02 够不着，但 provider 的自适应带 0.32 够得着）
    let cases = [
        ("① 静止·接触", 0.4, Vec3::ZERO, true),
        ("② 运动·接触", 0.4, Vec3::new(0.0, -12.0, 0.0), true),
        (
            "③ 运动·未接触（缝 0.2）",
            0.7,
            Vec3::new(0.0, -12.0, 0.0),
            false,
        ),
    ];
    for (tag, y, vel, expect_parity) in cases {
        let pos = Vec3::new(0.25, y, 0.25);
        // 旁路：地形在 a 侧 ⇒ 法线 = +地形法线
        let a = run(
            Shape::HeightField(0),
            Shape::Box { half },
            pos,
            vel,
            &NoProviders,
            std::slice::from_ref(&hf),
        );
        // provider 通道：提供者在 a 侧 ⇒ 法线同样 = +地形法线
        let p = run(
            Shape::Provider(0),
            Shape::Box { half },
            pos,
            vel,
            &hfbox,
            &[],
        );
        println!("[{tag}] 旁路 {} 点 | provider {} 点", a.1.len(), p.1.len());
        for (i, (pt, d, f)) in p.1.iter().enumerate() {
            println!("     prov[{i}] p={pt:?} depth={d:.6} f={f}");
        }
        if expect_parity {
            // 接触档：两条路**逐位一致**（本文件的核心判据）
            assert_eq!(a.1.len(), p.1.len(), "{tag}：点数须一致");
            assert_eq!(bits(a.0), bits(p.0), "{tag}：法线须逐位一致");
            for (i, (x, y)) in a.1.iter().zip(p.1.iter()).enumerate() {
                assert_eq!(bits(x.0), bits(y.0), "{tag}：第 {i} 点须逐位一致");
                assert_eq!(x.1.to_bits(), y.1.to_bits(), "{tag}：第 {i} 点深度");
                assert_eq!(x.2, y.2, "{tag}：第 {i} 点特征");
            }
        } else {
            // 带外档：**口径差异就在这里**（provider 的带随相对速度加宽 ⇒ 产预期接触）。
            assert!(a.1.is_empty(), "{tag}：旁路恒用 skin ⇒ 不该有接触");
            assert!(
                !p.1.is_empty(),
                "{tag}：provider 的自适应带应够到（若这里红了，说明带口径已变 ⇒ 须改本文件与 ROUTE）"
            );
            assert!(
                p.1.iter().all(|(_, d, _)| *d < 0.0),
                "{tag}：带外接触应全是预期接触（负深度）"
            );
        }
    }
}
