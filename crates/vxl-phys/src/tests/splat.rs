//! **高斯喷溅 + 介质场（单向/双向）判据** —— `tests` 的子模块。
//!
//! 2026-10-05 从 967 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use crate::{Aabb, PhysConfig, Quat, Shape, Vec3, World};

/// **M3 喷溅域**：盒落在一团高斯喷溅上并停住（隐式场 σ(p) = Σ 核；
/// 接触走统一提供者通道 `contacts_box`）。
#[test]
fn box_rests_on_gaussian_splat_field() {
    let mut w = World::new(PhysConfig::default());
    let mut f = vxl_phys_splat::GaussianSplatField::new(0.5);
    // 半径 1.2 球状栅格（间距 0.4、核半径 0.35）⇒ 顶面等值面 ≈ y = 1.6
    for i in -3..=3 {
        for j in -3..=3 {
            for k in -3..=3 {
                let c = Vec3::new(i as f32 * 0.4, j as f32 * 0.4, k as f32 * 0.4);
                if c.length() <= 1.2 {
                    f.push(vxl_phys_splat::Splat::isotropic(c, 0.35, 1.0));
                }
            }
        }
    }
    assert!(f.len() > 100);
    w.add_splat_field(f);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 3.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    for _ in 0..600 {
        w.step();
    }
    let y = w.bodies.position[b as usize].y;
    // 停在等值面之上（盒半长 0.4 + 顶面 ≈ 1.6 ⇒ 中心 ≈ 2.0）
    assert!(y > 1.5 && y < 2.6, "y = {y}");
    assert!(w.health().is_clean());
}

/// **介质耦合（喷溅域第三层）**：稀薄喷溅云（σ < iso ⇒ 不产生接触）作介质
/// ⇒ 落体被二次阻力减速；介质密度 0 时为纯自由落体（对照）。
/// 口径：两条 run 仅差 `medium_density`，其余位姿/初始条件完全相同。
#[test]
fn splat_medium_drag_slows_falling_body() {
    let run = |medium_density: f32| -> (f32, f32) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0); // iso 高 ⇒ 纯介质、无接触
        for k in 0..16 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
                0.45,
                0.6,
            ));
        }
        f.medium_density = medium_density;
        f.medium_velocity = Vec3::ZERO;
        w.add_splat_field(f);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 9.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..90 {
            w.step();
        }
        (
            w.bodies.position[b as usize].y,
            w.bodies.linvel[b as usize].y,
        )
    };
    let (y_free, v_free) = run(0.0);
    let (y_drag, v_drag) = run(6.0);
    println!("free: y={y_free:.3} v={v_free:.3} | drag: y={y_drag:.3} v={v_drag:.3}");
    assert!(
        y_drag > y_free + 0.2,
        "介质应显著减速：free y={y_free} drag y={y_drag}"
    );
    assert!(
        v_drag > v_free + 0.5,
        "末速应更高（落得更慢）：{v_free} vs {v_drag}"
    );
}

/// **D3 切片 1（介质双向，`PLAN-COUPLING.md` §4.3）**：场景与 `splat_medium_drag_slows_falling_body`
/// 同款（16 核稀薄云 + 落盒 90 tick），两条 run 仅差 `set_two_way`。判据三个方向：
/// ① **关档金丝雀**：`kernel_velocities()` 空、`absorbed_momentum()` 零（旧单向路径逐位）；
/// ② **开档有沉积**：核速度非零、审计动量非零，且**分量方向** = 体失去的（体下落 ⇒ 介质获 −y）；
/// ③ **反馈方向（有分辨力）**：介质被体带起 ⇒ 相对速度变小 ⇒ 阻力更小 ⇒ 落得更快。
#[test]
fn splat_medium_two_way_reduces_drag() {
    let run = |two_way: bool| -> (f32, f32, Vec<Vec3>, Vec3) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0); // iso 高 ⇒ 纯介质、无接触
        for k in 0..16 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
                0.45,
                0.6,
            ));
        }
        f.medium_density = 6.0;
        f.set_two_way(two_way);
        w.add_splat_field(f);
        let b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 9.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..90 {
            w.step();
        }
        assert!(w.providers.splat(0).is_some(), "splat field 已注册");
        // 无 `.expect`（unwrap 棘轮只准减）：断言存在 + if-let 兜底 ⇒ 门与判据都不破。
        let (kv, abs) = if let Some(fld) = w.providers.splat(0) {
            (fld.kernel_velocities().to_vec(), fld.absorbed_momentum())
        } else {
            (Vec::new(), Vec3::ZERO)
        };
        (
            w.bodies.position[b as usize].y,
            w.bodies.linvel[b as usize].y,
            kv,
            abs,
        )
    };
    let (y_off, v_off, kv_off, abs_off) = run(false);
    let (y_on, v_on, kv_on, abs_on) = run(true);
    // ① 关档金丝雀：不收集/不提交（旧单向语义零代际）。
    assert!(kv_off.is_empty(), "关档不建速度场");
    assert_eq!(abs_off, Vec3::ZERO, "关档无沉积");
    // ② 开档有沉积（含方向：介质获得体失去的动量 ⇒ −y）。
    assert!(
        kv_on.iter().any(|v| v.length() > 0.0),
        "开档核速度非零（沉积真发生）"
    );
    assert!(abs_on.length() > 0.0, "开档审计动量非零");
    assert!(
        abs_on.y < 0.0,
        "反作用方向：介质获 −y 动量（abs={abs_on:?}）"
    );
    // ③ 反馈方向：阻力被自己带起的流减小 ⇒ 末速更负、位置更低。
    assert!(
        v_on < v_off,
        "双向应减小阻力：v_off={v_off:.3} v_on={v_on:.3}"
    );
    assert!(
        y_on < y_off,
        "双向落得更快：y_off={y_off:.3} y_on={y_on:.3}"
    );
    println!(
        "off: y={y_off:.3} v={v_off:.3} | on: y={y_on:.3} v={v_on:.3} | abs={abs_on:?} | kv#={}",
        kv_on.iter().filter(|v| v.length() > 0.0).count()
    );
}

/// **D3 切片 2（平流）**：同场景两条 run 仅差 `set_two_way`（与上一条同款 16 核云 + 落盒）。
/// 判据：① 关档核中心**逐位不动**（advance 首行短路）；② 开档有位移且**净位移方向 = 体失去的
/// 动量方向**（体下落 ⇒ 介质获 −y ⇒ 核朝 −y 漂）。
#[test]
fn splat_medium_two_way_advects_kernels() {
    let run = |two_way: bool| -> (Vec<Vec3>, Vec<Vec3>) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0); // iso 高 ⇒ 纯介质、无接触
        for k in 0..16 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
                0.45,
                0.6,
            ));
        }
        f.medium_density = 6.0;
        f.set_two_way(two_way);
        let c0: Vec<Vec3> = f.splats().iter().map(|s| s.center).collect();
        w.add_splat_field(f);
        let _b = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.3),
            },
            Vec3::new(0.0, 9.0, 0.0),
            Quat::IDENTITY,
            1.0,
        );
        for _ in 0..90 {
            w.step();
        }
        assert!(w.providers.splat(0).is_some(), "splat field 已注册");
        if let Some(fld) = w.providers.splat(0) {
            let dc: Vec<Vec3> = fld
                .splats()
                .iter()
                .zip(&c0)
                .map(|(s, c)| s.center - *c)
                .collect();
            (dc, fld.kernel_velocities().to_vec())
        } else {
            (Vec::new(), Vec::new())
        }
    };
    let (dc_off, _) = run(false);
    let (dc_on, kv_on) = run(true);
    // ① 关档金丝雀：核中心逐位不动（默认档零代际的几何侧证据）。
    assert!(dc_off.iter().all(|d| *d == Vec3::ZERO), "关档核不动");
    // ② 开档：有位移 + 净向 −y（体失去的动量方向）+ 净位移量有分辨力。
    let sum = dc_on.iter().fold(Vec3::ZERO, |a, d| a + *d);
    let moved = dc_on.iter().filter(|d| **d != Vec3::ZERO).count();
    assert!(moved > 0, "开档至少一个核发生平流");
    assert!(kv_on.iter().any(|v| v.length() > 0.0), "开档速度场非空");
    assert!(
        sum.y < -0.01,
        "净位移应朝 −y（体失去的动量方向）：sum={sum:?}（核数 moved={moved}）"
    );
    println!(
        "平流：moved={moved}/16  ΣΔ={sum:?}（|Σ|={:.4} m）",
        sum.length()
    );
}

/// **D3 切片 4a（端到端动量账）**：双向档**逐 tick 对账**——**场侧** `absorbed` 增量 ==
/// **体侧**失去的动量（扣重力）：`Δabs = m·g·dt − m·Δv`。两侧来源独立（场累计 vs 体状态差），
/// 把"体收到的作用 == 场吸收的动量"这条反对称钉在端到端判据上（场景与 two_way 判据同款）。
#[test]
fn splat_medium_momentum_ledger_matches_body_delta() {
    let mut w = World::new(PhysConfig::default());
    let mut f = vxl_phys_splat::GaussianSplatField::new(4.0);
    for k in 0..16 {
        f.push(vxl_phys_splat::Splat::isotropic(
            Vec3::new(0.0, 1.0 + k as f32 * 0.5, 0.0),
            0.45,
            0.6,
        ));
    }
    f.medium_density = 6.0;
    f.set_two_way(true);
    w.add_splat_field(f);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.3),
        },
        Vec3::new(0.0, 9.0, 0.0),
        Quat::IDENTITY,
        1.0,
    ) as usize;
    let im = w.bodies.inv_mass[b];
    let m = if im > 0.0 { 1.0 / im } else { f32::NAN };
    let (g_y, dt) = (w.config.gravity.y, w.config.dt);
    let mut prev_v = w.bodies.linvel[b].y;
    let mut prev_abs = 0.0f32;
    let mut worst = 0.0f32;
    for _ in 0..90 {
        w.step();
        let v = w.bodies.linvel[b].y;
        let abs = match w.providers.splat(0) {
            Some(fld) => fld.absorbed_momentum().y,
            None => f32::NAN,
        };
        let dabs = abs - prev_abs;
        let expected = m * g_y * dt - m * (v - prev_v);
        let scale = (m * g_y * dt).abs().max((m * (v - prev_v)).abs()).max(1e-4);
        worst = worst.max((dabs - expected).abs() / scale);
        prev_v = v;
        prev_abs = abs;
    }
    println!("动量账（90 tick，逐 tick 对账）：最坏相对误差 {worst:.3e}");
    assert!(worst <= 1e-4, "逐 tick 账应平：最坏相对误差 {worst:.3e}");
}

/// **D3 切片 3（AABB 随核漂移刷新）**：单核场（σ=0.5 ⇒ 注册包围盒 ±2.0 m）被注入强 −y 速度
/// 平流 1 s ⇒ 核漂 ≈1.5 m。判据：① 关档 `provider_bounds` **逐位不动**（默认档零代际）；
/// ② 开档世界 AABB 跟随漂移（min.y 显著下移）；③ **修前会漏的几何**：漂移后的探点落在
/// 「新 AABB 内 ∧ 旧 AABB 外」——旧门会把它当真空跳过，新门放行（采样非真空）。
#[test]
fn drifting_medium_bounds_follow_kernels() {
    // 探点：旧 AABB（±2.0）之外、核漂后的截断域（√cut·σ = 2.0）之内（两条 run 共用）。
    let probe = Vec3::new(0.0, -3.0, 0.0);
    let run = |two_way: bool| -> (Aabb, Aabb, f32, Vec3) {
        let mut w = World::new(PhysConfig::default());
        let mut f = vxl_phys_splat::GaussianSplatField::new(4.0);
        f.push(vxl_phys_splat::Splat::isotropic(Vec3::ZERO, 0.5, 1.0));
        f.medium_density = 3.0;
        f.set_two_way(two_way);
        f.damping = 1.0; // 不衰减：本判据量的是"几何跟随"，把动力学因素取直
        w.add_splat_field(f);
        let bb0 = w.provider_bounds[0]; // 注册时快照
                                        // 直接给核注入 −y 动量（不依赖体动力学）：J = −1.5·m（质量取单一来源）。
        if let Some(fm) = w.providers.splat_mut(0) {
            use vxl_phys_core::interop::MediumField as _;
            let j = -1.5 * fm.splats()[0].mass(fm.medium_density);
            fm.deposit(Vec3::ZERO, Vec3::new(0.0, j, 0.0), 0.0, 0.0);
        }
        for _ in 0..60 {
            w.step(); // 平流 1 s
        }
        let center = match w.providers.splat(0) {
            Some(fld) => fld.splats()[0].center,
            None => Vec3::ZERO,
        };
        let dens = match w.providers.splat(0) {
            Some(fld) => {
                use vxl_phys_core::interop::MediumField as _;
                fld.sample(probe).density
            }
            None => 0.0,
        };
        (bb0, w.provider_bounds[0], dens, center)
    };
    let (bb0_off, bb1_off, dens_off, c_off) = run(false);
    let (bb0_on, bb1_on, dens_on, c_on) = run(true);
    // ① 关档金丝雀：核不动（逐位）+ AABB 不刷新（逐位）。
    assert_eq!(c_off, Vec3::ZERO, "关档核不动（逐位）");
    assert_eq!(
        bb1_off.min.y.to_bits(),
        bb0_off.min.y.to_bits(),
        "关档 AABB 不动"
    );
    assert_eq!(
        bb1_off.max.x.to_bits(),
        bb0_off.max.x.to_bits(),
        "关档 AABB 不动"
    );
    // ② 开档：核漂了、AABB 跟着漂。
    assert!(c_on.y < -1.0, "核漂移量级：center.y={:.3}", c_on.y);
    assert!(
        bb1_on.min.y < bb0_on.min.y - 1.0,
        "AABB 应跟随漂移：min.y {:.3} → {:.3}",
        bb0_on.min.y,
        bb1_on.min.y
    );
    // ③ 修前会漏的几何：探点在旧 AABB 外、新 AABB 内、且采样非真空。
    let old_contains = probe.y >= bb0_on.min.y && probe.y <= bb0_on.max.y;
    let new_contains = probe.y >= bb1_on.min.y && probe.y <= bb1_on.max.y;
    assert!(!old_contains, "探点应在旧 AABB 之外（否则用例不自证）");
    assert!(new_contains, "探点应落入新 AABB");
    assert!(dens_on > 0.0, "新几何下采样非真空（dens_on={dens_on}）");
    println!(
        "AABB：min.y {:.3} → {:.3}（核 y={:.3}）；探点旧内={old_contains} 新内={new_contains} 密度={dens_on:.3e}（关档 {:.3e}）",
        bb0_on.min.y, bb1_on.min.y, c_on.y, dens_off
    );
}
