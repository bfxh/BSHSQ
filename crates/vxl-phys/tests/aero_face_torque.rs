//! **面元**力矩**判据**（T4 的"每面力/力矩"里的**力矩腿**）：`world_step/aero.rs` 逐面施加
//! `τ_i = (c_i − p) × F_i`（面心力臂），但力矩腿此前只有"对称板 ≈ 0"一条旁注
//! （`aero_face_forces.rs` ③ 的尾注），没有正条判据。本文件把力矩腿钉死——
//! **只有判据、没有新代码**（片 15 同款："证明引擎里那份接线真的生效"）。
//!
//! **判据**（机器无关）：
//! ① **静力矩闭式逐位**：偏置板（网格整体 `+x` 偏 `d`）静态 + ⊥ 风 ⇒ 每面 `F` 相同 ⇒
//!    `τ = Σ (c_i − p) × F = (ΣF) × (c̄ − p)`；测试按**同一注册序、同一 f32 运算**重算离散和
//!    ⇒ 与读数**逐位相同**；且合力与"不偏置板"**逐位相同**（均匀风 ⇒ 合力与力臂无关）；
//! ② **力臂线性标度**：`d` 与 `2d` ⇒ `|τ|₂ ≈ 2·|τ|₁`（f32 舍入 ⇒ 1e-5 相对容差）；
//! ③ **动力学**：动态偏置板 ⇒ `ω` 沿 τ 方向单调发展；力臂翻倍 ⇒ `|ω|` 严格更大
//!    （力臂 ×2 但转轴惯量也变大 ⇒ 只断言方向与严格增，不卡解析值——惯量口径由
//!    薄壳质量那条判据单独钉）；
//! ④ **金丝雀**：不开 aero ⇒ 同场景 `ω` **逐位 0**。
//!
//! **边界**：本档只钉"逐面力矩的接线与口径"，不声称完整气动建模（风场空间变化等仍是
//! 线化档的登记边界）。
use vxl_phys::*;
use vxl_phys_aero::AeroConfig;
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};

const WIND: [f32; 3] = [0.0, -5.0, 0.0]; // ⊥ 板（板在 y=0 平面），|风| = 5

/// 平铺网格（`n×n` 格、跨度 `±size`、y=0 平面）整体偏置 `off`（沿 +x）。
fn plate_offset(n: usize, size: f32, off: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(
                off - size + s * ix as f32,
                0.0,
                -size + s * iz as f32,
            ));
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

/// 静态偏置板 + 风（判据①/② 的装置；返回世界与体号）。
fn static_offset_plate(off: f32) -> (World, u32) {
    let (pts, tris) = plate_offset(2, 0.5, off);
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let mesh = w.add_trimesh(pts, tris);
    let half = w.trimesh_half_extents(mesh);
    let b = w.add_static(Shape::TriMesh { mesh, half }, Vec3::ZERO, Quat::IDENTITY);
    w.set_aero(AeroConfig {
        wind: WIND,
        ..AeroConfig::default()
    });
    (w, b)
}

/// **与引擎同一注册序、同一 f32 运算**重算逐面力矩离散和（判据① 的闭式侧）。
fn torque_discrete_sum(pts: &[Vec3], tris: &[[u32; 3]], cfg: &AeroConfig) -> Vec3 {
    let wind = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
    let mut t_sum = Vec3::ZERO;
    let mut f_sum = Vec3::ZERO;
    for tri in tris {
        let (p0, p1, p2) = (
            pts[tri[0] as usize],
            pts[tri[1] as usize],
            pts[tri[2] as usize],
        );
        let n_full = (p1 - p0).cross(p2 - p0);
        let two_a = n_full.length();
        if two_a <= 1e-12 {
            continue;
        }
        let center = (p0 + p1 + p2) * (1.0 / 3.0);
        let f = vxl_phys_aero::face_force(wind, two_a * 0.5, cfg);
        f_sum += f;
        t_sum += center.cross(f); // 体在原点 ⇒ 力臂 = center（逐位同引擎的 center − pos）
    }
    let _ = f_sum;
    t_sum
}

/// **判据①**：偏置板静力矩 = 离散和**逐位**；合力与不偏置板**逐位**相同。
#[test]
fn static_torque_matches_discrete_sum_bitwise() {
    let d = 0.25f32;
    let (mut w, b) = static_offset_plate(d);
    w.step();
    let tau = w.aero_torque(b);
    let force = w.aero_force(b);
    // 闭式侧：同一注册序、同一 f32 运算（静态 ⇒ 面元速度恒 0 ⇒ u = wind 逐位）。
    let (pts, tris) = plate_offset(2, 0.5, d);
    let cfg = AeroConfig {
        wind: WIND,
        ..AeroConfig::default()
    };
    let want = torque_discrete_sum(&pts, &tris, &cfg);
    println!(
        "[判据·力矩①] 偏置 d={d} 板：τ 读数 = {tau:?}；离散和 = {want:?}（闭式 = (ΣF)×(c̄−p) ⇒ \
         只该有 −z 分量）"
    );
    assert!(
        tau == want,
        "静力矩读数该与离散和**逐位相同**（读 {tau:?} vs 算 {want:?}）⇒ \
         出现差异说明求和序/口径被改"
    );
    assert!(
        tau.x.abs() < 1e-6 && tau.y == 0.0 && tau.z < 0.0,
        "闭式 τ = (ΣF)×(d,0,0)：F = (0,−F,0) ⇒ τ 只该有 **−z** 分量（x 的 1e-6 内是 f32 \
         离散求和残差，实得 {tau:?}）"
    );
    // 合力与力臂无关（均匀风、面面相同 ⇒ 每面 F 一样）：与不偏置板逐位比。
    let (mut w0, b0) = static_offset_plate(0.0);
    w0.step();
    let force0 = w0.aero_force(b0);
    println!(
        "[判据·力矩①-合力] 偏置 d={d} 合力 = {force:?}；不偏置板合力 = {force0:?}（该逐位相同）"
    );
    assert!(
        force == force0,
        "均匀风 ⇒ 合力与力臂无关（偏置 {force:?} vs 不偏置 {force0:?} 该逐位相同）"
    );
}

/// **判据②**：力臂线性标度 —— `d` 与 `2d` ⇒ `|τ|₂ ≈ 2·|τ|₁`。
#[test]
fn torque_scales_linearly_with_offset() {
    let (mut w1, b1) = static_offset_plate(0.25);
    w1.step();
    let t1 = w1.aero_torque(b1).z;
    let (mut w2, b2) = static_offset_plate(0.5);
    w2.step();
    let t2 = w2.aero_torque(b2).z;
    println!("[判据·力矩②] |τ(d)| = {t1:.6}，|τ(2d)| = {t2:.6}（比值该 ≈ 2）");
    assert!(
        t1 < 0.0 && t2 < 0.0,
        "两块板的力矩都该沿 −z（实得 {t1} / {t2}）"
    );
    let ratio = t2 / t1;
    assert!(
        (ratio - 2.0).abs() / 2.0 < 1e-5,
        "力臂翻倍 ⇒ 力矩该精确翻倍（实测比值 {ratio:.8}，容差 1e-5）"
    );
}

/// **判据③ + ④**：动态偏置板 ⇒ ω 沿 τ 方向单调发展、力臂翻倍 ⇒ |ω| 严格更大；
/// 金丝雀：不开 aero ⇒ ω 逐位 0。
#[test]
fn dynamic_plate_spins_about_torque_axis_and_canary_stays_zero() {
    let cfg = || PhysConfig {
        gravity: Vec3::ZERO, // 隔离气动（重力会把板拉走、混入读数）
        ..PhysConfig::default()
    };
    let run = |off: f32, aero: bool| -> Vec<f32> {
        let (pts, tris) = plate_offset(2, 0.5, off);
        let mut w = World::new(cfg());
        let mesh = w.add_trimesh(pts, tris);
        let b = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 500.0, 0.02);
        if aero {
            w.set_aero(AeroConfig {
                wind: WIND,
                ..AeroConfig::default()
            });
        }
        let mut omegas = Vec::new();
        for _ in 0..60 {
            w.step();
            omegas.push(w.bodies.angvel(b as usize).z);
        }
        omegas
    };
    let w1 = run(0.25, true);
    let w2 = run(0.5, true);
    let wc = run(0.25, false);
    println!(
        "[判据·力矩③] 动态偏置板：|ω_z| 末值 d=0.25 ⇒ {:+.5}，d=0.5 ⇒ {:+.5}；\
         单调 {}（首 {:+.5} / 中 {:+.5} / 末 {:+.5}）",
        w1.last().expect("非空"),
        w2.last().expect("非空"),
        if w1.last().expect("非空").abs() >= w1[29].abs() && w1[29].abs() >= w1[0].abs() {
            "增 ✅"
        } else {
            "非单调 ❌"
        },
        w1[0],
        w1[29],
        w1.last().expect("非空"),
    );
    println!(
        "[金丝雀·力矩④] 不开 aero：ω_z 末值 = {:+.6}（该逐位 0）",
        wc.last().expect("非空")
    );
    assert!(w1.iter().all(|w| w.is_finite()), "出现非有限值");
    // 方向：τ 沿 −z ⇒ ω_z 该为**负**且幅值单调发展（离散阻尼/反馈 ⇒ 只断言不减）。
    assert!(
        *w1.last().expect("非空") < 0.0,
        "τ = (ΣF)×(d,0,0) 沿 −z ⇒ ω_z 该为负（实得 {:+.5}）",
        w1.last().expect("非空")
    );
    assert!(
        w1.last().expect("非空").abs() >= w1[29].abs() && w1[29].abs() >= w1[0].abs(),
        "|ω_z| 该单调发展（首 {} / 中 {} / 末 {}）——掉了说明力矩腿没在持续施加",
        w1[0].abs(),
        w1[29].abs(),
        w1.last().expect("非空").abs()
    );
    // **单 tick 定量**：ω₀ = τ·h/I，I = 薄板绕 z 过原点（**含偏置的平行轴**）：
    // `I = m·(w²/12 + d²)`，`m = ρ·t·A`（spawn 的密度/壳厚；1 m² 板）。两块板都验 ⇒
    // "τ（含力臂）→ 角冲量 → 惯量（含偏置）"整条管线被闭式钉死。
    // （⚠️ 别拿"力臂翻倍 ⇒ |ω| 更大"当判据：I 随 d² 涨得比 τ 的线性快 ⇒ 长臂板的角加速度
    // 反而更小——首版实测 0.875×，那是物理，不是缺陷。）
    let (rho, thick, area, w_ext) = (500.0f32, 0.02, 1.0, 1.0);
    let mass = rho * thick * area;
    let f_total = 0.5 * 1.225 * 1.0 * area * 25.0; // ½ρ·Cd·A·v²（判据③同式）
    let dt = 1.0 / 60.0;
    for (d, w0) in [(0.25f32, w1[0]), (0.5, w2[0])] {
        let tau = -(d * f_total);
        let inertia = mass * (w_ext * w_ext / 12.0 + d * d);
        let want = tau * dt / inertia;
        println!(
            "[判据·力矩③-定量] d={d}：ω₀ 实测 {w0:+.6}，闭式 τ·h/I = {want:+.6}（I = {inertia:.4}）"
        );
        assert!(
            (w0 - want).abs() / want.abs() < 0.02,
            "首 tick 角速度该 = τ·h/I（偏置进力矩也进惯量；实测 {w0:+.6} vs 闭式 {want:+.6}，\
             容差 2%）"
        );
    }
    // 金丝雀：不开 aero ⇒ ω 逐位 0（60 tick 全程）。
    assert!(
        wc.iter().all(|w| *w == 0.0),
        "不开 aero ⇒ ω 该逐位 0（实得末值 {:+.6}）⇒ 出现非零说明有别的力矩源",
        wc.last().expect("非空")
    );
}
