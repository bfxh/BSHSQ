//! **空间尾流判据**（`vxl_phys_aero::wake` / `AeroWakeExt`，`ROUTE.md` §4 风 × 障碍物）。
//!
//! 判据链（全部可解析对拍，不依赖"看起来像"）：
//! ① **默认档逐位不变**：`wakes` 为空 ⇒ `aero_wind_at` 与基准风**逐位相等**（`to_bits`）；
//! ② **下游亏损**：源点正下游 `s = 0.25·length` 处风速 < 基准风，且沿 `s` **单调递减**
//!    （`1 − s/length` 线性衰减）；
//! ③ **上游不受影响**：源点上游（`s < 0`）逐位等于基准风；
//! ④ **横向衰减**：横向距离越大亏损越小，`r ≥ radius` 处逐位等于基准风；
//! ⑤ **强度线性**：同一位置 `deficit(strength)` 与 `strength` 成正比（`f32` 容差 1e-6）；
//! ⑥ **端到端**：尾流里的三角网体受的风力**小于**无尾流时（面元气动真的用上了空间风场）；
//! ⑦ **未开气动域**：`set_aero_wakes` 返回 false、`aero_wind_at` 为零（不静默建槽）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys::{AeroExt, AeroWakeExt, MeshExt, PhysConfig, Quat, Vec3, World};
use vxl_phys_aero::{AeroConfig, WakeSource};

/// 基准风 `+x 10 m/s`（够大，亏损后可分辨）。
fn wind() -> [f32; 3] {
    [10.0, 0.0, 0.0]
}

fn aero_cfg() -> AeroConfig {
    AeroConfig {
        air_density: 1.225,
        drag_coefficient: 1.0,
        lift_slope: 5.0,
        wind: wind(),
        air_mass: 0.0,
    }
}

fn world_with_aero() -> World {
    let mut w = World::new(PhysConfig::default());
    w.set_aero(aero_cfg());
    w
}

fn bits(v: Vec3) -> (u32, u32, u32) {
    (v.x.to_bits(), v.y.to_bits(), v.z.to_bits())
}

#[test]
fn empty_wakes_is_bit_identical_to_uniform_wind() {
    let w = world_with_aero();
    let base = Vec3::new(wind()[0], wind()[1], wind()[2]);
    for x in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(3.0, 1.0, -2.0),
        Vec3::new(-5.0, 0.5, 7.0),
    ] {
        assert_eq!(
            bits(w.aero_wind_at(x)),
            bits(base),
            "空尾流必须逐位等于基准风"
        );
    }
}

#[test]
fn wake_deficit_grows_downstream_and_is_zero_upstream() {
    let mut w = world_with_aero();
    let src = WakeSource::new(Vec3::ZERO, 4.0, 1.0, 0.8);
    assert!(w.set_aero_wakes(vec![src]), "已开气动域，应接受尾流源");
    let base = 10.0f32;
    let upstream = w.aero_wind_at(Vec3::new(-1.0, 0.0, 0.0)).length();
    assert_eq!(upstream.to_bits(), base.to_bits(), "上游必须逐位等于基准风");

    // 亏损 `1 − s/length` 随下游**衰减** ⇒ 风速沿下游**回升**（s=0 处最慢）。
    let mut prev = 0.0f32;
    for s in [0.0f32, 1.0, 2.0, 3.0] {
        let v = w.aero_wind_at(Vec3::new(s, 0.0, 0.0)).length();
        assert!(
            v > prev,
            "亏损应随下游衰减（风速回升）：s={s} v={v} prev={prev}"
        );
        assert!(v < base, "尾流域内必须低于基准风：s={s} v={v}");
        prev = v;
    }
    // 解析对拍：s=0 处 deficit = strength ⇒ v = base·(1 − 0.8) = 2.0
    let at_origin = w.aero_wind_at(Vec3::ZERO).length();
    assert!(
        (at_origin - 2.0).abs() < 1e-4,
        "源点解析值应为 2.0，实得 {at_origin}"
    );
}

#[test]
fn wake_deficit_decays_laterally_and_vanishes_outside_radius() {
    let mut w = world_with_aero();
    w.set_aero_wakes(vec![WakeSource::new(Vec3::ZERO, 4.0, 1.0, 0.8)]);
    let base = 10.0f32;
    let center = w.aero_wind_at(Vec3::new(0.5, 0.0, 0.0)).length();
    let half = w.aero_wind_at(Vec3::new(0.5, 0.5, 0.0)).length();
    let edge = w.aero_wind_at(Vec3::new(0.5, 1.0, 0.0)).length();
    let outside = w.aero_wind_at(Vec3::new(0.5, 1.2, 0.0)).length();
    assert!(
        center < half,
        "横向越远亏损越小：center={center} half={half}"
    );
    assert!(half < edge, "横向越远亏损越小：half={half} edge={edge}");
    assert_eq!(
        edge.to_bits(),
        base.to_bits(),
        "r = radius 处 cos²(π/2) = 0"
    );
    assert_eq!(
        outside.to_bits(),
        base.to_bits(),
        "r > radius 必须逐位等于基准风"
    );
}

#[test]
fn wake_deficit_scales_linearly_with_strength() {
    let mut w = world_with_aero();
    let probe = Vec3::new(1.0, 0.0, 0.0);
    let base = 10.0f32;
    let mut deficit = [0.0f32; 2];
    for (k, strength) in [0.25f32, 0.5].iter().enumerate() {
        w.set_aero_wakes(vec![WakeSource::new(Vec3::ZERO, 4.0, 1.0, *strength)]);
        deficit[k] = base - w.aero_wind_at(probe).length();
    }
    let ratio = deficit[1] / deficit[0];
    assert!(
        (ratio - 2.0).abs() < 1e-5,
        "亏损应与 strength 成正比，实测比 {ratio}"
    );
}

/// 竖直方板（TriMesh，法线 +x 迎风）+ 可选尾流源。
fn plate_scene(with_wake: bool) -> World {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    // 边长 2 的正方形板，位于 y-z 平面（迎风面法线 +x）。
    let h = 1.0f32;
    let pts = vec![
        Vec3::new(0.0, -h, -h),
        Vec3::new(0.0, -h, h),
        Vec3::new(0.0, h, h),
        Vec3::new(0.0, h, -h),
    ];
    let tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let mesh = w.add_trimesh(pts, tris);
    let body = w.spawn_trimesh_body(mesh, Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY, 1.0, 0.02);
    w.set_aero(aero_cfg());
    if with_wake {
        // 尾流源放在板**上游**（x = 0），亏损区覆盖板所在的下游。
        assert!(w.set_aero_wakes(vec![WakeSource::new(
            Vec3::new(0.0, 0.0, 0.0),
            6.0,
            3.0,
            0.9
        )]));
    }
    let _ = body;
    w
}

#[test]
fn wake_reduces_face_aero_force_end_to_end() {
    let mut clean = plate_scene(false);
    let mut waked = plate_scene(true);
    clean.step();
    waked.step();
    let f_clean = clean.aero_force(0).length();
    let f_waked = waked.aero_force(0).length();
    assert!(f_clean > 1.0, "无尾流时应有可观风力，实得 {f_clean}");
    assert!(
        f_waked < f_clean * 0.5,
        "尾流必须显著削弱风力：clean={f_clean} waked={f_waked}"
    );
}

#[test]
fn wakes_are_ignored_without_aero_domain() {
    let mut w = World::new(PhysConfig::default());
    assert!(
        !w.set_aero_wakes(vec![WakeSource::new(Vec3::ZERO, 4.0, 1.0, 0.5)]),
        "未开气动域时应拒绝（返回 false），不静默建槽"
    );
    assert_eq!(
        bits(w.aero_wind_at(Vec3::new(1.0, 2.0, 3.0))),
        bits(Vec3::ZERO)
    );
}

#[test]
fn wake_field_is_deterministic_across_runs() {
    let mut a = world_with_aero();
    let mut b = world_with_aero();
    let src = WakeSource::new(Vec3::new(1.0, 0.0, 0.0), 5.0, 2.0, 0.7);
    a.set_aero_wakes(vec![src]);
    b.set_aero_wakes(vec![src]);
    for x in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.5, 0.7, -0.3),
        Vec3::new(4.9, 1.9, 1.9),
    ] {
        assert_eq!(
            bits(a.aero_wind_at(x)),
            bits(b.aero_wind_at(x)),
            "同输入必须逐位一致"
        );
    }
}
