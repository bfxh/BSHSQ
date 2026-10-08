//! **高斯粒子域的运动步判据**（②物理代理第一片，2026-10-08）。
//!
//! 判据链（**碰撞的落地判据在门面侧**：`crates/vxl-phys/tests/splat_dynamics_scene.rs` 用真体素
//! 地板；本文件只测不依赖世界几何的部分）：
//! ① **自由落体解析对拍**：`ids` 空 ⇒ 纯弹道，半隐式欧拉的**离散闭式** `Δy = −g·dt²·N(N+1)/2`；
//! ② **支撑半径口径**：沿各轴 = 对应半轴、45° = `√((σx²+σy²)/2)`、`max_radius` = 最大半轴；
//! ③ **确定性**：同场景两跑逐位一致。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::particles::{step_particles, ParticleStep};
use vxl_phys_splat::{GaussianSplatField, Splat};

fn one(scale: Vec3, y: f32) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.push(Splat {
        center: Vec3::new(0.0, y, 0.0),
        scale,
        rot: Mat3::IDENTITY,
        opacity: 1.0,
        color: [1.0, 1.0, 1.0],
    });
    f
}

#[test]
fn free_fall_matches_the_discrete_closed_form() {
    let mut f = one(Vec3::splat(0.2), 3.0);
    let (dt, n, g) = (1.0f32 / 60.0, 60usize, 9.81f32);
    let y0 = f.splats()[0].center.y;
    let mut hits = 0usize;
    for _ in 0..n {
        hits += step_particles(&mut f, dt, &NoProviders, &[], ParticleStep::default());
    }
    assert_eq!(hits, 0, "空 ids = 纯弹道，不该有接触");
    // 半隐式欧拉：v_k = k·g·dt、y_N = y0 − g·dt²·Σ_{k=1..N}k = y0 − g·dt²·N(N+1)/2
    let want_dy = -g * dt * dt * (n as f32 * (n as f32 + 1.0) / 2.0);
    let got_dy = f.splats()[0].center.y - y0;
    let rel = ((got_dy - want_dy) / want_dy).abs();
    assert!(rel < 1e-5, "自由落体应合离散闭式：rel={rel:e}");
    let want_v = -g * dt * n as f32;
    let got_v = f.kernel_velocities()[0].y;
    assert!(
        (got_v - want_v).abs() / want_v.abs() < 1e-5,
        "末速 ≈ −g·N·dt"
    );
    assert!(f.splats()[0].center.y < 1.0, "用例非平凡：确实落下来了");
}

#[test]
fn support_radius_follows_the_axes() {
    let s = Splat {
        center: Vec3::ZERO,
        scale: Vec3::new(0.1, 0.3, 0.5),
        rot: Mat3::IDENTITY,
        opacity: 1.0,
        color: [0.0; 3],
    };
    assert_eq!(s.radius_along(Vec3::X), 0.1);
    assert_eq!(s.radius_along(Vec3::Y), 0.3);
    assert_eq!(s.radius_along(Vec3::Z), 0.5);
    // 轴对齐椭球：r(n) = ‖(σx·nₓ, σy·n_y, σz·n_z)‖ ⇒ 45° 方向 = √((0.1² + 0.3²)/2)
    let diag = s.radius_along(Vec3::new(1.0, 1.0, 0.0).normalize());
    let want = ((0.1f32 * 0.1 + 0.3 * 0.3) / 2.0).sqrt();
    assert!(
        (diag - want).abs() < 1e-6,
        "斜向支撑半径应合解析值 {want}：r={diag}"
    );
    assert!(diag > 0.1 && diag < 0.3, "斜向应落在两半轴之间：r={diag}");
    assert_eq!(s.max_radius(), 0.5);
}

#[test]
fn particle_step_is_deterministic() {
    let run = || {
        let mut f = one(Vec3::new(0.12, 0.18, 0.12), 2.0);
        for _ in 0..180 {
            step_particles(
                &mut f,
                1.0 / 60.0,
                &NoProviders,
                &[],
                ParticleStep::default(),
            );
        }
        (f.splats()[0].center, f.kernel_velocities()[0])
    };
    let (c1, v1) = run();
    let (c2, v2) = run();
    assert_eq!(c1, c2, "同场景两跑位置逐位一致");
    assert_eq!(v1, v2, "同场景两跑速度逐位一致");
}
