//! **高斯粒子域的门面场景**（②物理代理接入，2026-10-08）—— `tests` 的子模块。
//!
//! 走 `World::step()` 而不是直接调 crate API；读回用 `w.providers.splat(id)`（**门面没有**公开的
//! 喷溅场读入口 —— 加方法会涨 god 门债务，所以本文件是 crate 内测试）。
//!
//! 判据：
//! ① **落体静置**：开档的单核场落到**真体素地板**上，静置高度 = 沿法线的**椭球支撑半径**（0.2）、
//!    不穿透、速度收敛、水平不漂；
//! ② **关档金丝雀**：`dynamics = None` ⇒ 核**逐位不动**（默认档不被改动）；
//! ③ **确定性**：开档场景两跑逐位一致。
use crate::{PhysConfig, Vec3, World};
use vxl_phys_splat::dynamics::SplatDynamics;
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 体素地板：10×10×1 格、格边 0.5 ⇒ 顶面 `y = 0`。
fn floor(w: &mut World) {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-2.5, -0.5, -2.5), 0.5, 10, 1, 10);
    for ix in 0..10u32 {
        for iz in 0..10u32 {
            vol.set(ix, 0, iz, true);
        }
    }
    w.add_voxel(vol);
}

/// 单核场（半径 0.2、介质密度 1000 ⇒ 质量口径与自密度都有定义）。
fn kernel(y: f32) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.medium_density = 1000.0;
    f.push(Splat::isotropic(Vec3::new(0.0, y, 0.0), 0.2, 1.0));
    f
}

fn scene(dynamics: bool) -> (World, u32) {
    let mut w = World::new(PhysConfig::default());
    floor(&mut w);
    let mut f = kernel(1.0);
    if dynamics {
        f.set_dynamics(Some(SplatDynamics::default()));
    }
    let id = w.add_splat_field(f);
    (w, id)
}

/// 读回 `(中心, 速度)`；场不存在时给零（不该发生，判据会因此红）。
fn state(w: &World, id: u32) -> (Vec3, Vec3) {
    match w.providers.splat(id) {
        Some(f) => {
            let v = f.kernel_velocities();
            (
                f.splats()[0].center,
                if v.is_empty() { Vec3::ZERO } else { v[0] },
            )
        }
        None => (Vec3::ZERO, Vec3::ZERO),
    }
}

#[test]
fn kernel_rests_on_the_voxel_floor() {
    let (mut w, id) = scene(true);
    for _ in 0..240 {
        w.step();
    }
    let (c, v) = state(&w, id);
    assert!(
        (c.y - 0.2).abs() < 0.03,
        "静置高度应 = 支撑半径 0.2（容差 0.03）：y={}",
        c.y
    );
    assert!(c.y > 0.05, "不许穿地：y={}", c.y);
    assert!(v.length() < 0.05, "应已收敛：|v|={}", v.length());
    assert!(
        c.x.abs() < 1e-6 && c.z.abs() < 1e-6,
        "水平向不该漂移：{c:?}"
    );
}

#[test]
fn dynamics_off_is_bitwise_still() {
    let (mut w, id) = scene(false);
    let (c0, _) = state(&w, id);
    for _ in 0..240 {
        w.step();
    }
    let (c1, v1) = state(&w, id);
    assert_eq!(c1, c0, "关档 ⇒ 核逐位不动");
    assert_eq!(v1, Vec3::ZERO, "关档 ⇒ 不建速度槽/不加速");
}

#[test]
fn splat_dynamics_scene_is_deterministic() {
    let run = || {
        let (mut w, id) = scene(true);
        for _ in 0..120 {
            w.step();
        }
        let (c, v) = state(&w, id);
        (c.x.to_bits(), c.y.to_bits(), c.z.to_bits(), v.y.to_bits())
    };
    assert_eq!(run(), run(), "同场景两跑逐位一致");
}
