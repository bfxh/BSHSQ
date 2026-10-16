//! 取数件：`quiet` / `quiet_shaped` / `run_yaw` —— 各量一个场景，返回标量读数 —— 同上拆出。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

use crate::scene::{tank, water};
use crate::stats::yaw_of;

pub(crate) fn quiet(couple: bool, layers: u32) -> (f32, f32, f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water(layers);
    if couple {
        w.add_fluid_with_boundary_coupling(sys, &[v]);
    } else {
        w.add_fluid(sys, &[v]);
    }
    // 轻盒从水面上方落入（干净开局：不落在已有水格上）。
    let half = 0.06f32;
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(half),
        },
        Vec3::new(0.0, 1.50, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    for _ in 0..300 {
        w.step(); // 落水 + 漂稳
    }
    let win = 180usize;
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut vlo, mut vhi) = (f32::MAX, f32::MIN);
    let (mut wlo, mut whi) = (f32::MAX, f32::MIN);
    let (mut ysum, mut fsum) = (0.0f64, 0.0f64);
    let (mut flo, mut fhi) = (f32::MAX, f32::MIN);
    let expect = 1000.0 * (2.0 * half).powi(3) * 9.81;
    for _ in 0..win {
        w.step();
        let p = w.bodies.position[b as usize];
        let lv = w.bodies.linvel[b as usize].length();
        let av = w.bodies.angvel(b as usize).length();
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        vlo = vlo.min(lv);
        vhi = vhi.max(lv);
        wlo = wlo.min(av);
        whi = whi.max(av);
        ysum += p.y as f64;
        // 反作用：2b 档读流体侧；2a 档没有反作用（读体受力替代不了）⇒ 只 2b 档有意义。
        if let Some(r) = w.fluids()[0]
            .0
            .boundary_reactions()
            .iter()
            .find(|r| r.0 == b)
        {
            fsum += r.1.y as f64;
            flo = flo.min(r.1.y);
            fhi = fhi.max(r.1.y);
        }
    }
    let n = win as f64;
    let fpp = if couple { (fhi - flo) / expect } else { 0.0 };
    let _ = fsum / n;
    (yhi - ylo, vhi - vlo, whi - wlo, (ysum / n) as f32, fpp)
}

/// 同 `quiet`，但可传体形/姿态，并额外返回（|τ| 窗口均值、累计偏航）。
pub(crate) fn quiet_shaped(couple: bool, shape: Shape, rot: Quat) -> (f32, f32, f32, f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    let sys = water(2);
    if couple {
        w.add_fluid_with_boundary_coupling(sys, &[v]);
    } else {
        w.add_fluid(sys, &[v]);
    }
    let b = w.add_dynamic(shape, Vec3::new(0.0, 1.50, 0.0), rot, 300.0);
    for _ in 0..300 {
        w.step(); // 落水 + 漂稳
    }
    let mut prev_yaw = 0.0f32;
    let mut unwrapped = 0.0f32;
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut vlo, mut vhi) = (f32::MAX, f32::MIN);
    let (mut wlo, mut whi) = (f32::MAX, f32::MIN);
    let mut tsum = 0.0f64;
    for _ in 0..180 {
        w.step();
        let p = w.bodies.position[b as usize];
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        let lv = w.bodies.linvel[b as usize].length();
        vlo = vlo.min(lv);
        vhi = vhi.max(lv);
        let av = w.bodies.angvel(b as usize).length();
        wlo = wlo.min(av);
        whi = whi.max(av);
        if let Some(r) = w.fluids()[0]
            .0
            .boundary_reactions()
            .iter()
            .find(|r| r.0 == b)
        {
            tsum += r.2.length() as f64;
        }
        // 累计偏航（unwrap：把每步 Δyaw 折到 (-π, π] 再累加）
        let yaw = yaw_of(w.bodies.rot(b as usize));
        let mut d = yaw - prev_yaw;
        while d > std::f32::consts::PI {
            d -= 2.0 * std::f32::consts::PI;
        }
        while d < -std::f32::consts::PI {
            d += 2.0 * std::f32::consts::PI;
        }
        unwrapped += d;
        prev_yaw = yaw;
    }
    (
        yhi - ylo,
        vhi - vlo,
        whi - wlo,
        (tsum / 180.0) as f32,
        unwrapped,
    )
}

/// 姿态的偏航角（绕 Y；用旋转矩阵的 x 轴投影求）。

pub(crate) fn run_yaw(w: &mut World, body: usize, settle: usize, win: usize) -> (f32, f32) {
    for _ in 0..settle {
        w.step();
    }
    let mut prev = yaw_of(w.bodies.rot(body));
    let mut unwrapped = 0.0f32;
    let mut wpeak = 0.0f32;
    for _ in 0..win {
        w.step();
        let y = yaw_of(w.bodies.rot(body));
        let mut d = y - prev;
        while d > std::f32::consts::PI {
            d -= 2.0 * std::f32::consts::PI;
        }
        while d < -std::f32::consts::PI {
            d += 2.0 * std::f32::consts::PI;
        }
        unwrapped += d;
        prev = y;
        wpeak = wpeak.max(w.bodies.angvel(body).length());
    }
    (unwrapped, wpeak)
}
