//! 窗口统计与读数量（`yaw_of` / `surface_height` / `Stats` / `window_stats`）—— 同上拆出。

use vxl_phys::{PhysConfig, Quat, Vec3, World};

use crate::scene::{tank, water};

pub(crate) fn yaw_of(q: Quat) -> f32 {
    let x = q.rotate_vec3(Vec3::X);
    x.z.atan2(x.x)
}

/// **自旋偏置来源隔离**（仪表，只打印）——上一轮已证"噪声与偏置是两个成分"，本测试只追偏置。
///
/// 三格对拍（同槽同体，窗口 180 tick）：
/// - **干槽**（无流体）：体落在槽底。若它也转 ⇒ 偏置在**刚体接触侧**（排序/法线选择）；
///   （干槽是刚体↔体素提供者接触，与 2b 无关，能一刀切开"流体 vs 刚体"。）
/// - **静态体**（marker，inv_mass=0）+ 流体：体不动 ⇒ 读**反作用力矩均值**——
///   非零即证明"流体对**静止**体也施加系统性转矩"（与体的转动反馈无关）。
/// - **自由体 + 流体**：基准（复现上一轮的 −0.142 rad/3 s）。

pub(crate) fn surface_height(couple: bool) -> f32 {
    let mut w0 = World::new(PhysConfig::default());
    let v0 = tank(&mut w0);
    let s0 = water(2);
    let f0 = if couple {
        w0.add_fluid_with_boundary_coupling(s0, &[v0])
    } else {
        w0.add_fluid(s0, &[v0])
    };
    for _ in 0..300 {
        w0.step();
    }
    w0.fluids()[f0]
        .0
        .positions()
        .iter()
        .filter(|p| p.x.abs() < 0.05 && p.z.abs() < 0.05)
        .map(|p| p.y)
        .fold(0.0f32, f32::max)
}

/// 窗口统计：**体动 vs 局部水动**（窗口 `win` tick，逐步 `w.step()`）。
///
/// 口径：体心 y 起伏、`|v|`/`|ω|` 窗口均值；邻近水 = 水平 `±0.08` m 内的平均/峰 `|v|`
/// 与**水面高度**（该窗口内邻近水最高点）的起伏。两档共用（2b 双向 / 2a 单向）；
/// 2a 侧不打印水面起伏，但统计照算（同一次遍历，不额外扫一遍）。
pub(crate) struct Stats {
    pub(crate) ylo: f32,
    pub(crate) yhi: f32,
    pub(crate) slo: f32,
    pub(crate) shi: f32,
    pub(crate) vsum: f64,
    pub(crate) vmax: f32,
    pub(crate) n: f64,
    pub(crate) bvsum: f64,
    pub(crate) bwsum: f64,
}

pub(crate) fn window_stats(w: &mut World, b: usize, fid: usize, win: usize) -> Stats {
    let (mut ylo, mut yhi) = (f32::MAX, f32::MIN);
    let (mut slo, mut shi) = (f32::MAX, f32::MIN);
    let (mut vsum, mut vmax, mut n) = (0.0f64, 0.0f32, 0.0f64);
    let (mut bvsum, mut bwsum) = (0.0f64, 0.0f64);
    for _ in 0..win {
        w.step();
        let p = w.bodies.position[b];
        ylo = ylo.min(p.y);
        yhi = yhi.max(p.y);
        bvsum += w.bodies.linvel[b].length() as f64;
        bwsum += w.bodies.angvel(b).length() as f64;
        let f = &w.fluids()[fid].0;
        let mut sh = 0.0f32;
        for (q, vv) in f.positions().iter().zip(f.velocities().iter()) {
            if (q.x - p.x).abs() < 0.08 && (q.z - p.z).abs() < 0.08 {
                let sp = vv.length();
                vsum += sp as f64;
                vmax = vmax.max(sp);
                n += 1.0;
                sh = sh.max(q.y);
            }
        }
        slo = slo.min(sh);
        shi = shi.max(sh);
    }
    Stats {
        ylo,
        yhi,
        slo,
        shi,
        vsum,
        vmax,
        n,
        bvsum,
        bwsum,
    }
}
