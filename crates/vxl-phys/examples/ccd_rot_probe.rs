//! **旋转扫掠探针**（2026-10-13，配合 `OPEN-PROBLEMS.md` P18）：2 m 细杆（`half = (1, 0.02, 0.02)`）
//! 无重力、质心静止、绕 y 轴自旋 40 rad/s ⇒ **尖端 40 m/s，旧口径（只看 `|v|`）根本不标记它**。
//! 三格障碍各跑「关档（= 旧行为）/ 开档（旋转扫掠）」两遍，打印：
//! 峰值**埋深**（杆材料点落进障碍 AABB 的最深值）、`min|φ|`（杆线朝向角到障碍角位的最小夹角，
//! **仅杆心未漂移时有意义**）、峰值 `|ω|`、末态 `|ω|`、**杆心最大漂移**。
//!
//! 判据只钉**小柱**那格（`crates/vxl-phys/tests/ccd_rotational_sweep.rs`）；另外两格留档：
//! - **正面薄墙**（刀锋沿自己的轴向"扎"进去）：关/开差异很小（埋深 0.0436 / 0.0480）——那格几何上
//!   杆尖必然穿过墙的平面，离散管线本来就接得住，**别拿它当判据**；
//! - **立着的薄板**（刀锋与其平行）与**小柱**同族，但小柱的角窗口更窄（±3.27°）、信号更干净。
//!
//! `substeps = 1` 是**有意**选的：一个 tick 就是一个积分步，亚步内埋深才能在 tick 末端看见
//! （默认 2 个子步会被第二个子步修掉大半）。
//!
//! 用法：`cargo run --release -p vxl-phys --example ccd_rot_probe`

use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

/// 子步数（见文件头注）。
const SUBSTEPS: u32 = 1;
/// 杆自旋角速度（绕 y）。
const OMEGA: f32 = 40.0;
/// 杆半尺寸（长 2 m、截面 4×4 cm）。
const ROD_HALF: Vec3 = Vec3::new(1.0, 0.02, 0.02);
/// 跑多少个 tick。
const TICKS: u32 = 240;

/// 障碍：`(半尺寸, 中心)`（轴对齐盒）。
type Obst = (Vec3, Vec3);

/// 杆材料采样点里**最深的埋入障碍**的深度（点全在外面 ⇒ 0）。沿杆轴 41 点 × 截面 4 角。
fn burial(w: &World, rod: usize, obs: Obst) -> f32 {
    let (p, q) = (w.bodies.position[rod], w.bodies.rot(rod));
    let ax = q.rotate_vec3(Vec3::X);
    let (half_o, c) = obs;
    let mut deepest = 0.0f32;
    for k in 0..=40 {
        let s = -1.0 + 2.0 * (k as f32) / 40.0;
        for t in [-ROD_HALF.y, ROD_HALF.y] {
            for u in [-ROD_HALF.z, ROD_HALF.z] {
                let d = (p + ax * s + q.rotate_vec3(Vec3::new(0.0, t, u))) - c;
                if d.x.abs() <= half_o.x && d.y.abs() <= half_o.y && d.z.abs() <= half_o.z {
                    deepest = deepest.max(
                        (half_o.x - d.x.abs())
                            .min(half_o.y - d.y.abs())
                            .min(half_o.z - d.z.abs()),
                    );
                }
            }
        }
    }
    deepest
}

/// 杆的**线**朝向角（mod 180，取 `[-90, 90)`）；`0` = 刀锋指向 `+x`。
fn phi_deg(w: &World, rod: usize) -> f32 {
    let ax = w.bodies.rot(rod).rotate_vec3(Vec3::X);
    let mut d = ax.z.atan2(ax.x).to_degrees();
    while d >= 90.0 {
        d -= 180.0;
    }
    while d < -90.0 {
        d += 180.0;
    }
    d
}

/// 一格障碍 × 一档 CCD：建场、跑 `TICKS`、打印一行读数。
fn probe(name: &str, obs: Obst, ccd_on: bool) {
    let cfg = PhysConfig {
        ccd_speed_threshold: if ccd_on { 10.0 } else { f32::INFINITY },
        gravity: Vec3::ZERO,
        substeps: SUBSTEPS,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(Shape::Box { half: obs.0 }, obs.1, Quat::IDENTITY);
    let rod = w.add_dynamic(
        Shape::Box { half: ROD_HALF },
        Vec3::ZERO,
        Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2),
        1000.0,
    ) as usize;
    w.bodies.set_angvel(rod, Vec3::new(0.0, OMEGA, 0.0));

    let (mut deep, mut min_phi, mut wmax, mut drift) = (
        burial(&w, rod, obs),
        phi_deg(&w, rod).abs(),
        OMEGA,
        w.bodies.position[rod].length(),
    );
    for _ in 0..TICKS {
        w.step();
        deep = deep.max(burial(&w, rod, obs));
        min_phi = min_phi.min(phi_deg(&w, rod).abs());
        wmax = wmax.max(w.bodies.angvel(rod).length());
        drift = drift.max(w.bodies.position[rod].length());
    }
    println!(
        "{name:<10} ccd={:<5} 峰值埋深={deep:.4} min|phi|={min_phi:7.3}° 峰值|w|={wmax:6.1} 末|w|={:6.2} 心漂={drift:8.4}",
        if ccd_on { "开" } else { "关" },
        w.bodies.angvel(rod).length()
    );
}

fn main() {
    let cases: [(&str, Obst); 3] = [
        // 正面薄墙（法线沿 x，刀锋沿自身轴向扎进去）
        (
            "正面薄墙",
            (Vec3::new(0.02, 2.0, 2.0), Vec3::new(0.9, 0.0, 0.0)),
        ),
        // 立着的薄板（刀锋与其平行，靠切向撞上去）
        (
            "立面薄板",
            (Vec3::new(0.2, 2.0, 0.02), Vec3::new(0.7, 0.0, 0.0)),
        ),
        // 小柱（判据用的那格：角窗口 ±3.27°）
        (
            "小柱",
            (Vec3::new(0.02, 1.0, 0.02), Vec3::new(0.7, 0.0, 0.0)),
        ),
    ];
    for (name, obs) in cases {
        for ccd_on in [false, true] {
            probe(name, obs, ccd_on);
        }
    }
}
