//! **自由旋转守恒探针**（`SURVEY-RIGID-HARD-CASES §1.2/§1.5` 的仪表）。
//!
//! 缺口：速度积分是 `ω += I⁻¹_world·τ·dt`，**没有陀螺项** `ω × (Iω)` ⇒ 无接触自由旋转的**非球体**
//! 不守恒角动量（`L = I_world·ω` 随体姿态转、而 `ω` 在世界系里恒定）。判据建议（该文件 §1.5）：
//! ① 自由旋转守恒（`|L|` 与动能 `½ωᵀIω` 的相对漂移有界）；② 对称陀螺解析对拍（进动率
//! `Ω = ω₃·(I₃ − I₁)/I₁`）；④ `ω ≡ 0` 金丝雀。
//!
//! 跑法：
//! ```text
//! cargo run -q --release -p vxl-phys --example free_rotation_probe
//! PROBE_TICKS=12000 cargo run -q --release -p vxl-phys --example free_rotation_probe
//! ```
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::Mat3;

fn ticks() -> usize {
    std::env::var("PROBE_TICKS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1200)
}

/// `I_local·ω`（世界系）：本仓的局部惯量是**对角**（`Vec3` 存 `I⁻¹`）⇒ 正向惯量 = 逐分量倒数。
fn momentum(bodies: &vxl_phys_core::BodySet, i: usize) -> (Vec3, Vec3) {
    let inv = bodies.local_inv_inertia[i];
    let r = Mat3::from_quat(bodies.rot(i));
    let w_l = r.transpose_mul_vec3(bodies.angvel(i));
    let l_l = Vec3::new(w_l.x / inv.x, w_l.y / inv.y, w_l.z / inv.z);
    (r.mul_vec3(l_l), w_l) // (L 世界系, ω 局部系)
}

/// 自由旋转一档：`shape` + 初始 `ω`，返回 `(|L| 相对漂移, KE 相对漂移, ω 局部系末态)`。
fn free_spin(name: &str, shape: Shape, w0: Vec3) -> (f32, f32) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let i = w.add_dynamic(shape, Vec3::ZERO, Quat::IDENTITY, 1000.0) as usize;
    w.bodies.linvel.set_ang(i, w0); // 直接写热组，避开 `set_angvel` 的唤醒语义（这里本就醒着）
    let (l0, _) = momentum(&w.bodies, i);
    let ke0 = 0.5 * w.bodies.angvel(i).dot(l0);
    let n = ticks();
    let mut l_rel = 0.0f32;
    let mut ke_rel = 0.0f32;
    let mut w3_min = f32::MAX;
    let mut w3_max = f32::MIN;
    // 局部系横向 ω 的累计转角（对称陀螺的进动率 = 累计角 / 秒）
    let (mut px, mut py) = (1.0f32, 0.0f32);
    let mut phase = 0.0f32;
    for k in 0..n {
        w.step();
        let (l, w_l) = momentum(&w.bodies, i);
        l_rel = l_rel.max((l - l0).length() / l0.length());
        let ke = 0.5 * w.bodies.angvel(i).dot(l);
        ke_rel = ke_rel.max((ke - ke0).abs() / ke0);
        w3_min = w3_min.min(w_l.z);
        w3_max = w3_max.max(w_l.z);
        let (nx, ny) = (w_l.x, w_l.y);
        if nx * nx + ny * ny > 1e-12 {
            phase += (px * ny - py * nx).atan2(px * nx + py * ny);
            (px, py) = (nx, ny);
        }
        if k + 1 == n {
            println!(
                "  [{name}] 末态 ω_local = ({:+.4},{:+.4},{:+.4})｜awake = {}",
                w_l.x, w_l.y, w_l.z, w.bodies.awake[i]
            );
        }
    }
    let secs = n as f32 / 60.0;
    println!(
        "  [{name}] {n} 步内 **L 矢量**最大相对漂移 = {:.6}、动能 = {:.6}；ω₃（局部）摆幅 = {:.4}…{:.4}；横向进动率 = {:+.3} rad/s",
        l_rel,
        ke_rel,
        w3_min,
        w3_max,
        phase / secs
    );
    (l_rel, ke_rel)
}

fn main() {
    println!("自由旋转探针（无重力、无接触、无外力矩）");
    // ① 不对称盒（三主惯量互不相等）+ 非主轴自旋：网球拍定理的那一档
    let w0 = Vec3::new(1.0, 2.0, 0.3).normalize() * 6.0;
    free_spin(
        "不对称盒 2×1×0.5 @ 非主轴 ω=6",
        Shape::Box {
            half: Vec3::new(1.0, 0.5, 0.25),
        },
        w0,
    );
    // ② 对称陀螺（I₁ = I₂ ≠ I₃）：z 轴自旋 + 一点横向分量 ⇒ 解析进动率 Ω = ω₃(I₃ − I₁)/I₁
    free_spin(
        "对称陀螺（方盒 1×1×0.5）@ ω=(1,0,6)",
        Shape::Box {
            half: Vec3::new(0.5, 0.5, 0.25),
        },
        Vec3::new(1.0, 0.0, 6.0),
    );
    // ④ 金丝雀：ω ≡ 0 ⇒ 一个字节都不该动
    free_spin(
        "金丝雀 ω=0",
        Shape::Box {
            half: Vec3::new(1.0, 0.5, 0.25),
        },
        Vec3::ZERO,
    );
    // ⑤ 细杆（极各向异性：长轴惯量比横向小 ~10³）：自旋绕**横向轴** —— 自由进动率
    //    `Ω = ω(I_⊥/I_∥ − 1)` 达到 ω·10³ 量级 ⇒ **远超 dt 能分辨的尺度**（这是"陀螺刚性"的
    //    经典边界：纯绕轴自转是对的，一旦有横向分量，隐式格式也解不动）。
    free_spin(
        "细杆 1×0.02×0.02 @ ω=40（⊥ 轴）",
        Shape::Box {
            half: Vec3::new(0.5, 0.01, 0.01),
        },
        Vec3::new(0.0, 0.0, 40.0),
    );
    // ⑤′ 同一条细杆，但自旋**偏离主轴**（撞墙后的典型姿态）：自由进动率 `ω·(I_⊥/I_∥ − 1)`
    //     ≈ 4·10² rad/s ⇒ **远超 dt 能分辨的尺度** ⇒ 本实现按"可分辨性"退回旧式（如实登记）。
    free_spin(
        "细杆 @ ω=(4,40,0)（偏离主轴 ⇒ 解不动档）",
        Shape::Box {
            half: Vec3::new(0.5, 0.01, 0.01),
        },
        Vec3::new(4.0, 40.0, 0.0),
    );
}
