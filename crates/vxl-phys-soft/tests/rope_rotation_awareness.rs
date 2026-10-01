//! **转动感知判据（计划 2c-1，§8.4.29）**：给定**预置角速度**的盒子，绳的接触几何必须**跟着转动走**。
//!
//! 判据两条，互为对照：
//! ① **转的盒子与不转的盒子轨迹必须不同** —— 若几何仍按"冻结朝向"算（2c-1 之前的行为），
//!    转动对接触**完全隐形** ⇒ 两条轨迹会相同 ⇒ 本条红；
//! ② 转的那条**钉住末态读数**（防回归：几何口径改坏了它就会变）。
//!
//! 为什么用"预置角速度"而不是"让接触把体转起来"：后者的**角反作用当前是停用的**（§8.4.10/§8.4.28），
//! 属 2c-3；本判据只验**"看得见转动"**这一步（体自转由外部给定 ⇒ 可复现、机器无关）。
use vxl_phys_core::{interop::NoProviders, Quat, Shape, Vec3};
use vxl_phys_soft::{RigidProxy, Rope};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const HALF: Vec3 = Vec3::new(0.3, 0.05, 0.3);
const TICKS: usize = 900;

/// 3 维自扮引擎（§8.4.16）+ **预置角速度** `omega`（绕 z 自转，恒定 —— 由外部给定，不是接触算出来的）。
fn run(omega: f32) -> (f32, f32, f32, u64) {
    // (末 y, 末 x, |x|max, 轨迹哈希)
    let mut r = Rope::line(
        Vec3::new(-0.5, 1.0, 0.0),
        Vec3::new(0.5, 1.0, 0.0),
        33,
        0.02,
    );
    r.damping = 0.999;
    for _ in 0..600 {
        r.step(DT, G, &NoProviders, 0, &[]);
    }
    let shape = Shape::Box { half: HALF };
    let (mut pos, mut vel) = (Vec3::new(0.0, 1.2, 0.0), Vec3::ZERO);
    let (mut x_max, mut h) = (0.0f32, 0xcbf2_9ce4_8422_2325u64);
    for _ in 0..TICKS {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: Quat::IDENTITY,
            linvel: vel,
            angvel: Vec3::new(0.0, 0.0, omega),
            local_inv_inertia: Vec3::ZERO, // 静态/未用（角反作用开关默认关）
            inv_mass: 1.0,
        };
        r.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        vel += G * DT;
        pos += vel * DT;
        if let Some(dv) = r.body_dv.first() {
            vel += *dv;
        }
        if let Some(dx) = r.body_dx.first() {
            pos += *dx;
        }
        x_max = x_max.max(pos.x.abs());
        for v in [pos.y, pos.x] {
            for b in v.to_bits().to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    (pos.y, pos.x, x_max, h)
}

#[test]
fn rope_contact_follows_an_imposed_spin() {
    let (y0, x0, _, h0) = run(0.0);
    let (ys, xs, x_max_s, hs) = run(3.0);
    println!(
        "ω=0 ：末 y={y0:+.5} x={x0:+.5} 哈希={h0:016x}\n\
         ω=3 ：末 y={ys:+.2} x={xs:+.2} |x|max={x_max_s:.4} 哈希={hs:016x}"
    );
    assert_ne!(
        h0, hs,
        "转的盒子与不转的盒子轨迹**必须不同**（两条哈希相同 = 接触几何仍按**冻结朝向**算、转动对它隐形）"
    );
    assert!(
        (ys - y0).abs() > 1e-4 || (xs - x0).abs() > 1e-3,
        "差异该是**实质**的（末态 y {ys:+.2} vs {y0:+.5}、x {xs:+.2} vs {x0:+.5}）——\
         只差末位说明转动项只渗进了舍入而不是几何"
    );
    // **② 钉住读数 + 说清这个场景的含义**：`ω` 是**外部强加**的（没有任何东西能吸收它——角反作用
    // 仍停用，属 2c-3），所以盒子会被"持续注入的能量"甩出去（物理上可解：不是"托住"测试）。
    // 因此这里钉的是**行为指纹**而不是"托住"：甩出去（`y` 深负）、横向被推到米级、
    // 且**轨迹哈希固定**（口径改了它就变 ⇒ 防回归）。
    assert!(
        ys < -100.0 && x_max_s > 0.5,
        "强加自转（无角反作用可吸收）该表现为**被甩出去**（末 y 深负、横向米级）：实测 y={ys:+.2} |x|max={x_max_s:.4}"
    );
    // **2026-10-01 重登记（§8.4.50：位置腿进 `body_disp`）**：本场景有动态体 ⇒ 位置腿在该
    // 路径上非零 ⇒ **两支轨迹都动**（ω=0：y 0.98113→0.96934、x 0.01284→0.01921、哈希
    // 4d5b6d9b…→df4decb3…；ω=3：y −692.05→−531.50、|x|max 2.1226→2.0101、哈希 6715e1be…→681d9ad4…）。
    // 行为语义不变（强加自转、无角反作用可吸收 ⇒ 被甩出去），上方两条断言继续承重。
    assert_eq!(
        hs, 0x681d_9ad4_957c_b6a0,
        "这条轨迹是**冻结基线**（转动/摩擦口径一改它就变）：改了 `angvel` 的用法或接触点速度口径，\
         必须在此重新登记——读数 y={ys:+.2} x={xs:+.2} |x|max={x_max_s:.4}"
    );
}
