//! **判据 ④（T4 收尾）：布片 × 动态刚体双向** —— **门面级**（`PLAN-triangle-first-class.md` 判据 ④）。
//!
//! 软体侧自扮引擎的判据在 `crates/vxl-phys-soft/tests/cloth_reaction.rs`（两腿的**机制**在那里量）；
//! 本条守的是**引擎里那份接线真的生效**：`cloth_pass` → `apply_two_leg_reactions`
//! （`linvel += dv` / `position += dx`）+ `rebuild_soft_proxies` 的时序 + 睡眠体对软体域呈现静态。
//!
//! **场景**：1.0 m 见方布片（8×8 格、**四周钉住**）先在门面里垂稳，再落一枚 **1 kg** 动态盒
//! ⇒ 长窗该**停在布片上**。**对照**：自由落体 1800 tick 该掉 **4415 m**。
//!
//! **为什么单独一个文件**（而不是塞进 `cloth_scene.rs`）：god 门对**既有**文件管"只准减"
//! （`cloth_scene.rs` 76 → 138 行 + 最长函数 45 → 52 会双双红）⇒ 按本仓"一片一文件"的惯例
//! （`cloth_{minimal,contact,body,bending,reaction,self_collision}.rs`）另立一个。

use vxl_phys::*;
use vxl_phys_core::{PhysConfig, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 `y = 0` 平面；行主序 = `z` 外层、`x` 内层）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
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

#[test]
fn box_is_held_by_a_pinned_cloth_in_a_world() {
    let mut w = World::new(PhysConfig::default());
    let (pts, tris) = plate(8, 0.5);
    let mut sheet = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    sheet.damping = 0.999;
    let side = 9usize; // n + 1
    for iz in 0..side {
        for ix in 0..side {
            if iz == 0 || ix == 0 || iz == side - 1 || ix == side - 1 {
                sheet.set_pinned(iz * side + ix, true); // 四周一圈钉住
            }
        }
    }
    assert_eq!(w.add_cloth(sheet), 0, "第一张布片的索引应为 0");
    for _ in 0..600 {
        w.step(); // 先垂稳：读数取**垂稳后**的形态（别把初态下坠混进来）
    }
    let y_rest = w.cloth(0).expect("just added").pos[4 * side + 4].y;
    // ⚠️ 第 4 参是**密度**（`mass_props(&shape, density)`），不是质量！
    // 盒 0.4×0.1×0.4 = 0.016 m³ ⇒ 要 1.0 kg 就得 62.5 kg/m³（与软体侧判据**同质量**）。
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::new(0.2, 0.05, 0.2),
        },
        Vec3::new(0.0, y_rest + 0.3, 0.0),
        Quat::IDENTITY,
        1.0 / 0.016,
    ) as usize;
    for _ in 0..1800 {
        w.step();
    }
    let (p, v) = (w.bodies.position[b], w.bodies.linvel[b]);
    let free_fall = 0.5 * 9.81 * (1800.0 / 60.0f32).powi(2);
    println!(
        "门面级 1800 tick：盒 y={:+.4}（布片垂稳 {:+.4}）| x={:+.4} | vy={:+.4}\n\
         自由落体对照：同样 1800 tick 该掉 {:.0} m",
        p.y, y_rest, p.x, v.y, free_fall
    );
    assert!(p.is_finite() && v.is_finite(), "出现非有限值（NaN/inf）");
    assert!(
        p.y > y_rest - 0.3,
        "长窗该**停在布片上**（实测 y={:+.4} vs 布片垂稳 {:+.4}）——红了说明门面的反作用两腿\
         没接上（`apply_two_leg_reactions` 的口径或时序被改坏；机制判据见 `cloth_reaction.rs`）",
        p.y,
        y_rest
    );
    assert!(
        p.x.abs() < 0.3,
        "横向该留在自身足迹内（半宽 0.2；实测 x={:+.4}）——红了说明有横向棘轮",
        p.x
    );
}
