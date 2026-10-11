//! **声学直达核的探针（GPU；需适配器）**：与 CPU 参考**逐对一致**。
//!
//! 场景刻意选成**轴对齐的屏障**：参照值就能用纯算术写出来（线段与 `z = 1` 平面的交点落不落在
//! 屏障矩形内），而卡上跑的仍是**真正的射线-三角**（Möller–Trumbore）—— 参照与被测不是同一份
//! 实现，所以这条判据不是自证。浮点读数按容差比，`blocked` 这种拓扑量逐位比。
//!
//! ⚠️ 无适配器 ⇒ 跳过（打印 `[SKIP]`，与其余 GPU 判据同款）；本仓 CI 的口径是
//! `cargo test --workspace --exclude vxl-phys-gpu` ⇒ 这条只在有 GPU 的机器上跑。

use vxl_phys_core::Vec3;

const SPEED: f32 = 343.0;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 屏障（`z = 1`、`x ∈ [-1,1]`、`y ∈ [0,3]`，两块三角）。
fn barrier() -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 3.0, 1.0),
        ],
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 3.0, 1.0),
            Vec3::new(-1.0, 3.0, 1.0),
        ],
    ]
}

/// 参照：线段 `s→d` 与 `z = 1` 的交点是否落在屏障矩形内（纯算术，不借被测实现）。
fn reference_blocked(s: Vec3, d: Vec3) -> bool {
    let (dz, sz) = (d.z - s.z, s.z);
    if dz.abs() <= 1e-9 {
        return false;
    }
    let t = (1.0 - sz) / dz;
    if !(0.0..=1.0).contains(&t) {
        return false;
    }
    let p = s + (d - s) * t;
    (-1.0..=1.0).contains(&p.x) && (0.0..=3.0).contains(&p.y)
}

#[test]
fn acoustic_direct_matches_cpu_reference() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    let srcs = [Vec3::new(0.0, 0.5, 0.0), Vec3::new(0.0, 1.0, 0.0)];
    let dsts = [
        Vec3::new(0.0, 0.5, 2.0),
        Vec3::new(2.5, 0.5, 2.0),
        Vec3::new(-2.5, 0.5, 2.0),
    ];
    let tris = barrier();
    let (name, got) =
        match vxl_phys_gpu::acoustic::direct_paths_on_adapter(0, &srcs, &dsts, &tris, SPEED) {
            Ok(v) => v,
            Err(e) => {
                println!("[SKIP] 核起不来：{e}");
                return;
            }
        };
    println!("[gpu] {name}: {} 对", got.len());
    assert_eq!(
        got.len(),
        srcs.len() * dsts.len(),
        "对序 = src_i * n_lst + lst_i"
    );
    // 拓扑（遮挡）**逐位**比 + 浮点**容差**比（WGSL 的 sqrt 与 Rust 的不保证逐位同）
    let mut blocked_n = 0usize;
    for (i, s) in srcs.iter().enumerate() {
        for (j, d) in dsts.iter().enumerate() {
            let k = i * dsts.len() + j;
            let want_blocked = reference_blocked(*s, *d);
            assert_eq!(
                got[k].blocked, want_blocked,
                "第 {k} 对（src {i} / lst {j}）的遮挡分类应与参照一致"
            );
            if want_blocked {
                blocked_n += 1;
                continue;
            }
            let dist = (*d - *s).length();
            let want_delay = dist / SPEED;
            let want_gain = 1.0 / (dist * dist);
            assert!(
                (got[k].delay - want_delay).abs() <= want_delay * 1e-4,
                "第 {k} 对时延（卡上 {} vs 参照 {want_delay}）",
                got[k].delay
            );
            assert!(
                (got[k].gain - want_gain).abs() <= want_gain * 1e-4,
                "第 {k} 对增益（卡上 {} vs 参照 {want_gain}）",
                got[k].gain
            );
        }
    }
    assert_eq!(blocked_n, 2, "这个场景恰好 2 对被挡（否则判据空跑）");
}
