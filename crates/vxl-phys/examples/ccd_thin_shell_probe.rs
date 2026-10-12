//! **薄壳 TOI 探针**（2026-10-13，配合 SURVEY-RIGID-HARD-CASES §7.2③）：薄壁（厚度 < 皮肤带 0.02）
//! 与三角网薄壳 × 高速盒，量三件事：
//! ① **贴合精度**：停住后「体前缘 → 壁面」的间距（现行实现是"钳到上一安全采样点"⇒ 间距可以
//!    有一整段采样步长那么长：盒 0.5 ⇒ 段长 0.25 ⇒ 停在墙前 0.25 m 且法向速度清零 = 悬空）；
//! ② **最坏相位**：起始偏移扫过一整段采样步长（16 相位），看有没有相位会穿；
//! ③ **薄壳招牌面**：5 mm 立板（比皮肤带还薄）+ 单页三角网（薄壳语义）两档都要挡住。
//!
//! 跑法：`cargo run --release -q -p vxl-phys --example ccd_thin_shell_probe`

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 盒半长（最小半长 = 0.5 ⇒ 段长上限 0.25）。
const HALF: f32 = 0.5;
/// 壁/板半厚（总厚 5 mm < 皮肤带 0.02）。
const TH: f32 = 0.0025;
/// 跑多少 tick。
const TICKS: u32 = 400;

/// 目标几何：`Box` = 立着的薄板 / `Mesh` = 单页三角网（薄壳语义）。
enum Target {
    Box,
    Mesh,
}

/// 建场景：`v` = 体初速（+x 撞壁），`d0` = 体心起始 x。返回 `(world, 体号)`。
fn scene(kind: &Target, v: f32, d0: f32) -> (World, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0, // 放开上限，让预算闸自己咬
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    match kind {
        Target::Box => {
            w.add_static(
                Shape::Box {
                    half: Vec3::new(TH, 2.0, 2.0),
                },
                Vec3::ZERO,
                Quat::IDENTITY,
            );
        }
        Target::Mesh => {
            let pts = vec![
                Vec3::new(0.0, -2.0, -2.0),
                Vec3::new(0.0, 2.0, -2.0),
                Vec3::new(0.0, 2.0, 2.0),
                Vec3::new(0.0, -2.0, 2.0),
            ];
            let tris = vec![[0u32, 1, 2], [0, 2, 3]];
            w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(pts, tris));
        }
    }
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(d0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(v, 0.0, 0.0);
    (w, b)
}

/// 跑一遍，返回 `(末态间距, 全程最深嵌入, 末态|v|, 末态是否清醒, 末 tick 号)`。
/// 「间距」= 体前缘到**壁近面** `x = −TH` 的距离（正 = 还没碰到；负 = 嵌进去多少）。
fn run(kind: &Target, v: f32, d0: f32) -> (f32, f32, f32, bool, u32) {
    let (mut w, b) = scene(kind, v, d0);
    let mut deepest = 0.0f32; // 最大嵌入深度（正 = 前缘越过近面多少）
    let mut last_move_tick = 0;
    let mut prev = w.bodies.position[b].x;
    for k in 0..TICKS {
        w.step();
        // 前缘（+x 面）到壁近面（x = −TH）的距离：正 = 还没碰上；负 = 嵌进去多少。
        let gap = -(w.bodies.position[b].x + HALF + TH);
        deepest = deepest.max(-gap);
        let moved = (w.bodies.position[b].x - prev).abs();
        if moved > 1e-4 {
            last_move_tick = k + 1;
        }
        prev = w.bodies.position[b].x;
    }
    (
        -(w.bodies.position[b].x + HALF + TH),
        deepest,
        w.bodies.linvel[b].length(),
        w.bodies.awake[b],
        last_move_tick,
    )
}

fn main() {
    if std::env::var_os("TRACE").is_some() {
        let (mut w, b) = scene(&Target::Box, 60.0, -5.0);
        for k in 0..24 {
            w.step();
            println!(
                "k={k:3} x={:9.4} v={:8.3} w={:7.3} 清醒={}",
                w.bodies.position[b].x,
                w.bodies.linvel[b].x,
                w.bodies.angvel(b).length(),
                w.bodies.awake[b]
            );
        }
        return;
    }
    // ① 单档：60 m/s 撞 5 mm 立板与单页三角网
    for kind in [Target::Box, Target::Mesh] {
        let name = if matches!(kind, Target::Box) {
            "5mm立板"
        } else {
            "单页三角网"
        };
        for v in [60.0f32, 3000.0] {
            let (gap, pen, sp, awake, last) = run(&kind, v, -5.0);
            println!(
                "{name}  v={v:7.0}  末态间距={gap:8.4}  最深嵌入={pen:8.4}  末|v|={sp:8.3}  清醒={awake}  末次移动@{last}"
            );
        }
    }
    // ② 最坏相位：起始偏移扫 16 个相位（覆盖一整段采样步长 0.25）
    for kind in [Target::Box, Target::Mesh] {
        let name = if matches!(kind, Target::Box) {
            "5mm立板"
        } else {
            "单页三角网"
        };
        let (mut min_gap, mut max_pen) = (f32::INFINITY, 0.0f32);
        for k in 0..16 {
            let d0 = -5.0 + (k as f32) * 0.25 / 16.0;
            let (gap, pen, _sp, _aw, _last) = run(&kind, 60.0, d0);
            min_gap = min_gap.min(gap);
            max_pen = max_pen.max(pen);
        }
        println!("{name}  相位扫描(16): 最小末态间距={min_gap:8.4}  最深嵌入={max_pen:8.4}");
    }
}
