//! **溅水（刚-液-风）三域贯通场景** —— `ROUTE.md` §4「必须有贯通示例」里的最后一格。
//!
//! **场景**：体素水槽 + 0.5 m 内腔、铸装水块 `[8,8,8]@0.05`、**开 2b 边界粒子耦合**
//! （刚×液那半）；一枚动态球从水面上方落下（推水 = "溅"）；另立一块**静态三角网风帆**
//! 并把 `set_aero` 打开（刚×风那半，解析对拍）。三域同帧共存、同帧推进。
//!
//! **判据**（每条都机器无关）：
//! ① **刚×液**：入水后柱区流体**峰值速度 > 0**（球把水推开了，不是幽灵穿水）；
//! ② **刚×风**：静态风帆上的气动合力 = `½ρ·Cd·A·|w|²` 解析值（相对 1e-3），方向 = 风向；
//! ③ **三域确定性**：同场景跑两遍，`state_hash()` **相同**（三域同帧可复现）；
//! ④ **域间正交**：开着气动域不影响落球轨迹（球是 `Sphere`，`aero_pass` 只对 `TriMesh` 施力）；
//!   全程无 NaN、流体粒子数不变。
//!
//! ⚠️ 场景**铸装**（`PLAN-0.3.md` §4.2）：水块按沉降后几何就位，避免"带落差入盆"的顶心喷泉。
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_aero::AeroConfig;
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// 水槽：5×5 格（外沿 2.5 m）地板 + **中心一格**围堰 ⇒ 内腔 0.5×0.5 m（同 `fluid_boundary.rs`）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

/// 铸装水块 `[8,8,8]@0.05`（占据 `y ∈ [1.05, 1.45]`，液面 ≈ 1.45）。
fn water() -> FluidSystem {
    FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

/// 静态三角网风帆（0.5×0.5、面法向 `x`、立在 `(0, 1, 1.6)` —— 在水槽**外**，不碰体素）。
fn sail() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
    for iy in 0..2u32 {
        for iz in 0..2u32 {
            pts.push(Vec3::new(0.0, 1.0 + 0.5 * iy as f32, 1.6 + 0.5 * iz as f32));
        }
    }
    tris.push([0, 2, 1]);
    tris.push([1, 2, 3]);
    (pts, tris)
}

/// 三域场景：水槽 + 2b 流体 + 落球（半径 0.08、ρ=1000 ⇒ ≈2.1 kg）+ 静态风帆 + 可选风。
/// 返回 `(世界, 球索引, 风帆索引, 流体粒子数)`。
fn build(wind: Option<[f32; 3]>) -> (World, usize, u32, usize) {
    let mut w = World::new(PhysConfig::default());
    let v = tank(&mut w);
    w.add_fluid_with_boundary_coupling(water(), &[v]);
    let b = w.add_dynamic(
        Shape::Sphere { radius: 0.08 },
        Vec3::new(0.0, 2.2, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    let (pts, tris) = sail();
    let mesh = w.add_trimesh(pts, tris);
    let half = w.trimesh_half_extents(mesh);
    let panel = w.add_static(Shape::TriMesh { mesh, half }, Vec3::ZERO, Quat::IDENTITY);
    if let Some(wind) = wind {
        w.set_aero(AeroConfig {
            wind,
            ..AeroConfig::default()
        });
    }
    let n = w.fluids()[0].0.len();
    (w, b, panel, n)
}

/// 柱区（球的入水柱）内流体的**峰值速度**：球推水 ⇒ 峰值明显 > 0。
fn column_peak_v(w: &World) -> f32 {
    let f = &w.fluids()[0].0;
    let mut peak = 0.0f32;
    for (p, v) in f.positions().iter().zip(f.velocities().iter()) {
        if p.x.abs() < 0.2 && p.z.abs() < 0.2 {
            peak = peak.max(v.length());
        }
    }
    peak
}

/// 全程健康：无 NaN、流体粒子数不变。
fn healthy(w: &World, n_fluid: usize) -> bool {
    let f = &w.fluids()[0].0;
    f.len() == n_fluid
        && f.positions().iter().all(|p| p.x.is_finite())
        && f.velocities().iter().all(|v| v.x.is_finite())
}

#[test]
fn ball_splashes_into_water_with_wind_and_stays_deterministic() {
    const TICKS: usize = 45;
    let (mut windy, b, panel, n) = build(Some([8.0, 0.0, 0.0]));
    let (mut calm, b2, _, _) = build(None);
    for _ in 0..TICKS {
        windy.step();
        calm.step();
    }
    let (x_wind, x_calm) = (windy.bodies.position[b].x, calm.bodies.position[b2].x);
    let peak = column_peak_v(&windy);
    let f_sail = windy.aero_force(panel);
    let want = 0.5 * 1.225 * 1.0 * 0.25 * 64.0; // ½ρCdA|w|²，A = 0.25、w = 8
    println!(
        "球 x（开气动域）={x_wind:.6e} /（无气动域）={x_calm:.6e} | 柱区峰值 |v|={peak:.3} m/s | \
         风帆 F={f_sail:?}（解析 {want:.4} N）"
    );
    assert!(peak > 0.3, "入水应推开流体（柱区峰值 |v|），实得 {peak:.3}");
    assert!(
        (f_sail.x - want).abs() / want < 1e-3 && f_sail.y.abs() < 1e-4 && f_sail.z.abs() < 1e-4,
        "风帆气动合力应 = ½ρCdA|w|² 沿 +x，实得 {f_sail:?}（解析 {want:.4}）"
    );
    assert!(
        (x_wind - x_calm).abs() < 1e-9,
        "气动域不该改动落球轨迹（球非 TriMesh）：{x_wind:.9} vs {x_calm:.9}"
    );
    assert!(healthy(&windy, n), "三域共存后流体应仍健康、粒子数不变");
    let (mut a, _, _, _) = build(Some([8.0, 0.0, 0.0]));
    let (mut c, _, _, _) = build(Some([8.0, 0.0, 0.0]));
    for _ in 0..TICKS {
        a.step();
        c.step();
    }
    assert_eq!(a.state_hash(), c.state_hash(), "三域同帧推进必须逐位可复现");
}
