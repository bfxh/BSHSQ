//! probes_domain：**本仓全域基准批** —— PhysArena 谱系之外的域（软体/流体/喷溅/
//! 破坏/耦合），每个场景注明来源（仓内哪个测试/示例的配方）与它的**判据形态**。
//!
//! 与 arena 复刻批的差别：这批没有"对手表"（arena 侧无对应场景），基准读数只作
//! 本仓自身的历史对照与优化迭代；协议相同（30 预热 + 180 测量、60 Hz 固定步长）。
use super::*;
use vxl_phys_aero::AeroConfig;
use vxl_phys_soft::{ClothSheet, Rope, Stiffness};

/// 8×8 平地板三角网（±4 m，y = 0）——与 `tests/rope_scene.rs` 同几何。
fn flat_mesh() -> vxl_phys_terrain::mesh::TriMesh {
    const N: usize = 8;
    const S: f32 = 4.0;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=N {
        for ix in 0..=N {
            let x = -S + (2.0 * S * ix as f32) / N as f32;
            let z = -S + (2.0 * S * iz as f32) / N as f32;
            verts.push(Vec3::new(x, 0.0, z));
        }
    }
    for iz in 0..N as u32 {
        for ix in 0..N as u32 {
            let a = iz * (N as u32 + 1) + ix;
            let c = a + 1;
            let d = a + N as u32 + 1;
            let e = d + 1;
            tris.push([a, d, c]);
            tris.push([c, d, e]);
        }
    }
    vxl_phys_terrain::mesh::TriMesh::new(verts, tris)
}

/// 竖直旗面（`n×n` 格、宽 `2·size`、顶边钉在横杆）——与 `tests/cloth_wind_scene.rs` 同几何。
/// 返回 `(点, 三角, 顶边点索引)`。
fn flag(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>, Vec<usize>) {
    let (mut pts, mut tris, mut rail) = (Vec::new(), Vec::new(), Vec::new());
    let s = 2.0 * size / n as f32;
    for iy in 0..=n {
        for iz in 0..=n {
            pts.push(Vec3::new(0.0, -(iy as f32) * s, -size + s * iz as f32));
            if iy == 0 {
                rail.push(pts.len() - 1);
            }
        }
    }
    for iy in 0..n as u32 {
        for iz in 0..n as u32 {
            let a = iy * (n as u32 + 1) + iz;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris, rail)
}

/// 5×5 体素水槽（地板 + 中心一格敞开围堰，内腔 0.5×0.5 m）——与 `tests/fluid_boundary.rs` 同几何。
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

/// 铸装水块 `[8,8,8]@0.05`（`y ∈ [1.05, 1.45]`）——与 `tests/splash_scene.rs` 同款。
fn water() -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

/// **软体 · 旗飘（布 × 风）**：源 `tests/cloth_wind_scene.rs`（门面 `set_aero` 下发风到布）。
pub(crate) fn scene_dom_cloth_wind(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let (pts, tris, rail) = flag(8, 0.6);
    let mut sc = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Standard);
    for i in rail {
        sc.set_pinned(i, true);
    }
    w.add_cloth(sc);
    w.set_aero(AeroConfig {
        wind: [20.0, 0.0, 0.0],
        ..AeroConfig::default()
    });
    w
}

/// **软体 · 绳落网格地形**：源 `tests/rope_scene.rs`（33 节点绳落在三角网 + 刚性盒代理通道）。
pub(crate) fn scene_dom_rope_terrain(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    w.add_mesh(flat_mesh());
    add_static_box(
        &mut w,
        Vec3::new(0.0, 0.25, 0.0),
        Vec3::new(1.0, 0.25, 1.0),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
    );
    add_box_r(
        &mut w,
        Vec3::new(1.5, 1.0, 0.0),
        Vec3::splat(0.2),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
        1000.0,
    );
    let mut rope = Rope::line(
        Vec3::new(-0.4, 1.2, 0.0),
        Vec3::new(0.4, 1.2, 0.0),
        33,
        0.02,
    );
    rope.set_pinned(0, false);
    rope.set_pinned(32, false);
    rope.damping = 0.999;
    w.add_rope(rope);
    w
}

/// **流体 · 溃坝金样场景**：源 `examples/m5_dam_break.rs`（体素盆 + 504 粒自由水柱 + 围堰）。
pub(crate) fn scene_dom_fluid_dam(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 0 || ix == 4 || iz == 0 || iz == 4 {
                vol.set(ix, 2, iz, true);
            }
        }
    }
    let voxel_id = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.73, 1.05, -0.125),
        [6, 6, 14],
        0.05,
    );
    w.add_fluid(sys, &[voxel_id]);
    w
}

/// **流体 · 浮箱金样场景**：源 `examples/m5_float_box.rs`（512 粒 + 水下轻盒靠浮力升上来）。
pub(crate) fn scene_dom_fluid_float(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
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
    let voxel_id = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    w.add_fluid(sys, &[voxel_id]);
    add_box_r(
        &mut w,
        Vec3::new(0.0, 1.10, 0.0),
        Vec3::splat(0.06),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
        300.0,
    );
    w
}

/// **流体 · SPH 规模档（32³ = 32768 粒）**：源 `vxl-phys-fluid/examples/sph_scale.rs` 的晶格形态。
///
/// 口径说明：无 provider（同 `sph_scale` —— 只量求解成本，不混体素查询）；m5_dam/float 用
/// 的 504/512 粒档见上两场景。
pub(crate) fn scene_dom_sph_scale(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let n = 32usize;
    let spacing = 0.05f32;
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    w.add_fluid(sys, &[]);
    w
}

/// **喷溅 · 场上的刚体静置**：源 `examples/splat_rest_probe.rs`（平场 σ=0.5 → 盒/球落定）。
pub(crate) fn scene_dom_splat_rest(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    // 平场：y=0 面按 0.25 m 铺各向同性核（σ=0.5、幅值 1.0，iso=0.5）——同探针几何。
    let mut f = vxl_phys_splat::GaussianSplatField::new(0.5);
    for ix in -10..=10 {
        for iz in -10..=10 {
            f.push(vxl_phys_splat::Splat::isotropic(
                Vec3::new(ix as f32 * 0.25, 0.0, iz as f32 * 0.25),
                0.5,
                1.0,
            ));
        }
    }
    w.add_splat_field(f);
    let m = mat(&mut w, 0.9, 0.02);
    let i = w.bodies.len();
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 3.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
        1000.0,
    );
    w.bodies.set_material(i, m);
    let j = w.bodies.len();
    w.add_dynamic(
        Shape::Sphere { radius: 0.3 },
        Vec3::new(1.2, 3.0, 0.0),
        vxl_phys_core::Quat::IDENTITY,
        1000.0,
    );
    w.bodies.set_material(j, m);
    let k = w.bodies.len();
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.35),
        },
        Vec3::new(-1.5, 3.6, 0.5),
        vxl_phys_core::Quat::IDENTITY,
        1000.0,
    );
    w.bodies.set_material(k, m);
    w
}

/// **破坏 · Voronoi 预断裂墙**：M3 预断裂管线（体素 → 碎块刚体）+ 弹丸整墙崩碎。
pub(crate) fn scene_dom_voronoi(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.5, 4.0));
    vol.fill_box(Vec3::new(0.0, 1.5, -2.0), Vec3::new(0.5, 3.5, 2.0));
    let marker = w.add_voxel(vol);
    let (min, max) = (Vec3::new(0.0, 1.5, -2.0), Vec3::new(0.5, 3.5, 2.0));
    if let Some(pid) = w.provider_id_of(marker) {
        let seeds = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(min, max, 40, 0.35);
        let pieces = w.fracture_voronoi(pid, min, max, &seeds, 1500.0);
        println!("  （预断裂：{pieces} 块碎块体）");
    }
    let b = add_box_r(
        &mut w,
        Vec3::new(-2.0, 3.0, 0.0),
        Vec3::splat(0.4),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
        2000.0,
    );
    w.bodies.set_linvel(b, Vec3::new(10.0, 0.0, 0.0));
    w
}

/// **破坏 · 冲击挖洞（M3 管线）**：源 `examples/m3_impact.rs`（体素墙 + 8 m/s 炮弹）。
/// 计时窗里**每步**调 `apply_impact_destruction`（破坏了才会真的发生）——见 `bench_impact`。
pub(crate) fn scene_dom_carve_impact(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.5, 4.0));
    vol.fill_box(Vec3::new(0.0, 1.5, -2.0), Vec3::new(0.5, 3.5, 2.0));
    w.add_voxel(vol);
    let b = add_box_r(
        &mut w,
        Vec3::new(-2.0, 3.0, 0.0),
        Vec3::splat(0.4),
        vxl_phys_core::Quat::IDENTITY,
        ARENA_DEFAULT,
        2000.0,
    );
    // 源默认弹速 8 m/s 正好压在阈值 8 上（实测窗口内只挖 1 块）⇒ 本场景提到 12 m/s
    // （0.2 m/step 仍 < 体半长 0.4 ⇒ 无隧穿），保证"冲击→挖洞→碎块"整条链被量到。
    w.bodies.set_linvel(b, Vec3::new(12.0, 0.0, 0.0));
    w
}

/// **耦合 · 溅水（刚-液-风三域）**：源 `tests/splash_scene.rs`（体素水槽 + 2b 流体 + 落球 + 风帆 + 风）。
pub(crate) fn scene_dom_splash(cfg: PhysConfig) -> World {
    let mut w = World::new(cfg);
    let v = tank(&mut w);
    w.add_fluid_with_boundary_coupling(water(), &[v]);
    add_ball(
        &mut w,
        Vec3::new(0.0, 2.2, 0.0),
        0.08,
        ARENA_DEFAULT,
        1000.0,
    );
    // 静态风帆（面法向 x，立在 (0, ·, 1.6)）：三角网一等形状。
    let mut pts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iy in 0..2u32 {
        for iz in 0..2u32 {
            pts.push(Vec3::new(0.0, 1.0 + 0.5 * iy as f32, 1.6 + 0.5 * iz as f32));
        }
    }
    tris.push([0, 2, 1]);
    tris.push([1, 2, 3]);
    let mesh = w.add_trimesh(pts, tris);
    let half = w.trimesh_half_extents(mesh);
    w.add_static(
        Shape::TriMesh { mesh, half },
        Vec3::ZERO,
        vxl_phys_core::Quat::IDENTITY,
    );
    w.set_aero(AeroConfig {
        wind: [12.0, 0.0, 0.0],
        ..AeroConfig::default()
    });
    w
}

/// 冲击破坏场景的**专用基准窗**：计时窗内每步 = `step()` + `apply_impact_destruction()`
/// （用通用 `bench()` 会把破坏关掉 —— 那样测的不是这个域）。
pub(crate) fn bench_impact(name: &str, mut w: World, provider: u32, threshold: f32, density: f32) {
    let mut carved = 0usize;
    for _ in 0..WARMUP {
        w.step();
        carved += w.apply_impact_destruction(provider, threshold, density);
    }
    w.reset_timings();
    let mut samples: Vec<f64> = Vec::with_capacity(MEASURE);
    for _ in 0..MEASURE {
        let t0 = std::time::Instant::now();
        w.step();
        carved += w.apply_impact_destruction(provider, threshold, density);
        samples.push(t0.elapsed().as_secs_f64() * 1000.0);
    }
    samples.sort_by(|a, b| a.total_cmp(b));
    let p50 = samples[samples.len() / 2];
    let p95 = samples[(samples.len() as f64 * 0.95) as usize];
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    let dynb = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .count();
    let h = w.health();
    let alive = w
        .providers()
        .voxel(provider)
        .map(|v| v.filled_count())
        .unwrap_or(0);
    println!(
        "{name}: {} 体（动 {dynb}） p50 {p50:.3} ms  p95 {p95:.3}  mean {mean:.3}  等效 {:>6.0} FPS",
        w.bodies.len(),
        1000.0 / mean
    );
    println!(
        "  破坏账（窗口内逐 step 调 `apply_impact_destruction`）：碎块累计 {carved}  残余体素 {alive}  干净={}",
        h.is_clean()
    );
}
