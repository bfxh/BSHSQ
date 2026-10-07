//! **M3 坍塌金样**（vxl-only 通道；`gold-sample/` 那条是 Rapier 对照，装不了体素破坏）。
//!
//! 场景：体素地板 + **悬空体素块**；`fracture_voronoi` 把整块预断裂成碎块 ⇒ 碎块自由下落
//! **坍塌**到地面并入睡。**冻结基线自检**只在门禁配方上判定（其余参数 = 自由实验，会显式打印
//! "跳过"）。
//!
//! ⚠️ **为什么不是"打塔"**：体素域是**静态地形** —— 打掉几格只会挖洞 + 生成碎块，剩下的格子
//! 仍是静态的、不会塌（实测：2×2×10 的塔被 8 m/s 炮弹打掉 8 格，塔纹丝不动）。"坍塌"必须让
//! **碎块**（动态体）成为主体 ⇒ 本档断整块、让碎块自己落。
//!
//! 运行：`cargo run --release -p vxl-phys --example m3_collapse -- [ticks] [seeds]`
//! 门禁配方：`1200 64`（见 `scripts/gate_gold.sh` 与 `docs/RECIPES.md` §金样门）。
//!
//! 为什么需要它：破坏路径此前只有单测（`tests/destruction_tiered.rs` / `mesh_carve`），**没有冻结
//! 基线** ⇒ 碎片数/末态漂了没人看得见。本档钉四项确定性读数（碎块数 / 末态动态体 / 末态清醒 /
//! 末态哈希）+ 一项健康量（最深穿透）；换代按 ADR-0004 记新旧值与理由。
//!
//! ⚠️ 规模：ROUTE 的「10 万碎片档」**不在本档**（那是参考机上的规模实验，不是 CI 金样）——
//! 本档 1200 tick、32 个多格碎块（**全睡**；CI 可负担）；放大/加密就改参数（同时会跳过基线判定）。

// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Vec3};

/// 门禁配方（`ticks / seeds`）。
const GATE_TICKS: usize = 1200;
const GATE_SEEDS: usize = 64;

/// 悬空体素块（世界坐标；4 m × 4 m × 4 m，0.5 格 ⇒ 8³ = 512 格）。
const BLOCK_MIN: Vec3 = Vec3::new(-2.0, 6.0, -2.0);
const BLOCK_MAX: Vec3 = Vec3::new(2.0, 10.0, 2.0);
/// 种子抖动（与 `world_step/destruction.rs` 的 `SEED_JITTER` 同值；两处都是"确定性起点锚点"）。
const SEED_JITTER: f32 = 0.25;

/// 冻结基线（配方 = `1200 64`）。
struct Frozen {
    debris: usize,
    dyn_n: usize,
    awake: usize,
    hash: u128,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（保证本二进制仍可自由做实验）。
fn frozen(ticks: usize, seeds: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS && seeds == GATE_SEEDS).then_some(Frozen {
        debris: 32,
        dyn_n: 32,
        awake: 0,
        hash: 0x22ca32fb3d285c1f2d6a9979a3bc6577,
    })
}

fn main() {
    let mut args = std::env::args().skip(1);
    let ticks = args
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let seeds = args
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_SEEDS);

    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, 0.0, -4.0), 0.5, 16, 16, 16);
    // 地板 3 层（薄地板会被砸穿 ⇒ 碎块掉出世界）；悬空块在 y ∈ [6,10)。
    vol.fill_box(Vec3::new(-4.0, 0.0, -4.0), Vec3::new(4.0, 1.5, 4.0));
    vol.fill_box(BLOCK_MIN, BLOCK_MAX);
    let filled0 = vol.filled_count();
    w.add_voxel(vol);

    // **预断裂整块**（Voronoi；种子走确定性格点抖动）⇒ 碎块是动态体、自己下落坍塌。
    let seeds_v = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
        BLOCK_MIN,
        BLOCK_MAX,
        seeds,
        SEED_JITTER,
    );
    let debris = w.fracture_voronoi(0, BLOCK_MIN, BLOCK_MAX, &seeds_v, 1000.0);
    for _ in 0..ticks {
        w.step();
    }
    let (mut dyn_n, mut awake) = (0usize, 0usize);
    for i in 0..w.bodies.len() {
        if w.bodies.is_dynamic(i) {
            dyn_n += 1;
            if w.bodies.awake[i] {
                awake += 1;
            }
        }
    }
    // `unwrap-gate` 对新增文件零基线 ⇒ 不用 `unwrap`（provider 缺失时退回初值）。
    let filled1 = match w.providers().voxel(0) {
        Some(v) => v.filled_count(),
        None => filled0,
    };
    let max_depth = w.health().max_depth;
    let hash = w.state_hash();
    println!(
        "M3-COLLAPSE ticks={ticks} seeds={seeds} 墙体素 {filled0}→{filled1} 碎块={debris} \
         动态={dyn_n} 清醒={awake} max_depth={max_depth:.4} hash={hash:#x}"
    );

    let Some(fz) = frozen(ticks, seeds) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方见 docs/RECIPES.md §金样门）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    if debris != fz.debris {
        bad.push(format!("碎块 {debris} ≠ 基线 {}", fz.debris));
    }
    if dyn_n != fz.dyn_n {
        bad.push(format!("动态体 {dyn_n} ≠ 基线 {}", fz.dyn_n));
    }
    if awake != fz.awake {
        bad.push(format!("清醒 {awake} ≠ 基线 {}", fz.awake));
    }
    if hash != fz.hash {
        bad.push(format!("哈希 {hash:#x} ≠ 基线 {:#x}", fz.hash));
    }
    if bad.is_empty() {
        println!(
            "✅ 金样基线 PASS（m3_collapse / {ticks} tick）：碎块 {debris}，动态 {dyn_n}，清醒 {awake}，hash {hash:#x}"
        );
    } else {
        println!(
            "❌ 金样基线 FAIL（m3_collapse / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
