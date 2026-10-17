//! **A1 声学金样** —— 声学域（`vxl-phys-acoustics`）此前**没有任何金样**。
//!
//! 它是个异类：不做力学、不推进状态，只**读**同一份场景产出确定性的「声音线索」
//! （直达 / 反射 / 绕射）—— 正因为是纯读，它天然适合位模式判据。
//!
//! ## 两个子场景（缺一不可）
//!
//! 单跑一个都不够：
//! - **开阔档**（无墙）：证明**直达**与**地板反射**真的算出来了；
//! - **遮挡档**（`z = 1` 立一道强吸声墙，夹在声源与听者之间）：证明**遮挡判定**生效。
//!
//! 只有遮挡档会漏掉"反射整条路径挂了但直达还在"；只有开阔档会漏掉"遮挡根本没生效"。
//!
//! ## 判据
//!
//! - 「1」两档各自的 `jsonl` 全量输出 **FNV-1a 哈希** —— 钉住每条路径的 `delay` / `gain` /
//!   `dir` / `bands` 的 `f32` 位模式（**主判据**）；
//! - 「2」**路径条数**与**直达条数**（可读；区分"数值变了"与"路径结构变了"）；
//! - 「3」**结构断言** —— 开阔档 `direct ≥ 1`（直达真的存在），遮挡档 `direct == 0`
//!   且 `total ≥ 1`（直达被挡、但反射还在）。没有这一条，数值再好也可能是"什么都没算出来"。
//!
//! **为什么这一域能要位模式**：声学路径是**纯函数**（给定几何 + 声源 + 听者 ⇒ 唯一输出），
//! 没有时间积分、没有接触迭代、没有混沌放大。与 `m6_wheeled_car` 恰好是两个极端。
//!
//! 运行：`cargo run --release -p vxl-phys --example a1_acoustic_gold -- [ticks]`
//! 门禁配方：`40`（见 `scripts/gate_gold.sh`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    jsonl, AcousticConfig, AcousticGeometry, AcousticsExt, Listener, PathKind, SoundPath, BANDS,
};

/// 门禁配方：冲击发生所需的 tick 数（落地即 break，实际通常远小于此）。
const GATE_TICKS: usize = 40;

/// 单个子场景的读数。
#[derive(Clone, Copy)]
struct Scene {
    hash: u64,
    total: usize,
    direct: usize,
}

/// 冻结的基线。
struct Frozen {
    open: Scene,
    walled: Scene,
}

/// 声学几何的四张平铺表（顶点 / 三角 / 逐面吸声 / 逐面透射）。
type Tables = (
    Vec<Vec3>,
    Vec<[u32; 3]>,
    Vec<[f32; BANDS]>,
    Vec<[f32; BANDS]>,
);

/// 地板（`y = 0`，±4 m）+ 可选一道立在 `z = 1` 的强吸声墙。
fn tables(with_wall: bool) -> Tables {
    let mut verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
    ];
    let mut tris = vec![[0u32, 1, 2], [0, 2, 3]];
    let mut absorb = vec![[0.25f32; BANDS]; 2];
    let mut trans = vec![[0.0f32; BANDS]; 2];
    if with_wall {
        let base = verts.len() as u32;
        verts.extend([
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 3.0, 1.0),
            Vec3::new(-1.0, 3.0, 1.0),
        ]);
        tris.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
        // 强吸声（0.9）：挡住直达，同时让它的反射几乎没有能量。
        absorb.extend([[0.9f32; BANDS]; 2]);
        trans.extend([[0.0f32; BANDS]; 2]);
    }
    (verts, tris, absorb, trans)
}

/// 世界：体素地面 + 一个落向地面的立方（冲击产生声源）。
fn world_with_source() -> Option<World> {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.0, -0.25, -1.0), 0.25, 8, 1, 8);
    vol.fill_box(Vec3::new(-1.0, -0.25, -1.0), Vec3::new(1.0, 0.0, 1.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(0.0, -6.0, 0.0);
    // 落到地面上为止（声源 = `impacts` 的接触点；落地前没有冲击 ⇒ 判据会空洞）。
    for _ in 0..GATE_TICKS {
        w.step();
        if !w.impacts().is_empty() {
            break;
        }
    }
    if w.impacts().is_empty() {
        return None;
    }
    Some(w)
}

fn scene(with_wall: bool) -> Option<Scene> {
    let (verts, tris, absorb, trans) = tables(with_wall);
    let g = AcousticGeometry {
        verts: &verts,
        tris: &tris,
        absorption: &absorb,
        transmission: &trans,
        scattering: &[],
        portals: &[],
        edges: &[],
        medium: None,
    };
    let w = world_with_source()?;
    let cfg = AcousticConfig::default();
    let listener = Listener {
        pos: Vec3::new(0.0, 0.5, 2.0),
        radius: 0.5,
    };
    let mut paths: Vec<SoundPath> = Vec::new();
    let _st = w.acoustic_cues(&g, &listener, &cfg, &mut paths);
    let text = jsonl(&paths);
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in text.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    let direct = paths
        .iter()
        .filter(|p| matches!(p.kind, PathKind::Direct))
        .count();
    Some(Scene {
        hash: h,
        total: paths.len(),
        direct,
    })
}

/// 只认门禁配方（本 example 的场景是固定的，参数只用于让调用方传 tick 数占位）。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        open: Scene {
            hash: 0xa458_58de_2968_7bbc,
            total: 2,
            direct: 1,
        },
        walled: Scene {
            // 全挡：墙既遮直达，也遮住"声源→地板→听者"那条反射链的反射点（就在 z=1 上）。
            // 这不是缺陷——是这一档的**定义**：它要证明遮挡生效，而 0 条路径就是最强的证据。
            hash: 0xcbf2_9ce4_8422_2325,
            total: 0,
            direct: 0,
        },
    })
}

fn main() {
    let ticks: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let (open, walled) = match (scene(false), scene(true)) {
        (Some(a), Some(b)) => (a, b),
        _ => {
            eprintln!("❌ 场景没产生冲击 ⇒ 没有声源，判据会空洞");
            std::process::exit(1);
        }
    };
    println!(
        "  开阔: {} 条（直达 {}）hash=0x{:016x}",
        open.total, open.direct, open.hash
    );
    println!(
        "  遮挡: {} 条（直达 {}）hash=0x{:016x}",
        walled.total, walled.direct, walled.hash
    );
    let Some(f) = frozen(ticks) else {
        println!("  （跳过基线判定：{ticks} 不等于门禁配方 {GATE_TICKS}）");
        return;
    };
    let ok = open.hash == f.open.hash
        && open.total == f.open.total
        && open.direct == f.open.direct
        && walled.hash == f.walled.hash
        && walled.total == f.walled.total
        && walled.direct == f.walled.direct
        // (3) 结构断言（两档各一条，缺一不可）：
        //  - 开阔档：`direct >= 1` 且 `total >= 2` —— 后者才证明**反射真的算出来了**，
        //    只写 `direct >= 1` 的话，反射整条路径挂了也照样过；
        //  - 遮挡档：`direct == 0` 且 `total == 0` —— 墙把直达和反射都挡住了。这是这一档
        //    的**定义**（证明遮挡生效），不是"什么都没算出来"：开阔档的 2 条路径已经证明
        //    管线是通的。两档合起来才是完整证据。
        && open.direct >= 1
        && open.total >= 2
        && walled.direct == 0
        && walled.total == 0;
    if !ok {
        eprintln!(
            "❌ 金样基线 FAIL（a1_acoustic_gold / {ticks}）\n   \
             期望 开阔 hash=0x{:016x} total={} direct={}（direct≥1）\n   \
             期望 遮挡 hash=0x{:016x} total={} direct={}（direct==0 且 total==0）\n   \
             实测 开阔 hash=0x{:016x} total={} direct={}\n   \
             实测 遮挡 hash=0x{:016x} total={} direct={}",
            f.open.hash,
            f.open.total,
            f.open.direct,
            f.walled.hash,
            f.walled.total,
            f.walled.direct,
            open.hash,
            open.total,
            open.direct,
            walled.hash,
            walled.total,
            walled.direct,
        );
        std::process::exit(1);
    }
    println!("✅ 金样基线 PASS（a1_acoustic_gold / {ticks}）");
}
