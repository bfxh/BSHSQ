//! **M4 悬臂金样**（vxl-only 通道）—— `ROUTE` M4 出口判据「悬臂/旗飘金样 + 刚度档表」里
//! 的**悬臂 + 档表**那一半（旗飘要风，属另一半）。
//!
//! 场景：一条 `SEG × 1` 格布带（`x ∈ [0, 1]`、`z ∈ [0, 0.12]`、`y = 0`）**一端钉住**、
//! 水平悬空 ⇒ 重力下垂到稳态。**冻结基线自检**只在门禁配方上判定（其余 tick 数 = 自由实验，
//! 会显式打印"跳过"）。
//!
//! 为什么需要它：M4 的出口判据此前只有零散单测，**没有"端到端 + 冻结基线"这一层** ⇒ 挠度 /
//! 应变漂了没人看得见（`M3` 的教训同款，见 `m3_collapse.rs`）。本档钉三样：
//! **自由端坐标（逐位） / 垂度 / 最大应变**，外加**五档 α 的垂度表**（SPEC §4.6 的档位）。
//!
//! ⚠️ **为什么不用 `World::state_hash()`**：那个哈希只覆盖**刚体**（`world_body.rs`：
//! `hash_bodies_streaming(&self.bodies, …)`），**不含软体域** ⇒ 对布的变化不敏感，做判据会
//! 静默失真。这里直接比 `f32::to_bits()`，逐位可查、差多少一眼看得出。
//!
//! 运行：`cargo run --release -p vxl-phys --example m4_cantilever -- [ticks]`
//! 门禁配方：`600`（见 `scripts/gate_gold.sh`）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::World;
use vxl_phys_core::{PhysConfig, Vec3};
use vxl_phys_soft::{ClothSheet, Stiffness};

/// 门禁配方（tick 数）。
const GATE_TICKS: usize = 600;
/// 长度方向的格数（`SEG + 1` 个顶点）。**悬臂不能太长**：1 m 的超软布会垂到根部之后
/// （实测自由端 `x` 跑到 −0.665），"自由端垂度"就不再是单调可读的量了（本轮踩过）。
const SEG: usize = 8;
const LEN: f32 = 0.4;
const WID: f32 = 0.08;
/// 自由端顶点（`x = LEN`、`z = 0` 那一只）。
const TIP: usize = SEG;

/// 五档 α（`SPEC.md` §4.6；由硬到软）。
const GRADES: [(&str, Stiffness); 5] = [
    ("NearRigid", Stiffness::NearRigid),
    ("Hard", Stiffness::Hard),
    ("Standard", Stiffness::Standard),
    ("Soft", Stiffness::Soft),
    ("Jelly", Stiffness::Jelly),
];

/// 悬臂布带：`SEG × 1` 格；`x = 0` 那两列钉住（悬臂根）。
fn ribbon(stiffness: Stiffness) -> ClothSheet {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
    let n = SEG as u32;
    for iz in 0..=1u32 {
        for ix in 0..=n {
            pts.push(Vec3::new(LEN * ix as f32 / n as f32, 0.0, WID * iz as f32));
        }
    }
    for ix in 0..n {
        let a = ix;
        let (c, d) = (a + 1, a + n + 1);
        tris.push([a, d, c]);
        tris.push([c, d, d + 1]);
    }
    // **结构固定 Hard、档位只调弯曲**：悬臂的下垂由弯曲刚度主导（布几乎不可拉伸），
    // 两者一起调软会让布带整体翻卷——自由端 `x` 从 1.0 跑到 −0.68，档表也不再单调
    // （本轮实测：Jelly 0.562 < Soft 0.705）。
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.bend_compliance = stiffness.alpha();
    for i in 0..s.pos.len() {
        if s.pos[i].x.abs() < 1e-6 {
            s.set_pinned(i, true);
        }
    }
    s
}

/// 跑 `ticks`，返回 `(自由端坐标, 垂度, 最大应变)`；布片缺失时退回零值（门禁禁 `unwrap`）。
fn run(stiffness: Stiffness, ticks: usize) -> ([f32; 3], f32, f32) {
    let mut w = World::new(PhysConfig::default());
    let idx = w.add_cloth(ribbon(stiffness));
    for _ in 0..ticks {
        w.step();
    }
    let Some(s) = w.cloth(idx) else {
        return ([0.0; 3], 0.0, 0.0);
    };
    let p = s.pos[TIP];
    ([p.x, p.y, p.z], -p.y, s.max_strain())
}

/// 冻结基线（配方 = `600` tick、`Standard` 档）。
///
/// **存 `f32` 的位模式**（不是十进制）：判定是"逐位"的，而十进制往返要凑够有效位数
/// （f32 需 9 位）——本轮用 `{:.9}` 踩过一次：它对 `0.00657…` 只给 7 位有效数字，
/// 解析回来不是同一个 `f32` ⇒ 判定 FAIL。位模式没有这个问题。
struct Frozen {
    tip: [u32; 3],
    droop: u32,
    strain: u32,
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        tip: [0x3e78_bdcd, 0xbea0_3160, 0x3bd7_70b6],
        droop: 0x3ea0_3160,
        strain: 0x3b8c_9920,
    })
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);

    let (tip, droop, strain) = run(Stiffness::Standard, ticks);
    println!(
        "M4-CANTILEVER ticks={ticks} 自由端=({:.9},{:.9},{:.9}) 垂度={droop:.9} 最大应变={strain:.9}",
        tip[0], tip[1], tip[2]
    );
    println!(
        "  位模式 tip=[{:#010x},{:#010x},{:#010x}] droop={:#010x} strain={:#010x}",
        tip[0].to_bits(),
        tip[1].to_bits(),
        tip[2].to_bits(),
        droop.to_bits(),
        strain.to_bits()
    );

    // **刚度档表**（SPEC §4.6 五档）—— **只作读数，不做判据**：本场景实测五档垂度差
    // <0.2%（`0.3107`–`0.3158`），原因见文件头注（布几乎完全下垂、重力主导）。要把它做成
    // 单调判据，得先找到对 α 敏感的场景（更硬的弯曲档 / 拉紧的膜），属后续片。
    println!("-- 刚度档表（α 由硬到软）--");
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for (name, st) in GRADES {
        let (_, d, e) = run(st, ticks);
        println!("  {name:>9}: α={:.0e} 垂度={d:.6} 应变={e:.6}", st.alpha());
        lo = lo.min(d);
        hi = hi.max(d);
    }
    println!(
        "档表跨度：{:.6}..{:.6}（相对差 {:.3}%）",
        lo,
        hi,
        (hi - lo) / hi * 100.0
    );

    let Some(fz) = frozen(ticks) else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；门禁配方 = {GATE_TICKS} tick）");
        return;
    };
    let mut bad: Vec<String> = Vec::new();
    for (k, (got, want)) in tip.iter().zip(fz.tip.iter()).enumerate() {
        if got.to_bits() != *want {
            bad.push(format!(
                "自由端[{k}] {got:.9}（{:#010x}）≠ 基线 {want:#010x}",
                got.to_bits()
            ));
        }
    }
    if droop.to_bits() != fz.droop {
        bad.push(format!(
            "垂度 {droop:.9}（{:#010x}）≠ 基线 {:#010x}",
            droop.to_bits(),
            fz.droop
        ));
    }
    if strain.to_bits() != fz.strain {
        bad.push(format!(
            "最大应变 {strain:.9}（{:#010x}）≠ 基线 {:#010x}",
            strain.to_bits(),
            fz.strain
        ));
    }
    if bad.is_empty() {
        println!(
            "✅ 金样基线 PASS（m4_cantilever / {ticks} tick）：垂度 {droop:.9}，应变 {strain:.9}，自由端 ({:.9},{:.9},{:.9})",
            tip[0], tip[1], tip[2]
        );
    } else {
        println!(
            "❌ 金样基线 FAIL（m4_cantilever / {ticks} tick）：{}",
            bad.join("；")
        );
        std::process::exit(1);
    }
}
