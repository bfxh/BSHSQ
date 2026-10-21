//! **M4 绳索金样**（vxl-only 通道）—— `ROUTE` M4 出口判据里**绳索**那一块。
//!
//! 场景：一根 40 节点的绳，**两端钉死**在 `x = ±0.6, y = 1.6`，初始水平（`y = 1.6`）⇒ 自由
//! 下垂成 U 形；一个**动态球**（半径 `0.12`、密度 `800`）从绳中段正上方 `y = 2.2` 落下，
//! 砸中绳后**被绳托住**。这一个场景同时压住三件事：
//!
//! 1. **绳索自身的伸展/弯曲约束**（下垂深度）——只有结构约束正确，U 形才会垂到该垂的位置；
//! 2. **绳↔刚体的双向耦合**（`rope_pass` 的位置回填 + 反作用）——球被托住而不是穿过去；
//! 3. **确定性**（末态位模式逐位）——绳**不收敛**，但末态逐位可复现，理由见下。
//!
//! **判据**（四项）：
//! - 「1」绳的**末态哈希**（`FNV-1a`，按 `f32::to_bits` 逐字节）—— 钉住整条绳的末态；
//! - 「2」绳的**最低点** `y` —— 可读的下垂深度，肉眼能验；
//! - 「3」球的**末态** `y` —— 被托住的平衡高度；
//! - 「4」**结构判据**（两条，都有上下界）—— 绳须真的垂下来（`low_y < 1.5`）、
//!   球须贴在绳上（`0.0 <= ball_y − low_y < 0.2`，下界防穿到绳下方）。
//!
//! ⚠️ **没有**「球速趋于零」这一条：球压着一根没摩擦的绳，持续微振才是物理对的。
//! 实测球 `|v| ≈ 6.2e-2`（600 tick）/ `6.6e-2`（1200 tick）—— **不降反升**。
//!
//! **为什么绳可以用位模式而车不行**：判据只看**有没有放大器**，与收敛性无关。
//! 绳有阻尼 + 约束，扰动能被**耗散**掉；车是接触驱动的**混沌**系统，1 ulp 会被放大成
//! 可见差异。两者都可以永远静不下来——绳就确实静不下来。
//! 判据口径见 `docs/PHYSICS-INVARIANTS.md` §2.4。
//!
//! 运行：`cargo run --release -p vxl-phys --example m4_rope -- [ticks]`
//! 门禁配方：`600`（见 `scripts/gate_gold.sh`）。
// 新文件：glob 门对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::SoftExt;
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_soft::Rope;

/// 门禁配方（`gate_gold.sh` 用这个）。
const GATE_TICKS: usize = 600;
/// 绳节点数。
const NODES: usize = 40;
/// 绳半径。
const RADIUS: f32 = 0.025;

/// 末态读数。
#[derive(Debug, Clone, Copy)]
struct Readings {
    /// 绳末态哈希（`FNV-1a` over `f32::to_bits`）。
    hash: u64,
    /// 绳最低点 `y`。
    low_y: f32,
    /// 球末态 `y`。
    ball_y: f32,
    /// 球末速模长。
    ball_v: f32,
    /// 绳节点末速模长最大值（区分"绳在抖"与"球在滑"）。
    rope_v: f32,
}

/// 冻结的基线（门禁配方下）。
#[derive(Debug, Clone, Copy)]
struct Frozen {
    hash: u64,
    low_y: f32,
    ball_y: f32,
}

fn readings(ticks: usize) -> Readings {
    let mut w = World::new(PhysConfig::default());
    let mut rope = Rope::line(
        Vec3::new(-0.6, 1.6, 0.0),
        Vec3::new(0.6, 1.6, 0.0),
        NODES,
        RADIUS,
    );
    // 两端钉死 ⇒ 中间自由下垂。这是场景的定义，不是"调参"。
    rope.set_pinned(0, true);
    rope.set_pinned(NODES - 1, true);
    rope.damping = 0.999;
    let _ri = w.add_rope(rope);

    let ball = w.add_dynamic(
        Shape::Sphere { radius: 0.12 },
        Vec3::new(0.0, 2.2, 0.0),
        Quat::IDENTITY,
        800.0,
    ) as usize;

    for _ in 0..ticks {
        w.step();
    }

    let r = match w.rope(0) {
        Some(r) => r,
        None => {
            eprintln!("❌ 绳 0 没注册上");
            std::process::exit(1);
        }
    };
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for p in &r.pos {
        for c in [p.x, p.y, p.z] {
            for b in c.to_bits().to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x100_0000_01b3);
            }
        }
    }
    let low_y = r.pos.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let rope_v = r.vel.iter().map(|v| v.length()).fold(0.0f32, f32::max);
    let v = w.bodies.linvel[ball];
    Readings {
        hash: h,
        low_y,
        ball_y: w.bodies.position[ball].y,
        ball_v: v.length(),
        rope_v,
    }
}

/// 只认门禁配方；其余返回 `None` ⇒ 打印"跳过"（本二进制仍可自由做实验）。
fn frozen(ticks: usize) -> Option<Frozen> {
    // 位模式写法（十进制超过 f32 有效位会触发 `clippy::excessive_precision`）。
    (ticks == GATE_TICKS).then_some(Frozen {
        hash: 0xa584_d2ef_c802_9259,
        low_y: f32::from_bits(0x3fb4_fa05),
        ball_y: f32::from_bits(0x3fc7_704e),
    })
}

fn main() {
    let ticks: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let r = readings(ticks);
    println!(
        "  绳低点 y={:.9} | 球 y={:.9} |v|={:.3e} | 绳|v|max={:.3e} | hash=0x{:016x}",
        r.low_y, r.ball_y, r.ball_v, r.rope_v, r.hash
    );
    let Some(f) = frozen(ticks) else {
        println!("  （跳过基线判定：{ticks} tick 不是门禁配方 {GATE_TICKS}）");
        return;
    };
    let ok = r.hash == f.hash
        && (r.low_y - f.low_y).abs() < 1e-6
        && (r.ball_y - f.ball_y).abs() < 1e-6
        // (4) 结构判据（可读、不依赖数值）。两条都要够**紧**，否则等于没判：
        //  - 绳必须**真的垂下来**：两端固定在 `y = 1.6`，实测低点 `1.4139`（垂度 0.186）。
        //    阈值取 `1.5`（垂度 ≥ 0.10 ≈ 半个垂度）—— 松到"参数偏了仍能过"就没意义了；
        //  - 球必须**贴在绳上**：实测 `ball_y − low_y = 0.1442`，与球半径 `0.12` 同量级。
        //    **上下界都要**：只写 `< 0.2` 的话，球**穿到绳下方**（差值为负）也满足
        //    —— 那正是 P19 记录过的「球穿过薄壳」情形，是这个判据最该拦住的一种失败。
        && r.low_y < 1.5
        && (0.0..0.2).contains(&(r.ball_y - r.low_y));
    // ⚠️ **不要**在这里断言"球速 ≈ 0"。球压着一根没摩擦的绳，持续微振是物理上正确的：
    // 实测球 `|v| ≈ 6.2e-2`、绳自己 `|v|max ≈ 1.2e-1`，跑多久都不降。而它**每次跑都逐位
    // 相同**（三次实测同一 hash）—— 金样守的是**确定性**，不是静止。把这两件事混为一谈，
    // 会写出一个永远不可能通过、或者必须靠放宽到无意义才能通过的判据。
    if !ok {
        eprintln!(
            "❌ 金样基线 FAIL（m4_rope / {ticks} tick）\n   \
             期望 hash=0x{:016x} low_y={:.9} ball_y={:.9} 且 low_y<1.5、ball_y-low_y<0.2\n   \
             实测 hash=0x{:016x} low_y={:.9} ball_y={:.9} |v|={:.3e}",
            f.hash, f.low_y, f.ball_y, r.hash, r.low_y, r.ball_y, r.ball_v,
        );
        std::process::exit(1);
    }
    println!("✅ 金样基线 PASS（m4_rope / {ticks} tick）");
}
