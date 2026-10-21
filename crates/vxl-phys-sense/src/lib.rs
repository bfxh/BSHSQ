//! # vxl-phys-sense —— 声音 / 震动（感官输出域）
//!
//! **定位**（`docs/ROUTE.md` §3.2）：本域**不改物理**，只把求解**已经产生**的事件归约成
//! 感官 cue（音频 / 触觉）。它是本仓第一条"输出侧"域，也是"新加一域不必再往 `World` 添方法"
//! 那条设计判据的第一次实考：域 API 走**扩展 trait**（[`SenseExt`]），`World` 一个方法都不加。
//!
//! **事件源（全部已存在，无需新管线）**：[`World::impacts()`]（逐 tick **峰值**冲击：provider /
//! 体 / 世界接触点 / 沿法向接近速度 / 解算前速度）、`bodies.material[i]` → `Material`。
//!
//! **三条契约**（`docs/ROUTE.md` §3.2；三条都有 `tests/sense_contracts.rs` 钉住）：
//!
//! 1. **零干扰**：归约**只读** —— 开/关它，末态 `state_hash` 逐位不变（全程不持有 `&mut World`）；
//! 2. **确定性**：同场景同输入 ⇒ **逐字相同**的 cue 序列（记录序扫描、无 HashMap 迭代序、
//!    排序键显式给平局判据）；
//! 3. **预算**：每 tick cue 数有**上界**，且超预算时按 gain **降序**丢最弱的（显式规则，
//!    不许默认全播——万级密堆一 tick 能产生上千次冲击，全播在听感上等于白噪）。
//!
//! **分期**：S1 = 冲击（峰值）→ cue + 预算/优先级 + JSONL dump；**S2 = 持续接触（滑 / 滚）→
//! 连续量 cue**（[`SlideCue`] / [`SenseExt::slide_cues`]，实现见 `slide.rs`）；**S3 = 材质表 /
//! 资产映射**（[`SoundTable`] / [`map_banks`]：材质槽 → 音色组 id，**缺项显式拒绝**，实现见
//! `table.rs`）；**S4 = 震动设备输出（包络 / 上钳 `[0,1]` / 更新率·延迟口径）**（[`HapticDriver`] /
//! [`HapticFrame`]，实现见 `haptic.rs`）。S1–S3 都是"事件 → cue"的**归约**（无状态），S4 才是
//! "cue → 设备帧"的**整形**——它带状态，所以另加两条口径（上钳闭在 `1.0`、帧号是 tick 的纯函数
//! ⇒ 延迟 0 帧）；四条契约的判据在 `tests/haptic_contracts.rs`。

#![forbid(unsafe_code)]

mod haptic;
mod slide;
mod table;

use vxl_phys::{ImpactExt, World};

pub use haptic::{
    haptic_jsonl, write_haptic_jsonl, HapticConfig, HapticDriver, HapticFrame, HapticStats,
};
pub use slide::{
    slide_jsonl, slide_jsonl_banked, write_slide_jsonl, write_slide_jsonl_banked, SlideCue,
};
pub use table::{map_banks, BankId, MaterialSlot, SoundTable, TableError};

/// **一条感官 cue**（一次冲击的归约结果；`Copy` ⇒ 可当值传递，无生命周期包袱）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cue {
    /// 产生它的 tick（`World::tick`）。
    pub tick: u64,
    /// 冲击体（动态体索引）。
    pub body: u32,
    /// 材质槽（`bodies.material[body]`）——音色表按它查（S3；本域**不**自带音色）。
    pub material: u32,
    /// 强度 `[0,1]`：由**沿法向接近速度**归一化（位置由接触点给）。
    pub gain: f32,
    /// 世界接触点（音频摆放 / 触觉定位）。
    pub pos: [f32; 3],
    /// 沿法向的接近速度（m/s，> 0 = 正在靠近）——原始量，供上层自定准则。
    pub approach: f32,
    /// **优先级**（0 最高）：由 gain 分档，超预算时低优先级先丢（阈值见 [`SenseConfig`]）。
    pub priority: u8,
}

/// **归约档**（预算 + 优先级）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SenseConfig {
    /// **每 tick cue 上界**（硬预算；超出按 gain 降序丢最弱的）。
    pub max_cues_per_tick: usize,
    /// 低于此接近速度的冲击**不产 cue**（"轻碰不算响"；`0` = 不过滤）。
    pub min_approach: f32,
    /// 满响的接近速度：`gain = clamp(approach / full_approach, 0, 1)`。
    pub full_approach: f32,
    /// 重击阈值：`gain ≥ heavy_gain` ⇒ 优先级 0。
    pub heavy_gain: f32,
    /// 轻击阈值：`gain ≥ light_gain` ⇒ 优先级 1（否则 2）。
    pub light_gain: f32,
    /// **S2：每 tick 持续接触 cue 上界**（硬预算；超出按切向速度降序丢最慢的）。
    pub max_slides_per_tick: usize,
    /// **S2：低于此切向相对速度的接触点不产 cue**（"静止体不产生循环音"；`0` = 不过滤）。
    pub min_slide_speed: f32,
    /// **S2：满响的切向速度**：`gain = clamp(speed / full_slide_speed, 0, 1)`。
    pub full_slide_speed: f32,
}

impl Default for SenseConfig {
    /// 默认档口径：`full_approach = 8 m/s`（落体 3.3 m 的着地速度量级）、
    /// `min_approach = 0.05 m/s`（睡眠阈 0.04 之上一点 ⇒ 静止堆不产 cue）、
    /// 预算 16 条/tick（远超常见场景的实际冲击数，只在密堆冲击帧才触发丢弃）。
    fn default() -> Self {
        Self {
            max_cues_per_tick: 16,
            min_approach: 0.05,
            full_approach: 8.0,
            heavy_gain: 0.6,
            light_gain: 0.25,
            max_slides_per_tick: 16,
            // 静置堆切向速度是数值噪声量级（远低于 0.05 m/s）；0.1 m/s 之上才当"真的在滑"
            // （人手推一把 / 落体擦墙的量级都远超它）。
            min_slide_speed: 0.1,
            // 4 m/s 满响：摩擦滑降的稳态速度量级；再快也封顶（避免 gain 长时间顶在 1 无分辨力）。
            full_slide_speed: 4.0,
        }
    }
}

/// **一次归约的统计**（工具链与判据读它：预算真的生效了没有，"非空洞"要在这里看）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SenseStats {
    /// 本 tick 冲击记录总数（`World::impacts().len()`）。
    pub records: usize,
    /// 通过 `min_approach` 的候选数。
    pub candidates: usize,
    /// 实际写入 `out` 的 cue 数。
    pub emitted: usize,
    /// 因低于 `min_approach` 被丢的条数。
    pub dropped_weak: usize,
    /// 因超预算被丢的条数。
    pub dropped_budget: usize,
}

/// **感官输出域的扩展 API**（`impl SenseExt for World`）——本域**不往 `World` 加方法**
/// （`god.gate.json` 把 `World` 的方法数登记为债务，处置就是这条扩展 trait）。
pub trait SenseExt {
    /// 归约**本 tick**的冲击 → cue，**追加**到 `out`（返回 [`SenseStats`]）。
    ///
    /// **只读契约**：实现只持有 `&self`，不借出任何 `&mut`；同一个 `out` 可跨 tick 反复追加
    /// （调用方自己决定要不要 `clear`）。
    fn sense_cues(&self, cfg: &SenseConfig, out: &mut Vec<Cue>) -> SenseStats;

    /// **S2**：归约**本 tick**的持续接触（滑 / 滚）→ [`SlideCue`]，**追加**到 `out`。
    ///
    /// 事件源是 `World` 的 `manifolds` 访问器（`DiagExt` 域侧 trait）：逐流形逐接触点算**切向相对速度**（相对速度去法向分量
    /// 后的模），按**体**取本 tick 峰值发一条。**只读契约同 [`SenseExt::sense_cues`]**（不借出
    /// `&mut`、不持有状态 ⇒ 零干扰；不记"上一 tick 是否在滑"，循环段由消费方连点成段）。
    ///
    /// `SenseStats` 字段在 S2 口径下的含义（与 S1 逐字段对应）：`records` = 扫过的接触点数；
    /// `candidates` = 过 `min_slide_speed` 的点数；`emitted` = 写出的 cue 数（≤ 滑动体数）；
    /// `dropped_weak` = 点级过滤数；`dropped_budget` = 因超 `max_slides_per_tick` 丢的**体**数。
    fn slide_cues(&self, cfg: &SenseConfig, out: &mut Vec<SlideCue>) -> SenseStats;
}

impl SenseExt for World {
    fn sense_cues(&self, cfg: &SenseConfig, out: &mut Vec<Cue>) -> SenseStats {
        let records = self.impacts();
        let mut stats = SenseStats {
            records: records.len(),
            ..SenseStats::default()
        };
        // ① 逐条记录判候选：**记录序**扫描（确定性序的第一层）⇒ `(增益, 记录序, cue)`。
        let mut cand: Vec<(f32, usize, Cue)> = Vec::with_capacity(records.len());
        for (i, r) in records.iter().enumerate() {
            // 非有限（NaN/±∞）与低于阈值都不产 cue——显式写 NaN 分支，别用 `!(a > min)`
            // （clippy 的 `neg_cmp_op_on_partial_ord`，本地踩过一次）。
            if !r.approach.is_finite() || r.approach <= cfg.min_approach {
                continue;
            }
            let gain = gain_of(r.approach, cfg.full_approach);
            cand.push((
                gain,
                i,
                Cue {
                    tick: self.tick,
                    body: r.body,
                    material: material_of(self, r.body),
                    gain,
                    pos: [r.point.x, r.point.y, r.point.z],
                    approach: r.approach,
                    priority: priority_of(gain, cfg),
                },
            ));
        }
        stats.candidates = cand.len();
        stats.dropped_weak = stats.records - stats.candidates;
        // ② 预算：`gain` 降序取前 `max` 条；**平局按记录序升序**（确定性序的第二层）。
        if cand.len() > cfg.max_cues_per_tick {
            cand.sort_by(|x, y| {
                y.0.partial_cmp(&x.0)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(x.1.cmp(&y.1))
            });
            cand.truncate(cfg.max_cues_per_tick);
        }
        // ③ 输出按**记录序**（不是 gain 序）——设备按事件时间线消费，序列也与预算取值无关。
        cand.sort_by_key(|c| c.1);
        stats.emitted = cand.len();
        stats.dropped_budget = stats.candidates - stats.emitted;
        out.extend(cand.into_iter().map(|c| c.2));
        stats
    }

    /// S2 持续接触归约（本体在 `slide::reduce_slides`；这里只委派，保持单一 `impl` 块）。
    fn slide_cues(&self, cfg: &SenseConfig, out: &mut Vec<SlideCue>) -> SenseStats {
        slide::reduce_slides(self, cfg, out)
    }
}

/// 冲击体的材质槽（体号越界 ⇒ 槽 0 = 默认材质，口径同求解器 `material_pair` 的兜底）。
fn material_of(world: &World, body: u32) -> u32 {
    world
        .bodies
        .material
        .get(body as usize)
        .copied()
        .unwrap_or(0)
}

/// `gain = clamp(approach / full, 0, 1)`；`full ≤ 0` 视为 1（防除零把 gain 变成 inf/NaN）。
fn gain_of(approach: f32, full: f32) -> f32 {
    let d = if full > 0.0 { full } else { 1.0 };
    let g = approach / d;
    if g > 1.0 {
        1.0
    } else if g > 0.0 {
        g
    } else {
        0.0
    }
}

/// 优先级分档（0 重 / 1 中 / 2 轻）；阈值来自 [`SenseConfig`]。gain 为 NaN 时两个比较全假 ⇒ 落最低档。
fn priority_of(gain: f32, cfg: &SenseConfig) -> u8 {
    if gain >= cfg.heavy_gain {
        0
    } else if gain >= cfg.light_gain {
        1
    } else {
        2
    }
}

/// **cue 流 → JSONL**（一行一条；浮点固定 6 位小数 ⇒ 逐字可对拍，见 `docs/ROUTE.md` §3.2）。
///
/// 字段序固定：`tick / body / mat / gain / pos / approach / prio`。不写时间戳、不写平台信息
/// （否则"机器无关"立刻失效）。
pub fn write_jsonl<W: std::io::Write>(w: &mut W, cues: &[Cue]) -> std::io::Result<()> {
    write_jsonl_banked(w, cues, None)
}

/// 同 [`write_jsonl`]，但可**附带音色组 id**（S3）：`banks` 为 `Some` 时每行多一个 `"bank"`
/// 字段（**追加在行尾**，前面字段逐字不变 ⇒ `None` 与 [`write_jsonl`] 逐字节相同，S1 基线不受
/// 影响）。`banks` 长度与 cue 数不符 ⇒ `InvalidInput`（不做"部分映射"：半张表比直接报错更难查）。
pub fn write_jsonl_banked<W: std::io::Write>(
    w: &mut W,
    cues: &[Cue],
    banks: Option<&[BankId]>,
) -> std::io::Result<()> {
    if let Some(b) = banks {
        if b.len() != cues.len() {
            return Err(bank_len_mismatch(b.len(), cues.len()));
        }
    }
    for (i, c) in cues.iter().enumerate() {
        write!(
            w,
            "{{\"tick\":{},\"body\":{},\"mat\":{},\"gain\":{:.6},\"pos\":[{:.6},{:.6},{:.6}],\
             \"approach\":{:.6},\"prio\":{}",
            c.tick,
            c.body,
            c.material,
            c.gain,
            c.pos[0],
            c.pos[1],
            c.pos[2],
            c.approach,
            c.priority
        )?;
        if let Some(b) = banks {
            write!(w, ",\"bank\":{}", b[i].0)?;
        }
        writeln!(w, "}}")?;
    }
    Ok(())
}

/// `banks` 与 cue 数不符时的 I/O 错（调用方应保证等长，见 [`map_banks`]）。
pub(crate) fn bank_len_mismatch(banks: usize, cues: usize) -> std::io::Error {
    std::io::Error::new(
        std::io::ErrorKind::InvalidInput,
        format!("banks 长度 {banks} 与 cue 数 {cues} 不一致"),
    )
}

/// 便捷：cue 切片 → JSONL 字符串（判据/工具链用）。
pub fn jsonl(cues: &[Cue]) -> String {
    let mut s = Vec::new();
    // 写内存 `Vec<u8>` 不会失败（它的 `Write` 实现不返回 Err）；真出错时给空串而不是 panic。
    if write_jsonl(&mut s, cues).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&s).into_owned()
}

/// 便捷：带音色组 id 的 cue 流 → JSONL 字符串（S3 工具链用；`banks` 需与 `cues` 等长）。
pub fn jsonl_banked(cues: &[Cue], banks: &[BankId]) -> String {
    let mut s = Vec::new();
    if write_jsonl_banked(&mut s, cues, Some(banks)).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&s).into_owned()
}
