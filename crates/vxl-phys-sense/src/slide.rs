//! slide：**S2 持续接触（滑 / 滚）→ 连续量 cue**（循环音 / 持续震动；`docs/ROUTE.md` §3.2）。
//!
//! S1 归约的是**逐 tick 峰值冲击**（`World::impacts()`：落地那一瞬的"砰"）；本片补上另一半——
//! **持续**的摩擦滑移 / 滚动：同一对体逐 tick 都在接触、且接触点有**切向相对速度** ⇒ 上层据此
//! 起一段**循环**音或**持续**震动包络。事件源是 `World::manifolds()`（接触流形：体对 / 法向 /
//! ≤4 接触点）+ `bodies` 速度热组，**同样只读**，不开新管线、不往 `World` 加方法。
//!
//! **"循环"由消费方连点成段，归约本身仍无状态**：本域**不记**"上一 tick 是否在滑"（那会引入
//! 状态、破坏纯函数），只把**每 tick** 的切向相对速度按体取峰值发一条 cue；消费方按
//! `(tick, body)` 把连续 tick 的 cue 连成一段，段停即静音。于是 S1 的三条契约逐条继承：
//! ① **零干扰**（只读 `&self`）；② **确定性**（流形序 + 点序扫描、无 HashMap、平局判据显式）；
//! ③ **预算**（每 tick 上界，超出按切向速度降序丢最慢的）。
//!
//! **判据"静止体不产生循环音"**：静置堆接触点的切向相对速度只有数值噪声量级，被
//! `SenseConfig::min_slide_speed` 挡掉 ⇒ 静置段零 cue（`tests/slide_contracts.rs` 钉住）。

use std::cmp::Ordering;

use vxl_phys::{DiagExt, Vec3, World};

use crate::{gain_of, material_of, priority_of, SenseConfig, SenseStats};

/// **一条持续接触 cue**（一次滑移 / 滚动归约的结果；`Copy` ⇒ 可当值传递）。
///
/// 与 S1 的 [`crate::Cue`] 分属**两条流**（冲击 vs 持续），刻意**不共用**结构：冲击看**接近
/// 速度**（沿法向），持续看**切向速度**——量纲同为 m/s 但语义正交，混进一个结构要靠额外字段
/// 分辨"哪条流"，得不偿失。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlideCue {
    /// 产生它的 tick（`World::tick`）。
    pub tick: u64,
    /// 滑动体（体对里"会动"的那一侧；两侧都会动时报索引小的 `a`）。
    pub body: u32,
    /// 材质槽（`bodies.material[body]`）——循环音色表按它查（S3；本域**不**自带音色）。
    pub material: u32,
    /// **切向相对速度**（m/s；接触点处去法向分量后的模）——"滑得多快"，即循环音 / 震动的强度源。
    pub speed: f32,
    /// 峰值接触点（世界系；音频摆放 / 触觉定位）。
    pub pos: [f32; 3],
    /// 该点接触法向（`a → b`，世界系）——给了滑动**平面**，供上层定"擦 / 滚"方向。
    pub normal: [f32; 3],
    /// 优先级（0 最高）：由 `gain = clamp(speed / full_slide_speed, 0, 1)` 分档，阈值同 S1。
    pub priority: u8,
}

/// 归约中间态：一个**体**在本 tick 的切向速度峰值（点级多条 → 体级一条）。
struct Agg {
    body: u32,
    speed: f32,
    pos: [f32; 3],
    normal: [f32; 3],
    /// 首次过阈的候选序号（输出按它排序 ⇒ 与"记录序"同口径的确定性序）。
    first: usize,
}

/// **S2 归约本体**（由 `SenseExt::slide_cues` 委派；自由函数 ⇒ 与 S1 的 `impl` 块不冲突）。
pub(crate) fn reduce_slides(
    world: &World,
    cfg: &SenseConfig,
    out: &mut Vec<SlideCue>,
) -> SenseStats {
    let mut stats = SenseStats::default();
    // ① 逐流形逐接触点扫：切向相对速度过阈才计候选；按**体**归约到本 tick 峰值。
    //    `slot`（体号 → 归约槽）是数组直查 ⇒ 无 HashMap（确定性第一层）。
    let mut slot: Vec<u32> = vec![u32::MAX; world.bodies.len()];
    let mut agg: Vec<Agg> = Vec::new();
    for m in DiagExt::manifolds(world) {
        let (a, b) = (m.a as usize, m.b as usize);
        let Some(body) = slide_body(world, a, b) else {
            continue;
        };
        let nrm = m.normal;
        for p in m.points.iter() {
            stats.records += 1;
            let speed = tangential_speed(world, a, b, p.point, nrm);
            // 非有限（NaN/±∞）与低于阈值都不产 cue——显式写 NaN 分支（同 S1 的踩坑注记）。
            if !speed.is_finite() || speed <= cfg.min_slide_speed {
                continue;
            }
            stats.candidates += 1;
            let at = slot[body as usize];
            if at == u32::MAX {
                slot[body as usize] = agg.len() as u32;
                agg.push(Agg {
                    body,
                    speed,
                    pos: xyz(p.point),
                    normal: xyz(nrm),
                    first: stats.candidates,
                });
            } else if speed > agg[at as usize].speed {
                let s = &mut agg[at as usize];
                s.speed = speed;
                s.pos = xyz(p.point);
                s.normal = xyz(nrm);
            }
        }
    }
    stats.dropped_weak = stats.records - stats.candidates;
    // ② 预算：`speed` 降序取前 `max` 个**体**；平局按体号升序（确定性第二层）。
    let bodies = agg.len();
    if bodies > cfg.max_slides_per_tick {
        agg.sort_by(|x, y| {
            y.speed
                .partial_cmp(&x.speed)
                .unwrap_or(Ordering::Equal)
                .then(x.body.cmp(&y.body))
        });
        agg.truncate(cfg.max_slides_per_tick);
    }
    stats.dropped_budget = bodies - agg.len();
    // ③ 输出按首次出现序（同 S1：设备按事件时间线消费，序列与预算取值无关）。
    agg.sort_by_key(|s| s.first);
    stats.emitted = agg.len();
    out.extend(agg.into_iter().map(|s| SlideCue {
        tick: world.tick,
        body: s.body,
        material: material_of(world, s.body),
        speed: s.speed,
        pos: s.pos,
        normal: s.normal,
        priority: priority_of(gain_of(s.speed, cfg.full_slide_speed), cfg),
    }));
    stats
}

/// 体对里"会动"的那一侧（动 / 运动学；都动报 `a`，口径固定 ⇒ 确定性）；两侧都不动 ⇒ `None`。
fn slide_body(world: &World, a: usize, b: usize) -> Option<u32> {
    if world.bodies.is_dynamic_or_kinematic(a) {
        Some(a as u32)
    } else if world.bodies.is_dynamic_or_kinematic(b) {
        Some(b as u32)
    } else {
        None
    }
}

/// 接触点处某体的**点速度** = 线速度 + 角速度 × 力臂（`r = p − center`）。
fn point_velocity(world: &World, i: usize, p: Vec3) -> Vec3 {
    let b = &world.bodies;
    b.linvel[i] + b.angvel(i).cross(p - b.position[i])
}

/// 接触点处的**切向相对速度模**：相对速度去掉沿法向的分量（`v − (v·n)n`）。
fn tangential_speed(world: &World, a: usize, b: usize, p: Vec3, n: Vec3) -> f32 {
    let vrel = point_velocity(world, b, p) - point_velocity(world, a, p);
    (vrel - n * vrel.dot(n)).length()
}

/// `Vec3` → 3 元组数组（`SlideCue` 的字段口径：便于 JSONL / 序列化）。
fn xyz(v: Vec3) -> [f32; 3] {
    [v.x, v.y, v.z]
}

/// **slide cue 流 → JSONL**（一行一条；浮点固定 6 位小数 ⇒ 逐字可对拍）。
///
/// 字段序固定：`tick / body / mat / speed / pos / normal / prio`。不写时间戳、不写平台信息
/// （否则"机器无关"立刻失效）。
pub fn write_slide_jsonl<W: std::io::Write>(w: &mut W, slides: &[SlideCue]) -> std::io::Result<()> {
    write_slide_jsonl_banked(w, slides, None)
}

/// 同 [`write_slide_jsonl`]，但可**附带音色组 id**（S3）：`banks` 为 `Some` 时每行多一个 `"bank"`
/// 字段（追加在行尾 ⇒ `None` 与 [`write_slide_jsonl`] 逐字节相同，S2 基线不受影响）。
/// 长度不符 ⇒ `InvalidInput`（口径同 `crate::write_jsonl_banked`）。
pub fn write_slide_jsonl_banked<W: std::io::Write>(
    w: &mut W,
    slides: &[SlideCue],
    banks: Option<&[crate::BankId]>,
) -> std::io::Result<()> {
    if let Some(b) = banks {
        if b.len() != slides.len() {
            return Err(crate::bank_len_mismatch(b.len(), slides.len()));
        }
    }
    for (i, c) in slides.iter().enumerate() {
        write!(
            w,
            "{{\"tick\":{},\"body\":{},\"mat\":{},\"speed\":{:.6},\
             \"pos\":[{:.6},{:.6},{:.6}],\"normal\":[{:.6},{:.6},{:.6}],\"prio\":{}",
            c.tick,
            c.body,
            c.material,
            c.speed,
            c.pos[0],
            c.pos[1],
            c.pos[2],
            c.normal[0],
            c.normal[1],
            c.normal[2],
            c.priority
        )?;
        if let Some(b) = banks {
            write!(w, ",\"bank\":{}", b[i].0)?;
        }
        writeln!(w, "}}")?;
    }
    Ok(())
}

/// 便捷：slide cue 切片 → JSONL 字符串（判据 / 工具链用）。
pub fn slide_jsonl(slides: &[SlideCue]) -> String {
    let mut s = Vec::new();
    // 写内存 `Vec<u8>` 不会失败（它的 `Write` 实现不返回 Err）；真出错时给空串而不是 panic。
    if write_slide_jsonl(&mut s, slides).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&s).into_owned()
}

/// 便捷：带音色组 id 的 slide 流 → JSONL 字符串（S3 工具链用；`banks` 需与 `slides` 等长）。
pub fn slide_jsonl_banked(slides: &[SlideCue], banks: &[crate::BankId]) -> String {
    let mut s = Vec::new();
    if write_slide_jsonl_banked(&mut s, slides, Some(banks)).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&s).into_owned()
}
