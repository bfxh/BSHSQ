//! **S4 震动设备输出的判据**（`docs/ROUTE.md` §3.2 分期表：S4 = 包络 / 上钳 `[0,1]` /
//! 更新率·延迟口径，判据 = **包络单调性与上钳边界**）。
//!
//! S4 与 S1–S3 的**结构差别**：前三片是"事件 → cue"的**归约**（无状态纯函数，判据是零干扰 /
//! 确定性 / 预算）；S4 是"cue → **设备帧**"的**整形**，它**必须带状态**（包络 `env`）。带状态
//! 的东西最容易"实现漂移而判据看不出来"，所以本片把三件事钉成可判的**结构性质**：
//!
//! - **上钳边界**：`drive ≥ 1` ⇒ 恰好 `1.0`（闭区间上界）、`drive = 0.999` ⇒ **不许**当满档；
//! - **包络单调**：攻击段逐帧非减且**不超调**；释放段逐帧非增且收敛到**恰好 `0.0`**；
//! - **更新率 / 延迟口径**：帧号是 tick 的**纯函数**（无隐藏补帧），cue 所属 tick 的**首帧**
//!   就生效（延迟 0 帧，且不泄漏到更早的帧）。
//!
//! S1 的两条契约照旧继承（**零干扰**：驱动器只吃 `&[Cue]`，结构上碰不到物理末态；**确定性**：
//! 帧流逐字可对拍，且驱动量取 `max` ⇒ **与同 tick 的 cue 顺序无关**）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{
    haptic_jsonl, Cue, HapticConfig, HapticDriver, HapticFrame, SenseConfig, SenseExt, SlideCue,
};

/// tick 步长取**引擎自己的** `PhysConfig::default().dt`（不写字面量 ⇒ 引擎改默认步长时本片跟着走）。
fn tick_dt() -> f32 {
    PhysConfig::default().dt
}

/// 合成一条冲击 cue（判据要的是**驱动量**，不必真跑物理；`approach` 只给自洽值）。
fn cue(tick: u64, gain: f32) -> Cue {
    Cue {
        tick,
        body: 0,
        material: 0,
        gain,
        pos: [0.0; 3],
        approach: gain * 8.0,
        priority: 0,
    }
}

/// 合成一条滑移 cue（S2 源；本片只用它验"两个源合成一个电平"与顺序无关）。
fn slide(tick: u64, speed: f32) -> SlideCue {
    SlideCue {
        tick,
        body: 0,
        material: 0,
        speed,
        pos: [0.0; 3],
        normal: [0.0, 1.0, 0.0],
        priority: 0,
    }
}

/// 场景：**体素地板** + 网格盒雨（与 `sense_dump impact` 的 CLI 场景同族口径；布局全确定性）。
///
/// ⚠️ 地板必须是 **provider**（同 S1 的踩坑）：`World::record_impacts` 只对 `Shape::Provider`
/// 接触记录冲击 ⇒ 拿静态盒当地板会得到**空 cue 流**，本片的非空洞断言会一起红。
fn scene(bodies: usize) -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, -0.5, -4.0), 0.5, 16, 3, 16);
    vol.fill_box(Vec3::new(-4.0, -0.5, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(
            -1.75 + 0.5 * ix,
            2.0 + 0.25 * (i % 5) as f32,
            -1.75 + 0.5 * iz,
        );
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

/// 跑一场并把本 tick 的冲击 cue 喂给震动驱动器；`drive = false` ⇒ 对照组（只推物理）。
///
/// 每 tick **清空** cue 缓冲：驱动器要的是"本 tick 的驱动量"（不是累积流）——这也让本片与
/// CLI（`sense_dump haptic`）的口径逐条对齐。
fn run(bodies: usize, ticks: usize, drive: bool) -> (World, Vec<HapticFrame>) {
    let sense = SenseConfig::default();
    let mut w = scene(bodies);
    let mut driver = HapticDriver::new(HapticConfig::default(), tick_dt());
    let mut cues: Vec<Cue> = Vec::new();
    let mut frames: Vec<HapticFrame> = Vec::new();
    for t in 0..ticks {
        w.step();
        if drive {
            cues.clear();
            w.sense_cues(&sense, &mut cues);
            driver.step(t as u64, &cues, &[], &mut frames);
        }
    }
    (w, frames)
}

/// **判据 ①：上钳边界（闭在 `1.0`）** —— 过驱动顶到恰好 `1.0`；差一点（`0.999`）**不许**当满档。
#[test]
fn clamp_boundary_is_closed_at_one() {
    // 过驱动：gain 0.9 × 4.0 = 3.6 ≫ 1 ⇒ 必须**恰好**顶到 1.0，且一帧都不越界。
    let over_cfg = HapticConfig {
        impact_scale: 4.0,
        ..HapticConfig::default()
    };
    let mut over = HapticDriver::new(over_cfg, tick_dt());
    let mut frames: Vec<HapticFrame> = Vec::new();
    let mut stats = vxl_phys_sense::HapticStats::default();
    for _ in 0..4 {
        stats = over.step(0, &[cue(0, 0.9)], &[], &mut frames);
    }
    assert!(
        frames.iter().all(|f| (0.0..=1.0).contains(&f.intensity)),
        "强度必须落在 [0,1] 内，实测越界"
    );
    assert_eq!(
        frames.last().map(|f| f.intensity),
        Some(1.0),
        "过驱动必须顶到**恰好** 1.0（上钳闭在区间上界）"
    );
    assert!(
        stats.clamped > 0,
        "上钳从没发生过 ⇒ 本判据空洞（先修场景/档位，别放宽判据）"
    );

    // 边界**下侧**：驱动量 0.999 ⇒ 收敛到 0.999 本身，**不许**被当成满档（边界是闭在 1.0，不是 0.999）。
    let unit_cfg = HapticConfig {
        impact_scale: 1.0,
        ..HapticConfig::default()
    };
    let mut near = HapticDriver::new(unit_cfg, tick_dt());
    let mut below: Vec<HapticFrame> = Vec::new();
    for _ in 0..4 {
        near.step(0, &[cue(0, 0.999)], &[], &mut below);
    }
    assert_eq!(
        below.last().map(|f| f.intensity),
        Some(0.999),
        "gain 0.999 的驱动量必须收敛到 0.999 本身（上钳边界不许内缩）"
    );

    // 恰在边界（drive == 1.0）⇒ 1.0 ⇒ 边界属于上钳内（闭区间）。
    let mut exact = HapticDriver::new(unit_cfg, tick_dt());
    let mut one: Vec<HapticFrame> = Vec::new();
    for _ in 0..4 {
        exact.step(0, &[cue(0, 1.0)], &[], &mut one);
    }
    assert_eq!(
        one.last().map(|f| f.intensity),
        Some(1.0),
        "drive == 1.0 必须落在上钳内（闭区间）"
    );
}

/// **判据 ②：包络单调** —— 释放段逐帧非增并收敛到恰好 `0.0`；攻击段逐帧非减、不超调、收敛到驱动量。
#[test]
fn envelope_is_monotone_and_never_overshoots() {
    // (a) 释放段：先顶到 1.0，再断驱动 60 tick（1020 帧）⇒ 逐帧非增，末帧**恰好** 0.0。
    let over_cfg = HapticConfig {
        impact_scale: 4.0,
        ..HapticConfig::default()
    };
    let mut down = HapticDriver::new(over_cfg, tick_dt());
    let mut frames: Vec<HapticFrame> = Vec::new();
    for _ in 0..8 {
        down.step(0, &[cue(0, 0.9)], &[], &mut frames);
    }
    let cut = frames.len();
    for t in 1..=60 {
        down.step(t, &[], &[], &mut frames);
    }
    let tail = &frames[cut..];
    assert!(!tail.is_empty(), "释放段为空 ⇒ 本判据空洞");
    assert!(
        tail.windows(2).all(|w| w[1].intensity <= w[0].intensity),
        "断驱动后包络必须逐帧非增"
    );
    assert_eq!(
        tail.last().map(|f| f.intensity),
        Some(0.0),
        "释放必须收敛到**恰好** 0.0（否则器件长期微振）"
    );

    // (b) 攻击段：从 0 起、常值驱动 0.75 ⇒ 逐帧非减、每帧 ≤ 0.75（不超调）、末帧恰 0.75。
    let rise_cfg = HapticConfig {
        impact_scale: 1.0,
        attack: 0.1,
        ..HapticConfig::default()
    };
    let mut up = HapticDriver::new(rise_cfg, tick_dt());
    let mut rise: Vec<HapticFrame> = Vec::new();
    for _ in 0..4 {
        up.step(0, &[cue(0, 0.75)], &[], &mut rise);
    }
    assert!(
        rise.windows(2).all(|w| w[1].intensity >= w[0].intensity),
        "常值驱动下包络必须逐帧非减"
    );
    assert!(
        rise.iter().all(|f| f.intensity <= 0.75),
        "不得超调（越过驱动量）"
    );
    assert_eq!(
        rise.last().map(|f| f.intensity),
        Some(0.75),
        "必须收敛到驱动量本身"
    );
}

/// **判据 ③：更新率 / 延迟口径** —— 每 tick 帧数 = `round(hz·dt)`（兜底 ≥1）；帧号是 tick 的纯函数；
/// cue 在所属 tick 的**首帧**生效（延迟 0 帧，不泄漏到更早的帧）。
#[test]
fn update_rate_and_latency_semantics() {
    let d = HapticDriver::new(HapticConfig::default(), tick_dt());
    assert_eq!(
        d.frames_per_tick(),
        17,
        "1000 Hz × 1/60 s ⇒ 每 tick 17 帧（四舍五入）"
    );
    assert_eq!(
        d.frame_of(3, 5),
        3 * 17 + 5,
        "帧号必须是 tick 的纯函数（驱动器不维护帧计数器）"
    );
    // 更新率与 tick 解耦：240 Hz ⇒ 恰好 4 帧/tick；低于 1 帧/tick 兜底为 1（0 帧 = 静默设备）。
    let hz240 = HapticConfig {
        update_hz: 240.0,
        ..HapticConfig::default()
    };
    let hz1 = HapticConfig {
        update_hz: 1.0,
        ..HapticConfig::default()
    };
    assert_eq!(
        HapticDriver::new(hz240, tick_dt()).frames_per_tick(),
        4,
        "240 Hz 必须给 4 帧/tick"
    );
    assert_eq!(
        HapticDriver::new(hz1, tick_dt()).frames_per_tick(),
        1,
        "低于一帧/tick 的更新率兜底为 1（不许 0 帧）"
    );

    // 帧数口径：10 tick ⇒ 恰好 170 帧，帧号连续且由 tick 唯一决定（无隐藏补帧 / 丢帧）。
    let mut seq = HapticDriver::new(HapticConfig::default(), tick_dt());
    let mut frames: Vec<HapticFrame> = Vec::new();
    for t in 0..10u64 {
        seq.step(t, &[], &[], &mut frames);
    }
    assert_eq!(frames.len(), 170, "10 tick × 17 帧");
    assert!(
        frames
            .iter()
            .enumerate()
            .all(|(i, f)| f.frame == i as u64 && f.tick == (i / 17) as u64),
        "帧号必须连续、且由 tick 唯一决定"
    );

    // **延迟 0 帧**：tick 5 的 cue 改的是 tick 5 的**首帧**（5×17 = 85），更早的帧一帧都不受影响。
    let mut on = HapticDriver::new(HapticConfig::default(), tick_dt());
    let mut off = HapticDriver::new(HapticConfig::default(), tick_dt());
    let (mut fa, mut fb) = (Vec::new(), Vec::new());
    for t in 0..10u64 {
        let cues = if t == 5 {
            vec![cue(5, 0.9)]
        } else {
            Vec::new()
        };
        on.step(t, &cues, &[], &mut fa);
        off.step(t, &[], &[], &mut fb);
    }
    let cut = 5 * 17;
    assert!(
        fa[..cut].iter().all(|f| f.intensity == 0.0),
        "cue 不许泄漏到更早的帧（延迟不得为负）"
    );
    assert!(
        fa[cut].intensity > 0.0,
        "cue 所属 tick 的**首帧**就必须带上它（延迟 = 0 帧）"
    );
    assert!(
        fa[cut..]
            .iter()
            .zip(fb[cut..].iter())
            .all(|(x, y)| x.intensity > y.intensity),
        "cue 之后的每一帧都该高于无 cue 组（电平确实被抬起来，且随后衰减）"
    );
}

/// **契约继承：零干扰 + 确定性 + 同 tick 无关** —— 真跑一场（体素地板 + 盒雨）。
#[test]
fn zero_interference_and_frame_stream_is_order_independent() {
    // ① 零干扰：把 cue 流真喂给驱动器，末态 state_hash 逐位不变（类型级：驱动器只吃 `&[Cue]`）。
    let (mut on, frames) = run(64, 240, true);
    let (mut off, _) = run(64, 240, false);
    assert!(!frames.is_empty(), "驱动器必须真的产出帧 ⇒ 否则本判据空洞");
    assert!(
        frames.iter().any(|f| f.intensity > 0.5),
        "场景必须真的把电平抬起来（否则上钳/包络都判不到）"
    );
    assert_eq!(on.state_hash(), off.state_hash(), "S4 驱动必须零干扰");

    // ② 确定性：同场景两跑帧流**逐字**一致（逐行字段数恒定 = 格式契约）。
    let (_, a) = run(64, 240, true);
    let (_, b) = run(64, 240, true);
    let (ja, jb) = (haptic_jsonl(&a), haptic_jsonl(&b));
    assert!(!ja.is_empty(), "空流对拍不算通过");
    assert_eq!(ja, jb, "两跑帧流必须逐字一致");
    for line in ja.lines() {
        assert_eq!(line.matches(':').count(), 3, "字段数变了：{line}");
    }

    // ③ 同 tick 顺序无关：两条 cue **倒序**喂入 ⇒ 帧流逐字相同（驱动量取 `max` ⇒ 与归约序无关；
    //    S1/S2 里那种"求和"归约喂倒序就会崩——这条把 S4 的确定性来源钉在实现上）。
    let mut p = HapticDriver::new(HapticConfig::default(), tick_dt());
    let mut q = HapticDriver::new(HapticConfig::default(), tick_dt());
    let (mut fp, mut fq) = (Vec::new(), Vec::new());
    p.step(0, &[cue(0, 0.4), cue(0, 0.9)], &[slide(0, 3.0)], &mut fp);
    q.step(0, &[cue(0, 0.9), cue(0, 0.4)], &[slide(0, 3.0)], &mut fq);
    assert_eq!(
        haptic_jsonl(&fp),
        haptic_jsonl(&fq),
        "同 tick 的 cue / slide 顺序不得影响帧流"
    );
}
