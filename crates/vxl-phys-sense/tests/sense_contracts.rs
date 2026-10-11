//! **感官输出域的判据**（`docs/ROUTE.md` §3.2 的三条契约，一条一个测试）。
//!
//! 为什么这三条缺一不可：本域是**只读消费者**——① 一旦某天它偷偷改了物理状态，`state_hash`
//! 就不再是那个"跨编译器/跨架构逐位一致"的指纹；② 一旦归约里出现 HashMap 迭代序或未定义
//! 的平局判据，cue 流就没法当基线对拍；③ 一旦"全播"，万级密堆一 tick 上千条 cue 在设备侧
//! 等于白噪，且"预算"这条设计约束会静默消失。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{jsonl, Cue, SenseConfig, SenseExt, SenseStats};

/// 场景：**体素地板** + 网格盒雨（与 `sense_dump` 的 CLI 场景同族口径；布局全确定性）。
///
/// ⚠️ 地板必须是 **provider**：`World::record_impacts` 只对 `Shape::Provider`（体素/网格/喷溅）
/// 接触记录冲击（它同时是破坏管线的输入）⇒ 拿静态盒当地板会得到**空 cue 流**（本地实测踩过，
/// 四个判据一起红）。
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

/// 跑一场：`(世界, cue 流, 每 tick 条数, 统计汇总)`。`sense = false` ⇒ 对照组（只推物理）。
#[allow(clippy::type_complexity)]
fn run(
    bodies: usize,
    ticks: usize,
    budget: usize,
    sense: bool,
) -> (World, Vec<Cue>, Vec<usize>, SenseStats) {
    let cfg = SenseConfig {
        max_cues_per_tick: budget,
        ..SenseConfig::default()
    };
    let mut w = scene(bodies);
    let mut cues: Vec<Cue> = Vec::new();
    let mut per_tick: Vec<usize> = Vec::with_capacity(ticks);
    let mut total = SenseStats::default();
    for _ in 0..ticks {
        w.step();
        let before = cues.len();
        if sense {
            let st = w.sense_cues(&cfg, &mut cues);
            total.records += st.records;
            total.candidates += st.candidates;
            total.emitted += st.emitted;
            total.dropped_weak += st.dropped_weak;
            total.dropped_budget += st.dropped_budget;
        }
        per_tick.push(cues.len() - before);
    }
    (w, cues, per_tick, total)
}

/// **契约 ①：零干扰** —— 开/关感官归约，末态 `state_hash` 逐位一致。
#[test]
fn zero_interference_state_hash_is_bit_identical() {
    let (mut on, cues, _, _) = run(64, 240, 16, true);
    let (mut off, _, _, _) = run(64, 240, 16, false);
    assert!(
        !cues.is_empty(),
        "场景必须真的产生 cue，否则本判据空洞（先修场景，别放宽判据）"
    );
    assert_eq!(on.state_hash(), off.state_hash(), "感官归约必须零干扰");
}

/// **契约 ②：确定性** —— 同场景两跑，cue 流**逐字**一致（JSONL 逐字节）。
#[test]
fn cue_stream_is_byte_identical_across_runs() {
    let (_, a, _, _) = run(64, 240, 16, true);
    let (_, b, _, _) = run(64, 240, 16, true);
    let (ja, jb) = (jsonl(&a), jsonl(&b));
    assert!(!ja.is_empty(), "空流对拍不算通过");
    assert_eq!(ja, jb, "两跑 cue 流必须逐字一致");
    // 逐行字段数恒定（格式契约；改格式必须同步改脚本 gate_sense.sh 的解析口径）
    for line in ja.lines() {
        assert_eq!(line.matches(":").count(), 7, "字段数变了：{line}");
    }
}

/// **契约 ③：预算** —— 每 tick 条数 ≤ 上界，且截断保留的是**最响**的那些（top-k ⊂ top-K）。
#[test]
fn budget_bounds_cues_and_keeps_the_loudest() {
    let (_, small, per_small, st_small) = run(128, 240, 2, true);
    let (_, big, _, _) = run(128, 240, 64, true);
    assert!(
        per_small.iter().all(|k| *k <= 2),
        "每 tick 必须 ≤ 预算，实测最大 {}",
        per_small.iter().copied().max().unwrap_or(0)
    );
    // 非空洞：预算真的被顶满过、真的丢过（否则场景没有密堆冲击帧）
    assert!(
        per_small.contains(&2) && st_small.dropped_budget > 0,
        "预算规则没被触发过 ⇒ 本判据空洞（dropped_budget = {}）",
        st_small.dropped_budget
    );
    assert!(small.len() < big.len(), "预算 2 必须真的比预算 64 少");
    for c in &small {
        assert!(
            big.iter()
                .any(|b| b.tick == c.tick && b.body == c.body && b.gain == c.gain),
            "预算截断必须保留最响的那些：tick {} body {} gain {} 不在大预算流里",
            c.tick,
            c.body,
            c.gain
        );
    }
}

/// **"轻碰不算响"**（`min_approach`）：静置期的微动不得产 cue——这条防的是"输出侧白噪"。
#[test]
fn resting_contacts_do_not_emit_cues() {
    let (_, cues, per_tick, _) = run(64, 240, 16, true);
    let tail: usize = per_tick.iter().skip(180).sum();
    assert!(
        per_tick.iter().take(60).sum::<usize>() > 0,
        "落地段必须有 cue（否则场景不适用）"
    );
    assert_eq!(
        tail, 0,
        "静置段仍然产 cue（{tail} 条）：min_approach 没起作用"
    );
    let weak: Vec<&Cue> = cues
        .iter()
        .filter(|c| c.approach <= SenseConfig::default().min_approach)
        .collect();
    assert!(weak.is_empty(), "存在低于阈值的 cue：{} 条", weak.len());
}
