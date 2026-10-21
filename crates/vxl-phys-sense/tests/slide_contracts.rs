//! **S2 持续接触（滑 / 滚）的判据**（`docs/ROUTE.md` §3.2 分期表：S2 判据 = S1 三条
//! 契约 + "静止体不产生循环音"，且非空洞）。
//!
//! S2 与 S1 是**两条流**（持续 vs 峰值），但共用同一套"只读消费者"纪律：① 开/关它末态
//! `state_hash` 逐位不变；② 同场景两跑 slide cue 流**逐字**一致；③ 每 tick 条数有上界且
//! 截断保留最**快**的。额外那条 S2 判据是"**静止体不产生循环音**"——静置堆滑速只是数值
//! 噪声，必须被 `min_slide_speed` 挡掉（否则"循环音"永远在响，等于白噪）。

use vxl_phys::{BodyBuildExt, DiagExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{slide_jsonl, SenseConfig, SenseExt, SenseStats, SlideCue};

/// 场景：**平滑静态盒地板** + N 个**贴地、带水平初速**的盒 ⇒ 落定即滑、摩擦减速到停。
///
/// 为什么用平滑地板而不是体素（`sense_dump` 的 S1 场景是体素）：S2 吃 `World::manifolds()`，
/// 它**不要求** provider；而体素地板的棱会让盒子**翻滚**（实测 `ωz` 冲到 ±8 rad/s）、切向速度
/// 信号被搅乱。平滑地板给的是干净、稳态的滑移——静止体不产 cue 这一条才判得干净。
///
/// 为什么这么摆：初速 3 m/s、μ=0.5 ⇒ 减速 4.9 m/s² ⇒ 约 37 tick 内滑行 0.92 m 停下（盒子
/// 0.4³ 不会翻倒：翻倒需 `a > w/h·g = 9.8 m/s²`）。同一条场景既能验"滑时有 cue"，又能验
/// "停下后静置不产 cue"。布局全确定性（无随机器），与 `sense_dump slide` 的 CLI 场景同口径。
fn scene(bodies: usize) -> World {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(4.0, 0.5, 4.0),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
    );
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(-1.75 + 0.5 * ix, 1.22, -1.75 + 0.5 * iz);
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
        w.bodies.linvel[id as usize] = Vec3::new(3.0, 0.0, 0.0);
    }
    w
}

/// 跑一场：`(世界, slide 流, 每 tick 条数, 统计汇总)`。`sense = false` ⇒ 对照组（只推物理）。
#[allow(clippy::type_complexity)]
fn run(
    bodies: usize,
    ticks: usize,
    budget: usize,
    sense: bool,
) -> (World, Vec<SlideCue>, Vec<usize>, SenseStats) {
    let cfg = SenseConfig {
        max_slides_per_tick: budget,
        ..SenseConfig::default()
    };
    let mut w = scene(bodies);
    let mut slides: Vec<SlideCue> = Vec::new();
    let mut per_tick: Vec<usize> = Vec::with_capacity(ticks);
    let mut total = SenseStats::default();
    for _ in 0..ticks {
        w.step();
        let before = slides.len();
        if sense {
            let st = w.slide_cues(&cfg, &mut slides);
            total.records += st.records;
            total.candidates += st.candidates;
            total.emitted += st.emitted;
            total.dropped_weak += st.dropped_weak;
            total.dropped_budget += st.dropped_budget;
        }
        per_tick.push(slides.len() - before);
    }
    (w, slides, per_tick, total)
}

/// **契约 ①：零干扰** —— 开/关 S2 归约，末态 `state_hash` 逐位一致。
#[test]
fn zero_interference_state_hash_is_bit_identical() {
    let (mut on, slides, _, _) = run(64, 240, 16, true);
    let (mut off, _, _, _) = run(64, 240, 16, false);
    assert!(
        !slides.is_empty(),
        "场景必须真的产生 slide cue，否则本判据空洞（先修场景，别放宽判据）"
    );
    assert_eq!(on.state_hash(), off.state_hash(), "S2 归约必须零干扰");
}

/// **契约 ②：确定性** —— 同场景两跑，slide 流**逐字**一致（JSONL 逐字节）。
#[test]
fn slide_stream_is_byte_identical_across_runs() {
    let (_, a, _, _) = run(64, 240, 16, true);
    let (_, b, _, _) = run(64, 240, 16, true);
    let (ja, jb) = (slide_jsonl(&a), slide_jsonl(&b));
    assert!(!ja.is_empty(), "空流对拍不算通过");
    assert_eq!(ja, jb, "两跑 slide 流必须逐字一致");
    // 逐行字段数恒定（格式契约；改格式必须同步改脚本 gate_sense.sh 与测试的解析口径）。
    for line in ja.lines() {
        assert_eq!(line.matches(":").count(), 7, "字段数变了：{line}");
    }
}

/// **契约 ③：预算** —— 每 tick 条数 ≤ 上界，且截断保留的是**切向速度最大**的那些（top-k ⊂ top-K）。
#[test]
fn budget_bounds_cues_and_keeps_the_fastest() {
    let (_, small, per_small, st_small) = run(64, 240, 2, true);
    let (_, big, _, _) = run(64, 240, 64, true);
    assert!(
        per_small.iter().all(|k| *k <= 2),
        "每 tick 必须 ≤ 预算，实测最大 {}",
        per_small.iter().copied().max().unwrap_or(0)
    );
    // 非空洞：预算真的被顶满过、真的丢过（否则场景里没有"多体同时滑"的帧）。
    assert!(
        per_small.contains(&2) && st_small.dropped_budget > 0,
        "预算规则没被触发过 ⇒ 本判据空洞（dropped_budget = {}）",
        st_small.dropped_budget
    );
    assert!(small.len() < big.len(), "预算 2 必须真的比预算 64 少");
    for c in &small {
        assert!(
            big.iter()
                .any(|b| b.tick == c.tick && b.body == c.body && b.speed == c.speed),
            "预算截断必须保留最快的那些：tick {} body {} speed {} 不在大预算流里",
            c.tick,
            c.body,
            c.speed
        );
    }
}

/// **S2 额外判据："静止体不产生循环音"** —— 滑停之后静置段的微动不得产 cue。
#[test]
fn resting_contacts_do_not_emit_slide_cues() {
    let (_, slides, per_tick, _) = run(64, 300, 16, true);
    assert!(
        per_tick.iter().take(60).sum::<usize>() > 0,
        "滑行段必须有 cue（否则场景不适用）"
    );
    let tail: usize = per_tick.iter().skip(120).sum();
    assert_eq!(
        tail, 0,
        "静置段仍然产 cue（{tail} 条）：min_slide_speed 没起作用"
    );
    let slow: Vec<&SlideCue> = slides
        .iter()
        .filter(|c| c.speed <= SenseConfig::default().min_slide_speed)
        .collect();
    assert!(slow.is_empty(), "存在低于阈值的 cue：{} 条", slow.len());
}
