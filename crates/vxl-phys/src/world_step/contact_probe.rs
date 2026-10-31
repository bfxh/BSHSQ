//! **接触点 churn / 预测覆盖率探针**（M1 只读仪表）。
//!
//! 两个问题：
//! 1. 一 tick 之内有多少接触点是**新出现**的（churn，`docs/M1-RULING.md` §3.5）；
//! 2. 速度预测视野（`NarrowPhase::set_predict_dt`）能提前看到多少**下一子步的真实接触**
//!    ——即"检测每 tick 一次"还差多少覆盖。
//!
//! 问题 2 走**反事实档**：tick 首子步在同一批候选对/同一速度上再跑一趟窄相
//! （`predict_dt = 整 tick`），**结果不喂解算器** ⇒ 动力学与不开探针时逐位相同。
//!
//! 关闭时每子步只多一次原子读 + 一次空调用（`predicted_pass` 立即返回）。
use std::cell::RefCell;
use std::collections::{BTreeSet, HashMap};
use std::sync::atomic::{AtomicU8, Ordering};

use vxl_phys_core::Vec3;
use vxl_phys_narrow::{ContactPoint, ContactPoints, Manifold, NarrowPhase};

use crate::World;

const ON: u8 = 1;
/// 反事实档：首子步额外跑一趟 `predict_dt = 整 tick` 的窄相（见模块注）。
const COUNTERFACTUAL: u8 = 2;

/// 一个接触点：身份 + 是否**投机**（`depth ≤ -skin` ⇒ 本趟不预测就不会出现）。
#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub id: u64,
    pub speculative: bool,
}

/// 一次采样。
#[derive(Clone, Debug)]
pub struct Sample {
    pub tick: u64,
    pub substep: u32,
    /// 本子步**实际**流形点（真实窄相 `predict_dt = 0` ⇒ `speculative` 恒 false）。
    pub points: Vec<Point>,
    /// 反事实档：本 tick 首子步按整 tick 视野预测出的点；其余子步为空。
    pub predicted: Vec<Point>,
    /// 反事实档：本 tick 首子步的宽相候选对（复用档里这份表整 tick 冻结）。
    /// 用来把"漏掉的点"分成**宽相没给候选**与**窄相视野拒收**两类。
    pub pairs: Vec<(u32, u32)>,
}

/// **位置判定**：tick 内"新点"按位置再分三类。
///
/// 动机：按特征 ID 判"新点"会把**同一接触位置换了个特征号**也算新。冻结接触集时，
/// 同位置的旧点仍在（深度/锚点按当前位姿重算）⇒ **几何不丢**，丢的是 warm-start 的
/// ID 匹配；真新位置/新体对才是几何上真正丢掉的部分。
#[derive(Clone, Copy, Debug, Default)]
pub struct LocStats {
    /// 同体对、与首子步某点距离 ≤ `max(2·skin, 1cm)`（= 窄相点去重阈值）⇒ 位置没变。
    pub same_loc: u64,
    /// 同体对但半径内无首子步点 ⇒ 该对上真新出现的接触位置。
    pub new_loc: u64,
    /// 体对在首子步的流形里根本不存在 ⇒ 新体对。
    pub new_pair: u64,
}

type Pending = (Vec<Point>, Vec<(u32, u32)>);

struct TickFirst {
    tick: u64,
    by_pair: HashMap<(u32, u32), Vec<(u64, Vec3)>>,
}

thread_local! {
    #[allow(clippy::missing_const_for_thread_local)]
    static MODE: AtomicU8 = const { AtomicU8::new(0) };
    #[allow(clippy::missing_const_for_thread_local)]
    static BUF: RefCell<Vec<Sample>> = const { RefCell::new(Vec::new()) };
    #[allow(clippy::missing_const_for_thread_local)]
    static PEND: RefCell<Pending> = const { RefCell::new((Vec::new(), Vec::new())) };
    #[allow(clippy::missing_const_for_thread_local)]
    static FIRST: RefCell<Option<TickFirst>> = const { RefCell::new(None) };
    #[allow(clippy::missing_const_for_thread_local)]
    static LOC: RefCell<LocStats> = const { RefCell::new(LocStats { same_loc: 0, new_loc: 0, new_pair: 0 }) };
}

fn pair_key(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

fn point_id(a: u32, b: u32, feature: u32) -> u64 {
    let (lo, hi) = pair_key(a, b);
    ((lo as u64) << 44) | ((hi as u64) << 24) | (feature & 0x000F_FFFF) as u64
}

/// 打开/关闭探针（关闭会清空缓冲并一并关掉反事实档）。
pub fn enable(on: bool) {
    MODE.with(|c| {
        if on {
            c.fetch_or(ON, Ordering::Relaxed);
        } else {
            c.store(0, Ordering::Relaxed);
        }
    });
    if !on {
        BUF.with(|b| b.borrow_mut().clear());
        PEND.with(|p| {
            let mut q = p.borrow_mut();
            q.0.clear();
            q.1.clear();
        });
        LOC.with(|l| *l.borrow_mut() = LocStats::default());
    }
}

/// 打开/关闭反事实预测趟（需先 `enable(true)`）。
pub fn counterfactual(on: bool) {
    MODE.with(|c| {
        if on {
            c.fetch_or(COUNTERFACTUAL, Ordering::Relaxed);
        } else {
            c.fetch_and(!COUNTERFACTUAL, Ordering::Relaxed);
        }
    });
}

/// 取出并清空采样。
pub fn take() -> Vec<Sample> {
    PEND.with(|p| {
        let mut q = p.borrow_mut();
        q.0.clear();
        q.1.clear();
    });
    BUF.with(|b| std::mem::take(&mut *b.borrow_mut()))
}

/// 探针档位（每子步读一次）。
pub(crate) fn mode() -> u8 {
    MODE.with(|c| c.load(Ordering::Relaxed))
}

/// 位置判定累计（`enable(false)` 时清零；`take()` 不清，便于取样后读取）。
pub fn loc_stats() -> LocStats {
    LOC.with(|l| *l.borrow())
}

/// 点身份 = (体对, 特征 ID 低 20 位)——刻意**不含几何**：
/// "新点"问的是"这个特征是否第一次出现在本 tick"，不是"它挪了多少"。
fn collect(manifolds: &[Manifold], skin: f32) -> Vec<Point> {
    let mut ids: BTreeSet<(u64, bool)> = BTreeSet::new();
    for m in manifolds {
        let pts: &ContactPoints = &m.points;
        for p in pts.iter() {
            let id = point_id(m.a, m.b, p.feature);
            ids.insert((id, p.depth <= -skin));
        }
    }
    ids.into_iter()
        .map(|(id, speculative)| Point { id, speculative })
        .collect()
}

/// tick 首子步的点落表（位置判定用）。
fn note_first(tick: u64, manifolds: &[Manifold]) {
    FIRST.with(|f| {
        let mut f = f.borrow_mut();
        let t = f.get_or_insert_with(|| TickFirst {
            tick,
            by_pair: HashMap::new(),
        });
        t.tick = tick;
        t.by_pair.clear();
        for m in manifolds {
            let v = t.by_pair.entry(pair_key(m.a, m.b)).or_default();
            for p in m.points.iter() {
                v.push((point_id(m.a, m.b, p.feature), p.point));
            }
        }
    });
}

/// 后续子步的每一点按"首子步有没有同位置的点"归类。
fn classify(tick: u64, manifolds: &[Manifold], skin: f32) {
    let r = (skin * 2.0).max(0.01);
    FIRST.with(|f| {
        let f = f.borrow();
        let Some(f) = f.as_ref().filter(|f| f.tick == tick) else {
            return;
        };
        LOC.with(|l| {
            let mut l = l.borrow_mut();
            for m in manifolds {
                for p in m.points.iter() {
                    classify_point(&mut l, f, m.a, m.b, p, r);
                }
            }
        });
    });
}

fn classify_point(l: &mut LocStats, first: &TickFirst, a: u32, b: u32, p: &ContactPoint, r: f32) {
    let Some(v) = first.by_pair.get(&pair_key(a, b)) else {
        l.new_pair += 1;
        return;
    };
    if v.iter().any(|(fid, _)| *fid == point_id(a, b, p.feature)) {
        return; // 同特征 = 不是新点
    }
    if v.iter().any(|(_, p0)| (p.point - *p0).length() <= r) {
        l.same_loc += 1;
    } else {
        l.new_loc += 1;
    }
}

/// 反事实档的预测点与候选对。
fn note_predictions(manifolds: &[Manifold], skin: f32, pairs: &[(u32, u32)]) {
    let pts = collect(manifolds, skin);
    PEND.with(|p| {
        let mut q = p.borrow_mut();
        q.0 = pts;
        q.1.clear();
        q.1.extend_from_slice(pairs);
    });
}

/// **反事实预测趟**：同一批候选对、同一速度，用整 tick 视野再跑一趟窄相。
///
/// 结果**不喂解算器** ⇒ 动力学与不开探针时逐位相同。由 `narrow_pass::run` 在
/// **与真窄相同点（解算前）**调用——时机挪到子步末会让读数虚高（见该文件注）。
pub(crate) fn predicted_pass(w: &mut World, pairs: &[(u32, u32)]) {
    let mode = mode();
    if mode & COUNTERFACTUAL == 0 {
        return;
    }
    let skin = w.config.contact_skin;
    let mut predicted = Vec::new();
    w.narrow.set_predict_dt(w.config.dt);
    w.narrow.collide(
        &w.bodies,
        pairs,
        w.terrain.slice(),
        &w.providers,
        &mut predicted,
        w.jobs.as_ref(),
    );
    w.narrow.set_predict_dt(0.0);
    note_predictions(&predicted, skin, pairs);
}

/// 每个子步末由 `World::step` 调用（探针关时只多一次原子读 + 一次调用）。
pub(crate) fn on_substep(w: &mut World, substep: u32) {
    let mode = mode();
    if mode & ON == 0 {
        return;
    }
    let skin = w.config.contact_skin;
    let tick = w.tick;
    if substep == 0 {
        note_first(tick, &w.manifolds);
    } else {
        classify(tick, &w.manifolds, skin);
    }
    let points = collect(&w.manifolds, skin);
    let (predicted, pairs) = PEND.with(|p| {
        let mut q = p.borrow_mut();
        (std::mem::take(&mut q.0), std::mem::take(&mut q.1))
    });
    BUF.with(|buf| {
        let mut b = buf.borrow_mut();
        if b.len() < 100_000 {
            b.push(Sample {
                tick,
                substep,
                points,
                predicted,
                pairs,
            });
        }
    });
}
