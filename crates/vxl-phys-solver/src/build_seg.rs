//! **约束构建内部分段**（诊断；`ISLAND_SEG_PROBE` 关时零成本）。
//!
//! 动机（`M1-RULING §3.11`）：构建占 solve 的 54–69%，但不知道钱花在哪。
//!
//! ⚠️ **单点内部拆不开**：一个点的构建约 140 ns，拆成四段后每段 ~40 ns，而 `Instant`
//! 在本机分辨率 **~100 ns** ⇒ 实测只能读到 11 ns/点（占 7%），其余被量化吃掉。
//! 所以本模块只量**流形级**两段：0 = 预处理（材质 / warm 槽查找 / 摩擦基 / 旋转矩阵 /
//! 锚点世界位置），1 = 点循环整体。点循环内部改用**消融**测：临时把 `match_warm_point`
//! 短路后点循环 100 → 97 ns/点 ⇒ 匹配只占 ~3%。
//!
//! 抽样：每 `SAMPLE_MOD` 个流形取 1 个（按 `m.a.is_multiple_of(SAMPLE_MOD)`，确定性、
//! 无计数器），只对样本流形读 2 次钟。累计**纳秒**（`probe::us` 会把亚微秒截断成 0）。
use std::cell::RefCell;

/// 抽样模数（每 N 个流形取 1 个）。
pub const SAMPLE_MOD: u32 = 16;

thread_local! {
    /// `[总, material_pair, friction_frame, 锚点世界, 点循环, 样本数]`。（1.99 连 `const { … }` 写法也判红，同 `contact_probe`。）
    #[allow(clippy::missing_const_for_thread_local)]
    static ACC: RefCell<[u64; 6]> = const { RefCell::new([0; 6]) };
}

/// 整次 `build_constraint` 的计时起点（调用方用；`None` = 未抽样）。
pub(crate) fn start_total(a: u32) -> Option<vxl_phys_core::probe::Stamp> {
    sampled(a).then(vxl_phys_core::probe::start)
}

/// **点循环计时器**（1 行插入 `build_constraint`；Drop 时记账）。
pub(crate) struct LoopGuard {
    t: Option<vxl_phys_core::probe::Stamp>,
}

impl LoopGuard {
    /// 起点在构造处（点循环之前），抽样判定按 `a.is_multiple_of(SAMPLE_MOD)`。
    pub(crate) fn new(a: u32) -> Self {
        Self {
            t: sampled(a).then(vxl_phys_core::probe::start),
        }
    }
}

/// 计时一个表达式（1 行插入；`seg` 为 false 时零成本）。
#[macro_export]
macro_rules! seg_time {
    ($seg:expr, $slot:expr, $e:expr) => {{
        let t = ($seg).then(vxl_phys_core::probe::start);
        let out = $e;
        $crate::build_seg::add($slot, t);
        out
    }};
}

impl Drop for LoopGuard {
    fn drop(&mut self) {
        add(4, self.t);
    }
}

/// 本流形是否抽样（`ISLAND_SEG_PROBE` 关 ⇒ 恒 false，一次比较）。
pub(crate) fn sampled(a: u32) -> bool {
    crate::ISLAND_SEG_PROBE && a.is_multiple_of(SAMPLE_MOD)
}

/// 记录一段（`None` = 未抽样，零成本）。
///
/// 槽位：0 = 整次构建（同时计样本流形数）、1 = `material_pair`、2 = `friction_frame + tangent_setup`、
/// 3 = 锚点世界位置、4 = 点循环；"其余" = 0 − (1+2+3+4)（入口 / warm 槽查找 / 2×quat→Mat3 / push）。
pub(crate) fn add(slot: usize, t: Option<vxl_phys_core::probe::Stamp>) {
    if let Some(t) = t {
        let ns = vxl_phys_core::probe::ns(t);
        ACC.with(|c| {
            let mut v = c.borrow_mut();
            v[slot] += ns;
            if slot == 0 {
                v[5] += 1;
            }
        });
    }
}

/// 读取并清零：`([总, material_pair, friction, 锚点世界, 点循环], 样本流形数)`。
pub fn take() -> ([u64; 5], u64) {
    ACC.with(|c| {
        let mut b = c.borrow_mut();
        let v = std::mem::take(&mut *b);
        ([v[0], v[1], v[2], v[3], v[4]], v[5])
    })
}
