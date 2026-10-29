//! **差分定位**（确定性产品化第一刀）：两条哈希记录从哪一 tick 开始不一致。
//!
//! 独立成文件：`lib.rs` 受 god 门**文件行数棘轮**（基线只准减）。

use crate::Recorder;

/// **首个发散点**（差分调试的载体）：两条记录从哪一 tick 开始不一致。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Divergence {
    /// 第一个**不再相同**的采样 tick。
    pub first_bad: u64,
    /// 它之前最后一个**仍然相同**的采样 tick（`None` = 第一条就不同）。
    pub last_good: Option<u64>,
    /// 该 tick 两侧的哈希。
    pub left: u128,
    pub right: u128,
    /// 已经比过的采样点数（定位成本，用于诊断"是否只差在末尾"）。
    pub compared: usize,
}

impl Divergence {
    /// 人类可读的一行报告（进日志/进断言消息都用它，避免各处格式不一）。
    pub fn report(&self) -> String {
        match self.last_good {
            Some(g) => format!(
                "首个发散 tick={}（上一个一致 tick={}），比较了 {} 个采样；left={:#034x} right={:#034x}",
                self.first_bad, g, self.compared, self.left, self.right
            ),
            None => format!(
                "**第一条采样就不一致**（tick={}），比较了 {} 个采样；left={:#034x} right={:#034x}",
                self.first_bad, self.compared, self.left, self.right
            ),
        }
    }
}

impl Recorder {
    /// **差分定位**：按 tick **对齐**比较两条记录，返回首个发散点；全同则 `None`。
    ///
    /// 为什么按 tick 而不是按下标：两条记录的 `period` 可以不同（例如定位时一边调密），
    /// 若按下标比，period 一改就会把"采样不同步"误报成"物理发散"。
    pub fn divergence(&self, other: &Recorder) -> Option<Divergence> {
        let mut last_good = None;
        let mut compared = 0usize;
        let mut i = 0usize;
        let mut j = 0usize;
        while i < self.hashes.len() && j < other.hashes.len() {
            let (ti, hi) = self.hashes[i];
            let (tj, hj) = other.hashes[j];
            if ti < tj {
                i += 1;
            } else if tj < ti {
                j += 1;
            } else {
                compared += 1;
                if hi != hj {
                    return Some(Divergence {
                        first_bad: ti,
                        last_good,
                        left: hi,
                        right: hj,
                        compared,
                    });
                }
                last_good = Some(ti);
                i += 1;
                j += 1;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use crate::Recorder;
    /// 同一轨迹 ⇒ 无发散。
    #[test]
    fn divergence_none_for_identical() {
        let mut a = Recorder::new(60);
        let mut b = Recorder::new(60);
        for t in 1..=240u64 {
            a.observe(t, (t as u128) * 7);
            b.observe(t, (t as u128) * 7);
        }
        assert!(a.divergence(&b).is_none());
    }

    /// 第 180 tick 起不同 ⇒ 精确定位到 180，且上一个一致点是 120。
    #[test]
    fn divergence_locates_first_bad_tick() {
        let mut a = Recorder::new(60);
        let mut b = Recorder::new(60);
        for t in 1..=300u64 {
            a.observe(t, t as u128);
            let h = if t >= 180 { t as u128 + 1 } else { t as u128 };
            b.observe(t, h);
        }
        let Some(d) = a.divergence(&b) else {
            assert!(false, "应当检出");
            return;
        };
        assert_eq!(d.first_bad, 180);
        assert_eq!(d.last_good, Some(120));
        println!("[差分] {}", d.report());
    }

    /// **period 不同不能误报**（按 tick 对齐）：同一物理、一边每 60、一边每 30。
    #[test]
    fn divergence_aligns_by_tick_not_index() {
        let mut a = Recorder::new(60);
        let mut b = Recorder::new(30);
        for t in 1..=240u64 {
            a.observe(t, (t as u128) % 97);
            b.observe(t, (t as u128) % 97);
        }
        assert!(a.divergence(&b).is_none(), "period 不同被误判成发散");
    }

    /// 第一条采样就不同 ⇒ `last_good = None`。
    #[test]
    fn divergence_at_first_sample() {
        let mut a = Recorder::new(60);
        let mut b = Recorder::new(60);
        a.observe(60, 1);
        b.observe(60, 2);
        let Some(d) = a.divergence(&b) else {
            assert!(false, "应当检出");
            return;
        };
        assert_eq!(d.first_bad, 60);
        assert_eq!(d.last_good, None);
    }
}
