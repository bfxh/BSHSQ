//! **弯曲约束族**（`cloth.rs` 的子模块）：二环对 + 注册长度 + XPBD 乘子**并成一组**。
//!
//! **为什么并组**：`ClothSheet` 是 god 门**成员棘轮**下的记录型结构（只准减），三个**同生共死**
//! 的 Vec 收成一个成员 ⇒ 腾出 2 个位给逐面反作用冲量 scratch（`ContactTuning` /
//! `BodyCoupling` / `SelfContacts` / `damage::Damage` 同款先例）。
//!
//! 三个字段全是 `pub(crate)`（**不是**公开 API）⇒ 纯内部整理，行为逐位不变，判据 =
//! 既有布片测试全绿 + 三条冻结哈希不动。

/// 弯曲约束族（二环对的距离约束；`cloth.rs::project_bend` 消费）。
#[derive(Clone, Default)]
pub(crate) struct BendSet {
    /// 二环对（`[min, max]` 有序；插入序 = `bend_pairs` 的扫描序 ⇒ 确定性）。
    pub(crate) pairs: Vec<[u32; 2]>,
    /// 各对的注册长度（注册态即零应变态；**塑性会改它**，见 `cloth/plastic.rs`）。
    pub(crate) rest: Vec<f32>,
    /// XPBD 乘子（每子步清零、子步内按迭代累加）。
    pub(crate) lambda: Vec<f32>,
}

impl BendSet {
    /// 由二环对 + 注册长度构造（XPBD 乘子留空，`step` 里按长度补齐）。
    pub(crate) fn new(pairs: Vec<[u32; 2]>, rest: Vec<f32>) -> Self {
        Self {
            pairs,
            rest,
            lambda: Vec::new(),
        }
    }

    /// 供 `mark_torn` 之类的**双切片**消费点用（`(pairs, rest)`；避免在调用点写两次字段路径）。
    pub(crate) fn parts(&self) -> (&[[u32; 2]], &[f32]) {
        (&self.pairs, &self.rest)
    }
}
