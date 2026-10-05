//! types：从 lib.rs 按域拆出（纯搬移，语义未改）。

/// **效应键**（`PLAN-COUPLING.md` §3.4 的 I3，本仓第一次落地；V2 片只登记两个
/// **消费体素格**的效应）。语义：一个键在一个体素域、一个 tick 内只允许**一条路径**施加；
/// 第二条路径 ⇒ fail-loud（`Err`），不静默二选一。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EffectKey {
    /// 破坏域提取（`extract_boxes` / `extract_sphere` / `extract_where` / `fracture_voronoi` 一族）。
    VoxelExtraction,
    /// 体素 → 网格的表示转换（`world_step/conversion.rs` 的转换窗口）。
    Conversion,
}

/// 转换请求被拒的原因（**一律 `Err`**：不静默跳过、不静默降级）。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConversionError {
    /// `PhysConfig::conversion.enabled == false`（默认档）。
    Disabled,
    /// 该 id 不是体素域（转换的源表示必须是有格可提取的体素体）。
    NotVoxel(u32),
    /// 同一域同一 tick 重复请求转换。
    Duplicate { provider: u32, tick: u64 },
    /// I3：同一域同一 tick 已被另一个消费型效应占用。
    EffectConflict {
        provider: u32,
        tick: u64,
        already: EffectKey,
    },
    /// 每 tick 事件预算（`ConversionConfig::max_events`）已满。
    EventBudget { tick: u64, max: usize },
}

/// 转换被**拒绝**的原因（如实登记；拒绝 = 保持源表示，不切换）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConversionReject {
    /// 提取出的零等值面为空（空体素域 / 无表面格）。
    EmptyMesh,
    /// 提取结果含越界索引的三角（**静默丢弃的显式化**：`PLAN-CONVERSION.md` §3.1 要求
    /// 提取器的判据断言"零丢弃"，这里把丢弃变成拒绝而不是悄悄少几个三角）。
    DegenerateMesh,
    /// 网格顶点数超 `ConversionConfig::max_vertices`（降级档属 V4，本片只拒绝）。
    VertexBudget,
    /// 影子对拍不通过（I5：流形摘要差超阈值 ⇒ 拒绝切换）。
    Reconciliation,
}

/// 一次转换窗口的**结果快照**（诊断与判据出口；`PLAN-CONVERSION.md` §4 的 I5/I6）。
///
/// 全部字段都是**当次实测值**，不是期望值——判据读它、不另算一份（仪器与被测同源）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConversionReport {
    /// 源体素域 id。
    pub provider: u32,
    /// 发起本次转换的 tick（契约卡：时间戳 = tick 末）。
    pub tick: u64,
    /// 提取出的影子网格规模（顶点 / 三角）。
    pub vertices: usize,
    pub tris: usize,
    /// 对拍：两侧的流形点数（源 provider vs 影子网格，同一批探针）。
    pub source_points: usize,
    pub shadow_points: usize,
    /// 对拍：两侧的最大穿透深度（m）。
    pub source_max_depth: f32,
    pub shadow_max_depth: f32,
    /// 对拍：法向分布一致性（逐探针法向点积的**最小值**；无探针时为 1.0）。
    pub min_normal_dot: f32,
    /// 是否提交了切换。
    pub committed: bool,
    /// 提交产物（`add_trimesh` 的 mesh id / 新动态体 id）。
    pub mesh: Option<u32>,
    pub body: Option<u32>,
    /// 被拒绝时的原因。
    pub rejected: Option<ConversionReject>,
}

/// **转换窗口簿记**（效应键账 + 上一 tick 的结果）。挂在 [`Providers`] 上而不是 `World`：
/// `world_struct.rs` 的成员位顶在 god 门棘轮（23 个成员、只准减）⇒ 新域状态一律收进域自身。
/// 账目按 tick 过滤（`mark` 时裁掉旧 tick）⇒ 体量 = **本 tick 的消费型效应数**，与运行时长无关。
#[derive(Default)]
pub(crate) struct ConversionBook {
    /// 本 tick 已被消费型效应占用的域：`(provider id, tick, key)`。
    pub(crate) effects: Vec<(u32, u64, EffectKey)>,
    /// 上一 tick 的转换结果（无转换 = `None`）。
    pub(crate) last: Option<ConversionReport>,
}
