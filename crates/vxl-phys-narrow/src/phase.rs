//! phase：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 默认窄相（SAT + 解析球 + 高度场采样）。工作缓冲全程复用（无逐步分配）。
/**
 * 每对求解用的 scratch 与世界几何缓存（2026-09-30 从 `DefaultNarrowPhase` 收出来）。
 *
 * 为什么单独成结构：原来 34 个字段把「仓库 / 配置 / 每对 scratch / 诊断计数」混在一个类型上，
 * 62 个方法全挂在它身上 ⇒ 门按「字段 + 方法双超」判它是上帝对象（登记在 `god.gate.json`）。
 * 收出 scratch 是那一刀的第一步：**纯机械搬移，无任何判定变化**（四哈希逐位不变是验收判据）。
 * 留在宿主上的只有：三个仓库、借用缓冲、三条配置、以及本结构 + 诊断计数。
 */
#[derive(Clone, Default)]
pub(crate) struct PairWorkspace {
    /// 本对的充气量（scratch）：由 SAT 预筛处按 (a, b, n) 计算，`clip` 的逐点过滤共用。
    pub(crate) inflate: f32,
    pub(crate) polys: Vec<ConvexPolytope>,
    pub(crate) poly_index: HashMap<u64, usize>,
    pub(crate) poly_a: WorldPoly,
    pub(crate) poly_b: WorldPoly,
    /// 世界多面体填充缓存（T3 快路径）：pair 按 (a,b) 排序 ⇒ 同一体的对连续，
    /// 单条缓存即可让每体每帧只填一次（大场景实测同一体每帧被填 ~50 次）。
    /// 纯函数（poly, pos, rot）→ 命中即跳过 33 次旋转；逐位一致。
    pub(crate) cached_a: (u32, u64),
    pub(crate) cached_b: (u32, u64),
    pub(crate) axes: Vec<Vec3>,
    pub(crate) clip_in: Vec<(Vec3, u32)>,
    pub(crate) clip_out: Vec<(Vec3, u32)>,
    pub(crate) cand: Vec<ContactPoint>,
    /// select_contacts 去重取点 scratch（T3：免每接触一次堆分配）。
    pub(crate) kept_buf: Vec<ContactPoint>,
    /// 盒对 SAT 快路径参数（half, 世界中心）：两 poly 均为盒时用 extents
    /// 投影公式（O(1)/体/轴）替代逐顶点 min/max（24 点/体/轴）。由盒-盒
    /// 分支设置；圆柱等其它形状为 None（走通用逐顶点路径）。
    pub(crate) box_a: Option<(Vec3, Vec3)>,
    pub(crate) box_b: Option<(Vec3, Vec3)>,
    /// 盒对专属体轴（T3 专用路径）：由 `(rot, half)` 直生（每体 3 次旋转），
    /// 与预筛共用；置位时 SAT/clip 走免填充路径，其余形状为 None。
    pub(crate) box_axes_a: Option<[Vec3; 3]>,
    pub(crate) box_axes_b: Option<[Vec3; 3]>,
    /// 体轴缓存（T3）：键 = (体号, 姿态指纹)。盒对路径原先**每对**都重算两体的
    /// **外壳世界点缓存**（两侧各一份）：键 = (体号, 姿态指纹)，与 `cached_ax_*`
    /// 同机制 ⇒ 同体同帧多对时只做一次 O(n) 世界变换（顶点采样/EPA 细化共用）。
    pub(crate) hull_pts: [Vec<Vec3>; 2],
    pub(crate) cached_hull: [(u32, u64); 2],
    /// 3 次旋转（同体在 (a,b) 序下连续出现，与 `cached_a/cached_b` 同机制）
    /// ⇒ 纯函数、命中即同值 ⇒ **逐位透明**。两槽分别给对的两个侧别。
    pub(crate) cached_ax_a: (u32, u64, [Vec3; 3]),
    pub(crate) cached_ax_b: (u32, u64, [Vec3; 3]),
    /// 裁剪顶点 scratch（参考面顶点）：通用与专用路径共用同一裁剪实现，
    /// 避免两份易漂移的裁剪代码。
    pub(crate) ref_v: Vec<Vec3>,
    /// 上一帧产出的流形数（下一帧并行块输出缓冲的容量提示）。纯性能提示：
    /// 不参与任何判定，故与确定性无关。
    pub(crate) out_hint: usize,
}

/// 诊断计数器（零开销，仅整数自增）；从 `DefaultNarrowPhase` 收出，见 `PairWorkspace` 头注。
#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeCounters {
    /// 诊断计数器（**零开销**，仅整数自增）：裁剪调用数 / 内层顶点迭代总数 /
    /// 交点插值总数 / 进 `select_contacts` 前的候选点数。用途：用"每次调用的
    /// 迭代数"反推成本落在**固定开销**（面选择 + 装配 + 归约）还是**内层行走**
    /// ——计时探针在 1361 次/步下自身就要 ~200 µs/步，会淹没被测段。
    pub clip_calls: u64,
    pub clip_iters: u64,
    pub clip_xings: u64,
    pub cand_pts: u64,
    /// 观测到的**最大裁剪多边形长度**（`clip_in` 峰值）。用途：为"把
    /// `clip_in`/`clip_out` 换成定长数组"提供**实测上界**——盒对快路径理论上界
    /// 是 8（4 顶点入射面 + 4 次半平面裁剪，每次凸多边形裁剪 ≤ m+1），但通用
    /// 路径的面顶点数无小常数上界（圆柱面 = n 边形、凸包面可达数十顶点），
    /// 只有实测峰值才能支撑"定长 16 是否安全"的判断。
    pub clip_max: u64,
    /// **窄相四段剖析（2026-10-01 第一刀）**：把"固定开销 vs 内层行走"的归因补齐到四段
    /// （多面体填充 / SAT / 裁剪 / 特征+装配）——前两段原先没有计数：
    /// - `sat_pairs` / `sat_pairs_box`：进 SAT 的对数 / 其中走**盒对快路径**（轴由体轴直生、
    ///   **不填多面体**）的对数 ⇒ 盒对占比 = 快路径覆盖率；
    /// - `poly_fills` / `poly_fill_verts`：世界多面体**未命中**填充次数 / 填充顶点总数
    ///   （圆柱/圆锥等通用路径的固定开销来源；盒对恒 0）；
    /// - `hull_fills` / `hull_fill_verts`：外壳世界点缓存未命中填充（GJK 路径）。
    ///
    /// 判读：`poly_fill_verts/步` 与 `clip_iters/步` 同尺比 ⇒ 填充 vs 裁剪谁在吃时间；
    /// 二者之和远小于 `narrowphase_us` ⇒ 大头在**每对固定开销**（面选择/装配/归约）。
    pub sat_pairs: u64,
    pub sat_pairs_box: u64,
    pub poly_fills: u64,
    pub poly_fill_verts: u64,
    pub hull_fills: u64,
    pub hull_fill_verts: u64,
    /// **provider 分派段（2026-10-01 第二刀）**：trimesh 场景的窄相时间在"体×提供者"这条路上，
    /// 这里把它拆成两类：
    /// - `prov_pairs`：进 `provider_shape_contacts` 分派的对数；
    /// - `prov_bulk`：**一次解析查询解决整个形状**的次数（盒 `contacts_box` / 球 `contacts_sphere`）；
    /// - `prov_samples`：**逐样本查询**总数（外壳顶点 / 胶囊沿轴球 / 圆柱圆锥环点与顶点）。
    ///
    /// 判读：`prov_samples/步` 与 `prov_pairs/步` 之比 ⇒ 平均每对样本数；两类查询的成本再与
    /// `narrowphase_us` 对照（提供者侧的桶/三角搜索在 `vxl-phys-terrain` 内，不属本计数器
    /// ——若两步都对不上，下一刀进提供者实现）。
    pub prov_pairs: u64,
    pub prov_bulk: u64,
    pub prov_samples: u64,
}

#[derive(Clone)]
pub struct DefaultNarrowPhase {
    /// 凸体外壳仓库（多边形域；点云注册后由 shape 引用）。**`Arc` = 并行 clone 的只读面**，理由与读数见 `PERF-REVIEW-2026-09-27.md` §1.4。
    pub(crate) hulls: std::sync::Arc<HullStore>,
    /// 复合体仓库（子形状表；由 `Shape::Compound { compound, .. }` 引用）。`Arc` 理由同上。
    pub(crate) compounds: std::sync::Arc<CompoundStore>,
    /// 三角网仓库（薄壳：布片/薄板/碎片；支持矩阵见 `docs/SURVEY-SHAPE-SUPPORT-MATRIX.md`）。`Arc` 理由同上。
    pub(crate) meshes: std::sync::Arc<MeshStore>,
    /// 子形状表 scratch（`kids_take`/`kids_put` 借出，避开 `&self`/`&mut self` 借用冲突）。
    pub(crate) kids_buf: Vec<CompoundChild>,
    pub(crate) skin: f32,
    /// **速度充气视野的预测时长**（s；0 = 不预测 ＝ 现行行为）。由 `World` 每子步设为
    /// **检测间隔**（每子步检测时为 `dt`、每 tick 检测时为整 tick）：窄相的接受判据
    /// 从 `sep ≤ skin` 放宽为 `sep ≤ skin + max(0, 接近速度)·predict_dt`，使
    /// "**下一次检测之前会碰上的接触**"提前成流形——求解器的 spec 项
    /// （`sep·inv_dt`）负责把逼近平滑拦停，不需要额外机制。
    ///
    /// 依据（`EXPERIMENTS.md` 末节 L/M）：检测每步一次时塔崩（KE 30 万）的真实机理是
    /// **逼近中的接触晚生**；把 skin 从 0.01 加到 0.02/0.04/0.08 即可让 KE 降到
    /// 2 386/1 992/1 271 且 y 带完整 ⇒ 视野不足而非语义不可行。
    /// `0` 时整条路径逐位不变（含三哈希）。
    pub(crate) predict_dt: f32,
    /// 接触点空间去重最小间距（m）：2×skin，且 ≥ 1 cm。
    pub(crate) min_point_sep: f32,
    /// 每对求解的 scratch/缓存（见 `PairWorkspace`）。
    pub(crate) ws: PairWorkspace,
    /// 诊断计数器（见 `ProbeCounters`）。
    pub probe: ProbeCounters,
}

impl DefaultNarrowPhase {
    /// 诊断读数（字段语义与判读见 `ProbeCounters`）。
    pub fn probe_stats(&self) -> ProbeCounters {
        self.probe
    }
}

/// 并行分块的输出槽预分配（T3 结构项②的第一片）：8B 实测对/流形 ≈ 15:1；旧实现每块
/// 从空 Vec 逐次增长（~8 次重分配 + memcpy）、最终拼接再搬一遍 ⇒ 按下界预留即免掉这段。
///
/// **为什么槽位是元组**：`collide` 的并行档每块 `clone` 一份独立计数器（见
/// `ProbeCounters::merge`）——不回流则并行档的诊断读数恒为 0。槽位顺路把它带回宿主。
///
/// 放在**模块级**（不是 `DefaultNarrowPhase` 的方法）：类型账对它的方法数已登记为债务
/// （只准减），加方法会顶红。
pub(crate) fn chunk_outs(n_chunks: usize, out_hint: usize) -> Vec<(Vec<Manifold>, ProbeCounters)> {
    let cap = out_hint / n_chunks + 8;
    (0..n_chunks)
        .map(|_| (Vec::with_capacity(cap), ProbeCounters::default()))
        .collect()
}

impl ProbeCounters {
    /// **并行分块回流**：`collide` 的并行档每块 `clone` 一份独立计数器（见 `entry.rs`）——
    /// 不回流的话，**并行档读出来恒为全 0**（2026-10-04 实测：8B 场景 8 线程读 0、
    /// 同一份码单线程读 238 442 次裁剪/步）。⇒ 每块跑完必须把它的计数加回宿主。
    ///
    /// `clip_max` 是**峰值**（取 max），其余是**累计量**（相加）——两者语义不同，别一把加。
    pub(crate) fn merge(&mut self, o: &Self) {
        self.clip_calls += o.clip_calls;
        self.clip_iters += o.clip_iters;
        self.clip_xings += o.clip_xings;
        self.cand_pts += o.cand_pts;
        self.sat_pairs += o.sat_pairs;
        self.sat_pairs_box += o.sat_pairs_box;
        self.poly_fills += o.poly_fills;
        self.poly_fill_verts += o.poly_fill_verts;
        self.hull_fills += o.hull_fills;
        self.hull_fill_verts += o.hull_fill_verts;
        self.prov_pairs += o.prov_pairs;
        self.prov_bulk += o.prov_bulk;
        self.prov_samples += o.prov_samples;
        self.clip_max = self.clip_max.max(o.clip_max);
    }
}
