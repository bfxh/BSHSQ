//! solve_tier：求解**卡上档**的门面侧（`PLAN-gpu.md` §30 B 片：B1+B3+B4 接进 `World` 的求解位置）。
//!
//! **这一片换掉的是什么**：默认档里 solve 占 tick 的 80%、其中**构建**占 54–69%（§3.8/§3.11）。
//! 本档把「构建 + 解算」交给卡（`vxl_phys_solver::tier_phase` 编排、`vxl_phys_gpu::solve_tier` 执行），
//! **宽相、窄相、分岛、休眠全部留在主机**（§3.23：卡上窄相单对慢 4–6×，搬上去是负优化）。
//!
//! **包一层 `SolverSlot` 而不是给 `World` 加字段**（同 `NarrowSlot` 的理由）：`world_struct.rs`
//! 加行即红、`World` 成员数顶在棘轮上 ⇒ 换字段类型不动行数与成员数；`Deref`/`DerefMut` 让
//! 既有的 `w.solver.<方法|字段>` 调用点（含 8 个 example）一行都不用改。
//!
//! ⚠️ **口径**（B6 另立质量判据，这里只把差别摊开）：卡上 = Jacobi + 无暖启动 + 无锚点回收，
//! 与默认档的顺序 GS + 暖启动**不同源** ⇒ 本档只能显式注册，未注册时一行都不走这里
//! （金样、`FINAL_HASH` 逐位不变）。
use crate::World;
use vxl_phys_core::solve_tier::SolveTierBackend;
use vxl_phys_solver::tier_phase::TierCtx;
use vxl_phys_solver::ImpulseSolver;

/// 卡上求解档的**分项读数**（µs 累计 + 规模 + 回退计数）。
///
/// 为什么要进生产代码：一趟 `solve_run` 里"上传/派发/回读"三项只差几行，从外面量不出来；
/// 而 B 路线的未决问题正是"整 tick 的钱在哪一项"（§3.22 就是被这个问题翻案的）。
#[derive(Default, Clone, Copy, Debug)]
pub struct SolveTierStats {
    /// 打包体表 + 速度表 + 接触表。
    pub pack_us: u64,
    /// CSR 计数排序（主机）。
    pub csr_us: u64,
    /// 后端一趟：差量打包 + 上传 + 派发 + **同步回读**。
    pub run_us: u64,
    /// 回写速度到 `BodySet`。
    pub writeback_us: u64,
    /// 分岛 + 休眠（主机 `sleep_pass`，本档没搬走它）。
    pub sleep_us: u64,
    /// 走了卡上档的趟数 / 回退 CPU 的趟数（含超容量、后端报错、未注册）。
    pub calls: u64,
    pub fallbacks: u64,
    /// 后端报错次数（`fallbacks` 里"注册了但没跑成"的那部分）。
    pub errors: u64,
    pub contacts_sum: u64,
    pub bodies_sum: u64,
}

/// **宿主求解器 + 可选卡上档**。`host` = `ImpulseSolver`（默认档，也是本档的回退目标）。
pub struct SolverSlot {
    pub host: ImpulseSolver,
    pub(crate) tier: Option<Box<dyn SolveTierBackend>>,
    /// Jacobi 扫描轮数（注册档时给定；`set_solve_sweeps` 可改）。
    pub(crate) sweeps: u32,
    pub(crate) stats: SolveTierStats,
    /// 最近一次后端报错（`None` = 没有；诊断用——**别把它当"没报错"的证据**，只看 `errors` 计数）。
    pub(crate) last_err: Option<String>,
}

impl SolverSlot {
    /// 注册/替换卡上求解档（`sweeps` = Jacobi 轮数）。传 `None` = 摘掉档、回到默认档。
    pub fn set_tier(&mut self, tier: Option<Box<dyn SolveTierBackend>>, sweeps: u32) {
        self.tier = tier;
        self.sweeps = sweeps.max(1);
        // 换档 = 常驻状态作废（后端自己管差量基线，但容量/适配器可能变了）⇒ 读数清零重算。
        self.stats = SolveTierStats::default();
        self.last_err = None;
    }

    /// 已注册的档名（`None` = 默认档）。
    pub fn tier_name(&self) -> Option<&str> {
        self.tier.as_deref().map(|t| t.tier_name())
    }

    /// 分项读数。
    pub fn tier_stats(&self) -> SolveTierStats {
        self.stats
    }

    /// 最近一次后端报错。
    pub fn tier_error(&self) -> Option<&str> {
        self.last_err.as_deref()
    }
}

impl From<ImpulseSolver> for SolverSlot {
    fn from(host: ImpulseSolver) -> Self {
        Self {
            host,
            tier: None,
            sweeps: 12,
            stats: SolveTierStats::default(),
            last_err: None,
        }
    }
}

impl std::ops::Deref for SolverSlot {
    type Target = ImpulseSolver;
    fn deref(&self) -> &ImpulseSolver {
        &self.host
    }
}

impl std::ops::DerefMut for SolverSlot {
    fn deref_mut(&mut self) -> &mut ImpulseSolver {
        &mut self.host
    }
}

impl World {
    /// **卡上求解一趟**：`true` = 本趟已由卡上档做完（速度已回写、分岛/休眠已跑）；
    /// `false` = 调用方走 CPU（未注册 / 超容量 / 后端报错 ⇒ **整趟**回退，不半途混用）。
    pub(crate) fn solve_tier_pass(&mut self, dt: f32, cleanup: bool) -> bool {
        let Some(tier) = self.solver.tier.take() else {
            self.solver.stats.fallbacks += 1;
            return false;
        };
        let sweeps = self.solver.sweeps;
        // `take` 出来再放回（同 `narrow_tier_pass`）：调用要可变借 `self.solver.host` 与 `self.bodies`，
        // 而 `tier` 是 `self.solver` 的字段 ⇒ 不先摘出来会把 `self.solver` 整份借住。
        let ctx = TierCtx {
            manifolds: &self.manifolds,
            config: &self.config,
            dt,
            jobs: self.jobs.as_ref(),
            cleanup,
            sweeps,
        };
        let r = self
            .solver
            .host
            .solve_phase_tier(&mut self.bodies, &ctx, tier.as_ref());
        self.solver.tier = Some(tier);
        match r {
            Ok(b) => {
                let s = &mut self.solver.stats;
                s.pack_us += b.pack_us;
                s.csr_us += b.csr_us;
                s.run_us += b.run_us;
                s.writeback_us += b.writeback_us;
                s.sleep_us += b.sleep_us;
                s.calls += 1;
                s.contacts_sum += b.contacts_sum;
                s.bodies_sum += b.bodies_sum;
                true
            }
            Err(e) => {
                let s = &mut self.solver.stats;
                s.fallbacks += 1;
                s.errors += 1;
                self.solver.last_err = Some(e);
                false
            }
        }
    }
}
