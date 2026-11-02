//! tier_phase：求解**卡上档**的编排（`PLAN-gpu.md` §30 B 片：B1+B3+B4 接进 `World` 的求解位置）。
//!
//! **留什么在主机、搬什么上卡**（这是 §3.23 读数逼出来的分工）：
//! - 宽相/窄相**全留在主机**（卡上窄相单对 155–254 ns vs 主机 40 ns，搬上去是负优化）；
//! - 分岛 + 休眠**留在主机**（`sleep_pass` 是岛级唤醒/入睡语义的唯一出处，绕开它 M1 的
//!   "入睡数"就不再是同一个量）；
//! - **构建**（力臂、有效质量、软接触 rhs/cfm）与**解算**（Jacobi 扫）上卡。
//!
//! **与默认档不同的三处口径**（都写进 `PLAN-gpu.md` B6，不在本文件里辩护）：
//! ① 跨接触是 Jacobi（同一次扫掠内所有接触读同一份速度表），默认档是顺序 GS；
//! ② **无暖启动**（冲量表每次求解归零）、无锚点回收 ⇒ 深度取窄相裁剪点的原值；
//! ③ 切向基由法线现算（`tangents`），不认各向异性摩擦基与传送带。
//! 因此本档**只做显式 opt-in**，默认路径一行都不走这里（金样与三哈希逐位不变）。
use crate::{ImpulseSolver, SolverParams};
use vxl_phys_core::solve_tier::{self as st, SolveTierBackend};
use vxl_phys_core::{BodySet, JobSystem, Material, PhysConfig, Vec3};
use vxl_phys_narrow::Manifold;

/// 一次卡上求解趟的上下文（形参棘轮 >7 ⇒ 收成一个结构）。
pub struct TierCtx<'a> {
    pub manifolds: &'a [Manifold],
    pub config: &'a PhysConfig,
    pub dt: f32,
    pub jobs: &'a dyn JobSystem,
    /// `true` = 无偏置趟（与 CPU `solve_unbiased` 同位：去穿透偏置归零）。
    pub cleanup: bool,
    /// Jacobi 扫描轮数（一次提交内的 dispatch 对数）。
    pub sweeps: u32,
}

/// 一趟的**分项读数**（微秒 + 规模）。门面把它累进自己的快照——"每 tick 多出来的那几毫秒
/// 在哪一项"是这条线唯一还没定论的问题（同 `NarrowTierStats` 的理由）。
#[derive(Default, Clone, Copy, Debug)]
pub struct TierBreakdown {
    /// 打包体表 + 速度表 + 接触表（O(体数 + 接触数)）。
    pub pack_us: u64,
    /// CSR 计数排序（主机侧，O(接触数)）。
    pub csr_us: u64,
    /// 后端一趟：差量打包 + `write_buffer` + 派发 + **速度回读**（同步往返在这一项里）。
    pub run_us: u64,
    /// 回写速度到 `BodySet`（只写清醒动体）。
    pub writeback_us: u64,
    /// 分岛 + 休眠（仍走主机 `sleep_pass`）。
    pub sleep_us: u64,
    /// 趟数与规模（折算每接触/每体成本用）。
    pub calls: u64,
    pub contacts_sum: u64,
    pub bodies_sum: u64,
}

/// 卡上软接触的标量：**从 CPU 同一个 `SolverParams` 导出**（别在这儿重算 ω/ζ 公式）。
pub(crate) fn tier_params(
    cfg: &PhysConfig,
    dt: f32,
    cleanup: bool,
    sweeps: u32,
) -> st::SolveTierParams {
    let sp = SolverParams::from_config(cfg, dt);
    st::SolveTierParams {
        dt,
        inv_dt: sp.inv_dt,
        erp_dyn: sp.erp_inv_dt_dyn,
        erp_static: sp.erp_inv_dt_static,
        cfm_dyn: sp.cfm_dyn,
        cfm_static: sp.cfm_static,
        slop: sp.slop,
        max_corr: sp.max_corr,
        e_threshold: cfg.restitution_threshold,
        // 无偏置趟 = 去掉 erp 偏置（`solve_unbiased` 的 `max_corr = 0` 同义），保留 speculative 项。
        bias: if cleanup { 0.0 } else { 1.0 },
        sweeps,
    }
}

/// 体材质（与 `constraint::friction::material_of` 同式；那边是模块私有，这里不跨模块开口子）。
fn material_of(bodies: &BodySet, i: usize) -> Material {
    bodies
        .material
        .get(i)
        .and_then(|&id| bodies.materials.get(id as usize))
        .copied()
        .unwrap_or_default()
}

impl ImpulseSolver {
    /// **卡上档的一次求解**：打包 → 后端 → 回写 → 分岛/休眠。
    ///
    /// 失败（超容量 / 无适配器 / 核跑不通）⇒ `Err`，**一个字的速度都没写回**⇒ 调用方整趟走 CPU。
    pub fn solve_phase_tier(
        &mut self,
        bodies: &mut BodySet,
        ctx: &TierCtx,
        tier: &dyn SolveTierBackend,
    ) -> Result<TierBreakdown, String> {
        let p = tier_params(ctx.config, ctx.dt, ctx.cleanup, ctx.sweeps);
        let t0 = vxl_phys_core::probe::start();
        let (bd, vel) = st::pack_bodies(bodies);
        let n_b = bodies.len();
        let mut ct: Vec<f32> = Vec::with_capacity(ctx.manifolds.len() * st::CT_WORDS * 2);
        for m in ctx.manifolds {
            let (a, b) = (m.a as usize, m.b as usize);
            // **参与求解的两侧判据 = 主机岛规则的逐对写法**：主机只解"被清醒动体激活的岛"里的流形
            // （`fill_island_buckets`），双睡体的接触**根本不被构建**。这里若把它打包上卡，两边质量
            // 都是 0（睡眠体按无限质量打包，见 `solve_tier::pack_bodies`）⇒ 有效质量退化 ⇒ 护栏把
            // 每一趟整趟判死 ⇒ 堆一旦入睡档就再也跑不了（§3.25 实测：96 扫时 24/24 全回退）。
            let solves =
                |i: usize| (bodies.awake[i] && bodies.is_dynamic(i)) || bodies.is_kinematic(i);
            if !solves(a) && !solves(b) {
                continue;
            }
            let (mu, e) = Material::combine(&material_of(bodies, a), &material_of(bodies, b));
            for cp in &m.points {
                st::pack_contact(
                    &mut ct,
                    &st::ContactRow {
                        a: m.a,
                        b: m.b,
                        point: cp.point,
                        normal: m.normal,
                        depth: cp.depth,
                        mu,
                        e,
                    },
                );
            }
        }
        let d_pack = vxl_phys_core::probe::us(t0);
        let t1 = vxl_phys_core::probe::start();
        let (off, items) = st::build_csr(&ct, n_b);
        let d_csr = vxl_phys_core::probe::us(t1);
        let t2 = vxl_phys_core::probe::start();
        let out = tier.solve_run(&st::SolveTierIn {
            bd: &bd,
            vel: &vel,
            ct: &ct,
            off: &off,
            items: &items,
            params: p,
        })?;
        let d_run = vxl_phys_core::probe::us(t2);
        if out.len() != n_b * st::VEL_WORDS {
            return Err(format!(
                "后端速度表长度不符：{}（应为 {}）",
                out.len(),
                n_b * st::VEL_WORDS
            ));
        }
        // ⚠️ **非有限值一律整趟作废**（与 `vxl-phys::world_step::nan_guard` 同族契约）：NaN/inf 一旦
        // 写进 `BodySet`，位置积分会把它扩散到姿态与之后所有相位，`deep`/`awake`/沉降这些判据全部
        // 失真（§3.25 实测：`sweeps=24` 那趟的沉降读数直接变 NaN）。宁可回退 CPU，不许污染世界。
        if let Some(i) = out.iter().position(|v| !v.is_finite()) {
            return Err(format!(
                "后端速度表第 {i} 字非有限（{}）⇒ 整趟回退 CPU",
                out[i]
            ));
        }
        let t3 = vxl_phys_core::probe::start();
        for i in 0..n_b {
            if !bodies.awake[i] || !bodies.is_dynamic(i) {
                continue;
            }
            let o = i * st::VEL_WORDS;
            bodies.linvel[i] = Vec3::new(out[o], out[o + 1], out[o + 2]);
            bodies.set_angvel_raw(i, Vec3::new(out[o + 3], out[o + 4], out[o + 5]));
        }
        let d_wb = vxl_phys_core::probe::us(t3);
        // 分岛 + 休眠（**照旧走主机**：岛级唤醒语义与入睡计数是本仓稳定性的判据所在）。
        let t4 = vxl_phys_core::probe::start();
        let mut parent = self.build_union_find(bodies, ctx.manifolds);
        let (pool, used) = self.fill_island_buckets(bodies, ctx.manifolds, &mut parent);
        self.parent = parent;
        self.island_count = pool[..used].len();
        self.island_diag.manifolds = ctx.manifolds.len() as u32;
        self.island_diag.islands = used as u32;
        self.sleep_pass(
            bodies,
            &pool[..used],
            ctx.manifolds,
            ctx.config,
            ctx.dt,
            ctx.cleanup,
        );
        self.island_pool = pool;
        let d_sleep = vxl_phys_core::probe::us(t4);
        Ok(TierBreakdown {
            pack_us: d_pack,
            csr_us: d_csr,
            run_us: d_run,
            writeback_us: d_wb,
            sleep_us: d_sleep,
            calls: 1,
            contacts_sum: (ct.len() / st::CT_WORDS) as u64,
            bodies_sum: n_b as u64,
        })
    }
}

#[cfg(test)]
mod tests {
    // glob 门对新文件零基线 => 显式导入（含测试模块）。
    use super::{SolveTierBackend, TierCtx};
    use crate::{BodySet, ImpulseSolver, PhysConfig, Vec3};
    use vxl_phys_core::solve_tier::{self as st};
    use vxl_phys_core::SerialJobSystem;
    use vxl_phys_narrow::{ContactPoint, Manifold};

    /// 假后端：把速度表写成给定值（用来证明"非有限值整趟作废"这条锁真的会红）。
    struct Fake {
        fill: f32,
    }

    impl SolveTierBackend for Fake {
        fn solve_run(&self, inp: &st::SolveTierIn) -> Result<Vec<f32>, String> {
            Ok(vec![self.fill; inp.bd.len() / st::BD_WORDS * st::VEL_WORDS])
        }
    }

    fn one_box() -> (BodySet, Vec<Manifold>, PhysConfig) {
        let mut b = BodySet::new();
        b.push_dynamic(
            vxl_phys_core::Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 1.0, 0.0),
            vxl_phys_core::Quat::IDENTITY,
            1.0,
        );
        let g = b.push_static(
            vxl_phys_core::Shape::Box {
                half: Vec3::new(10.0, 0.5, 10.0),
            },
            Vec3::ZERO,
            vxl_phys_core::Quat::IDENTITY,
        );
        let m = Manifold {
            a: 0,
            b: g,
            normal: Vec3::new(0.0, 1.0, 0.0),
            points: [ContactPoint {
                point: Vec3::new(0.0, 0.5, 0.0),
                depth: 0.01,
                feature: 3,
            }]
            .into(),
        };
        (b, vec![m], PhysConfig::default())
    }

    #[test]
    fn nan_from_backend_never_reaches_bodies() {
        let (mut bodies, ms, cfg) = one_box();
        let before = bodies.linvel[0];
        let mut solver = ImpulseSolver::new(cfg.contact_skin);
        for fill in [f32::NAN, f32::INFINITY] {
            let ctx = TierCtx {
                manifolds: &ms,
                config: &cfg,
                dt: cfg.dt,
                jobs: &SerialJobSystem,
                cleanup: false,
                sweeps: 4,
            };
            let r = solver.solve_phase_tier(&mut bodies, &ctx, &Fake { fill });
            // 判据不是"返回了 Err"，而是 **Err 之后世界的速度一个字都没被改过**。
            // （写成本地变量再断言，不用 `panic!`/`expect`：那两条都顶棘轮。）
            let msg = match r {
                Ok(_) => String::new(),
                Err(e) => e,
            };
            assert!(!msg.is_empty(), "非有限速度必须判红（fill={fill}）");
            assert!(msg.contains("非有限"), "报错文案要说明原因：{msg}");
            assert_eq!(bodies.linvel[0], before, "回退时不许写脏体速度");
        }
    }

    #[test]
    fn finite_backend_writes_back() {
        // 同一把锁的反面：有限值必须真的落进 `BodySet`（否则上一条测试可以是"永远 Err"的假绿）。
        let (mut bodies, ms, cfg) = one_box();
        let mut solver = ImpulseSolver::new(cfg.contact_skin);
        let ctx = TierCtx {
            manifolds: &ms,
            config: &cfg,
            dt: cfg.dt,
            jobs: &SerialJobSystem,
            cleanup: false,
            sweeps: 4,
        };
        struct Fixed;
        impl SolveTierBackend for Fixed {
            fn solve_run(&self, inp: &st::SolveTierIn) -> Result<Vec<f32>, String> {
                let n = inp.bd.len() / st::BD_WORDS * st::VEL_WORDS;
                let mut v = vec![0.0f32; n];
                v[1] = -1.25; // 体 0 的 y 速度
                Ok(v)
            }
        }
        let ok = solver.solve_phase_tier(&mut bodies, &ctx, &Fixed).is_ok();
        assert!(ok, "有限速度表应当通过");
        assert_eq!(bodies.linvel[0].y, -1.25);
    }

    /// 假后端：只记录这趟真正打包了多少字，速度回零。
    #[derive(Default)]
    struct Counting {
        ct_words: std::sync::atomic::AtomicUsize,
    }

    impl SolveTierBackend for Counting {
        fn solve_run(&self, inp: &st::SolveTierIn) -> Result<Vec<f32>, String> {
            self.ct_words
                .store(inp.ct.len(), std::sync::atomic::Ordering::Relaxed);
            Ok(vec![0.0; inp.bd.len() / st::BD_WORDS * st::VEL_WORDS])
        }
    }

    #[test]
    fn asleep_and_static_contacts_are_not_packed() {
        // 岛规则的逐对写法必须真的体现在接触数上：双睡体、双静态都不打包；醒×睡、醒×静 要打包。
        // （这条锁防的是 §3.25 那次事故：把双睡体接触打包上卡 ⇒ 有效质量退化 ⇒ 每趟整趟判死。）
        let cfg = PhysConfig::default();
        let mut bodies = BodySet::new();
        let half = Vec3::splat(0.5);
        for k in 0..3 {
            bodies.push_dynamic(
                vxl_phys_core::Shape::Box { half },
                Vec3::new(k as f32, 1.0, 0.0),
                vxl_phys_core::Quat::IDENTITY,
                1.0,
            );
        }
        let ground = bodies.push_static(
            vxl_phys_core::Shape::Box {
                half: Vec3::new(10.0, 0.5, 10.0),
            },
            Vec3::ZERO,
            vxl_phys_core::Quat::IDENTITY,
        );
        bodies.awake[1] = false;
        bodies.awake[2] = false;
        let mk = |a: u32, b: u32| Manifold {
            a,
            b,
            normal: Vec3::Y,
            points: [ContactPoint {
                point: Vec3::ZERO,
                depth: 0.01,
                feature: 1,
            }]
            .into(),
        };
        let ms = vec![
            mk(0, 1),      // 醒 × 睡 ⇒ 进
            mk(1, 2),      // 睡 × 睡 ⇒ 不进（主机也不解）
            mk(0, ground), // 醒 × 静 ⇒ 进
            mk(ground, ground),
        ];
        let tier = Counting::default();
        let mut solver = ImpulseSolver::new(cfg.contact_skin);
        let ctx = TierCtx {
            manifolds: &ms,
            config: &cfg,
            dt: cfg.dt,
            jobs: &SerialJobSystem,
            cleanup: false,
            sweeps: 4,
        };
        let ok = solver.solve_phase_tier(&mut bodies, &ctx, &tier).is_ok();
        assert!(ok, "四条流形里两条有效，应当跑通");
        assert_eq!(
            tier.ct_words.load(std::sync::atomic::Ordering::Relaxed),
            2 * st::CT_WORDS,
            "只有 2 条接触该被打包"
        );
    }
}
