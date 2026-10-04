//! solver_impl：从 lib.rs 按域拆出（纯搬移，语义未改）。
//!
//! `solve_phase` 是唯一的大分发：按相位拆成「参数 / 分岛 / 岛桶 / gather / 解算 /
//! scatter / warm 回写 / 休眠」八个 helper（纯搬移，探针位置与状态写入序逐条保持）。
use super::*;

// 休眠族拆成**子模块**（理由见 `solver_impl/sleep.rs` 头注：`solver/lib.rs` 只剩 0 行预算，
// crate 级新文件要加 `mod` 声明；子模块还能看见父模块的私有项）。
#[path = "solver_impl/sleep.rs"]
mod sleep;
impl ImpulseSolver {
    pub fn new(skin: f32) -> Self {
        Self {
            warm_slots: Vec::new(),
            warm_free: Vec::new(),
            warm_index: HashMap::new(),
            warm_stamp: 0,
            island_count: 0,
            match_dist: (skin * 4.0).max(0.02),
            sleep_resets: 0,
            wake_streak: Vec::new(),
            build_bufs: Vec::new(),
            warm_outs: Vec::new(),
            group_lv: Vec::new(),
            group_av: Vec::new(),
            group_iw: Vec::new(),
            group_im: Vec::new(),
            local_of: Vec::new(),
            last_phase_us: (0, 0, 0, 0),
            last_detail_us: [0; 4],
            last_points: (0, 0),
            island_diag: IslandDiag::default(),
            parent: Vec::new(),
            island_pool: Vec::new(),
        }
    }

    /// 求解 + 岛级休眠。调用方顺序：积分速度 → 检测 → 本函数 → 积分位置。
    /// 唤醒语义：岛整体睡/醒（Box2D 同族）——岛内任一成员被外部唤醒（用户冲量、
    /// 新接触带入的动体）即全岛唤醒；杜绝"醒体反复唤醒睡体"。
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
    ) {
        self.solve_phase(bodies, manifolds, config, dt, jobs, false);
    }

    /// **无偏置趟**（Rapier TGS-Soft 的末趟，`rhs_wo_bias` 同义）：位置积分之后
    /// 对同一批流形重解，去掉**去穿透偏置**（`max_corr = 0`），把"修正速度"从
    /// **最终速度**里移除——位置已由带偏置趟的积分推进，去穿透不受影响；最终速度
    /// 只留"不接近"（speculative）语义。切向漂移回拉**保留**（Rapier 的切向
    /// `rhs_wo_bias` = 材料点漂移率；实测归零会让 125 体留缝角点场景退化）。
    /// `config.stabilization_iterations == 0` 时本函数不应被调用。
    pub fn solve_unbiased(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
    ) {
        self.solve_phase(bodies, manifolds, config, dt, jobs, true);
    }

    /// 相位编排：参数 → 分岛 → 岛桶 → gather → 解算 → scatter → warm 回写 → 休眠。
    ///
    /// **四个计时点（`t_island`/`t_fill`/`t_solve`/`t_sleep`）必须留在本函数**——
    /// 诊断分界（`fill_us`/`island_build_us`/`scope_us`/`scatter_us`）依赖它们的相对位置
    /// （B24 踩过同型：`t_fill` 夹在"分岛"与"填桶"之间，合成一个函数就丢分界）。
    pub(crate) fn solve_phase(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        jobs: &dyn JobSystem,
        cleanup: bool,
    ) {
        let p = PhaseParams::from_config(config, dt, cleanup, self.match_dist);
        let threads = jobs.threads().max(1);
        // 计时走跨目标探针（wasm32 无时钟；原生不变）。
        let t_island = vxl_phys_core::probe::start();

        // 1) 并查集分岛（直接对流形；双静态对不入岛）。固定规则：小索引为根（确定性）。
        //    缓冲跨帧复用（`self.parent`），避免每帧 20 万级 alloc/fill。
        let mut parent = self.build_union_find(bodies, manifolds);

        // 2) 岛桶（见 `fill_island_buckets`）。诊断拆点（T4 第三刀）：`t_fill` 覆盖
        //    「建岛 + 按组 gather」——后者实测占求解相位约 24%（36000 体 × 约 41 ns），
        //    是唯一还串行的重活。⚠️ 并行化它**反而更慢**（见 `gather_groups` 处注）：
        //    该段与解算同属**访存带宽受限**。
        let t_fill = vxl_phys_core::probe::start();
        let (pool, islands_used) = self.fill_island_buckets(bodies, manifolds, &mut parent);
        let islands = &pool[..islands_used];
        self.parent = parent;
        self.island_count = islands.len();

        // 3) 清醒岛分组 + gather（§6 契约：组间写槽位不相交、组内 = 串行语义）。
        let awake: Vec<usize> = (0..islands.len()).collect();
        // 并行门槛：spawn ≈ 90µs/个（Windows 实测）；流形 < 4096 时并行不划算
        // （解算工作量 ≈ 1µs/接触/帧），走单组串行（数值路径不变）。
        let g_count = if threads <= 1 || manifolds.len() < 4096 {
            1
        } else {
            awake.len().min(threads).max(1)
        };
        let mut bufs = GroupBufs::take(self);
        let gr = self.gather_groups(bodies, islands, awake, &mut bufs, g_count);

        let d_fill = vxl_phys_core::probe::us(t_fill);
        let d_island_all = vxl_phys_core::probe::us(t_island);
        let d_island = d_island_all.saturating_sub(d_fill);
        self.island_diag.fill_us = d_fill;
        self.island_diag.island_build_us = d_island;
        let t_solve = vxl_phys_core::probe::start();

        // 4) 解算 → scatter → warm 回写（顺序 = 数据流序，不可换）。
        // **两段实测（2026-10-01）**：串行档下原 `scatter_us = d_solve − scope` 把
        // "解算 + 散射 + warm 回写"整体都算进 scatter（arena 场景全是单组串行 ⇒ 该列退化）。
        // 这里另测两个真实点（并行档下 `solve_call_us` 即 scope 墙钟，语义同样有定义）。
        let t_sg = vxl_phys_core::probe::start();
        self.solve_groups(bodies, manifolds, islands, &gr, &mut bufs, &p);
        let d_sg = vxl_phys_core::probe::us(t_sg);
        self.scatter_groups(bodies, islands, &gr, &bufs);
        let t_wc = vxl_phys_core::probe::start();
        self.commit_warm_slots(bodies, manifolds, &mut bufs);
        let d_wc = vxl_phys_core::probe::us(t_wc);
        bufs.restore(self);

        // （M1 软接触形态起，位置修正走 erp 偏置速度进速度通道 + CFM 正则化，
        //  独立「分裂冲量偏置通道 + 位移写回」已退役——见 SolverParams。）

        let d_solve = vxl_phys_core::probe::us(t_solve);
        // 诊断收尾（T4）：并行段墙钟 / 每组耗时 / 每组流形数 → `island_diag`。
        // 串行占比 = 求解相位 − scope；它就是扩展比的上限（判据写在 `IslandDiag` 文档里）。
        self.island_diag.islands = islands.len() as u32;
        self.island_diag.manifolds = manifolds.len() as u32;
        self.island_diag.g_count = g_count as u32;
        self.island_diag.gather_us = d_island;
        // `d_solve` 只覆盖 scope 之后的部分（t_solve 在 gather 之后才起）⇒ 别重复减 gather。
        // `scatter_us` 保持旧语义（并行档：d_solve − scope = 串行尾段上界）；两个新字段是真实测点。
        self.island_diag.scatter_us = d_solve.saturating_sub(self.island_diag.scope_us);
        self.island_diag.solve_call_us = d_sg;
        self.island_diag.warm_commit_us = d_wc;
        self.island_diag.warm_count = self.warm_slots.len() as u32;
        self.island_diag.warm_bytes_per_slot = std::mem::size_of::<WarmManifold>() as u32;

        let t_sleep = vxl_phys_core::probe::start();
        // 5) 岛级休眠与唤醒（§4.11 / §3 稳定性）——见 `sleep_pass` 及其两条分支。
        self.sleep_pass(bodies, islands, manifolds, config, dt, cleanup);
        self.island_pool = pool;
        self.last_phase_us = (d_island, d_solve, vxl_phys_core::probe::us(t_sleep), 0);
    }

    /// 1) 并查集分岛（直接对流形；双静态对不入岛）。固定规则：小索引为根（确定性）。
    ///    缓冲跨帧复用（`self.parent`），避免每帧 20 万级 alloc/fill。
    fn build_union_find(&mut self, bodies: &BodySet, manifolds: &[Manifold]) -> Vec<u32> {
        let n = bodies.len();
        let mut parent = std::mem::take(&mut self.parent);
        parent.clear();
        parent.extend(0..n as u32);
        for m in manifolds {
            let (a, b) = (m.a as usize, m.b as usize);
            if bodies.is_dynamic(a) && bodies.is_dynamic(b) {
                union_small_root(&mut parent, m.a, m.b);
            }
        }
        parent
    }

    /// 2) 岛桶。体按索引升序；岛内流形按全局流形序（§4.14 确定性模式）。
    ///    只有「与清醒体连通」的体参与建岛：
    ///    - 第一遍：清醒动体建岛（睡眠体不建岛 → 全睡眠帧岛构建 ≈ O(查是否为空)）；
    ///    - 第二遍：睡眠动体若其连通分量已被激活（root 已在槽位表）则并入——
    ///      保证「被撞唤醒」的接触对里有沉睡侧的体（求解冲量要施加到它并唤醒）。
    ///
    ///    岛池跨帧复用（Vec 容量保留），全清醒场景（10 万岛）不再逐帧分配。
    fn fill_island_buckets(
        &mut self,
        bodies: &BodySet,
        manifolds: &[Manifold],
        parent: &mut [u32],
    ) -> (Vec<Island>, usize) {
        let n = bodies.len();
        let mut root_slot: HashMap<u32, usize> = HashMap::new();
        let mut pool = std::mem::take(&mut self.island_pool);
        let mut islands_used = 0usize;
        for i in 0..n {
            if !(bodies.is_dynamic(i) && bodies.awake[i]) {
                continue;
            }
            let r = find_small_root(parent, i as u32);
            let slot = *root_slot.entry(r).or_insert_with(|| {
                if islands_used == pool.len() {
                    pool.push(Island {
                        bodies: Vec::new(),
                        manifs: Vec::new(),
                    });
                }
                let s = islands_used;
                pool[s].bodies.clear();
                pool[s].manifs.clear();
                islands_used += 1;
                s
            });
            pool[slot].bodies.push(i as u32);
        }
        if !root_slot.is_empty() {
            // 睡眠侧并入（其根已被清醒体激活的连通分量）。
            for i in 0..n {
                if !bodies.is_dynamic(i) || bodies.awake[i] {
                    continue;
                }
                let r = find_small_root(parent, i as u32);
                if let Some(&slot) = root_slot.get(&r) {
                    pool[slot].bodies.push(i as u32);
                }
            }
            for (mi, m) in manifolds.iter().enumerate() {
                let (a, b) = (m.a as usize, m.b as usize);
                if !bodies.is_dynamic(a) && !bodies.is_dynamic(b) {
                    continue;
                }
                let root = if bodies.is_dynamic(a) {
                    find_small_root(parent, m.a)
                } else {
                    find_small_root(parent, m.b)
                };
                if let Some(&slot) = root_slot.get(&root) {
                    pool[slot].manifs.push(mi);
                }
            }
        }
        (pool, islands_used)
    }

    /// 3) 清醒岛分组 + gather：把组内岛体速度拷进组内 scratch（顺序 = 岛序 = scatter 序）。
    ///    分组切分用比例式（g·n/g_count）：`岛数 < 组数` 时尾部组为空区间
    ///    ——旧式 `g*ceil(n/g)` 会产出 start > n 的越界区间（9 岛 8 组实测 panic）。
    ///    每体 4 次 push + 世界逆惯量矩阵是 gather 的全部工作（占求解相位约 24%）；
    ///    ⚠️ 并行化它**反而更慢**：该段与解算同属**访存带宽受限**。
    fn gather_groups(
        &mut self,
        bodies: &BodySet,
        islands: &[Island],
        awake: Vec<usize>,
        bufs: &mut GroupBufs,
        g_count: usize,
    ) -> Groups {
        bufs.build.resize_with(g_count, Vec::new);
        bufs.warm_out.resize_with(g_count, Vec::new);
        bufs.lv.resize_with(g_count, Vec::new);
        bufs.av.resize_with(g_count, Vec::new);
        bufs.iw.resize_with(g_count, Vec::new);
        bufs.im.resize_with(g_count, Vec::new);
        bufs.local_of.clear();
        bufs.local_of.resize(bodies.len(), u32::MAX);
        let mut ranges: Vec<(usize, usize)> = Vec::with_capacity(g_count);
        let mut group_manifs_diag: Vec<u32> = Vec::with_capacity(g_count);
        for g in 0..g_count {
            bufs.build[g].clear();
            bufs.warm_out[g].clear();
            bufs.lv[g].clear();
            bufs.av[g].clear();
            bufs.iw[g].clear();
            bufs.im[g].clear();
            let s0 = g * awake.len() / g_count;
            let e0 = (g + 1) * awake.len() / g_count;
            ranges.push((s0, e0));
            // 诊断（T4）：每组流形数 = 工作量代理（与 group_us 一起判负载不均）。
            let mut mf_here = 0u32;
            for &ii in &awake[s0..e0] {
                mf_here += islands[ii].manifs.len() as u32;
            }
            group_manifs_diag.push(mf_here);
            for &ii in &awake[s0..e0] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    bufs.local_of[i] = bufs.lv[g].len() as u32;
                    if SUBISLAND_SLEEP && !bodies.awake[i] {
                        // 睡眠体落在清醒岛里（子块睡眠开启时会发生）：**求解期按静态处理**
                        // （质量/惯量置 0 + 速度置 0），否则它会被每子步写入速度
                        // ——"位置冻结、速度被写"的僵尸体，唤醒时会跳。
                        bufs.lv[g].push(Vec3::ZERO);
                        bufs.av[g].push(Vec3::ZERO);
                        bufs.iw[g].push(Mat3::world_inv_inertia(bodies.rot(i), Vec3::ZERO));
                        bufs.im[g].push(0.0);
                        continue;
                    }
                    bufs.lv[g].push(bodies.linvel[i]);
                    bufs.av[g].push(bodies.angvel(i));
                    // 每帧一次：世界逆惯量矩阵（帧内姿态不变，求解只改速度；
                    // M = R·diag(inv_local)·Rᵀ ⇒ 求解环每点每轮只需一次矩阵乘）。
                    bufs.iw[g].push(Mat3::world_inv_inertia(
                        bodies.rot(i),
                        bodies.local_inv_inertia[i],
                    ));
                    bufs.im[g].push(bodies.inv_mass[i]);
                }
            }
        }
        self.island_diag.group_manifs = group_manifs_diag;
        Groups { awake, ranges }
    }

    /// 4) 解算：并行门槛（`组数 > 1`）之内走 `solve_groups_parallel`，否则走
    ///    `solve_groups_serial`（数值路径相同）。诊断写回 `island_diag` 的
    ///    scope/build/warm/iter/group 五项与 `points`（后者取 `last_points`，
    ///    **无清醒岛时不刷新**——保持旧行为：空帧沿用上一帧读数）。
    fn solve_groups(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        islands: &[Island],
        gr: &Groups,
        bufs: &mut GroupBufs,
        p: &PhaseParams,
    ) {
        let (group_us, det, scope_us) = if gr.ranges.len() > 1 {
            self.solve_groups_parallel(bodies, manifolds, islands, gr, bufs, p)
        } else if !gr.awake.is_empty() {
            self.solve_groups_serial(bodies, manifolds, islands, gr, bufs, p)
        } else {
            (vec![0; gr.ranges.len()], [0u64; 3], 0)
        };
        self.island_diag.scope_us = scope_us;
        self.island_diag.build_us = det[0];
        self.island_diag.warm_us = det[1];
        self.island_diag.iter_us = det[2];
        self.island_diag.group_us = group_us;
        self.island_diag.points = self.last_points.0 as u32;
    }

    /// 4a) 并行解算（§6 契约：组间写槽位不相交，组内 = 串行语义）。
    ///      返回 (每组墙钟 µs, [构建/热启动/迭代] 合计 µs, scope 墙钟 µs)。
    fn solve_groups_parallel(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        islands: &[Island],
        gr: &Groups,
        bufs: &mut GroupBufs,
        p: &PhaseParams,
    ) -> (Vec<u64>, [u64; 3], u64) {
        let g_count = gr.ranges.len();
        // warm 表在本段只读（求解环把新值写进各组 `warm_out`，回写在 `commit_warm_slots`）
        // ⇒ take 出来才能与 `bufs` 的可变借用共存，出段立刻还回去。
        let warm_slots = std::mem::take(&mut self.warm_slots);
        let warm_index = std::mem::take(&mut self.warm_index);
        let bodies_ref: &BodySet = bodies;
        let awake_ref: &[usize] = &gr.awake;
        let islands_ref: &[Island] = islands;
        let warm_index_ref: &HashMap<WarmKey, u32> = &warm_index;
        let warm_slots_ref: &[(WarmKey, WarmManifold)] = &warm_slots;
        let local_ref: &[u32] = &bufs.local_of;
        let iw_ref: &[Vec<Mat3>] = &bufs.iw;
        let im_ref: &[Vec<f32>] = &bufs.im;
        let sp_ref: &SolverParams = &p.sp;
        // 诊断细分：每组一份累加器（闭包按 move 捕获，不能共享一个可变借用）。
        let mut details: Vec<[u64; 5]> = vec![[0; 5]; g_count];
        let mut group_us_diag: Vec<u64> = vec![0; g_count];
        let t_scope = vxl_phys_core::probe::start();
        std::thread::scope(|s| {
            // iter_mut 逐容器取出元素可变借用（按 g 索引整体借用会跨迭代重叠）。
            for (g, ((((lv, av), (cbuf, wout)), det), t_slot)) in bufs
                .lv
                .iter_mut()
                .zip(bufs.av.iter_mut())
                .zip(bufs.build.iter_mut().zip(bufs.warm_out.iter_mut()))
                .zip(details.iter_mut())
                .zip(group_us_diag.iter_mut())
                .enumerate()
            {
                let (s0, e0) = gr.ranges[g];
                let iw_g: &[Mat3] = &iw_ref[g];
                let im_g: &[f32] = &im_ref[g];
                let job = move || {
                    // 诊断（T4）：本组墙钟（组间最大值 = scope 墙钟 ⇒ 离散度 = 负载不均）。
                    let t0 = vxl_phys_core::probe::start();
                    solve_island_group(
                        &awake_ref[s0..e0],
                        islands_ref,
                        manifolds,
                        bodies_ref,
                        warm_index_ref,
                        warm_slots_ref,
                        local_ref,
                        iw_g,
                        im_g,
                        lv,
                        av,
                        cbuf,
                        wout,
                        p.tuning(),
                        sp_ref,
                        det,
                    );
                    *t_slot = vxl_phys_core::probe::us(t0);
                };
                if g + 1 == g_count {
                    let mut job = job;
                    job();
                } else {
                    s.spawn(job);
                }
            }
        });
        let scope_us = vxl_phys_core::probe::us(t_scope);
        let mut tot = [0u64; 5];
        for d in &details {
            for (k, v) in d.iter().enumerate() {
                tot[k] += v;
            }
        }
        for (k, v) in tot.iter().enumerate().take(3) {
            self.last_detail_us[k + 1] += v;
        }
        self.last_points = (tot[3], tot[4]);
        self.warm_slots = warm_slots;
        self.warm_index = warm_index;
        (group_us_diag, [tot[0], tot[1], tot[2]], scope_us)
    }

    /// 4b) 串行解算（`g_count == 1`：单线程或流形数低于并行门槛）。
    ///      返回 (每组墙钟 µs, [构建/热启动/迭代] 合计 µs, scope 墙钟 µs = 0)。
    fn solve_groups_serial(
        &mut self,
        bodies: &mut BodySet,
        manifolds: &[Manifold],
        islands: &[Island],
        gr: &Groups,
        bufs: &mut GroupBufs,
        p: &PhaseParams,
    ) -> (Vec<u64>, [u64; 3], u64) {
        let mut det = [0u64; 5];
        let t_ser = vxl_phys_core::probe::start();
        solve_island_group(
            &gr.awake,
            islands,
            manifolds,
            bodies,
            &self.warm_index,
            &self.warm_slots,
            &bufs.local_of,
            &bufs.iw[0],
            &bufs.im[0],
            &mut bufs.lv[0],
            &mut bufs.av[0],
            &mut bufs.build[0],
            &mut bufs.warm_out[0],
            p.tuning(),
            &p.sp,
            &mut det,
        );
        for (k, v) in det.iter().enumerate().take(3) {
            self.last_detail_us[k + 1] += v;
        }
        self.last_points = (det[3], det[4]);
        let group_us = vxl_phys_core::probe::us(t_ser);
        (vec![group_us], [det[0], det[1], det[2]], 0)
    }

    /// scatter：组序 = gather 序 → 局部索引一一对应（确定性）。
    fn scatter_groups(
        &self,
        bodies: &mut BodySet,
        islands: &[Island],
        gr: &Groups,
        bufs: &GroupBufs,
    ) {
        for g in 0..gr.ranges.len() {
            let mut k = 0usize;
            for &ii in &gr.awake[gr.ranges[g].0..gr.ranges[g].1] {
                for &bi in &islands[ii].bodies {
                    let i = bi as usize;
                    bodies.linvel[i] = bufs.lv[g][k];
                    bodies.set_angvel_raw(i, bufs.av[g][k]);
                    k += 1;
                }
            }
        }
    }

    /// warm 回写（按槽号原位写）+ 剪枝（对稠密槽单遍扫）。
    ///
    /// 设计（DESIGN-staged-solver §11）：`HashMap` 桶遍历/插入曾占 33.5ms/tick
    /// （合并 13.1 + 剪枝 20.7，均为桶访存）。槽位表把「值」搬到连续内存：
    /// 回写零哈希（直接按槽号写）、剪枝单遍顺序扫、索引只存 (键 → u32)。
    ///
    /// 剪枝规则不变：流形已消失的**双清醒**对才删——睡眠体不移动，其接触
    /// 不会真正消失（睡眠期不被检测只是省算力），若一并剪掉，唤醒后 warm
    /// 起点归零会导致数帧收敛变弱（穿透加深）。以「本调用是否刷新过」的
    /// 印章判定「流形是否仍在」：有流形且≥1 体清醒 ⇒ 必属清醒岛 ⇒ 必被
    /// 求解盖章；双睡的有流形但不被求解，两条规则都因「非双清醒」保留。
    fn commit_warm_slots(
        &mut self,
        bodies: &BodySet,
        manifolds: &[Manifold],
        bufs: &mut GroupBufs,
    ) {
        let mut warm_slots = std::mem::take(&mut self.warm_slots);
        let mut warm_free = std::mem::take(&mut self.warm_free);
        let mut warm_index = std::mem::take(&mut self.warm_index);
        self.warm_stamp = self.warm_stamp.wrapping_add(1);
        let stamp = self.warm_stamp;
        for wo in bufs.warm_out.drain(..) {
            for (slot, key, mut v) in wo {
                v.seen = stamp;
                if slot != u32::MAX {
                    warm_slots[slot as usize] = (key, v); // 原位写：零哈希
                } else if let Some(free) = warm_free.pop() {
                    warm_slots[free as usize] = (key, v);
                    warm_index.insert(key, free);
                } else {
                    warm_slots.push((key, v));
                    warm_index.insert(key, (warm_slots.len() - 1) as u32);
                }
            }
        }
        if manifolds.is_empty() {
            warm_slots.clear();
            warm_index.clear();
            warm_free.clear();
        } else {
            // 稠密单遍剪枝（顺序访存）。
            for (i, (key, v)) in warm_slots.iter_mut().enumerate() {
                let (a, b, _space) = *key;
                if a == u32::MAX {
                    continue; // 已是空洞
                }
                let both_awake = bodies.awake[a as usize] && bodies.awake[b as usize];
                if v.seen != stamp && both_awake {
                    warm_index.remove(key);
                    *key = DEAD_KEY;
                    warm_free.push(i as u32);
                }
            }
        }
        self.warm_slots = warm_slots;
        self.warm_free = warm_free;
        self.warm_index = warm_index;
    }
}

/// `solve_phase` 的每帧参数集（同一 `config`/`dt`/`cleanup` 下恒等）。
///
/// 打包成一个结构的原因：解算段三个 helper 都要它，拆成标量参数会让签名越界
/// （`clippy::too_many_arguments`）；实测把段整段提成函数时，光参数列表就 18–24 行
/// ⇒ 辅助函数自己超 120 行（记录在 `docs/SESSION-2026-09-24-GOD-OBJECTS.md` §5 第 14 条）。
struct PhaseParams {
    e_threshold: f32,
    settled_hold: u32,
    hold_max_vn: f32,
    iters: u32,
    shock: u32,
    normal_inner: u32,
    match_dist: f32,
    sp: SolverParams,
}

impl PhaseParams {
    /// 并组出 `solve_island_group` 要的旋钮包（存在理由见 `island::GroupTuning`：
    /// 原先 23 个形参撞 args-gate 硬禁，这 7 个同族且只读 ⇒ 并组不动数值路径）。
    fn tuning(&self) -> crate::island::GroupTuning {
        crate::island::GroupTuning {
            iters: self.iters,
            e_threshold: self.e_threshold,
            match_dist: self.match_dist,
            shock_iterations: self.shock,
            normal_inner: self.normal_inner,
            settled_hold: self.settled_hold,
            hold_max_vn: self.hold_max_vn,
        }
    }

    fn from_config(config: &PhysConfig, dt: f32, cleanup: bool, match_dist: f32) -> Self {
        // 准静态"安座"趟（`settled_hold_iterations`；0 = 关闭 = 不走该路径，逐位同旧）。
        // **两趟都跑**：睡眠判定看的是**带偏置趟**之后的速度（`solve_phase` 末尾逐岛判），
        // 而无偏置趟的速度才是下一子步的初值 ⇒ 两边都要清。
        let settled_hold = config.settled_hold_iterations;
        let hold_max_vn = 4.0 * config.sleep_linear;
        let mut sp = SolverParams::from_config(config, dt);
        if cleanup {
            // 无偏置趟：法向只留 speculative（去穿透偏置 0）——对齐 Rapier
            // `rhs_wo_bias`。**切向漂移回拉保留**：Rapier 的切向 `rhs_wo_bias` =
            // `solver_contact.tangent_velocity`（材料点漂移率），即在无偏置趟里
            // 仍然生效；本仓实测把它一并归零会让 125 体留缝角点场景退化
            // （|ω| 0.13→0.45、末态 KE 1.6→19.0）——它正是那个场景的承重件。
            sp.max_corr = 0.0;
        }
        Self {
            e_threshold: config.restitution_threshold,
            settled_hold,
            hold_max_vn,
            iters: if cleanup {
                config.stabilization_iterations.max(1)
            } else {
                config.velocity_iterations.max(1)
            },
            shock: if cleanup { 0 } else { config.shock_iterations },
            normal_inner: if cleanup {
                1
            } else {
                config.normal_inner.max(1)
            },
            match_dist,
            sp,
        }
    }
}

/// 解算期「按组缓冲」打包（gather → 解算 → scatter → warm 回写 四段共用）。
///
/// `solve_phase` 持有 take/restore：这些缓冲在 `self` 上跨帧复用容量，段间以
/// 单参数传递，避免每段 6–7 个 `&mut Vec<Vec<..>>` 参数（见 `PhaseParams` 注）。
struct GroupBufs {
    lv: Vec<Vec<Vec3>>,
    av: Vec<Vec<Vec3>>,
    iw: Vec<Vec<Mat3>>,
    im: Vec<Vec<f32>>,
    build: Vec<Vec<ContactConstraint>>,
    warm_out: Vec<Vec<WarmOutEntry>>,
    local_of: Vec<u32>,
}

impl GroupBufs {
    fn take(s: &mut ImpulseSolver) -> Self {
        Self {
            lv: std::mem::take(&mut s.group_lv),
            av: std::mem::take(&mut s.group_av),
            iw: std::mem::take(&mut s.group_iw),
            im: std::mem::take(&mut s.group_im),
            build: std::mem::take(&mut s.build_bufs),
            warm_out: std::mem::take(&mut s.warm_outs),
            local_of: std::mem::take(&mut s.local_of),
        }
    }

    fn restore(self, s: &mut ImpulseSolver) {
        s.group_lv = self.lv;
        s.group_av = self.av;
        s.group_iw = self.iw;
        s.group_im = self.im;
        s.build_bufs = self.build;
        s.warm_outs = self.warm_out;
        s.local_of = self.local_of;
    }
}

/// 解算分组表：`awake[i]` 是第 i 个清醒岛，`ranges[g]` 是第 g 组的 `awake` 区间
/// （组内串行、组间体集合不相交 ⇒ 可并行）。
struct Groups {
    awake: Vec<usize>,
    ranges: Vec<(usize, usize)>,
}
