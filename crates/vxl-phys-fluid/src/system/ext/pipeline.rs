//! **FluidPipelineExt**：`FluidSystem` 的**管线内部步**——本族**不往 `FluidSystem` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys-fluid::FluidSystem"]` 写明——
//! 数据侧已按域打包（**字段 26 → 4**），剩余待做是「**数据+管线同体的那一半**」：
//! `FluidSystem` 的 40 个方法里有一半是**管线内部步**（`density_pass` / `force_pass` /
//! `boundary_pass` / `substep` / `for_neighbors` / …），它们与「SoA 数据访问器」混在同一个
//! 类型上 ⇒ **新加一个相位就往本类型添方法**。处置是「**管线内部步搬进扩展 trait**」，
//! 判据是「新加一个相位**不必再往 `FluidSystem` 添方法**」。
//!
//! **门口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! ⇒ 搬进 trait 后 `FluidSystem` 的方法数**不计入本类型**（门账 40 → 25）。
//!
//! **先例**（同一判据的既有落法）：`crates/vxl-phys/src/world_struct/ext/` 的 11 个
//! `World` 域侧扩展 trait（`World` 76 → 22）、`crates/vxl-phys-soft/src/cloth/ext/` 的
//! 5 个 `ClothSheet` 约束族扩展 trait（`ClothSheet` 36 → 14）。
//!
//! **搬走的是哪一半**（**不扩大公开面**）：
//! - **搬**：15 个**管线内部步**（原先 `pub(crate)` 或私有）—— 外部 crate 一个都不用。
//! - **不搬**：25 个**公开访问器与公开入口**（`len` / `positions` / `velocities` / `densities` /
//!   `pressures` / `accelerations` / `boundary_reactions` / `set_boundary_particles` / `step` /
//!   `phase_us` / `reset_phase_us` / `boundary_spans` / `gather_wall_*` …）—— 它们是
//!   `vxl-phys` / `vxl-phys-gpu` / `vxl-phys-fluid/tests/` 都在用的**公开 API**
//!   ⇒ 搬了要改所有调用方 + 破坏源码兼容性 ⇒ **不搬**。
//!
//! **逐位不变**：方法体逐字搬过来，未改一个算式、一个遍历序、一个分支。
//!
//! **可见性**：本 trait 是 `pub(crate)`（它的方法原本全是 crate 内可见 ⇒ 不外扩；
//! 顺带避免 `pub` trait 的签名里出现 crate 私有类型 ⇒ Rust `private_interfaces` lint 不响）。
use crate::fluid_density::kernel::{
    density_chunk_parallel, density_fluid_particle, DensityCtx, DensityKind, WallPlanes,
};
use crate::fluid_force::kernel::{force_fluid_particle, ForceCtx};
use crate::FluidSystem;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::Vec3;

/// **FluidPipelineExt**：`FluidSystem` 的**管线内部步**（密度/压力/力/边界/积分 各相）。
pub(crate) trait FluidPipelineExt {
    /// 清空边界粒子（截断回流体前缀；容量保留 ⇒ 稳态零分配）。
    fn truncate_to_fluid(&mut self);

    /// 单个子步（**管线内部**）：网格 → 密度 → 压力 → 力 → 积分 → 边界。
    fn substep(&mut self, dt: f32, providers: &dyn ProviderColliders);

    /// 邻域公共体：粒子 i 的 27 邻域格（钳边）内、r ≤ h 的 j 交给 `f`。
    /// 访问序 = 格坐标序（dz, dy, dx 固定）× 格内索引序 ⇒ 求和序是位置的确定函数。
    /// **转发给 `UniformGrid::for_neighbors_in`**（并行相位在分块闭包里用同一实现）。
    fn for_neighbors(&self, i: usize, f: impl FnMut(usize, Vec3, f32));

    /// **密度相位**（串行档）。
    fn density_pass(&mut self, providers: &dyn ProviderColliders);

    /// **密度相位**（并行档）—— 与串行档**调同一个核**（`density_fluid_particle`）⇒ 逐位一致。
    fn density_pass_parallel(
        &mut self,
        providers: &dyn ProviderColliders,
        nf: usize,
        nt: usize,
        threads: usize,
    );

    /// **压力相位**（Tait EOS）。
    fn pressure_pass(&mut self);

    /// **力相位**（入口：按档分派串/并）。
    fn force_pass(&mut self);

    /// **力相位**（并行档）—— 与串行档**调同一个核**（`force_fluid_particle`）⇒ 逐位一致。
    fn force_pass_parallel(&mut self, nf: usize, nt: usize);

    /// **力相位**（串行档）—— 与并行档**调同一个核**（`force_fluid_particle`）⇒ 逐位一致。
    fn force_pass_serial(&mut self, nf: usize);

    /// **边界反作用聚合**（每子步末；逐体累加 `breact`）。
    fn aggregate_reactions(&mut self);

    /// **边界相位**（接触带投影 + 反作用收集）。
    fn boundary_pass(&mut self, providers: &dyn ProviderColliders);

    /// **重合粒子分离**（边界相位的收尾）。
    fn separate_coincident(&mut self, pushed: &[(usize, Vec3)]);
}

impl FluidPipelineExt for FluidSystem {
    // ── 来自 `fluid_step.rs` ──
    /// 清空边界粒子（截断回流体前缀；容量保留 ⇒ 稳态零分配）。
    fn truncate_to_fluid(&mut self) {
        let nf = self.n_fluid;
        let ps = &mut self.dom.ps;
        ps.pos.truncate(nf);
        ps.vel.truncate(nf);
        ps.dens.truncate(nf);
        ps.press.truncate(nf);
        ps.acc.truncate(nf);
        ps.xsph.truncate(nf);
        ps.pmass.truncate(nf);
        ps.bforce.truncate(nf);
        self.dom.bnd.spans.clear();
        self.dom.bnd.breact.clear();
    }

    fn substep(&mut self, dt: f32, providers: &dyn ProviderColliders) {
        let nf = self.n_fluid;
        if nf == 0 {
            return; // 边界粒子不单独驱动（它们只随体走）
        }
        // 网格含**全部**粒子（邻域必须看得见边界粒子）；边界粒子的位置/速度每 tick
        // 由 `set_boundary_particles` 整体重建，本子步内不动（运动学冻结）。
        let t = vxl_phys_core::probe::start();
        self.dom.grid.rebuild(&self.dom.ps.pos, self.dom.kern.h);
        self.phase_us[0] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.density_pass(providers);
        self.phase_us[1] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.pressure_pass();
        self.phase_us[2] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        self.force_pass();
        self.phase_us[3] += vxl_phys_core::probe::us(t);
        // 半隐式欧拉：v ← v + dt·a + ε·xsph；CFL 限速防穿隧。
        // **只积分流体粒子**（索引前缀）——边界粒子不作积分。
        let vmax = self.cfg.max_speed_frac * self.dom.kern.h / dt;
        let vmax2 = vmax * vmax;
        let eps = self.cfg.xsph_viscosity;
        for i in 0..nf {
            let mut v = self.dom.ps.vel[i] + self.dom.ps.acc[i] * dt + self.dom.ps.xsph[i] * eps;
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            self.dom.ps.vel[i] = v;
            self.dom.ps.pos[i] += v * dt;
        }
        let t = vxl_phys_core::probe::start();
        self.boundary_pass(providers);
        self.phase_us[4] += vxl_phys_core::probe::us(t);
    }

    fn for_neighbors(&self, i: usize, f: impl FnMut(usize, Vec3, f32)) {
        self.dom
            .grid
            .for_neighbors_in(&self.dom.ps.pos, self.dom.kern.h2, i, f);
    }

    // ── 来自 `fluid_density.rs` ──
    /// **密度相位·唯一入口**（原 `density_pass`）。
    ///
    /// 算式在**一个核**里（`density_fluid_particle` 自由函数）：串行档与并行档
    /// **调同一个核** ⇒ 逐位一致靠「只有一份算式」保证，不再靠「两份手抄的算式恰好一致」。
    /// 并行档的 `DensityKind` 与串行档相同 ⇒ **同一相位只有一种口径**。
    fn density_pass(&mut self, providers: &dyn ProviderColliders) {
        let nf = self.n_fluid;
        let nt = self.dom.ps.pos.len();
        let threads = self.cfg.threads.max(1);
        if threads > 1 {
            self.density_pass_parallel(providers, nf, nt, threads);
            return;
        }
        // 串行档：逐粒调**同一个核**。
        // 密度缓冲**整体借出**（`Vec::take` 是 O(1)，不分配）⇒ 上下文（`&self`）与
        // 输出缓冲（`&mut [f32]`）两处借用互不相交（同 `force_pass_serial` 的做法）。
        let mut dens = std::mem::take(&mut self.dom.ps.dens);
        let ctx = density_ctx(self, nf);
        let mut scratch: Vec<InteropContact> = Vec::new();
        for (i, d_out) in dens.iter_mut().enumerate().take(nf) {
            let mut walls = WallPlanes::new();
            *d_out = density_fluid_particle(
                ctx,
                DensityKind::Fluid,
                i,
                providers,
                &mut scratch,
                &mut walls,
            );
        }
        // 2b 边界粒子：同一核、**另一个 kind**（口径是参数，不是新方法）。
        for (i, d_out) in dens.iter_mut().enumerate().skip(nf).take(nt - nf) {
            let mut walls = WallPlanes::new();
            *d_out = density_fluid_particle(
                ctx,
                DensityKind::Boundary,
                i,
                providers,
                &mut scratch,
                &mut walls,
            );
        }
        self.dom.ps.dens = dens;
    }

    /// **密度相位·并行档**（`threads > 1`）：逐粒独立（每粒只读他人的 pos/pmass、
    /// 只写自己的 `dens[i]`）⇒ 分块并行；壁面查询用**每块一份 scratch**
    /// （无共享可变状态）⇒ 结果与串行**逐位一致**。
    ///
    /// **与串行档调同一个核**（`density_fluid_particle`）⇒ 算式只有一份。
    fn density_pass_parallel(
        &mut self,
        providers: &dyn ProviderColliders,
        nf: usize,
        nt: usize,
        threads: usize,
    ) {
        // 流体段（`0..nf`）与 2b 边界段（`nf..nt`）= 同一 helper、只差 `kind`
        // 与索引范围 ⇒ 「边界口径是参数」在并行档上同样成立。
        // 密度缓冲**整体借出**（O(1)，不分配）⇒ 只读上下文与可写输出两处借用不相交。
        // ⚠️ 传给 `density_chunk_parallel` 的是**整段** `dens`（它自己按 `range` 切片），
        // 不是预切片 —— 预切片会与 `range` 的绝对下标叠加 ⇒ 越界。
        let mut dens = std::mem::take(&mut self.dom.ps.dens);
        let ctx = density_ctx(self, nf);
        density_chunk_parallel(
            ctx,
            &mut dens[..],
            0..nf,
            DensityKind::Fluid,
            providers,
            threads,
        );
        if nt > nf {
            density_chunk_parallel(
                ctx,
                &mut dens[..],
                nf..nt,
                DensityKind::Boundary,
                providers,
                threads,
            );
        }
        self.dom.ps.dens = dens;
    }

    // ── 来自 `fluid_force.rs` ──
    /// 压力：Tait p = B·((ρ/ρ0)^γ − 1)；γ=7 走整数次幂乘法展开。
    /// `tensile_instability_suppression`（默认开）= Monaghan 自由面钳制
    /// p ≥ 0：表面密度截断（q ≈ 0.5） otherwise 会给出 p ≈ −B，经对称
    /// spiky 形式变成巨大粒子间吸力（单子步 Δv ~ 10² m/s）⇒ 全体炸散。
    /// 覆盖**全部**粒子（含 2b 边界粒子——它们要参与压力对，也吃同一套钳制：
    /// 悬空/真空里的边界粒子 ρ 很小 ⇒ p 钳到 0 ⇒ 不产生凭空吸力）。
    fn pressure_pass(&mut self) {
        let rho0 = self.cfg.rest_density;
        let g = self.cfg.gamma_tait;
        let clamp = self.cfg.tensile_instability_suppression;
        let b_tait = self.dom.kern.b_tait;
        for i in 0..self.dom.ps.pos.len() {
            let q = self.dom.ps.dens[i] / rho0;
            let mut p = if (g - 7.0).abs() < 1e-6 {
                let q2 = q * q;
                let q4 = q2 * q2;
                b_tait * (q * q2 * q4 - 1.0)
            } else {
                b_tait * (q.powf(g) - 1.0)
            };
            if clamp && p < 0.0 {
                p = 0.0;
            }
            self.dom.ps.press[i] = p;
        }
    }

    /// **力相位·唯一入口**（原 `force_pass`）。
    ///
    /// 算式在**一个核**里（`force_fluid_particle` 自由函数）：串行档与并行档
    /// **调同一个核** ⇒ 逐位一致靠「只有一份算式」保证，不再靠「两份手抄的算式恰好一致」。
    fn force_pass(&mut self) {
        let nf = self.n_fluid;
        let nt = self.dom.ps.pos.len();
        // 并行档（`cfg.threads > 1`）：逐粒独立 ⇒ 分块并行（逐位一致，见 config 注）。
        if self.cfg.threads > 1 {
            self.force_pass_parallel(nf, nt);
        } else {
            self.force_pass_serial(nf);
        }
        // 物理层流黏度（μ = 0 自短路）——必须在上面**赋值** `acc` 之后累加（见 `laminar.rs`）。
        crate::laminar::pass(self);
    }

    /// **力相位·并行档**（`threads > 1`）：`acc`/`xsph` 逐粒独立 ⇒ 分块并行；
    /// 唯一跨粒子的 `bforce`（2b 反作用）在有边界粒子时走**串行补趟** ⇒ 保逐位一致。
    fn force_pass_parallel(&mut self, nf: usize, nt: usize) {
        let threads = self.cfg.threads.max(1);
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        for f in &mut self.dom.ps.bforce[nf..] {
            *f = Vec3::ZERO;
        }
        {
            let Self { dom, .. } = self;
            let (h, h2, k6, ks) = (dom.kern.h, dom.kern.h2, dom.kern.k6, dom.kern.ks);
            let (pos, vel, dens, press) = (&dom.ps.pos, &dom.ps.vel, &dom.ps.dens, &dom.ps.press);
            let (acc, xsph, pmass) = (&mut dom.ps.acc, &mut dom.ps.xsph, &dom.ps.pmass);
            let grid = &dom.grid;
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, (acc_c, xs_c)) in acc[..nf]
                    .chunks_mut(per)
                    .zip(xsph[..nf].chunks_mut(per))
                    .enumerate()
                {
                    let base = ci * per;
                    let (pos, vel, dens, press, pmass, grid) =
                        (&*pos, &*vel, &*dens, &*press, &*pmass, &*grid);
                    s.spawn(move || {
                        for (k, (a_out, x_out)) in acc_c.iter_mut().zip(xs_c.iter_mut()).enumerate()
                        {
                            let i = base + k;
                            let (a, xs) = force_fluid_particle(
                                ForceCtx {
                                    pos,
                                    vel,
                                    dens,
                                    press,
                                    pmass,
                                    grid,
                                    h,
                                    h2,
                                    k6,
                                    ks,
                                    g,
                                    alpha_c,
                                    n_fluid: nf,
                                },
                                i,
                                None,
                            );
                            *a_out = a;
                            *x_out = xs;
                        }
                    });
                }
            });
        }
        // 2b 反作用：**串行补趟**（只有边界粒子存在时才走）⇒ 单独成相位（见文件尾）。
        if nt > nf {
            boundary_reaction_serial(self, nf, alpha_c);
        }
        self.aggregate_reactions();
    }

    /// **力相位·串行档**（默认档：`threads == 1` ⇒ 与历史行为逐位一致）。
    fn force_pass_serial(&mut self, nf: usize) {
        // `for_neighbors` 借 `&self`，闭包里要写边界粒子受力 ⇒ 把缓冲整体借出
        // （`Vec::take` 是 O(1)，不分配），循环后归还。
        let mut bforce = std::mem::take(&mut self.dom.ps.bforce);
        for f in &mut bforce[nf..] {
            *f = Vec3::ZERO; // 每子步重新累加（反作用 = 本子步的力）
        }
        // 核要写 `bforce[j]`（2b 反作用）⇒ 把缓冲**借给核**（`&mut bforce[..]`）；
        // 借用作用域 = 单次调用 ⇒ 循环结束即释放，随后整块归还。
        // 上下文在循环**外**建（借 `&self`），而 `acc`/`xsph` 的回填在循环**内**
        // ⇒ 用 `split`（逐字段借出：只读视图 + 可写缓冲）避开借用冲突。
        let (ctx, acc, xsph) = split_force(self);
        for i in 0..nf {
            let (a, xs) = force_fluid_particle(ctx, i, Some(&mut bforce[..]));
            acc[i] = a;
            xsph[i] = xs;
        }
        self.dom.ps.bforce = bforce;
        self.aggregate_reactions();
    }

    // ── 来自 `fluid_boundary.rs` ──
    /// 反作用聚合：逐粒 `bforce` → 每体 `(力, 绕**体原点**的力矩)`；求和序 = 段序 × 段内索引序。
    /// **C2（2026-10-01，`PLAN-COUPLING.md` §5 / D1(b)）**：向 `breact` **逐子步累加**（不再清空
    /// 重写）——清零归 `step` 开头、÷ 子步数归 `step` 末尾 ⇒ `boundary_reactions()` =
    /// **dt_sub 时间加权的 tick 平均力**。段序在 tick 内不变 ⇒ 按段位置就地累加；
    /// 段表中途被换（不该发生）时就地改标签重起账。
    fn aggregate_reactions(&mut self) {
        for (si, &(body, origin, start, end)) in self.dom.bnd.spans.iter().enumerate() {
            let mut f = Vec3::ZERO;
            let mut tau = Vec3::ZERO;
            for k in start..end {
                let fk = self.dom.ps.bforce[k as usize];
                f += fk;
                tau += (self.dom.ps.pos[k as usize] - origin).cross(fk);
            }
            match self.dom.bnd.breact.get_mut(si) {
                Some(e) if e.0 == body => {
                    e.1 += f;
                    e.2 += tau;
                }
                Some(e) => *e = (body, f, tau),
                None => self.dom.bnd.breact.push((body, f, tau)),
            }
        }
    }

    /// 边界投影：`contacts_point` 接触带内，只推**真穿透**（sdf < 0，由
    /// 接触点/外法线恢复，与 provider 的 depth 口径解耦）+ 法向速度归零
    /// （非弹性 e=0）。推到 sdf = +skin 驻留线（穿透量 + skin）：静隙由
    /// 壁压自平衡，投影只定下限——且驻留点 sdf > 0 让密度轮的镜像平面
    /// 收集始终覆盖得到（sdf = 0 会被 `sdf > 0` 滤掉 → 壁邻鬼影丢失）。
    /// **投影坍缩消除**：多平面深穿透被逐轴推到各平面交集 = 同一个点
    /// （凹角/棱），同位粒子对互喂 W(0) ⇒ ρ ≈ 2ρ0 ⇒ Tait q⁷ 爆压
    /// （实测角点堆粒子 ρ=2153 起爆喷泉）；且同位对 d = 0 ⇒ 压力梯度
    /// 零方向 ⇒ 永久僵局。解析：低索引驻留、高索引沿自身接触法线和
    /// （流体侧）外移 r_min = 0.2h；索引升序 + 法线序固定 ⇒ 确定。
    /// 预滤余量 = h：必须 ≥ 单子步最大行程（CFL 上限 0.4h）+ 接触带，
    /// 否则快速粒子一步跨过查询带 → 永久脱离所有接触查询（自由落体逃逸）。
    /// 复用统一提供者通道：体素/三角网/喷溅无改动即为边界。
    /// **只投影流体粒子**（索引前缀）：边界粒子在体内、由体运动学带着走，
    /// 既不该被提供者推出，也不该被推出体外。
    fn boundary_pass(&mut self, providers: &dyn ProviderColliders) {
        if self.dom.bnd.boundaries.is_empty() {
            return;
        }
        // 核半径与接触带宽度（域①/域③ 各自读出 ⇒ 下面几处不必再走 `self.dom.…`）。
        let (h, skin) = (self.dom.kern.h, self.dom.bnd.skin);
        // 本子步被投影粒子：(索引, 接触法线和)。升序登记 ⇒ 消解序确定。
        let mut pushed: Vec<(usize, Vec3)> = Vec::new();
        for i in 0..self.n_fluid {
            let pi = self.dom.ps.pos[i];
            // 投影累计跨所有边界（角部粒子同帧吃地面+墙的多笔推出）。
            let mut p = pi;
            let mut v = self.dom.ps.vel[i];
            let mut n_sum = Vec3::ZERO;
            let mut hit = false;
            for &bid in &self.dom.bnd.boundaries {
                // AABB 预滤（外扩 h；None = 无信息 ⇒ 仍查询）。
                if let Some(bb) = providers.bounds(bid) {
                    if pi.x < bb.min.x - h
                        || pi.x > bb.max.x + h
                        || pi.y < bb.min.y - h
                        || pi.y > bb.max.y + h
                        || pi.z < bb.min.z - h
                        || pi.z > bb.max.z + h
                    {
                        continue;
                    }
                }
                self.dom.bnd.contacts.clear();
                // 流体边界口径（内点鲁棒）：体素等截断 SDF 提供者的内部
                // 梯度被格间内面主导可指向固体深处 ⇒ 投影穿壁隧逃（实测
                // 1–2 格厚壁均复现）；解析面提供者默认原样转 contacts_point。
                if providers.contacts_point_boundary(bid, pi, skin, &mut self.dom.bnd.contacts) {
                    for c in &self.dom.bnd.contacts {
                        let sdf = (pi - c.point).dot(c.normal);
                        let pen = -sdf;
                        if pen <= 0.0 {
                            continue;
                        }
                        p += c.normal * (pen + skin).min(h);
                        n_sum += c.normal;
                        hit = true;
                        let vn = v.dot(c.normal);
                        if vn < 0.0 {
                            v -= c.normal * vn;
                        }
                    }
                    self.dom.ps.pos[i] = p;
                    self.dom.ps.vel[i] = v;
                }
            }
            if hit {
                pushed.push((i, n_sum));
            }
        }
        self.separate_coincident(&pushed);
    }

    /// 同位坍缩消解：仅在本子步被投影粒子间做最小间距（r_min = 0.2h，
    /// 远小于静置间距，常态零触发）。高索引者沿 n_sum 内移——投影把
    /// 深穿粒子逐轴推到平面交集点，法线和指向流体侧，一步拉开后
    /// 常规压强接管。位移 ≤ r_min，不注入爆发能量。
    fn separate_coincident(&mut self, pushed: &[(usize, Vec3)]) {
        let r_min = 0.2 * self.dom.kern.h;
        for a in 0..pushed.len() {
            let (ia, na) = pushed[a];
            let nl = na.length();
            if nl <= 1e-9 {
                continue;
            }
            let dir = na * (1.0 / nl);
            let mut pa = self.dom.ps.pos[ia];
            for &(ib, _) in &pushed[..a] {
                let d = pa - self.dom.ps.pos[ib];
                let r2 = d.length_squared();
                if r2 < r_min * r_min {
                    let r = r2.sqrt();
                    // 同位（r=0）也拉满 r_min；方向 = 自身法线和，不依赖 d。
                    pa += dir * (r_min - r);
                }
            }
            self.dom.ps.pos[ia] = pa;
        }
    }
}

// ── 原 `crates/vxl-phys-fluid/src/fluid_density.rs` 的自由函数 `density_ctx` ──
/// **密度核的只读上下文**（域② + 域③ + 域④ 的按需借出 ⇒ 核是自由函数）。
///
/// **为什么是自由函数而不是方法**：`FluidSystem` 的方法数是 god 门的**已登记债务**
/// （只准减）⇒ 新增能力不占方法位（同 `laminar::pass` / `fluid_medium::scan` 先例）。
fn density_ctx(sys: &FluidSystem, nf: usize) -> DensityCtx<'_> {
    DensityCtx {
        pos: &sys.dom.ps.pos,
        pmass: &sys.dom.ps.pmass,
        grid: &sys.dom.grid,
        h: sys.dom.kern.h,
        h2: sys.dom.kern.h2,
        k6: sys.dom.kern.k6,
        w0: sys.dom.kern.w0,
        mass: sys.dom.kern.mass,
        boundaries: &sys.dom.bnd.boundaries,
        n_fluid: nf,
    }
}

// ── 原 `crates/vxl-phys-fluid/src/fluid_force.rs` 的自由函数 `split_force` ──
/// **力相位的"只读上下文 + 可写缓冲"**（域② + 域④ 的按需借出 ⇒ 核是自由函数）。
///
/// **为什么是自由函数而不是方法**：`FluidSystem` 的方法数是 god 门的**已登记债务**
/// （只准减）⇒ 新增能力不占方法位（同 `laminar::pass` / `fluid_medium::scan` 先例）。
///
/// **为什么拆成"只读视图 + 两个可写切片"**：串行档要**同时**借 `&self`（喂核）与
/// 写 `acc`/`xsph`（回填）⇒ 整借 `&FluidSystem` 会拒；逐字段借出则两处互不相交
/// （同 `laminar.rs::split` 的做法）。
fn split_force(sys: &mut FluidSystem) -> (ForceCtx<'_>, &mut [Vec3], &mut [Vec3]) {
    // 先把标量读出来（`Copy`）⇒ 后面只借**叶子字段**，借用互不相交。
    let (h, h2, k6, ks) = (
        sys.dom.kern.h,
        sys.dom.kern.h2,
        sys.dom.kern.k6,
        sys.dom.kern.ks,
    );
    let g = sys.cfg.gravity;
    let alpha_c = sys.cfg.artificial_viscosity * sys.cfg.sound_speed;
    let n_fluid = sys.n_fluid;
    let ps = &mut sys.dom.ps;
    let ctx = ForceCtx {
        pos: &ps.pos,
        vel: &ps.vel,
        dens: &ps.dens,
        press: &ps.press,
        pmass: &ps.pmass,
        grid: &sys.dom.grid,
        h,
        h2,
        k6,
        ks,
        g,
        alpha_c,
        n_fluid,
    };
    (ctx, &mut ps.acc, &mut ps.xsph)
}

// ── 原 `crates/vxl-phys-fluid/src/fluid_force.rs` 的自由函数 `boundary_reaction_serial` ──
/// **2b 反作用·串行补趟**（从 `force_pass_parallel` 按相位拆出；纯搬移，语义未改）：
/// 逐粒 `bforce`（只有边界粒子存在时才走）——与串行路径同一算式、同一遍历序 ⇒ 逐位一致。
/// 缓冲用 `take` 借出以便闭包内写。**自由函数**：`FluidSystem` 的方法数是 god 门的
/// 登记债务（只准减），相位拆出不占方法位（同 `laminar::pass` / `fluid_medium::scan` 先例）。
fn boundary_reaction_serial(sys: &mut FluidSystem, nf: usize, alpha_c: f32) {
    let mut bforce = std::mem::take(&mut sys.dom.ps.bforce);
    for i in 0..nf {
        let vi = sys.dom.ps.vel[i];
        let rho_i = sys.dom.ps.dens[i];
        let ci2 = sys.dom.ps.press[i] / (rho_i * rho_i);
        let mi = sys.dom.ps.pmass[i];
        let h = sys.dom.kern.h;
        let h2 = sys.dom.kern.h2;
        let ks = sys.dom.kern.ks;
        sys.for_neighbors(i, |j, d, r2| {
            if j < nf {
                return;
            }
            let rho_j = sys.dom.ps.dens[j];
            let cj2 = sys.dom.ps.press[j] / (rho_j * rho_j);
            let r = r2.sqrt();
            let t = h - r;
            let mj = sys.dom.ps.pmass[j];
            let coef = mj * (ks * t * t) * (ci2 + cj2);
            let denom = r.max(1e-9);
            let cv = {
                let vij = vi - sys.dom.ps.vel[j];
                let vdn = -vij.dot(d);
                if vdn > 0.0 {
                    let mu = vdn * h / (r2 + 0.01 * h2);
                    mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ks * t * t)
                } else {
                    0.0
                }
            };
            bforce[j] -= d * (mi * (coef + cv) / denom);
        });
    }
    sys.dom.ps.bforce = bforce;
}
