//! fluid_step：**推进面**（`set_boundary_particles` / `step` / `phase_us` /
//! `reset_phase_us`）—— **公开入口**留在 `FluidSystem` 上。
//!
//! **推进的内部步**（`substep` / `truncate_to_fluid` / `for_neighbors`）已搬进扩展 trait
//! [`FluidPipelineExt`](crate::system::ext::FluidPipelineExt)（`system/ext/pipeline.rs`）——
//! god 门处置方案「数据+管线同体的那一半」的执行面
//! （见 `docs/KNOWLEDGE.md`「2026-10-22 · 三条 god 债务同时出榜」一节）。
use super::*;

impl FluidSystem {
    /// **Akinci 边界粒子（2b）**：用给定体的两层表面粒子**整体替换**当前边界集
    /// （facade 每 tick 重建一次；空列表 = 回落纯流体）。返回边界粒子总数。
    ///
    /// - 局部采样按 `(形状, spacing)` **缓存复用**（形状集小、线性查找；稳态零分配）；
    ///   形状不受支持（复合体/高度场/provider/凸壳）⇒ 出 0 粒、**不进缓存**，
    ///   调用方据此回退粗档（`M1-EXIT.md` §4）。
    /// - 世界变换与体面速度在本函数内完成（`v + ω×r`，`r` 自体原点量起）。
    /// - 数组按**体序 × 层序 × 面内序**排布 ⇒ 求和序确定。
    pub fn set_boundary_particles(&mut self, bodies: &[(u32, Shape, BodyPose)]) -> usize {
        self.truncate_to_fluid();
        let mut cache = std::mem::take(&mut self.dom.bnd.lattice_cache);
        for &(body, shape, pose) in bodies {
            // 取或建局部两层采样（缓存已移出 `self` ⇒ 与下面的粒子写入无借用冲突）。
            let idx = match cache.iter().position(|(s, _)| *s == shape) {
                Some(i) => i,
                None => {
                    let lat = boundary::lattice(
                        &shape,
                        self.dom.kern.spacing,
                        self.dom.kern.h,
                        self.cfg.boundary_layers,
                    );
                    if lat.is_empty() {
                        continue; // 不支持 ⇒ 不造粒、不缓存（回退粗档的信号）
                    }
                    cache.push((shape, lat));
                    cache.len() - 1
                }
            };
            let start = self.dom.ps.pos.len() as u32;
            self.dom
                .ps
                .append_boundary(self.cfg.rest_density, &cache[idx].1, pose);
            let end = self.dom.ps.pos.len() as u32;
            if end > start {
                self.dom.bnd.spans.push((body, pose.pos, start, end));
            }
        }
        self.dom.bnd.lattice_cache = cache;
        self.dom.ps.bforce.resize(self.dom.ps.pos.len(), Vec3::ZERO);
        self.dom.bnd.breact.clear();
        self.boundary_count()
    }

    /// 推进一个完整 tick（内部按 `cfg.substeps` 等分子步，经 `pbf::substep_of` 分派族；边界经统一提供者通道）。
    ///
    /// **C2（2026-10-01，`PLAN-COUPLING.md` §5 / D1(b)）**：`breact` 是 **tick 级累加器**——
    /// 此处清零、每子步力相末尾累加、末尾 ÷ 子步数 ⇒ `boundary_reactions()` = **dt_sub
    /// 时间加权的 tick 平均力**（替代"末子步快照"）。**子步 = 1 不做除法** ⇒ 单子步档逐位不变。
    ///
    /// FLIP 档的相位缓冲提到 **tick 级**（`flip_sc`）：活过本 tick 的各子步 ⇒ 压力场热启动
    /// （稳态下第 2..N 子步几乎不再迭代），分配也从每子步一次降到每 tick 一次。每 tick 现建
    /// ⇒ 结果只依赖 tick 起点状态，不继承上一 tick 的历史（契约见 `flip.rs` 模块头"热启动"）。
    pub fn step(&mut self, dt_tick: f32, providers: &dyn ProviderColliders) {
        let sub = self.cfg.substeps.max(1);
        let dt = dt_tick / sub as f32;
        self.dom.bnd.breact.clear();
        let mut flip_sc = crate::flip::FlipScratch::new();
        for _ in 0..sub {
            crate::pbf::substep_of(self, dt, providers, &mut flip_sc);
        }
        if sub > 1 {
            let inv = 1.0 / sub as f32; // 等分子步 ⇒ 即 dt_sub/dt_tick
            for e in &mut self.dom.bnd.breact {
                e.1 *= inv;
                e.2 *= inv;
            }
        }
    }

    /// **相位计时**（诊断）：`[网格, 密度, 压力, 力, 积分+边界]` 的**累计微秒**。
    /// 与 `reset_phase_us()` 配合取窗口差值（本函数不改行为、不进哈希）。
    pub fn phase_us(&self) -> [u64; 5] {
        self.phase_us
    }

    /// 清零相位计时累加器（取窗口前调用）。
    pub fn reset_phase_us(&mut self) {
        self.phase_us = [0; 5];
    }
}
