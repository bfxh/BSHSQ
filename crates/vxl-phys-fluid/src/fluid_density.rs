//! fluid_density：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

mod wall_gather;

mod kernel;
use kernel::{density_chunk_parallel, density_fluid_particle, DensityCtx, DensityKind, WallPlanes};

impl FluidSystem {
    /// **密度相位·唯一入口**（原 `density_pass`）。
    ///
    /// 算式在**一个核**里（`density_fluid_particle` 自由函数）：串行档与并行档
    /// **调同一个核** ⇒ 逐位一致靠「只有一份算式」保证，不再靠「两份手抄的算式恰好一致」。
    /// 并行档的 `DensityKind` 与串行档相同 ⇒ **同一相位只有一种口径**。
    pub(crate) fn density_pass(&mut self, providers: &dyn ProviderColliders) {
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
}

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
