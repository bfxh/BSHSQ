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
        let nt = self.pos.len();
        let threads = self.cfg.threads.max(1);
        if threads > 1 {
            self.density_pass_parallel(providers, nf, nt, threads);
            return;
        }
        // 串行档：逐粒调**同一个核**。
        let mut scratch: Vec<InteropContact> = Vec::new();
        for i in 0..nf {
            let mut walls = WallPlanes::new();
            self.dens[i] = density_fluid_particle(
                DensityCtx {
                    pos: &self.pos,
                    pmass: &self.pmass,
                    grid: &self.grid,
                    h: self.h,
                    h2: self.h2,
                    k6: self.k6,
                    w0: self.w0,
                    mass: self.mass,
                    boundaries: &self.boundaries,
                    n_fluid: nf,
                },
                DensityKind::Fluid,
                i,
                providers,
                &mut scratch,
                &mut walls,
            );
        }
        // 2b 边界粒子：同一核、**另一个 kind**（口径是参数，不是新方法）。
        for i in nf..nt {
            let mut walls = WallPlanes::new();
            self.dens[i] = density_fluid_particle(
                DensityCtx {
                    pos: &self.pos,
                    pmass: &self.pmass,
                    grid: &self.grid,
                    h: self.h,
                    h2: self.h2,
                    k6: self.k6,
                    w0: self.w0,
                    mass: self.mass,
                    boundaries: &self.boundaries,
                    n_fluid: nf,
                },
                DensityKind::Boundary,
                i,
                providers,
                &mut scratch,
                &mut walls,
            );
        }
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
        let ctx = DensityCtx {
            pos: &self.pos,
            pmass: &self.pmass,
            grid: &self.grid,
            h: self.h,
            h2: self.h2,
            k6: self.k6,
            w0: self.w0,
            mass: self.mass,
            boundaries: &self.boundaries,
            n_fluid: nf,
        };
        // 流体段（`0..nf`）与 2b 边界段（`nf..nt`）= 同一 helper、只差 `kind`
        // 与索引范围 ⇒ 「边界口径是参数」在并行档上同样成立。
        let Self { dens, .. } = self;
        density_chunk_parallel(ctx, dens, 0..nf, DensityKind::Fluid, providers, threads);
        if nt > nf {
            let Self { dens, .. } = self;
            density_chunk_parallel(ctx, dens, nf..nt, DensityKind::Boundary, providers, threads);
        }
    }
}
