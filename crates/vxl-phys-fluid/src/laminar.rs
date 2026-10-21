//! laminar：**物理动力黏度 μ（Pa·s）→ 流体内的层流黏度项**（`SURVEY-RIGID-HARD-CASES §4.2①` 的另一半）。
//!
//! `#288` 已经把 `FluidConfig::dynamic_viscosity` 接到了**刚体侧**（介质 Stokes 阻力）；
//! 本模块把它接到**液体内部**：SPH 动量方程里的层流黏度项（**Brookshaw 形式**的 ∇²v）——
//!
//! ```text
//! a_i += Σ_j  (v_j − v_i) · [ m_j · (2ν/(ρ̄)) · |∂W/∂r| / r ]      ν = μ/ρ0、ρ̄ = (ρ_i+ρ_j)/2
//! ```
//!
//! 与既有 **Monaghan 人工黏度**（`α`）的两点区别（别混）：
//! 1. **方向**：人工黏度沿**径向** `d̂`（只耗散接近对 ⇒ 专治压缩波）；本项沿 **(v_j − v_i)** ⇒ 对**剪切**同样做功；
//! 2. **系数**：人工黏度用 `α·c`（**无量纲数值旋钮**、不可迁移）；本项用 `2ν`（**物理量**，调 μ 就等价于调
//!    "水的黏度"）。
//!
//! 逐对系数对称（`m_i·m_j·2ν/(ρ̄)`）⇒ **动量守恒**；`j ≥ n_fluid`（边界粒子）在本档**不参与**
//! （该通道留给"边界剪切反作用"下一刀 ⇒ 本档不会向刚体漏/多给动量）。
//!
//! ⚠️ **绝对标定**：`ν_eff/(μ/ρ0)` 由核离散决定（不是 1）—— 探针/判据把它**量出来**（见
//! [`tests`] 的 `shear_mode_decay_is_linear_in_mu`），本档只保证**线性与只耗散**。
//! 默认 `dynamic_viscosity = 0` ⇒ 整段不进（四道哈希逐位未变）。
use crate::{FluidSystem, Vec3};

/// 层流黏度段（自由函数：`FluidSystem` 的方法数是 god 门的登记债务，只准减）。
/// 默认档（μ = 0）⇒ 立即返回；`threads > 1` 走分块并行（逐粒独立 ⇒ 与串行**逐位一致**）。
pub(crate) fn pass(sys: &mut FluidSystem) {
    let nu2 = 2.0 * sys.cfg.dynamic_viscosity / sys.cfg.rest_density;
    if sys.n_fluid == 0 || nu2 <= 0.0 || nu2.is_nan() {
        return; // 默认档（μ = 0）零成本短路；NaN 也走这里
    }
    if sys.cfg.threads > 1 {
        pass_parallel(sys, nu2);
    } else {
        pass_serial(sys, nu2);
    }
}

/// 只读视图（**逐字段**借出，不用整包 `&FluidSystem` —— 那样会把 `acc` 也借成只读，
/// 与并行档的 `&mut sys.acc` 冲突；同 `force_pass_parallel` 的 destructure 做法）。
#[derive(Clone, Copy)]
struct View<'a> {
    pos: &'a [Vec3],
    vel: &'a [Vec3],
    dens: &'a [f32],
    pmass: &'a [f32],
    grid: &'a crate::grid::UniformGrid,
    h: f32,
    h2: f32,
    ks: f32,
    n_fluid: usize,
}

/// 一个粒子在给定 `ν2 = 2ν` 下的层流黏度加速度（邻居遍历序 = 网格固定序 ⇒ 确定性）。
fn acc_of(v: &View, i: usize, nu2: f32) -> Vec3 {
    let vi = v.vel[i];
    let rho_i = v.dens[i];
    let mut add = Vec3::ZERO;
    v.grid.for_neighbors_in(v.pos, v.h2, i, |j, _d, r2| {
        if j >= v.n_fluid {
            return; // 边界粒子不参与（见头注）
        }
        let r = r2.sqrt();
        let t = v.h - r;
        // 系数 = m_j·(2ν/ρ̄)·|∂W/∂r| / r；|∂W/∂r| = ks·t²（spiky）
        let c = v.pmass[j] * (nu2 / (0.5 * (rho_i + v.dens[j]))) * (v.ks * t * t);
        add += (v.vel[j] - vi) * (c / r.max(1e-9));
    });
    add
}

/// 从 `&mut FluidSystem` 逐字段借出：只读视图 + `acc` 可变（**字段级**借用 ⇒ 不冲突）。
fn split(sys: &mut FluidSystem) -> (View<'_>, &mut [Vec3]) {
    // 先把标量读出来（`Copy`）⇒ 后面只借**叶子字段**，借用互不相交。
    let (h, h2, ks, nf) = (
        sys.dom.kern.h,
        sys.dom.kern.h2,
        sys.dom.kern.ks,
        sys.n_fluid,
    );
    let ps = &mut sys.dom.ps;
    let v = View {
        pos: &ps.pos,
        vel: &ps.vel,
        dens: &ps.dens,
        pmass: &ps.pmass,
        grid: &sys.dom.grid,
        h,
        h2,
        ks,
        n_fluid: nf,
    };
    (v, &mut ps.acc)
}

fn pass_serial(sys: &mut FluidSystem, nu2: f32) {
    let (view, acc) = split(sys);
    for (i, a_out) in acc[..view.n_fluid].iter_mut().enumerate() {
        *a_out += acc_of(&view, i, nu2);
    }
}

/// 分块并行：逐粒独立（每粒的邻居序不随分块变化）⇒ 与 [`pass_serial`] **逐位一致**。
fn pass_parallel(sys: &mut FluidSystem, nu2: f32) {
    let nf = sys.n_fluid;
    let threads = sys.cfg.threads.max(1);
    let per = nf.div_ceil(threads).max(1);
    // 借出只读视图 + `acc` 可变（逐块 `chunks_mut`）
    let (view, acc) = split(sys);
    std::thread::scope(|s| {
        for (ci, acc_c) in acc[..nf].chunks_mut(per).enumerate() {
            let base = ci * per;
            s.spawn(move || {
                for (k, a_out) in acc_c.iter_mut().enumerate() {
                    let i = base + k;
                    *a_out += acc_of(&view, i, nu2);
                }
            });
        }
    });
}

#[cfg(test)]
mod tests {
    // 新文件禁 glob import（`glob-gate`）⇒ 显式列名
    use crate::{FluidConfig, FluidSystem, Vec3};

    const N: usize = 10;
    const SPACING: f32 = 0.05;
    const AMP: f32 = 0.01;
    const OFFSET: f32 = 0.005;

    /// 简单剪切波晶格：`v_x = AMP·sin(k·(y−y0))`（+ 均匀偏移 `v_z = OFFSET`）。
    /// **纯剪切不改变密度**（映射的行列式 = 1）⇒ 密度/压强几乎不动 ⇒ 干净的黏度实验。
    fn shear(mu: f32, threads: usize) -> (FluidSystem, Vec<f32>, f32) {
        let cfg = FluidConfig {
            dynamic_viscosity: mu,
            gravity: Vec3::ZERO,
            artificial_viscosity: 0.0, // 人工黏度对剪切无作用，仍清零以排除干扰
            xsph_viscosity: 0.0,       // XSPH 会平滑剪切 ⇒ 必须关
            threads,
            ..FluidConfig::default()
        };
        let mut f = FluidSystem::new(cfg, Vec3::new(-0.2, -0.2, -0.2), [N, N, N], SPACING);
        let span = (N as f32 - 1.0) * SPACING; // 一个整波
        let k = core::f32::consts::TAU / span;
        let y0 = f.positions()[0].y;
        let mut phase = Vec::with_capacity(f.positions().len());
        let pos: Vec<Vec3> = f.positions().to_vec();
        for p in &pos {
            phase.push((k * (p.y - y0)).sin());
        }
        let vel: Vec<Vec3> = phase
            .iter()
            .map(|s| Vec3::new(AMP * s, 0.0, OFFSET))
            .collect();
        f.set_velocities(&vel);
        (f, phase, k)
    }

    /// 剪切模幅（只取**内部**粒子：离任一边界至少 2 层 ⇒ 表层密度亏空的压强扰动不进投影）。
    /// 用 f32 累加（f64 在本仓是登记债务、只准减 ⇒ 新文件零 f64；这里 10³ 项、量级 ~0.01，够）。
    fn amp(f: &FluidSystem, phase: &[f32]) -> f32 {
        let pos = f.positions();
        let vel = f.velocities();
        let (mut num, mut den) = (0.0f32, 0.0f32);
        for i in 0..pos.len() {
            let p = pos[i];
            let edge = p.x < -0.2 + 2.0 * SPACING
                || p.x > -0.2 + (N as f32 - 3.0) * SPACING
                || p.y < -0.2 + 2.0 * SPACING
                || p.y > -0.2 + (N as f32 - 3.0) * SPACING
                || p.z < -0.2 + 2.0 * SPACING
                || p.z > -0.2 + (N as f32 - 3.0) * SPACING;
            if edge {
                continue;
            }
            let s = phase[i];
            num += s * vel[i].x;
            den += s * s;
        }
        num / den
    }

    /// 最小二乘斜率（`y = a + b·x` 的 `b`）。
    fn slope(xs: &[f32], ys: &[f32]) -> f32 {
        let n = xs.len() as f32;
        let (mx, my) = (xs.iter().sum::<f32>() / n, ys.iter().sum::<f32>() / n);
        let (mut sxy, mut sxx) = (0.0f32, 0.0f32);
        for (x, y) in xs.iter().zip(ys) {
            sxy += (x - mx) * (y - my);
            sxx += (x - mx) * (x - mx);
        }
        sxy / sxx
    }

    /// 返回 `(λ_实测, 模幅比, k, KE 终/初)`。
    ///
    /// λ 用**最小二乘**在 `ln A` 上拟合，且**只取 `A > 0.3·A0` 段**：离散 Laplacian 的特征向量
    /// 不等于采样后的 sin 模 ⇒ 衰减越深、投影里混进来的其它模越多、λ 会被**低估**
    /// （实测深衰减段会把 μ = 200 的 λ 压到 9.94 而真值 ~12 ⇒ 线性判据假红）。
    fn run(mu: f32, threads: usize, ticks: usize) -> (f32, f32, f32, f32) {
        let (mut f, phase, k) = shear(mu, threads);
        let a0 = amp(&f, &phase);
        let pz0: f32 = f.velocities().iter().map(|v| v.z).sum();
        let ke0: f32 = f.velocities().iter().map(|v| v.length_squared()).sum();
        let dt = 1.0 / 60.0;
        let (mut xs, mut ys) = (Vec::new(), Vec::new());
        for t in 0..ticks {
            f.step(dt, &vxl_phys_core::interop::NoProviders);
            if (t + 1) % 4 == 0 {
                let a = amp(&f, &phase).abs();
                if a > 0.3 * a0.abs() {
                    xs.push((t + 1) as f32 * dt);
                    ys.push(a.ln());
                }
            }
        }
        let a1 = amp(&f, &phase);
        let pz1: f32 = f.velocities().iter().map(|v| v.z).sum();
        let ke1: f32 = f.velocities().iter().map(|v| v.length_squared()).sum();
        // 动量守恒：黏度的**逐对力严格反对称**（同一对的两个方向用同一串乘子 ⇒ f32 逐位相同）
        // ⇒ 物理上总动量不变；剩下的只有"逐粒速度更新各自的舍入"，不能按粒逐位比较。
        assert!(
            (pz1 - pz0).abs() <= 1e-4 * pz0.abs() + 1e-6,
            "均匀偏移方向的动量必须守恒（只剩 f32 逐粒更新的舍入）：{pz0} → {pz1}"
        );
        // 动能：黏度只耗散**剪切**；μ = 0 档的**压强残余**（表层密度亏空）会给块做一点功
        // ⇒ 这里只钉"不许显著增能"，跨档比较放在下面的测试里（μ > 0 必须比 μ = 0 低）。
        assert!(
            ke1 <= ke0 * 1.001,
            "黏度档不许显著增能：KE {ke0:.6e} → {ke1:.6e}"
        );
        assert!(k > 0.0, "波数必须为正（否则 ν_eff 读数无意义）");
        assert!(xs.len() >= 3, "拟合点太少（{}）", xs.len());
        (-slope(&xs, &ys), a1 / a0, k, ke1 / ke0)
    }

    /// **剪切模衰减随 μ 线性**（本档的核心判据；绝对标定由打印的 `ν_eff/(μ/ρ0)` 给出）。
    #[test]
    fn shear_mode_decay_is_linear_in_mu() {
        let (r0, _, _, ke0) = run(0.0, 1, 40);
        let (r1, ratio1, k, ke1) = run(20.0, 1, 40);
        let (r2, ratio2, _, ke2) = run(40.0, 1, 40);
        let (ex1, ex2) = (r1 - r0, r2 - r0);
        let nu = 20.0 / 1000.0; // μ/ρ0
        println!(
            "① μ = 0/20/40 Pa·s：λ = {r0:.3} / {r1:.3} / {r2:.3} 1/s（模幅比 {ratio1:.4} / {ratio2:.4}）\
             ｜KE 终/初 = {ke0:.6} / {ke1:.6} / {ke2:.6}｜ν_eff/(μ/ρ0) = {:.3}（k = {k:.3}）",
            ex1 / (k * k * nu)
        );
        assert!(
            ke2 < ke1 && ke1 < ke0,
            "μ 越大耗散越多（KE 终/初）：μ = 0/100/200 ⇒ {ke0:.6} / {ke1:.6} / {ke2:.6}"
        );
        assert!(
            ex1 > 0.0,
            "μ > 0 必须比 μ = 0 更快耗散剪切：λ = {r0:.3} vs {r1:.3}"
        );
        assert!(
            (ex2 / ex1 - 2.0).abs() <= 0.25,
            "剪切耗散必须**线性**于 μ：μ = 100 给 {ex1:.3} /s、μ = 200 给 {ex2:.3} /s（比 {:.2}）",
            ex2 / ex1
        );
    }

    /// **并行 = 串行逐位一致**（并行档的契约；黏度段逐粒独立 ⇒ 必须成立）。
    #[test]
    fn parallel_matches_serial_bitwise() {
        let (mut a, phase, _) = shear(100.0, 1);
        let (mut b, ..) = shear(100.0, 4);
        let dt = 1.0 / 60.0;
        for _ in 0..16 {
            a.step(dt, &vxl_phys_core::interop::NoProviders);
            b.step(dt, &vxl_phys_core::interop::NoProviders);
        }
        let (va, vb) = (a.velocities(), b.velocities());
        for i in 0..va.len() {
            assert!(
                va[i].x.to_bits() == vb[i].x.to_bits()
                    && va[i].y.to_bits() == vb[i].y.to_bits()
                    && va[i].z.to_bits() == vb[i].z.to_bits(),
                "并行档必须与串行逐位一致（粒子 {i}：{:?} vs {:?}）",
                va[i],
                vb[i]
            );
        }
        assert!(amp(&a, &phase).abs() > 0.0);
    }
}
