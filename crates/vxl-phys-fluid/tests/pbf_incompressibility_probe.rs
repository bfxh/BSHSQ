//! **不可压误差档**（M5 出口判据之一）：同一个水槽、同一个 `dt`、同一套边界口径下，把
//! `CpuSph`（WCSPH + Tait 状态方程）与 `CpuPbf`（Müller 2013 密度约束投影）的**密度误差**
//! 摆在一张表上——而不是各跑各的"看起来不错"。
//!
//! 判据量（都在**末段窗口**上取峰值；单帧快照不可信，`boundary_accuracy_probe` 已踩过）：
//! - `over  = max(ρ/ρ0 − 1)`：**过密**——这才是不可压性的真实违约；
//! - `over_in`：同上，但只统计**体内部**粒子：核完整（`ρ/ρ0 ≥ 0.9`）**且**离所有边界
//!   `> h`。两个条件各挡一种**离散伪影**（都不是不可压性）：
//!   ① `ρ/ρ0 < 0.9` 是自由表面的核截断——离散本身的性质，两族都躲不掉；
//!   ② 地板-侧壁**棱线**上镜像鬼影按两个面各反射一次 ⇒ 该角区被双计。实测该棱 `C=+0.18`
//!   且**两族同现** ⇒ 是边界几何口径，不该算在求解器账上（`fluid_density.rs` 头注：鬼影
//!   随流体局部分布逐粒同步，单面下是准的，两面相加才是伪影）。
//! - `under = min(ρ/ρ0)`：表面截断深度（**只读数**，不作判据）；
//! - `vmax` / `reach`：**表面完整性**——格式若把表面炸开，这两项先现形。
//!
//! 场景 = `set_boundaries(&[0])` 的**镜像鬼影**通道 + `Basin` provider（地板 y=0 + 四壁
//! x/z = ±0.35，堰高 0.9；与 `boundary_accuracy_probe.rs::Basin` 同口径）。水柱**已在摊开
//! 平衡位起步**（足印 0.68、深 0.35 = 3.5h，首层离壁 0.01）⇒ 不再塌、无飞溅、无孤立粒子。
//! 2b 边界粒子的反作用另由 `pbf_boundary_reaction.rs` 单独考。60 tick × 4 子步（`dt=1/60`）。
//!
//! ⚠️ **别拿"腔太窄/墙把流体夹住"的水槽来考 PBF**：约束投影要把 `C → 0`，而壁面质量在核内
//! **不动**，流体挪不开 ⇒ 只能卡在墙上硬顶 ⇒ 读数反映的是"腔太窄"（本轮踩过：4³ 块 + 四面墙
//! 贴到 0.05；`boundary_accuracy_probe` 头注同款：大体被夹 ⇒ 虚高 10×）。上表的水柱 3.5h 深、
//! 只在**底部一层**贴壁，是能摊开的口径。
//!
//! ⚠️ **自由表面必须一边倒**（`pbf.rs::lambda_pass` 的 `λ ≤ 0`）：表面核被截断 ⇒ `C < 0` ⇒
//! `λ > 0` ⇒ 成对项是**吸引**，而"把邻域移回截断的核内"几何上做不到 ⇒ 表面成了**永不满足**的
//! 常量拉力（自己缩 → 过密 → 再顶回 ⇒ 湍流）。实测（自由空间均匀晶格 + 自由表面，单子步）：
//! 不钳 ⇒ 表面位移 0.24 格、`vmax` 2.6~3.0 m/s，且**迭代越多越差**；钳后 ⇒ `vmax = 0.0000`、
//! 位移 0。WCSPH 侧对应"负压钳零"（同一个 `tensile_instability_suppression` 开关，见
//! `fluid_force.rs`）⇒ 两族口径统一。
//!
//! 实测（60 tick 末段峰值）：wcsph `over` 1.87e-1 / `over_in` 4.49e-2；pbf×3 1.27e-1 / 3.37e-2；
//! pbf×4 9.30e-2 / 2.76e-2 ⇒ PBF 内部过密**优于** WCSPH 且随迭代单调 ↓。
//!
//! 跑法：`cargo test -p vxl-phys-fluid --test pbf_incompressibility_probe -- --nocapture`
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Vec3};
use vxl_phys_fluid::{DensityIterations, FluidConfig, FluidFamily, FluidSystem};

/// 水槽（半空间族）：地板 y=0 + 四壁内面 x/z = ±`cav`，堰高 `wall`（堰上不设墙）。
/// 与 `boundary_accuracy_probe.rs::Basin` 同口径（外法线朝流体、`depth = probe − sdf`）。
struct Basin {
    cav: f32,
    wall: f32,
}

impl Basin {
    fn faces(&self) -> [(usize, f32, f32); 5] {
        [
            (1, 0.0, 1.0),
            (0, -self.cav, 1.0),
            (0, self.cav, -1.0),
            (2, -self.cav, 1.0),
            (2, self.cav, -1.0),
        ]
    }
}

impl ProviderColliders for Basin {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(Aabb {
            min: Vec3::new(-50.0, -50.0, -50.0),
            max: Vec3::new(50.0, 50.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, probe: f32, out: &mut Vec<InteropContact>) -> bool {
        for &(axis, plane, sign) in &self.faces() {
            let comp = [p.x, p.y, p.z][axis];
            if axis != 1 && comp > self.wall {
                continue;
            }
            let sdf = (comp - plane) * sign;
            if sdf > 2.0 * probe {
                continue;
            }
            let point = match axis {
                0 => Vec3::new(plane, p.y, p.z),
                1 => Vec3::new(p.x, plane, p.z),
                _ => Vec3::new(p.x, p.y, plane),
            };
            let normal = match axis {
                0 => Vec3::new(sign, 0.0, 0.0),
                1 => Vec3::new(0.0, sign, 0.0),
                _ => Vec3::new(0.0, 0.0, sign),
            };
            out.push(InteropContact {
                point,
                normal,
                depth: probe - sdf,
                feature: 0,
            });
        }
        true
    }
}

/// 一次运行的读数（`over`/`under` 取末段窗口峰值，`digest` 为末态位模式）。
struct Metrics {
    over: f32,
    over_in: f32,
    under: f32,
    vmax: f32,
    reach: f32,
    digest: u64,
    /// 窗口内**内部样本数**的最大值。非空样本守卫：`over_in` 以 `f32::MIN` 哨兵起步，
    /// 若某档一个内部粒子都没有，`over_in` 比较会**空通过**（哨兵恒小于真值）。只做非空性
    /// 判据，不参与物理比较。
    n_in: usize,
}

/// 位模式指纹（FNV-1a，同 `src/tests.rs` 口径）：位置 + 速度的 `f32` 位。
fn digest(f: &FluidSystem) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut eat = |b: u32| {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    for p in f.positions().iter().chain(f.velocities()) {
        eat(p.x.to_bits());
        eat(p.y.to_bits());
        eat(p.z.to_bits());
    }
    h
}

/// 跑 `ticks` tick 并取读数。两族的**边界口径、几何、种子、dt 全同**（只换 `family`）。
fn measure(family: FluidFamily, iters: DensityIterations, ticks: usize) -> Metrics {
    let h = 0.1f32;
    let s = h / 2.0;
    let (cav, wall) = (0.35f32, 0.9f32);
    let cfg = FluidConfig {
        family,
        density_iterations: iters,
        smoothing_radius: h,
        ..FluidConfig::default()
    };
    let rho0 = cfg.rest_density;
    // 水柱**已在平衡摊开位**起步：足印 0.68 × 0.68（腔 0.7）、深 0.35（= 3.5h），
    // 首层离壁 0.01（> 0 ⇒ 驻留线在 +skin 内侧、壁邻鬼影收得到；< skin ⇒ 不算穿透）。
    // 关键：块**不再塌**（足印已等于腔宽）⇒ 无飞溅、无孤立粒子（`under` 不会被
    // "核被截光"的 −0.806 污染）。
    let n = 14usize;
    let nz = 7usize;
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.34, 0.010, -0.34), [n, nz, n], s);
    f.set_boundaries(&[0]);
    let mut m = Metrics {
        over: f32::MIN,
        over_in: f32::MIN,
        under: f32::MAX,
        vmax: 0.0,
        reach: 0.0,
        digest: 0,
        n_in: 0,
    };
    // 末 10 tick 为窗口：起步阶段晶格还没被重力压实，读数不反映静水态。
    let window = 10.min(ticks);
    for t in 0..ticks {
        f.step(1.0 / 60.0, &Basin { cav, wall });
        if t + window >= ticks {
            let mut over = f32::MIN;
            let mut over_in = f32::MIN;
            let mut under = f32::MAX;
            let mut n_in = 0usize;
            for (&rho, p) in f.densities().iter().zip(f.positions()) {
                let r = rho / rho0 - 1.0;
                over = over.max(r);
                under = under.min(r);
                // **体内部**：核完整（ρ/ρ0 ≥ 0.9）**且**离所有边界 > h —— 后者挡掉
                // 地板-侧壁棱线的镜像鬼影重叠加（实测该棱 C=+0.18，两族同现，
                // 属边界几何口径、不是不可压性）。
                let dw = (cav - p.x.abs()).min(cav - p.z.abs()).min(p.y);
                if r >= -0.1 && dw > h {
                    over_in = over_in.max(r);
                    n_in += 1;
                }
            }
            m.over = m.over.max(over);
            m.over_in = m.over_in.max(over_in);
            m.under = m.under.min(under);
            m.n_in = m.n_in.max(n_in);
            for v in f.velocities() {
                m.vmax = m.vmax.max(v.length());
            }
            for p in f.positions() {
                m.reach = m.reach.max(p.length());
            }
        }
    }
    m.digest = digest(&f);
    m
}

#[test]
fn pbf_holds_hydrostatic_density_better_than_wcsph() {
    let sph = measure(FluidFamily::CpuSph, DensityIterations::Three, 60);
    let pbf3 = measure(FluidFamily::CpuPbf, DensityIterations::Three, 60);
    let pbf4 = measure(FluidFamily::CpuPbf, DensityIterations::Four, 60);
    eprintln!(
        "不可压误差档（60 tick × 4 子步，dt=1/60，足印 0.68/腔半宽 0.35/堰高 0.9，水柱深 0.35）\n\
         {:8} {:11} {:11} {:11} {:9} {:9}\n\
         {:8} {:11.3e} {:11.3e} {:11.3e} {:9.4} {:9.4}\n\
         {:8} {:11.3e} {:11.3e} {:11.3e} {:9.4} {:9.4}\n\
         {:8} {:11.3e} {:11.3e} {:11.3e} {:9.4} {:9.4}",
        "档位",
        "over",
        "over_in",
        "under",
        "vmax",
        "reach",
        "wcsph",
        sph.over,
        sph.over_in,
        sph.under,
        sph.vmax,
        sph.reach,
        "pbf×3",
        pbf3.over,
        pbf3.over_in,
        pbf3.under,
        pbf3.vmax,
        pbf3.reach,
        "pbf×4",
        pbf4.over,
        pbf4.over_in,
        pbf4.under,
        pbf4.vmax,
        pbf4.reach,
    );

    // 非空样本守卫：`over_in` 以 `f32::MIN` 起步 ⇒ 某档若"一个内部粒子都没有"，两条
    // `over_in` 比较都会**空通过**（哨兵恒小于真值）。先钉住样本量，`over_in` 才算证据。
    eprintln!(
        "内部样本数（末 10 tick 窗口内最大）：wcsph {} / pbf×3 {} / pbf×4 {}",
        sph.n_in, pbf3.n_in, pbf4.n_in
    );
    assert!(
        sph.n_in >= 32 && pbf3.n_in >= 32 && pbf4.n_in >= 32,
        "内部样本不足（wcsph {}、pbf×3 {}、pbf×4 {}）⇒ over_in 比较是空断言",
        sph.n_in,
        pbf3.n_in,
        pbf4.n_in
    );
    // 非平凡：场景必须真的压出了密度偏差，否则下面的比较是空断言。
    assert!(
        sph.over > 1e-3 && pbf4.over > 1e-4,
        "场景没压出密度偏差（wcsph over={}、pbf over={}）",
        sph.over,
        pbf4.over
    );
    // 迭代单调：密度约束迭代是 Gauss-Seidel 式松弛 ⇒ 多一次不许更差（容许末位浮点级回退）。
    assert!(
        pbf4.over_in <= pbf3.over_in * 1.02,
        "4 次迭代的不可压误差不应劣于 3 次：over_in {} → {}",
        pbf3.over_in,
        pbf4.over_in
    );
    // 主判据：同 dt、同边界口径下，PBF 的内部过密误差优于 WCSPH（Tait 的可压性）。
    assert!(
        pbf4.over_in < sph.over_in,
        "PBF 的内部过密误差应小于 WCSPH：pbf {} vs wcsph {}",
        pbf4.over_in,
        sph.over_in
    );
    // 表面完整性：任一档都不许炸开（本轮量纲/符号踩坑的现形处）。
    // ⚠️ 阈值按**本场景已理解的实测**定：地板-壁棱的鬼影过密（C=+0.18）会顶出 ~2 m/s 的
    // 角射流，两族都有 ⇒ `vmax` 取 4.0（真炸开在 K=3e-3 扫描里是 9.6）、`reach` 取 0.8
    // （水槽内最远点 |p| ≈ 0.61）。`under` 不做判据：孤立粒子两族都能到 −0.806。
    for (name, m) in [("wcsph", &sph), ("pbf×3", &pbf3), ("pbf×4", &pbf4)] {
        assert!(
            m.reach < 0.8 && m.vmax < 4.0,
            "{name} 表面炸开：reach {} / vmax {}",
            m.reach,
            m.vmax
        );
    }
}

#[test]
fn both_families_are_bitwise_reproducible() {
    // 确定性契约（`adr/0004`）：同输入两跑逐位一致——两族各守一遍。
    for (name, family) in [("wcsph", FluidFamily::CpuSph), ("pbf", FluidFamily::CpuPbf)] {
        let a = measure(family, DensityIterations::Four, 12);
        let b = measure(family, DensityIterations::Four, 12);
        assert_eq!(a.digest, b.digest, "{name} 两跑不逐位一致");
        assert!(
            a.reach > 0.0 && a.vmax > 0.0,
            "{name} 用例没动起来（reach {} / vmax {}）",
            a.reach,
            a.vmax
        );
    }
}
