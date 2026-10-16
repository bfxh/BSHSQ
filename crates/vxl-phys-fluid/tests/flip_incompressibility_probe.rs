//! **FLIP 静水/低耗散判据**（M5 第二半：`CpuFlip`）：与 `pbf_incompressibility_probe.rs` **同一个
//! 水槽、同一个 `dt`、同一套边界口径**，只换族的读数表。
//!
//! ⚠️ **为什么这里没有"FLIP 的 ρ 比 WCSPH 好"这条**：FLIP 的不可压约束装在**网格**上，粒子之间
//! **没有**排斥项——两粒叠在一起时网格只当它们是"一团"，没有任何力把它们分开（PIC 的重置会让
//! 重叠**冻住**而非散开）。所以 `densities()`（`density_pass` 出的被动诊断，求解器根本不读）
//! 对本族**不是判据**：拿它去比 WCSPH/PBF 是在拿别人的量尺量本族。本表照印 `over`/`over_in`
//! 作为**读数**（不藏数），但断言只落在本族真正控制的量上（见下 ①②③）。
//!
//! 判据（都取**末段窗口**峰值，口径与 PBF 档逐字相同）：
//!
//! - ① **静水支撑**`height_keep` = 末态流体最高点 y / 起步最高点 y。投影若形同虚设，柱子会被
//!   自身重力压成饼 ⇒ 这个比值先塌。这条**有判别力**的证据是：同一个场景只把迭代数降到
//!   20，`height_keep` 必须显著更差（本档实测 0.48 vs 默认档 0.85）⇒ 说明它能真的抓到病。
//! - ② **净沉降**`sink` = 池内质心 y 相对起点的下降量，与**自由落体** `½gT²` 比（1 s ⇒ 4.91 m）。
//! - ③ **低耗散**（本族立身之本）：同场景同迭代、只改 FLIP/PIC 混合比 θ，θ = 0.95 的末态
//!   `Σ|v|²` 必须高于纯 PIC（θ = 0）——若桥接项误写成 PIC，这条先红。
//! - ④ 逐位确定（`flip_is_bitwise_reproducible`）；表面完整性 `vmax`/`reach` 阈值与 PBF 档同。
//!
//! 实测（60 tick，`h = 0.1`，水柱足印 0.68/腔 0.7/深 0.35，θ = 0.95）：
//!
//! ```text
//! 迭代数   over      over_in   under     vmax    reach   sink      height_keep  ke
//!  20      3.83e0    1.38e0   -5.3e-1    0.46    0.51   1.11e-1      0.482      21.4
//!  40      3.42e0    1.04e0   -5.6e-1    0.46    0.51   8.26e-2      0.712      17.4
//!  80      3.59e0    3.81e-1  -5.1e-1    0.45    0.53   5.92e-2      0.854      15.5
//! 160      3.71e0    7.21e-1  -5.0e-1    0.45    0.53   4.56e-2      0.930      15.4
//! 320      3.76e0    6.97e-1  -5.0e-1    —       —      3.60e-2      1.016      14.2
//! ```
//!
//! ⇒ `height_keep`/`sink` **随迭代单调收敛**且 320 次回到 1.02（离散公式正确，只是 Jacobi
//! 预算不够，见 `flip` 模块头"验证①"）；`over`（粒子级重叠）几乎不随迭代变 ⇒ 印证它是
//! 结构性读数、不是收敛问题。默认 `flip_pressure_iterations = 80`（水线掉幅 ≤ 15% 与线性
//! 成本之间的折中，其中"掉幅"就是本表的 `height_keep`）。
//!
//! 跑法：`cargo test -p vxl-phys-fluid --test flip_incompressibility_probe -- --nocapture`
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Vec3};
use vxl_phys_fluid::{FluidConfig, FluidFamily, FluidSystem};

/// 水槽（半空间族）：地板 y=0 + 四壁内面 x/z = ±`cav`，堰高 `wall`（堰上不设墙）。
/// 与 `pbf_incompressibility_probe.rs::Basin` / `boundary_accuracy_probe.rs::Basin` 同口径。
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

/// 一次运行的读数。`over`/`over_in`/`under` 对 FLIP 是**被动读数**（见文件头 ⚠️）。
struct Metrics {
    over: f32,
    over_in: f32,
    under: f32,
    vmax: f32,
    reach: f32,
    /// 池内质心 y 相对起点的**净沉降**（m）：`y0 − y_end`。
    sink: f32,
    /// 末态流体最高点 y / 起步最高点 y（判据 ①：静水支撑的判别量）。
    height_keep: f32,
    /// 末态总动能代理 `Σ|v|²`（等质量 ⇒ 与 Σ½m|v|² 单调同序）。
    ke: f32,
    digest: u64,
    n_in: usize,
}

/// 位模式指纹（FNV-1a，与 `pbf_incompressibility_probe.rs` / `src/tests.rs` 同口径）。
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

/// 读一份场景（族 + 压力迭代 + FLIP 混合比），几何/种子/dt 与 PBF 档**逐字相同**。
/// `diag` = 打印末态过密**空间分布**（只给 `#[ignore]` 诊断用；正常判据不动）。
fn run(
    family: FluidFamily,
    pressure_iters: u32,
    flip_ratio: f32,
    ticks: usize,
    diag: bool,
) -> Metrics {
    let h = 0.1f32;
    let s = h / 2.0;
    let (cav, wall) = (0.35f32, 0.9f32);
    let cfg = FluidConfig {
        family,
        smoothing_radius: h,
        flip_pressure_iterations: pressure_iters,
        flip_ratio,
        ..FluidConfig::default()
    };
    let rho0 = cfg.rest_density;
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
        sink: 0.0,
        height_keep: 0.0,
        ke: 0.0,
        digest: 0,
        n_in: 0,
    };
    let y0 = com_y(&f);
    let ymax0 = max_y(&f);
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
                // **体内部**：核完整（ρ/ρ0 ≥ 0.9）且离所有边界 > h（口径同 PBF 档）。
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
    m.sink = y0 - com_y(&f);
    m.height_keep = max_y(&f) / ymax0.max(f32::MIN_POSITIVE);
    m.ke = f.velocities().iter().map(|v| v.length_squared()).sum();
    m.digest = digest(&f);
    if diag {
        diag_over_density(&f, rho0);
    }
    m
}

/// 流体最高点 y。
fn max_y(f: &FluidSystem) -> f32 {
    f.positions().iter().map(|p| p.y).fold(f32::MIN, f32::max)
}

/// 流体段质心 y。
fn com_y(f: &FluidSystem) -> f32 {
    let n = f.positions().len().max(1);
    f.positions().iter().map(|p| p.y).sum::<f32>() / n as f32
}

/// 自由落体参照：`T` 秒自由落体下沉量（判据 ② 的分母）。
fn free_fall(t_sec: f32) -> f32 {
    0.5 * 9.81 * t_sec * t_sec
}

/// 打印读数表（`god_gate` 的单函数 120 行硬线：表格与断言分开，两边都短）。
fn print_table(ff: f32, rows: &[(&str, &Metrics)]) {
    eprintln!(
        "FLIP 档（60 tick × 4 子步，dt=1/60，水柱/水槽口径同 PBF 档；自由落体参照 ½gT² = {ff:.3} m）"
    );
    eprintln!(
        "{:10} {:11} {:11} {:11} {:8} {:8} {:10} {:11} {:10}",
        "档位", "over", "over_in", "under", "vmax", "reach", "sink", "height_keep", "ke"
    );
    for (name, m) in rows {
        eprintln!(
            "{name:10} {:11.3e} {:11.3e} {:11.3e} {:8.4} {:8.4} {:10.3e} {:11.4} {:10.3e}",
            m.over, m.over_in, m.under, m.vmax, m.reach, m.sink, m.height_keep, m.ke
        );
    }
    eprintln!(
        "内部样本数（末 10 tick 窗口内最大）：{}",
        rows.iter()
            .map(|(n, m)| format!("{n} {}", m.n_in))
            .collect::<Vec<_>>()
            .join(" / ")
    );
}

#[test]
fn flip_holds_hydrostatic_column_and_low_dissipation() {
    let sph = run(FluidFamily::CpuSph, 0, 0.0, 60, false);
    let coarse = run(FluidFamily::CpuFlip, 20, 0.95, 60, false);
    let base = run(FluidFamily::CpuFlip, 80, 0.95, 60, false);
    let fine = run(FluidFamily::CpuFlip, 160, 0.95, 60, false);
    let pic = run(FluidFamily::CpuFlip, 80, 0.0, 60, false);
    let ff = free_fall(1.0);
    let rows: [(&str, &Metrics); 5] = [
        ("wcsph(读)", &sph),
        ("flip×20", &coarse),
        ("flip×80", &base),
        ("flip×160", &fine),
        ("flipPIC×80", &pic),
    ];
    print_table(ff, &rows);
    // 非空样本守卫：`over_in` 以 `f32::MIN` 起步（哨兵恒小于真值）⇒ 样本为 0 时
    // `over_in` 比较会**空通过**。这条只钉样本量（读数口径的可信度），不是物理判据。
    for (name, m) in &rows {
        if *name == "flip×20" {
            continue; // 判别力档：水柱都撑不住 ⇒ 内部样本本来就少，不作读数可信度要求
        }
        assert!(
            m.n_in >= 32,
            "{name} 内部样本不足（{}）⇒ over_in 读数不可信",
            m.n_in
        );
    }
    // 表面完整性：任一档不许炸开（阈值同 PBF 档：棱线鬼影角射流两族都有 ⇒ vmax 4.0，
    // 水槽内最远点 |p| ≈ 0.61 ⇒ reach 0.8）。
    for (name, m) in &rows {
        assert!(
            m.reach < 0.8 && m.vmax < 4.0,
            "{name} 表面炸开：reach {} / vmax {}",
            m.reach,
            m.vmax
        );
    }
    // 判据 ① 的**判别力**：同一个场景只把压力迭代降到 20（默认档的 1/4），水柱必须肉眼可见地
    // 撑不住（实测 0.482 vs 0.854）。这条保证 ① 不是恒真断言——投影若真的形同虚设，
    // `height_keep` 就是走这条路径塌下去的。
    assert!(
        coarse.height_keep < 0.60,
        "低迭代档竟撑住了水柱（height_keep {}）⇒ 本判据没有判别力，先查场景/读数",
        coarse.height_keep
    );
    // 判据 ① 正题：默认档（80 次 Jacobi）必须撑住水柱（实测 0.854 ⇒ 阈值 0.80 留 6% 余量）。
    assert!(
        base.height_keep >= 0.80,
        "默认压力迭代档没撑住静水柱：height_keep {} < 0.80（水线掉幅 > 20%）",
        base.height_keep
    );
    // 判据 ① 收敛单调：迭代越多不许更差（容许末位浮点级回退）。实测 0.482 → 0.854 → 0.930。
    assert!(
        coarse.height_keep < base.height_keep && fine.height_keep >= base.height_keep * 0.98,
        "静水支撑没随压力迭代单调变好：×20 {} / ×80 {} / ×160 {}",
        coarse.height_keep,
        base.height_keep,
        fine.height_keep
    );
    // 判据 ② 净沉降：相对自由落体（实测 80 档 0.059 m = 自由落体的 1.2%；阈值取 5% 粗防线，
    // 抓"重力加了两次/完全没投影"这类量纲级错误）。
    assert!(
        base.sink < 0.05 * ff,
        "FLIP 池内质心沉降 {} m 超过自由落体 {:.3} m 的 5%",
        base.sink,
        ff
    );
    // 判据 ② 的收敛单调：迭代更多 ⇒ 沉降不许更大（实测 0.111 → 0.0592 → 0.0456）。
    assert!(
        base.sink < coarse.sink && fine.sink <= base.sink * 1.05,
        "沉降没随压力迭代单调变好：×20 {} / ×80 {} / ×160 {}",
        coarse.sink,
        base.sink,
        fine.sink
    );
    // 判据 ③ 低耗散：同一场景、同一压力求解，只换混合比 θ（0.95 vs 0）⇒ FLIP 档末态动能必须
    // 高于纯 PIC（实测 15.5 vs 0.85，余量 18×）。PIC 每步把粒子速度重置成网格插值 ⇒ 逐子步抹平。
    assert!(
        pic.ke > 0.0,
        "PIC 档没动起来（ke {}）⇒ 下面的比较是空断言",
        pic.ke
    );
    assert!(
        base.ke > pic.ke,
        "FLIP 的末态动能未高于 PIC（flip {} vs pic {}）⇒ 混合比没起作用或桥接项写成了 PIC",
        base.ke,
        pic.ke
    );
}

#[test]
fn flip_is_bitwise_reproducible() {
    // 确定性契约（`adr/0004`）：Jacobi（全量读旧、写新）+ 固定遍历序 ⇒ 同输入两跑逐位一致。
    let a = run(FluidFamily::CpuFlip, 80, 0.95, 12, false);
    let b = run(FluidFamily::CpuFlip, 80, 0.95, 12, false);
    assert_eq!(a.digest, b.digest, "FLIP 两跑不逐位一致");
    assert!(
        a.reach > 0.0 && a.vmax > 0.0,
        "用例没动起来（reach {} / vmax {}）",
        a.reach,
        a.vmax
    );
}

/// `#[ignore]` 诊断（不进 CI）：打印 FLIP/WCSPH 末态过密的**空间分布**，用来分辨
/// "网格投影造会聚流"（过密聚集在壁角/底面/贴邻对）与"纯几何起始条件"（均匀过密）。
/// 跑法：`cargo test -p vxl-phys-fluid --test flip_incompressibility_probe -- --ignored --nocapture`
#[test]
#[ignore]
fn diag_over_density_layout() {
    eprintln!("== wcsph ==");
    let _ = run(FluidFamily::CpuSph, 0, 0.0, 60, true);
    eprintln!("== flip×40 θ=0.95 ==");
    let _ = run(FluidFamily::CpuFlip, 40, 0.95, 60, true);
    eprintln!("== flip×80 θ=0.95 ==");
    let _ = run(FluidFamily::CpuFlip, 80, 0.95, 60, true);
}

/// `#[ignore]` 诊断（不进 CI）：FLIP 的**压力迭代收敛趋势**——静水支撑到底是受限于求解器
/// 迭代数（⇒ 加迭代/CG 能治）还是卡在平台上（⇒ 公式本身有问题）。跑法同上。
#[test]
#[ignore]
fn diag_pressure_iteration_convergence() {
    eprintln!("迭代数  over       over_in    sink       height_keep  ke");
    for it in [10u32, 20, 40, 80, 160, 320] {
        let m = run(FluidFamily::CpuFlip, it, 0.95, 60, false);
        eprintln!(
            "{it:6}  {:10.3e} {:10.3e} {:10.3e} {:11.4} {:10.3e}",
            m.over, m.over_in, m.sink, m.height_keep, m.ke
        );
    }
}

/// 过密**空间分布**诊断：分桶计数 + 最差的 8 个粒子的位置（`#[ignore]` 用，不参与判据）。
///
/// 目的：分清"网格投影造出了会聚流"（⇒ 过密集中在**壁角/底面**或成**贴邻对**）与
/// "纯几何/初始条件"（⇒ 均匀过密）。SPH 密度对 FLIP 只是**被动诊断**（求解器不读 ρ），
/// 所以这条诊断是**定位**用，不是判据。
fn diag_over_density(f: &FluidSystem, rho0: f32) {
    let mut bins = [0usize; 6];
    for &rho in f.densities() {
        let r = rho / rho0 - 1.0;
        let b = if r < 0.05 {
            0
        } else if r < 0.2 {
            1
        } else if r < 0.5 {
            2
        } else if r < 1.0 {
            3
        } else if r < 2.0 {
            4
        } else {
            5
        };
        bins[b] += 1;
    }
    eprintln!(
        "  过密分布 r-1∈ [..0.05) {} | [0.05,0.2) {} | [0.2,0.5) {} | [0.5,1) {} | [1,2) {} | [2,..) {}",
        bins[0], bins[1], bins[2], bins[3], bins[4], bins[5]
    );
    let ymin = f.positions().iter().map(|p| p.y).fold(f32::MAX, f32::min);
    let ymax = f.positions().iter().map(|p| p.y).fold(f32::MIN, f32::max);
    let xmax = f
        .positions()
        .iter()
        .map(|p| p.x.abs().max(p.z.abs()))
        .fold(0.0f32, f32::max);
    eprintln!("  末态几何：y∈[{ymin:.4},{ymax:.4}] |x|z max {xmax:.4}");
    let mut idx: Vec<usize> = (0..f.densities().len()).collect();
    idx.sort_by(|&a, &b| (f.densities()[b] / rho0).total_cmp(&(f.densities()[a] / rho0)));
    for &i in idx.iter().take(8) {
        let p = f.positions()[i];
        eprintln!(
            "  #{i} r-1 {:8.3} p=({:7.4},{:7.4},{:7.4}) 边距 x/z {:6.4} y {:6.4} |v| {:.4}",
            f.densities()[i] / rho0 - 1.0,
            p.x,
            p.y,
            p.z,
            0.35 - p.x.abs().max(p.z.abs()),
            p.y,
            f.velocities()[i].length()
        );
    }
}
