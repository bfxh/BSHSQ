//! **候选网格 vs 全扫：逐位一致 + 加速读数**（2026-10-08）。
//!
//! 压力/黏性的对循环走 `candidate_ids`（有网格 = 只取所在格的登记表；无网格 = 全扫）。
//! 网格**只裁候选、不改求和序** ⇒ 两条路必须逐位一致；不建网格时每核对全扫 = O(n²)。
//!
//! 判据分两层（与仓库既有先例同款）：
//! ① **逐位一致**（机器无关，进 CI）；
//! ② 微基准**只报数不判**（时间类断言在 CI 上不稳）。
//!
//! ⚠️ **量具修正（2026-10-08 晚）**：旧版"否证"是**无效测量**——两臂都没建网格
//! （`rebuild_grid` 从未被调用），所谓差值来自运行顺序/热漂移 ⇒ 旧结论作废。本版
//! `Arm::GridPerStep` 每步平移后**真的建表**（库内单测断言 `grid.is_some()`），两臂**交替**跑
//! 并报极差（差 ≤ 极差 = 测不出）；且分「稠密云 / 局部云」两场景——核影响半径 ≈ 场半宽时，
//! 任何网格都不可能加速（求 σ 本就等于全和），这正是旧场景测不出收益的结构性原因。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Mat3, Vec3};
use vxl_phys_splat::dynamics::{step_dynamics, SplatDynamics};
use vxl_phys_splat::pressure::{apply_self_pressure, SelfPressure};
use vxl_phys_splat::viscosity::{apply_self_viscosity, SelfViscosity};
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 稠密随机云（旧量具场景）：σ = 0.3、中心在 [-1,1]³ ⇒ 4σ = 1.2 ≈ 场半宽，
/// 每核影响域覆盖几乎全场（无局部性可用）。留作**对照**（说明此处网格不该有收益）。
fn field(n: usize, grid_min_splats: usize) -> GaussianSplatField {
    field_scaled(n, grid_min_splats, 0.3, 1.0)
}

/// 物理量级核云：σ = 0.1、场半宽 = cbrt(n)·σ ⇒ 核间距与 σ 同量级（有真实局部性）。
fn field_local(n: usize, grid_min_splats: usize) -> GaussianSplatField {
    let half = 0.1 * (n as f32).cbrt();
    field_scaled(n, grid_min_splats, 0.1, half)
}

/// 核场骨架：`grid_min_splats` 决定"建不建网格"（0 = 总建；`usize::MAX` = 永不建 = 全扫）。
fn field_scaled(n: usize, grid_min_splats: usize, sigma: f32, half: f32) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.medium_density = 1000.0;
    f.grid_min_splats = grid_min_splats;
    let mut st = 0x1234_5678u32;
    let mut next = move || {
        st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (st >> 8) as f32 / (1u32 << 24) as f32
    };
    for _ in 0..n {
        f.push(Splat {
            center: Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0) * half,
            scale: Vec3::splat(sigma),
            rot: Mat3::IDENTITY,
            opacity: 0.5 + next(),
            color: [1.0, 1.0, 1.0],
        });
    }
    f
}

/// 两臂：`Brute` = 不建网格（每核全扫）；`GridPerStep` = 每步（平移置脏后）**建一次网格**，
/// 让对循环真的吃到候选表 —— 这才是"网格 vs 全扫"的有效对照。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Arm {
    Brute,
    GridPerStep,
}

/// 两个量具场景：`Dense` = 旧随机云（无局部性，作对照）；`Local` = 物理量级核云。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Scene {
    Dense,
    Local,
}

fn scene_field(scene: Scene, n: usize, grid_min_splats: usize) -> GaussianSplatField {
    match scene {
        Scene::Dense => field(n, grid_min_splats),
        Scene::Local => field_local(n, grid_min_splats),
    }
}

/// 混合尺度 + 各向异性核：专门压"逐核半径登记"这条覆盖前提——bin 按**中位**尺度取，
/// 个别大核的登记范围必须自己撑开，否则大核覆盖到的查询点会被漏掉。
fn mixed_field(grid_min_splats: usize) -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    f.grid_min_splats = grid_min_splats;
    let mut st = 0x9e37_79b9u32;
    let mut next = move || {
        st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (st >> 8) as f32 / (1u32 << 24) as f32
    };
    for i in 0..300usize {
        let base = 0.02 + (i % 7) as f32 * 0.03; // 0.02–0.20
        let s = if i % 11 == 0 { 1.0 } else { base }; // 少数大核把分布拉开
        f.push(Splat {
            center: Vec3::new(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0),
            scale: Vec3::new(s, s * 0.5, s * 1.5),
            rot: Mat3::IDENTITY,
            opacity: 0.5 + next(),
            color: [1.0, 1.0, 1.0],
        });
    }
    f
}

/// 跑 `steps` 步「压力 + 黏性」，返回逐核速度（位模式比较用）。
fn run(n: usize, arm: Arm, steps: usize, scene: Scene) -> Vec<u32> {
    // `grid_min_splats = 0` ⇒ `rebuild_grid()` 一定会建（被 0 门槛挡住的分支走不到）
    let grid_min_splats = if arm == Arm::GridPerStep {
        0
    } else {
        usize::MAX
    };
    let mut f = scene_field(scene, n, grid_min_splats);
    let v0: Vec<Vec3> = (0..n)
        .map(|k| {
            let t = k as f32 * 0.37;
            Vec3::new(t.sin(), t.cos(), -t.sin())
        })
        .collect();
    assert!(f.set_kernel_velocities(&v0));
    for _ in 0..steps {
        // 模拟"每步位置都变"：先平移置脏、再按臂决定是否重建候选表（Brute 臂永不建）。
        f.translate(Vec3::new(0.001, 0.0, 0.0));
        if arm == Arm::GridPerStep {
            f.rebuild_grid();
        }
        apply_self_pressure(&mut f, 1.0 / 120.0, SelfPressure::default());
        apply_self_viscosity(&mut f, 1.0 / 120.0, SelfViscosity::default());
    }
    f.kernel_velocities()
        .iter()
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

#[test]
fn grid_and_brute_force_agree_bitwise() {
    let grid = run(120, Arm::GridPerStep, 8, Scene::Dense);
    let brute = run(120, Arm::Brute, 8, Scene::Dense);
    assert_eq!(grid.len(), brute.len());
    assert_eq!(grid, brute, "候选网格与全扫必须逐位一致");
    assert!(grid.iter().any(|b| *b != 0), "用例非平凡：速度确实变了");
    // 金丝雀在库内单测 `grid_matches_brute_force_bitwise`（同参数断言 `grid.is_some()`）：
    // 集成测试看不到私有字段 ⇒ 这里守"结果逐位一致"，"真建了表"由单元测试守。
}

/// 覆盖前提的**对抗用例**：混合尺度 + 各向异性核，网格查询与全扫在 200 个点上逐位一致。
/// 若登记范围误用"统一半径/中位半径"，大核覆盖的点就会被漏 ⇒ 这条先红。
#[test]
fn mixed_scale_grid_never_drops_kernels() {
    let mut grid = mixed_field(0);
    grid.rebuild_grid();
    // 该参数下必建表（n=300、dims≈32³ ≪ GRID_MAX_BINS）；"真的建了表"由库内单测
    // `grid_matches_brute_force_bitwise` 的 `grid.is_some()` 断言守（集成测试看不到私有字段）。
    let brute = mixed_field(usize::MAX);
    for i in 0..200usize {
        let t = i as f32 * 0.173;
        let p = Vec3::new(t.sin() * 1.6, (t * 1.3).cos() * 1.6, (t * 0.7).sin() * 1.6);
        let (sg, gg) = grid.density_grad(p);
        let (sb, gb) = brute.density_grad(p);
        assert_eq!(sg.to_bits(), sb.to_bits(), "点 {i}：σ 不一致");
        for (a, b) in [(gg.x, gb.x), (gg.y, gb.y), (gg.z, gb.z)] {
            assert_eq!(a.to_bits(), b.to_bits(), "点 {i}：∇σ 不一致");
        }
    }
}

/// 单点读数：`rounds` 轮、两臂**交替**跑（奇偶轮换先后）+ 报极差。本仓 A/B 纪律——
/// 均值之差 ≤ 两臂极差就标"测不出"，不看单跑。
fn bench_one(
    label: &str,
    scene: Scene,
    n: usize,
    steps: usize,
    rounds: usize,
    run_fn: fn(usize, Arm, usize, Scene) -> Vec<u32>,
) {
    let mut grid = Vec::new();
    let mut brute = Vec::new();
    for r in 0..rounds {
        let arms = if r % 2 == 0 {
            [Arm::GridPerStep, Arm::Brute]
        } else {
            [Arm::Brute, Arm::GridPerStep]
        };
        for arm in arms {
            let t = std::time::Instant::now();
            let _ = run_fn(n, arm, steps, scene);
            let ms = t.elapsed().as_secs_f64() * 1e3;
            match arm {
                Arm::GridPerStep => grid.push(ms),
                Arm::Brute => brute.push(ms),
            }
        }
    }
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let spread = |v: &[f64]| {
        let (lo, hi) = v
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), &x| (lo.min(x), hi.max(x)));
        hi - lo
    };
    let (mg, mb) = (mean(&grid), mean(&brute));
    let inconclusive = (mg - mb).abs() <= spread(&grid).max(spread(&brute));
    println!(
        "[{label}] n={n:5} × {steps} 步：网格 {:8.2} ms（极差 {:.2}）| 全扫 {:8.2} ms（极差 {:.2}）| 加速 {:.2}×{}",
        mg,
        spread(&grid),
        mb,
        spread(&brute),
        mb / mg,
        if inconclusive { "  ⚠️测不出（差 ≤ 极差）" } else { "" }
    );
}

#[test]
fn grid_speedup_microbench_report_only() {
    // 只报数不判（时间类断言在 CI 上不稳）。
    for (label, scene) in [("稠密云", Scene::Dense), ("局部云", Scene::Local)] {
        for n in [200usize, 400, 800, 1600] {
            bench_one(label, scene, n, 10, 4, run);
        }
    }
}

/// 走**完整动力学档**（压力 → 黏性 → 重力积分）`steps` 步，返回末态逐核速度（位模式比较用）。
/// `GridPerStep` 臂靠 `step_dynamics` 内部**每步** `rebuild_grid`；`Brute` 臂把门槛设成
/// `usize::MAX` ⇒ 重建短路 ⇒ 全扫。
fn run_dynamics(n: usize, arm: Arm, steps: usize, scene: Scene) -> Vec<u32> {
    let grid_min_splats = if arm == Arm::GridPerStep {
        0
    } else {
        usize::MAX
    };
    let mut f = scene_field(scene, n, grid_min_splats);
    let cfg = SplatDynamics {
        pressure: Some(SelfPressure::default()),
        viscosity: Some(SelfViscosity::default()),
        ..SplatDynamics::default()
    };
    let providers = NoProviders;
    for _ in 0..steps {
        step_dynamics(&mut f, 1.0 / 120.0, &providers, &[], cfg);
    }
    f.kernel_velocities()
        .iter()
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

/// **世界路径判据**：`step_dynamics` 每步重建候选表后，压力/黏性与全扫**逐位一致**。
/// 若重建漏掉某核（覆盖前提坏了），这条直接红。
#[test]
fn dynamics_grid_matches_brute_force_bitwise() {
    let grid = run_dynamics(120, Arm::GridPerStep, 8, Scene::Local);
    let brute = run_dynamics(120, Arm::Brute, 8, Scene::Local);
    assert_eq!(grid.len(), brute.len());
    assert_eq!(grid, brute, "动力学档：候选网格与全扫必须逐位一致");
    assert!(grid.iter().any(|b| *b != 0), "用例非平凡：速度确实变了");
}

/// 世界路径读数：完整动力学步（含每步重建）的两臂耗时。
/// ⚠️ 稠密云在动力学档下会**发散**（σ=0.3 核重叠 ⇒ 压力爆炸）⇒ `world_bounds` 到天文数字、
/// 网格按格数上限退回全扫 ⇒ 读数 ≈1.00×。该场景只作"发散不 panic"的健壮性证据；收益读局部云。
#[test]
fn dynamics_speedup_microbench_report_only() {
    for (label, scene) in [("稠密云", Scene::Dense), ("局部云", Scene::Local)] {
        for n in [400usize, 800, 1600] {
            bench_one(label, scene, n, 10, 4, run_dynamics);
        }
    }
}
