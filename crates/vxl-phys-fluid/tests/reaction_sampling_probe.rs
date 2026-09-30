//! **C2 的前置测量（仪表，只打印不断言）**：量"**末子步快照当 tick 常量**"这个采样口径的偏置
//! ——`PLAN-COUPLING.md` §2 C1/§5 C2 与 D1 决策的输入。
//!
//! **被量的是什么**：门槛档（facade）把反作用当**整 tick 常量力**施加，而 `boundary_reactions()`
//! 给的是**末子步**的力（`bforce` 每子步清零重累，`fluid_force.rs:171-174`）。若力在一个 tick 内
//! 变化，则"末子步采样"与"tick 时间平均"不是同一个数 ⇒ 交付的冲量有偏置。
//!
//! **怎么隔离它（零引擎改动，只用公开 API）**：把物理 tick 拆成 `SUB` 次**细 tick** 调用
//! （`FluidConfig{substeps:1}` + `step(dt_fine)`）⇒ 每次调用后读 `boundary_reactions()`，就拿到
//! 一个 tick 内 `SUB` 个**逐子步**样本 ⇒ 直接算：
//! - `F_last`（末子步，现口径） vs `F̄ = Σ F_s/SUB`（时间平均，C2 口径）；
//! - 相对偏置 `|F_last − F̄| / |F̄|`（信号 `y` 与噪声 `x/z` 分开看）；
//! - **P7 那个精度读数**（窗口均值 `F̄_y/ρVg`）在两种口径下差多少 —— 即"C2 会不会动 P7 表"。
//!
//! 跑法（**release**，探针只打印）：
//! ```bash
//! cargo test --release -p vxl-phys-fluid --test reaction_sampling_probe -- --ignored --nocapture
//! ```

use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};

/// 物理 tick 内的细 tick 数（= 该 tick 内的子步采样数）。
const SUB: usize = 4;
/// 静置物理 tick 数（窗口前瞬态；与 `boundary_accuracy_probe` 同口径）。
const SETTLE: usize = 60;
/// 统计窗口（物理 tick 数）。
const WIN: usize = 120;

/// 水槽（半空间族：地板 + 四壁内面）——**围水必须走提供者通道**（`M1-EXIT.md` §4：只用边界
/// 粒子搭盆会漏，实测水会从缝里走空 ⇒ 本探针第一版就栽在这，水面掉到 y ≈ −25）。
/// 与 `boundary_accuracy_probe.rs` 的 `Basin` 同款（那里是 crate 外的既有先例）。
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
                continue; // 墙只到堰顶
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

fn still_pose(pos: Vec3) -> BodyPose {
    BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    }
}

/// 取某一体的反作用 `(F, τ)`。
fn reaction_of(f: &FluidSystem, body: u32) -> (Vec3, Vec3) {
    f.boundary_reactions()
        .iter()
        .find(|r| r.0 == body)
        .map(|r| (r.1, r.2))
        .unwrap_or((Vec3::ZERO, Vec3::ZERO))
}

/// 一个物理 tick：`SUB` 次细 tick，返回 `(逐子步力样本, 逐子步力矩样本)`。
fn tick_samples(
    f: &mut FluidSystem,
    bodies: &[(u32, Shape, BodyPose)],
    providers: &Basin,
    body: u32,
) -> (Vec<Vec3>, Vec<Vec3>) {
    let dt_fine = 1.0 / 60.0 / SUB as f32;
    let (mut fs, mut ts) = (Vec::with_capacity(SUB), Vec::with_capacity(SUB));
    let nb = f.set_boundary_particles(bodies);
    assert!(nb > 0, "边界粒子为 0 ⇒ 场景退化");
    for _ in 0..SUB {
        // 与门面同频：边界每个**物理 tick** 重建一次（引擎的子步循环内部不重建）。
        f.step(dt_fine, providers);
        let (ff, tt) = reaction_of(f, body);
        fs.push(ff);
        ts.push(tt);
    }
    (fs, ts)
}

/// 场景诊断（探针的职责：先说清"场景到底在不在"）——边界粒数 / 流体数 / 水面 / 盒反作用。
fn scene_diag(f: &mut FluidSystem, bodies: &[(u32, Shape, BodyPose)], body: u32) -> String {
    let nb = f.set_boundary_particles(bodies);
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for p in f.positions() {
        lo = lo.min(p.y);
        hi = hi.max(p.y);
    }
    let bf_max = f
        .boundary_forces()
        .iter()
        .map(|v| v.length())
        .fold(0.0f32, f32::max);
    let (fr, _) = reaction_of(f, body);
    format!(
        "场景诊断：边界粒子 {nb} | 流体 {} | 水面 y ∈ [{lo:.3}, {hi:.3}] | 逐粒 |bforce| max {bf_max:.4} | 盒反作用 {fr:?}",
        f.len()
    )
}

#[test]
#[ignore = "仪表：只打印（需 release 才跑得快）"]
fn last_substep_vs_tick_average() {
    // 场景与 `boundary_accuracy_probe.rs` 同款（水块摊到 0.8² 腔 ⇒ 水深 0.675、盒摆柱中）。
    let h = 0.1f32;
    let s = h / 2.0;
    let (cav, wall) = (0.4f32, 0.9f32);
    let half = 0.06f32;
    let n = (0.6 / s).round() as usize;
    let nz = (1.2 / s).round() as usize;
    let cfg = FluidConfig {
        smoothing_radius: h,
        substeps: 1, // 细 tick 驱动 ⇒ 每次 `step` 恰好一个子步（物理与 `substeps = SUB` 同构）
        xsph_viscosity: 0.05,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.3, 0.0, -0.3), [n, nz, n], s);
    f.set_boundaries(&[0]); // 围水 = Basin 提供者（id 0）
    let vol = (n as f32 * s) * (n as f32 * s) * (nz as f32 * s);
    let depth = vol / ((2.0 * cav) * (2.0 * cav));
    let bodies = vec![(
        7u32,
        Shape::Box {
            half: Vec3::splat(half),
        },
        still_pose(Vec3::new(0.0, 0.5 * depth, 0.0)),
    )];
    let box_body = 7u32;
    let basin = Basin { cav, wall };

    for _ in 0..SETTLE {
        let _ = tick_samples(&mut f, &bodies, &basin, box_body);
    }
    println!("{}", scene_diag(&mut f, &bodies, box_body));

    let mut rel_y = Vec::new();
    let mut rel_lat = Vec::new();
    let mut rel_tau = Vec::new();
    let mut max_spread_y = 0.0f32;
    let (mut sum_last, mut sum_mean) = (Vec3::ZERO, Vec3::ZERO);
    let (mut tau_last, mut tau_mean) = (Vec3::ZERO, Vec3::ZERO);
    for _ in 0..WIN {
        let (fs, ts) = tick_samples(&mut f, &bodies, &basin, box_body);
        let mean = fs.iter().fold(Vec3::ZERO, |a, b| a + *b) * (1.0 / SUB as f32);
        let mean_t = ts.iter().fold(Vec3::ZERO, |a, b| a + *b) * (1.0 / SUB as f32);
        let last = *fs.last().unwrap();
        let last_t = *ts.last().unwrap();
        sum_last += last;
        sum_mean += mean;
        tau_last += last_t;
        tau_mean += mean_t;
        // 相对偏置：|末 − 均| / |均|（逐分量只为看 x/z 噪声，故用模长）
        rel_y.push((last.y - mean.y).abs() / mean.y.abs().max(1e-9));
        rel_lat.push(((last.x - mean.x).abs() + (last.z - mean.z).abs()) / mean.y.abs().max(1e-9));
        rel_tau.push((last_t - mean_t).length() / mean_t.length().max(1e-9));
        // tick 内**逐子步**离散度（相对 |F̄_y|）
        let hi = fs.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
        let lo = fs.iter().map(|v| v.y).fold(f32::INFINITY, f32::min);
        max_spread_y = max_spread_y.max((hi - lo) / mean.y.abs().max(1e-9));
    }
    let inv = 1.0 / WIN as f32;
    let (avg_last, avg_mean) = (sum_last * inv, sum_mean * inv);
    let (t_last, t_mean) = (tau_last * inv, tau_mean * inv);
    let stat = |v: &[f32]| {
        let mut s = v.to_vec();
        s.sort_by(|a, b| a.total_cmp(b));
        (
            s[s.len() / 2],
            s[s.len() - 1],
            v.iter().sum::<f32>() / v.len() as f32,
        )
    };
    let (my, xy, ay) = stat(&rel_y);
    let (ml, xl, al) = stat(&rel_lat);
    let (mt, xt, at) = stat(&rel_tau);
    let expect = f.config().rest_density * (2.0 * half).powi(3) * 9.81;
    println!("== C2 采样口径偏置（SUB = {SUB}，窗口 {WIN} 物理 tick，静置 {SETTLE}）==");
    println!(
        "窗口均值 F_y：末子步口径 {:.4} / 时间平均口径 {:.4} N（差 {:+.2}%）| ρVg = {:.4} N",
        avg_last.y,
        avg_mean.y,
        100.0 * (avg_last.y - avg_mean.y) / avg_mean.y.abs().max(1e-9),
        expect
    );
    println!(
        "  ⇒ P7 读数 F_y/ρVg：末子步 {:.3} / 时间平均 {:.3}（**C2 会不会动这张表**看这一行）",
        avg_last.y / expect,
        avg_mean.y / expect
    );
    println!(
        "  侧向（窗口均值）：末子步 ({:+.4}, {:+.4}) / 时间平均 ({:+.4}, {:+.4}) N",
        avg_last.x, avg_last.z, avg_mean.x, avg_mean.z
    );
    println!(
        "  |τ| 窗口均值：末子步 {:.4} / 时间平均 {:.4}",
        t_last.length(),
        t_mean.length()
    );
    println!(
        "逐 tick 相对偏置 |末 − 均|/|均|：F_y 中位 {my:.3} 最大 {xy:.3} 均值 {ay:.3}；\
         侧向/|F̄_y| 中位 {ml:.3} 最大 {xl:.3} 均值 {al:.3}；|τ| 中位 {mt:.3} 最大 {xt:.3} 均值 {at:.3}"
    );
    println!("tick 内逐子步离散度（峰峰/|F̄_y|）最大 {max_spread_y:.3}");

    // **金丝雀**（防空跑）：反作用必须真是非零的，否则上面全是"0/0 的统计"。
    assert!(
        sumsq_ok(avg_mean) || avg_mean.y.abs() > 1e-6,
        "反作用恒为零 ⇒ 场景没建立起接触 ⇒ 本探针空跑（mean = {avg_mean:?}）"
    );
}

/// 小工具：给金丝雀用（避免把断言写成对 `Vec3` 的直接比较）。
fn sumsq_ok(v: Vec3) -> bool {
    v.length_squared().is_finite() && v.length_squared() > 0.0
}
