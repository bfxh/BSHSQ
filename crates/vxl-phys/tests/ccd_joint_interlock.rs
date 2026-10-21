//! **已知互锁：CCD 扫掠 × 关节约束**（`OPEN-PROBLEMS.md` P17 的判据入口）。
//!
//! 场景 = 真实性基准 R6 的同款（10 节球铰链，一端挂静态体，水平摆放后大幅摆动）。
//! 读数（`sep` = 关节两侧锚点的世界距离）：
//!
//! | CCD 档 | `sep_p99` | `>5cm 违规率` | R6 门（≤0.012 / =0） |
//! |---|---|---|---|
//! | 默认（关，`INFINITY`） | **0.00966** | **0.000%** | ✅ 过 |
//! | 显式开（10 m/s） | **0.12382** | **1.967%** | ❌ 13× 超阈 |
//!
//! **机制**（推断，未逐行验证）：扫掠命中后把体**钳位到上一安全采样点**并清法向速度，
//! 这一步只看单体的位移、**不看关节约束** ⇒ 相邻链节的钳位彼此不一致 ⇒ 锚点分离被拉大。
//!
//! ✅ **2026-10-13 已修**（`joints/project.rs`：CCD 之后按关节连通分量做**位置级投影**）：
//! 上表"开档"那行现在是 **`sep_p99 = 0.00986`、`>5cm = 0.000%`**（与关档同一量级）。
//! 本文件因此从"缺陷快照"变成**回归判据**：两档都必须过 R6 门。
use vxl_phys::{BodyBuildExt, Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::Mat3;

/// 10 节球铰链（照抄 `bench/realism-probe` 的 R6 场景，`ccd` 是显式阈值）。
fn chain(ccd: f32, ticks: usize) -> (f32, f32) {
    let mut w = World::new(PhysConfig {
        ccd_speed_threshold: ccd,
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    });
    let anchor = w.add_static(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 20.0, 0.0),
        Quat::IDENTITY,
    );
    let mut prev = anchor;
    let mut links = Vec::new();
    for k in 0..10 {
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::new(0.25, 0.05, 0.05),
            },
            Vec3::new(0.25 + 0.5 * k as f32, 20.0, 0.0),
            Quat::IDENTITY,
            1000.0,
        );
        let la = if k == 0 {
            Vec3::ZERO
        } else {
            Vec3::new(0.25, 0.0, 0.0)
        };
        let lb = Vec3::new(-0.25, 0.0, 0.0);
        w.add_joint(Joint::new(JointKind::Spherical, prev, id, la, lb));
        links.push((prev as usize, id as usize, la, lb));
        prev = id;
    }
    let mut seps = Vec::new();
    for _ in 0..ticks {
        w.step();
        for &(a, b, la, lb) in &links {
            let (pa, qa) = w.bodies.pose(a);
            let (pb, qb) = w.bodies.pose(b);
            let wa = pa + Mat3::from_quat(qa).mul_vec3(la);
            let wb = pb + Mat3::from_quat(qb).mul_vec3(lb);
            seps.push((wb - wa).length());
        }
    }
    seps.sort_by(f32::total_cmp);
    let p99 = seps[(seps.len() * 99) / 100];
    let v5 = 100.0 * seps.iter().filter(|&&s| s > 0.05).count() as f32 / seps.len() as f32;
    (p99, v5)
}

/// **回归判据（P17 修好之后）**：开/关两档的关节分离都必须过 R6 门（`≤0.012 / =0`）。
/// 修法见 `vxl-phys-solver/src/joints/project.rs`（CCD 之后按**关节连通分量**做位置级投影；
/// 只修被钳体那一条会把误差推给邻接 ⇒ 必须闭包整条链）。
#[test]
fn ccd_does_not_break_jointed_chain() {
    let (p99_off, v5_off) = chain(f32::INFINITY, 600);
    let (p99_on, v5_on) = chain(10.0, 600);
    println!(
        "[互锁快照] 10 节球链：关档 sep_p99 = {p99_off:.5}、>5cm {v5_off:.3}%｜开档（10 m/s）sep_p99 = {p99_on:.5}、>5cm {v5_on:.3}%"
    );
    assert!(
        p99_off <= 0.012 && v5_off == 0.0,
        "关档（默认）必须过 R6 门（sep_p99 ≤ 0.012、>5cm = 0）：实得 {p99_off} / {v5_off}"
    );
    assert!(
        p99_on <= 0.012 && v5_on == 0.0,
        "开 CCD（10 m/s）也必须过 R6 门（P17 的互锁已修：位置级投影把锚点拉回）——实得 {p99_on} / {v5_on}"
    );
}
