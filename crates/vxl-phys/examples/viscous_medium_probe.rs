//! **黏性介质探针**（`SURVEY-RIGID-HARD-CASES §4.2②`：高黏介质里的刚体）。
//!
//! 介质 = **"冻结蜂蜜"**：流体自身 `gravity = 0` + 数值黏度（α / ε）全 0 ⇒ 粒子基本不动
//! （`VISC_TRACE=1` 会打出流体侧 `max|v|`）⇒ 采样是**静态精确**的 ⇒ 体的运动只由阻力律决定。
//! （常规水盆做不了定量对拍：`OPEN-PROBLEMS` P7 —— 2a/2b 是**定性档**。）
//!
//! 量什么（球在介质正中）：
//! ① **自由衰减**（`PhysConfig::gravity = 0` ⇒ 无浮力干扰）：与解析 `v(t) = v₀·e^{−t/τ}`、
//!    `τ = m/(6πμ·r)` 对拍；② **μ = 0 对照**（旧的二次阻力口径 ⇒ 更慢）；③ **静止金丝雀**
//!    （`v ≡ 0` ⇒ 介质残余噪声把体推动多少 —— 本档的"观测下限"）。
//!
//! 跑法：
//! ```text
//! cargo run -q --release -p vxl-phys --example viscous_medium_probe
//! VISC_MU=2.0 VISC_V0=0.5 VISC_TRACE=1 cargo run -q --release -p vxl-phys --example viscous_medium_probe
//! ```
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 水槽（与 `tests/fluid_coupling.rs` 同款：5×5 格外沿、中心一格围堰 ⇒ 内腔 0.5 m）。
fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

fn env_f32(name: &str, d: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(d)
}

const R: f32 = 0.05;
/// 体**密度**（`add_dynamic` 的第 4 参是密度，不是质量！）：r = 0.05 ⇒ m ≈ 0.600 kg。
const RHO_BODY: f32 = 1146.0;

/// 一档：`(μ, 初速, 重力, 打印前缀)` ⇒ 返回 `(tick, |v|)` 采样序列。
fn run(mu: f32, v0: Vec3, g: Vec3, label: &str, ticks: usize) -> Vec<f32> {
    let cfg = PhysConfig {
        gravity: g,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let v = tank(&mut w);
    // **"冻结蜂蜜"介质**：流体自身 `gravity = 0` + **数值黏度全 0** ⇒ 晶格无外力、粒子基本不动
    // ⇒ 介质采样是**静态精确**的（`ρ ≈ ρ0`、`v_med ≈ 0`）⇒ 体的运动只由**本刀的阻力律**决定。
    // （常规水盆里体-液的定量对拍做不到：`OPEN-PROBLEMS` P7 说 2a/2b 是**定性档**。）
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            dynamic_viscosity: mu,
            gravity: Vec3::ZERO,
            xsph_viscosity: 0.0,
            artificial_viscosity: 0.0,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    // 不需要挖空：介质是**冻结**的（流体自身零重力/零数值黏度 ⇒ 粒子基本不动），体与粒子之间
    // 只经**介质场**耦合（2a）—— 挖空反而会在体周围留下密度亏空、把 `frac_sub` 压低。
    let _ = w.add_fluid(sys, &[v]);
    let i = w.add_dynamic(
        Shape::Sphere { radius: R },
        Vec3::new(0.0, 1.25, 0.0),
        Quat::IDENTITY,
        RHO_BODY,
    ) as usize;
    w.bodies.linvel[i] = v0;
    let m = RHO_BODY * 4.0 / 3.0 * core::f32::consts::PI * R * R * R;
    let tau = m / (6.0 * core::f32::consts::PI * mu * R);
    println!(
        "  [{label}] m = {m:.4} kg、μ = {mu:6.2} Pa·s、τ = m/(6πμr) = {tau:.4} s（dt/τ = {:.4}）、\
         Re₀ = {:.3}",
        (1.0 / 60.0) / tau,
        1000.0 * v0.length() * 2.0 * R / mu
    );
    let mut out = Vec::with_capacity(ticks);
    for k in 0..ticks {
        w.step();
        let v = w.bodies.linvel[i].length();
        if std::env::var_os("VISC_TRACE").is_some() && k % 10 == 9 {
            let sys = &w.fluids()[0].0;
            let vs = sys.velocities();
            let maxv = vs.iter().fold(0.0f32, |a, x| a.max(x.length()));
            println!("      (流体侧：粒子 {}/max|v| = {maxv:.5} m/s)", vs.len());
        }
        if std::env::var_os("VISC_TRACE").is_some() && k < 14 {
            let p = w.bodies.position[i];
            let man: Vec<String> = w
                .manifolds()
                .iter()
                .filter(|m| m.a as usize == i || m.b as usize == i)
                .map(|m| {
                    let d = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
                    format!(
                        "({},{}) n=({:+.2},{:+.2},{:+.2}) d={d:+.4} pts={}",
                        m.a,
                        m.b,
                        m.normal.x,
                        m.normal.y,
                        m.normal.z,
                        m.points.len()
                    )
                })
                .collect();
            println!(
                "      ·t={k:>2} p=({:+.4},{:+.4},{:+.4}) v=({:+.4},{:+.4},{:+.4}) |v|={v:.5} 流形[{}]",
                p.x,
                p.y,
                p.z,
                w.bodies.linvel[i].x,
                w.bodies.linvel[i].y,
                w.bodies.linvel[i].z,
                man.join(" | ")
            );
        }
        out.push(v);
        if k % 10 == 9 || k == 0 {
            let t = (k + 1) as f32 / 60.0;
            let want = v0.length() * (-t / tau).exp();
            let ratio = if want > 1e-6 { v / want } else { f32::NAN };
            println!("      t = {t:5.3} s：|v| = {v:.5}（解析 {want:.5}，比 {ratio:.3}）");
        }
    }
    out
}

fn main() {
    let mu = env_f32("VISC_MU", 5.0);
    let v0 = env_f32("VISC_V0", 0.5);
    println!("黏性介质探针（球 r = {R} m、体密度 {RHO_BODY} kg/m³、静水介质）");
    run(mu, Vec3::new(v0, 0.0, 0.0), Vec3::ZERO, "自由衰减 μ>0", 30);
    run(0.0, Vec3::new(v0, 0.0, 0.0), Vec3::ZERO, "对照 μ=0", 30);
    // 静止金丝雀：介质（含数值噪声）把静止体推动多少 —— 本档的**观测下限**。
    let still = run(mu, Vec3::ZERO, Vec3::ZERO, "静止金丝雀", 60);
    let peak = still.iter().fold(0.0f32, |a, b| a.max(*b));
    println!("  ⇒ 静止体在 μ > 0 的介质里被推到的峰值 |v| = {peak:.5} m/s（观测下限）");
}
