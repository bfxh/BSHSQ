//! config：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// WCSPH 参数（PLAN-0.3 §1）。
#[derive(Clone, Debug)]
pub struct FluidConfig {
    /// **执行族选择器**：本 crate 受理 `CpuSph`（WCSPH）、`CpuPbf`（PBF 约束投影，见 `pbf`）与
    /// `CpuFlip`（FLIP 网格投影，见 `flip`）；GPU 族请走
    /// 门面的 `World::set_fluid_stepper`（`vxl-phys-gpu` 的 `GpuFluidStepper`）——设成 GPU 族
    /// **不会**触发 GPU 求解，`assemble` 会显式拒绝（不静默跑 CPU）。
    pub family: FluidFamily,
    /// **PBF 档消费**：密度约束迭代数（§4.8 的 3~4；每迭代 = 重建邻域 + 重评密度 + 一次投影）。
    /// `CpuSph` 档的密度是直接求和（单趟、无迭代）⇒ 该档下本字段不影响任何结果（`SPEC.md` §4.8）。
    pub density_iterations: DensityIterations,
    /// 静止密度 ρ0（kg/m³，水 = 1000）。
    pub rest_density: f32,
    /// 核半径 h。
    pub smoothing_radius: f32,
    /// XSPH 黏度系数 ε（速度平滑；晶格标定下权重和 ≈ 1）。
    pub xsph_viscosity: f32,
    /// 张力不稳定性抑制（spiky 梯度；CPU 档恒用 spiky，保留为口径标记）。
    pub tensile_instability_suppression: bool,
    /// 声速 c（m/s）⇒ Tait 刚度 B = c²ρ0/γ。
    pub sound_speed: f32,
    /// Tait 指数 γ（默认 7）。
    pub gamma_tait: f32,
    /// 每 tick 的子步数。
    pub substeps: u32,
    /// **相位并行线程数**（1 = 串行，默认）。
    ///
    /// 并行对象 = 逐粒独立的相位：**密度**与**力+黏度**（`phase_us` 实测这两项占
    /// 30 万粒档的 **96%**：密度 32% / 力 64%）。**逐位一致契约**：每个粒子的邻域
    /// 遍历序不随分块变化（见 `UniformGrid::for_neighbors_in`）⇒ 并行结果与串行
    /// **逐位相同**（不是"近似相同"）；唯一跨粒子的量 `bforce`（2b 反作用）在有
    /// 边界粒子时走**串行补趟**，同样保逐位。
    /// ⇒ 默认 1 时**完全不进入并行路径**（与旧行为逐位一致）。
    pub threads: usize,
    /// **2b 边界粒子的层数**（默认 2；SPEC §4.8 的两层形态）。
    /// 2026-09-22 起可调：实测用于"层数 ↑ ⇒ 反作用波动 ↓"这条候选
    /// （波动会透传到体上——漂浮体 y 峰峰 19.9 mm、|ω| 峰峰 1.22 rad/s，见
    /// `crates/vxl-phys/tests/float_quiet_probe.rs`）。
    pub boundary_layers: u32,
    /// 重力（m/s²）。
    pub gravity: Vec3,
    /// CFL 限速：|v| ≤ frac·h/dt（防穿隧/防爆炸）。
    pub max_speed_frac: f32,
    /// Monaghan 人工黏度 α（Π = α·c·μ/ρ̄，仅作用于接近对）：
    /// 耗散法向压缩波——没有它，钉在边界上的底层是个无阻尼硬弹簧，
    /// 柱体落位时反弹成尘（互穿死区 + 飞散），静水态永远建不起来。
    pub artificial_viscosity: f32,
    /// **动力黏度 μ（Pa·s）** —— 这是**物理材料参数**（水 ≈ 1.0e-3、机油 ≈ 0.1、甘油 ≈ 1.4、
    /// 蜂蜜 ≈ 10），不是上面那个无量纲的数值黏度。
    ///
    /// 消费方是**介质通道**（`world_step/viscous.rs`）：体在介质里的 Stokes 线性阻力
    /// `F = −6πμ·r_eq·v_rel` + 转动阻力矩 `T = −8πμ·r_eq³·ω`（`r_eq` = 体积等效球半径）。
    /// **默认 0 = 关** ⇒ 阻力退回旧的数值口径（`Cd = 1` 的二次阻力，冻结行为）⇒ 既有场景逐位不变。
    ///
    /// ⚠️ **本档的有效窗口是 `τ ≳ dt`**（`τ = m/(6πμ·r_eq·frac_sub)`）：更黏的介质（沥青/焦油）
    /// 会让 τ ≪ 子步长，半隐式写法在那一档**饱和**（终端速度偏高 `1 + dt/τ` 倍）
    /// ⇒ 那一档需要半解析积分（`v ← v_med + aτ + (v − v_med − aτ)e^{−dt/τ}`），登记为下一刀。
    pub dynamic_viscosity: f32,
    /// **FLIP 档消费**：FLIP/PIC 混合比 θ（θ = 1 纯 FLIP、θ = 0 纯 PIC；见 `flip` 模块）。
    /// `CpuSph`/`CpuPbf` 档下本字段不影响任何结果。
    pub flip_ratio: f32,
    /// **FLIP 档消费**：压力 Poisson 的迭代**上限**。默认 **80**（`solve_cg` 与 `solve_jacobi`
    /// 共用这一上限，见 `flip` 模块头"已知简化 2"）。`CpuSph`/`CpuPbf` 档下无影响。
    ///
    /// 默认 80 是实测折中（`tests/flip_incompressibility_probe.rs`，水槽静水池 60 tick，判别量
    /// = 末态柱高 / 起步柱高 `height_keep`）。**关键差别**在谁在用这个预算：
    /// - **CG（默认）**：20 次就满足 `CG_RTOL` 并**自行停手** —— 实测 20/40/80/160/320 档逐位
    ///   相同（`height_keep` 1.0367、净沉降 3.21e-2）⇒ 上限对它是"保险丝"而非成本。
    /// - **Jacobi（A/B）**：永远用满，且 N 格高柱上要 ~N² 次才收敛 ⇒ 迭代 10/20/40/80/160/320
    ///   ⇒ `0.30/0.48/0.71/0.85/0.93/1.02`（40 档水线掉 29%、80 档 15% 以内）。
    ///
    /// 更细的预条件/多重网格仍是下一刀（当前收敛率已不是瓶颈，见 `flip/solve.rs` 的 CG 头注）。
    pub flip_pressure_iterations: u32,
    /// **FLIP 档消费**：压力 Poisson 的**求解器**选择——`true`（默认）= **CG**（共轭梯度 +
    /// 对角预处理，见 `flip::solve_cg`）、`false` = 固定次数 **Jacobi** 扫（A/B 与复现实验用）。
    ///
    /// 两个开关是**乘法**关系：`flip_pressure_cg` 选方法，`flip_pressure_iterations` 是**迭代
    /// 上限**（CG 通常远早于此就满足 `CG_RTOL` 的相对残差 => 上限不常咬住；Jacobi 则永远用满）。
    /// `CpuSph`/`CpuPbf` 档下无影响。
    pub flip_pressure_cg: bool,
    /// **FLIP 档消费**：P2G 是否走 **APIC 仿射传输**（**默认 `false` = 关**；理由见"实测"）。
    /// 开时散射到网格角点的是 `v + C·(x_角点 − x_p)`，其中 3×3 的 `C`（= 速度场梯度）由
    /// `flip/affine.rs::estimate_from_field` **每子步从粒子场现算**（加权最小二乘 + 岭正则 +
    /// 拟合质量收缩，`(pos, vel)` 的纯函数）⇒ `C` **不进粒子状态**，逐位复现/保存恢复/状态
    /// 继承三份契约原样成立。
    ///
    /// **实测**（`tests/flip_rotation_probe.rs::diag_apic_rotation` +
    /// `flip_incompressibility_probe.rs::diag_apic_hydrostatic`）：
    /// ① 旋转团（零重力/无边界，30 tick）：`θ = 0` 的角动量保持率 PIC 0.145 → **APIC 0.981**
    ///    （`rot_err` 0.864 → 0.429；`ke` 4.77 → 1.61e2 —— 按 `ke ≈ L²/(2I)` 与 `l_keep` 自洽，
    ///    不是能量爆走）。上一版"**tick 级** `C`"（从网格重建、每 tick 清零）只有 0.155 ⇒
    ///    红利来自 `C` **从粒子场现算**这一刀，而不是 APIC 这个口径本身。
    /// ② 静水槽（θ = 0.95，80 CG）：`ke` 17.24 → 20.35（+18%）、`sink` 3.21e-2 → 3.50e-2（+9%，
    ///    自由落体参照 4.91 m ⇒ 两者都可忽略）；代价集中在**近壁**——全体过密 `over` 3.603 → 4.138
    ///    （+15%）而**体内部** `over_in` 0.856 → 0.332（**−61%**）⇒ 近静场代价温和但**不为零**，
    ///    故默认仍关。
    ///
    /// 派生 `C` 的两处驯服（不驯服时静水槽 `ke` 17.2 → 80.1、过密 `over` 3.6 → 18.5，不可用）：
    /// **岭** `λ·(tr D / 3)`（表面粒子邻域单侧 ⇒ `D` 欠定，不压住会把偏置拟合成假梯度）+
    /// **拟合质量收缩** `C ← (⟨C, M⟩ / T)·C`（近静场里信号几乎全是噪声 ⇒ `s → 0`，`C` 哑火）。
    /// 本开关保留为 A/B 实验档（`false` = 冻结旧口径，与 `dynamic_viscosity = 0` 同一先例）。
    pub flip_apic: bool,
}

impl Default for FluidConfig {
    fn default() -> Self {
        Self {
            family: FluidFamily::CpuSph,
            density_iterations: DensityIterations::Three,
            rest_density: 1000.0,
            smoothing_radius: 0.1,
            xsph_viscosity: 0.1,
            tensile_instability_suppression: true,
            // 声速 c：声学 CFL c·dt/h ≤ ~0.5（dt = 1/(60·substeps)，h = 0.1 ⇒ c = 10 时 c·dt/h ≈ 0.42）；c = ρ0gH 压缩误差 ~ ρ0gH/c²。
            sound_speed: 10.0,
            gamma_tait: 7.0,
            substeps: 4,
            threads: 1,
            boundary_layers: 2,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            max_speed_frac: 0.4,
            artificial_viscosity: 1.0,
            dynamic_viscosity: 0.0,
            flip_ratio: 0.95,
            flip_pressure_iterations: 80,
            flip_pressure_cg: true,
            // APIC 仿射传输：派生 `C` 已买到旋转红利（l_keep 0.145 → 0.981），但近静场要付
            // +18% ke / +15% 近壁过密 over（见字段 doc 的"实测"节）⇒ 默认仍关，留作 A/B 档。
            flip_apic: false,
        }
    }
}

/// 均匀格总数上限；超出则格边逐级加倍粗化（粗化不减正确性：
/// r > h 的候选被核函数零剔除）。
/// **单一来源已移到 `vxl_phys_core::grid::GRID_MAX_BINS`**——GPU 侧每子步重算箱子要用同一个数。
pub(crate) use vxl_phys_core::grid::GRID_MAX_BINS;
