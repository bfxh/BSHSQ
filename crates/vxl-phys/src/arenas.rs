//! arenas：从 lib.rs 按域拆出（纯搬移，语义未改）。
//! `FrameArenas` = 帧级相位暂存（相内 bump、相末 reset、全帧零 free；M0 消费者 = 帧末哈希缓冲）；
//! `HealthReport` = §3 稳定性报告 + **耦合审计**（`PLAN-COUPLING` C0）。
use super::*;

#[derive(Debug)]
pub struct FrameArenas {
    pub hash: PhaseArena,
}

impl FrameArenas {
    pub fn new() -> Self {
        Self {
            hash: PhaseArena::with_capacity(HASH_SCRATCH_BYTES),
        }
    }
}

impl Default for FrameArenas {
    fn default() -> Self {
        Self::new()
    }
}

/// 哈希规范化暂存预算（固定 8KB；流式打包与体数无关，永不溢出）。
pub const HASH_SCRATCH_BYTES: usize = 8 << 10;

/// §3 稳定性健康报告（NaN/Inf、静默穿透计数）+ **耦合审计**（C0）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HealthReport {
    pub nan_bodies: u32,
    /// 深度 > skin×4 的接触计数（§3：静默穿透 = 0）。
    pub deep_penetrations: u32,
    pub max_depth: f32,
    pub awake_bodies: u32,
    pub contacts: u32,
    /// **耦合审计**（`PLAN-COUPLING.md` §5 C0）：收不到跨域作用的体数 + 被挡下的合力模长；不进 `is_clean`。
    pub coupling_dropped_bodies: u32,
    pub coupling_dropped_force: f32,
}

impl HealthReport {
    pub fn is_clean(&self) -> bool {
        self.nan_bodies == 0 && self.deep_penetrations == 0
    }
}

/// 分相耗时计数（§11 性能计数器：每帧物理 ms 按阶段拆分）。
/// 累计微秒，跨 `step` 累加，`reset_timings()` 归零。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhaseTimings {
    pub fields_us: u64,
    pub integrate_vel_us: u64,
    pub broadphase_us: u64,
    pub narrowphase_us: u64,
    /// 接触求解 + 关节求解（同一阶段计时；关节通道无关节时零成本）。
    pub solve_us: u64,
    pub integrate_pos_us: u64,
    pub ccd_us: u64,
}

impl PhaseTimings {
    pub fn total_us(&self) -> u64 {
        self.fields_us
            + self.integrate_vel_us
            + self.broadphase_us
            + self.narrowphase_us
            + self.solve_us
            + self.integrate_pos_us
            + self.ccd_us
    }
}
