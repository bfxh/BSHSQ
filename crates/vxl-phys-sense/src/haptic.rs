//! haptic：**S4 震动设备输出**（包络 / 上钳到 `[0,1]` / 更新率·延迟口径；`docs/ROUTE.md` §3.2）。
//!
//! S1/S2/S3 都是"事件 → cue"的**归约**（无状态纯函数）；S4 是"cue → **设备帧**"的**整形**
//! （envelope shaping）——它**必须带状态**：一次冲击是"一下"，而震动器件要的是"电平抬起来、
//! 再按器件的时间常数落下去"。于是本模块引入前三片刻意避开的那样东西：**状态**——而且刻意
//! 只留一个（[`HapticDriver`] 的唯一状态就是包络 `env` 本身，没有缓冲队列、没有帧计数器）。
//!
//! 带状态就必须把三件事**钉死**，否则"包络"会随实现漂移（判据见 `tests/haptic_contracts.rs`）：
//!
//! 1. **上钳到 `[0,1]`**：设备量纲是归一化强度；`drive ≥ 1` ⇒ **恰好** `1.0`（上界是闭的），
//!    `drive < 1` ⇒ 收敛到 `drive` 本身（**不许**被当成满档）。
//! 2. **包络单调**：常值驱动下逐帧**非减且不超调**（每帧 ≤ 驱动量）；断驱动后逐帧**非增**并
//!    收敛到**恰好 `0.0`**（否则设备长期微振）。两条都是 [`env_step`] 递推的**结构性质**，
//!    不是"看着差不多"：一个分支夹在 `[env, target]`、另一个夹在 `[target, env]`。
//! 3. **更新率 / 延迟口径**：设备帧率与 tick 步长解耦——每 tick **恰好** `frames_per_tick` 帧
//!    （`round(update_hz · dt)`，兜底 ≥ 1），帧号是 tick 的**纯函数**
//!    `frame = tick · frames_per_tick + k` ⇒ **延迟 0 帧**（cue 所属 tick 的**首帧**就带上它，
//!    驱动器内部**没有**缓冲队列；要额外缓冲是消费方的事，本模块基线口径写死为 0）。
//!
//! **驱动量怎么来**：本 tick 的冲击 cue（S1）与滑移 cue（S2）合成**一个**电平——取 `max`
//! （同时发生多下 ⇒ 取最强的那一下）。`max` 与顺序无关 ⇒ S1/S2 那种"求和归约顺序"的确定性
//! 风险在这里天然不存在（同 tick 的 cue 倒序喂入，帧输出逐字相同；判据里钉住）。
//!
//! **零干扰是类型级的**：本模块**不**借 `&World`（只吃 `&[Cue]` / `&[SlideCue]`）⇒ 结构上碰不到
//! 物理末态（同 S3 的映射）。

use crate::{gain_of, Cue, SlideCue};

/// **震动档**（器件时间常数 + 两个源的驱动量标定）。
///
/// `attack` / `release` 是**每设备帧**的系数（不是每 tick）：
/// `attack = 0.15` ⇒ 1000 Hz 下 7 帧 ≈ 7 ms 到顶；`release = 0.86` ⇒ ≈50 帧 ≈ 50 ms 衰到近零，
/// 再往后按 f32 下溢**恰好**到 0（判据 ② 依赖这条）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HapticConfig {
    /// 设备更新率（Hz）。与 tick 步长一起定每 tick 帧数（见 [`HapticDriver::frames_per_tick`]）。
    pub update_hz: f32,
    /// 每帧**上升**步长（`[0,1]`；`1` ⇒ 一帧到顶）。非有限值按 `0`（不响应，安全侧）。
    pub attack: f32,
    /// 每帧**衰减**系数（`[0,1]`；`0` ⇒ 一帧放干）。非有限值按 `0`。
    pub release: f32,
    /// 冲击 `Cue::gain` → 驱动量的比例。**> 1 是常态**：器件要过驱动才能让重击顶到满档
    /// （触感弱的重击与轻击分不出来），`1.4` ⇒ 场景里 `gain ≥ 0.72` 的冲击会上钳。
    pub impact_scale: f32,
    /// 滑移（S2）→ 驱动量的比例。持续接触是**背景**震动，默认压低（`0.35`）以免盖过冲击。
    pub slide_scale: f32,
    /// 滑移量的满档速度（m/s）——**与 `SenseConfig::full_slide_speed` 同口径**（S2 没把 `gain`
    /// 写进 `SlideCue`，只给了原始 `speed`；本模块自己归一化，避免两个模块互相依赖）。
    pub slide_full_speed: f32,
}

impl Default for HapticConfig {
    /// 1000 Hz 器件（消费级手柄/手机的典型触觉更新率）、7 ms 起、50 ms 落、冲击过驱动 1.4×、
    /// 滑移压到 0.35×。
    fn default() -> Self {
        Self {
            update_hz: 1000.0,
            attack: 0.15,
            release: 0.86,
            impact_scale: 1.4,
            slide_scale: 0.35,
            slide_full_speed: 4.0,
        }
    }
}

/// **一帧设备输出**（`intensity` 已保证落在 `[0,1]`）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HapticFrame {
    /// 采样它的 tick（`frame` 由它唯一决定）。
    pub tick: u64,
    /// 全局帧号：`tick · frames_per_tick + k`（`k ∈ [0, frames_per_tick)`）。
    pub frame: u64,
    /// 归一化强度（`[0,1]`；上钳闭在 `1.0`，释放收敛到恰好 `0.0`）。
    pub intensity: f32,
}

/// **一段驱动（通常 = 一个 tick）的统计**（工具链拿它判"非空洞"与"上钳真的发生过"）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HapticStats {
    /// 本段产出的帧数（= `frames_per_tick`）。
    pub frames: usize,
    /// 强度**顶在 `1.0`** 的帧数（上钳真的发生过；恒 0 ⇒ 场景里没有重到该满档的驱动）。
    pub clamped: usize,
    /// 强度**恰为 `0.0`** 的帧数（包络已释放干净）。
    pub silent: usize,
    /// 本段强度峰值。
    pub peak: f32,
}

/// **震动驱动器**：`cue 流 → 设备帧`。唯一的状态是包络 `env`（没有队列、没有帧计数器）。
#[derive(Clone, Copy, Debug)]
pub struct HapticDriver {
    cfg: HapticConfig,
    /// tick 步长（秒）——`PhysConfig::default().dt`；与 [`HapticConfig::update_hz`] 定帧数。
    tick_dt: f32,
    env: f32,
}

impl HapticDriver {
    /// 新驱动器（包络从 `0.0` 起：**上电静默**，不继承上一次会话的电平）。
    pub fn new(cfg: HapticConfig, tick_dt: f32) -> Self {
        Self {
            cfg,
            tick_dt,
            env: 0.0,
        }
    }

    /// 当前包络（设备此刻的输出电平，`[0,1]`）。
    pub fn env(&self) -> f32 {
        self.env
    }

    /// **每 tick 的设备帧数** = `round(update_hz · dt)`，兜底 ≥ 1（`< 1` 帧/tick 等于静默设备）。
    ///
    /// 非有限的 `update_hz` / `tick_dt` 按默认值算；上限 `100_000` 是**防呆硬钳**：
    /// `update_hz = inf` 那种配置否则会把调用方一帧循环到挂死（float→int 的饱和转换会给出
    /// `u64::MAX`，故这里自己夹）。
    pub fn frames_per_tick(&self) -> u64 {
        let hz = finite_or(self.cfg.update_hz, 1000.0);
        let dt = finite_or(self.tick_dt, 1.0 / 60.0);
        let n = (hz * dt).round();
        if n < 1.0 {
            1
        } else if n > 100_000.0 {
            100_000
        } else {
            n as u64
        }
    }

    /// 帧号 = `tick · frames_per_tick + k`（**纯函数** ⇒ 无内部缓冲、无漂移；延迟口径见模块头）。
    pub fn frame_of(&self, tick: u64, k: u64) -> u64 {
        tick * self.frames_per_tick() + k
    }

    /// **驱动一个 tick**：吃本 tick 的冲击 cue 与滑移 cue，产出该 tick 的 `frames_per_tick` 帧
    /// （**追加**到 `out`），返回统计。
    ///
    /// `tick` 由调用方给（帧号由它算，见 [`HapticDriver::frame_of`]）⇒ 本模块不维护帧计数器：
    /// 跳 tick / 重跑同一个 tick 都只是"帧号按 tick 算"，行为可预测（不做隐藏的补帧/去重）。
    pub fn step(
        &mut self,
        tick: u64,
        cues: &[Cue],
        slides: &[SlideCue],
        out: &mut Vec<HapticFrame>,
    ) -> HapticStats {
        let target = drive_of(cues, slides, &self.cfg);
        let fpt = self.frames_per_tick();
        let mut stats = HapticStats::default();
        for k in 0..fpt {
            self.env = env_step(self.env, target, &self.cfg);
            stats.frames += 1;
            if self.env >= 1.0 {
                stats.clamped += 1;
            }
            if self.env <= 0.0 {
                stats.silent += 1;
            }
            if self.env > stats.peak {
                stats.peak = self.env;
            }
            out.push(HapticFrame {
                tick,
                frame: self.frame_of(tick, k),
                intensity: self.env,
            });
        }
        stats
    }
}

/// **包络递推**（本模块唯一的动力学；两个分支都把结果夹在 `[env, target]` / `[target, env]`）：
///
/// - `target ≥ env`（**上升**）：每帧 `+attack`，且 **`.min(target)`** ⇒ 逐帧非减、**不超调**、
///   有限帧内**恰好**到 `target`（`attack > 0`）；
/// - `target < env`（**下降**）：每帧 `×release`，且 **`.max(target)`** ⇒ 逐帧非增、不下穿；
///   且**衰减必须真的减小**——否则直接归零（见下面的踩坑注记）。
///
/// 由此：`env ∈ [0,1]` 是**不变量**（两个分支的结果都落在两个端点之间；初始 `env = 0`）
/// ⇒ "上钳"不是事后 `clamp` 出来的，而是递推本身到不了 1 以上。
///
/// ⚠️ **踩坑（判据 ② 首次就抓到的设计缺陷）**：只写 `(env * r).max(target)` 时，包络**永远到不了**
/// `0.0` —— f32 最小正数 `4e-45` 乘 `0.86` 按最近舍入又回到 `4e-45`（次正规数的**不动点**），
/// 于是"释放干净"变成常驻微振（器件长期嗡嗡）。`release` 取到 `1.0` 附近时同理会卡在任意 `env`。
/// 故下降分支显式要求 `decayed < env`：不满足就**落到 `target`**（`release = 0` 时即"一帧放干"）。
/// 这条不是补丁而是判据的形状——`tests/haptic_contracts.rs` 的"收敛到恰好 0.0"就是它的守门人。
fn env_step(env: f32, target: f32, cfg: &HapticConfig) -> f32 {
    let a = gain_of(cfg.attack, 1.0);
    let r = gain_of(cfg.release, 1.0);
    if target >= env {
        (env + a).min(target)
    } else {
        let decayed = env * r;
        if decayed < env {
            decayed.max(target)
        } else {
            target
        }
    }
}

/// **本 tick 的驱动量** = 两个源取最强，再上钳到 `[0,1]`（`gain_of` 同时兜住 NaN：非有限 ⇒ `0`）。
fn drive_of(cues: &[Cue], slides: &[SlideCue], cfg: &HapticConfig) -> f32 {
    let mut drive = 0.0_f32;
    for c in cues {
        drive = drive.max(c.gain * cfg.impact_scale);
    }
    for s in slides {
        drive = drive.max(gain_of(s.speed, cfg.slide_full_speed) * cfg.slide_scale);
    }
    gain_of(drive, 1.0)
}

/// 有限值取本身，非有限（NaN/±∞）取兜底 —— 配置字段的防呆（`clamp` 对 NaN 是透传的，不能用）。
fn finite_or(x: f32, dflt: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        dflt
    }
}

/// **设备帧流 → JSONL**（一行一帧；浮点固定 6 位小数 ⇒ 逐字可对拍，同 S1/S2 口径）。
///
/// 字段序固定：`tick / frame / intensity`。不写时间戳、不写平台信息（否则"机器无关"立刻失效）。
pub fn write_haptic_jsonl<W: std::io::Write>(
    w: &mut W,
    frames: &[HapticFrame],
) -> std::io::Result<()> {
    for f in frames {
        writeln!(
            w,
            "{{\"tick\":{},\"frame\":{},\"intensity\":{:.6}}}",
            f.tick, f.frame, f.intensity
        )?;
    }
    Ok(())
}

/// 便捷：设备帧切片 → JSONL 字符串（判据 / 工具链用）。
pub fn haptic_jsonl(frames: &[HapticFrame]) -> String {
    let mut s = Vec::new();
    // 写内存 `Vec<u8>` 不会失败（它的 `Write` 实现不返回 Err）；真出错时给空串而不是 panic。
    if write_haptic_jsonl(&mut s, frames).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&s).into_owned()
}
