//! **A6「离线音频渲染」**：把 cue 流（[`SoundPath`]）变成**能听的波形**，外加一对最小 WAV 读写。
//!
//! 为什么有它（调研文档 §8 第 3 条的落地）：本层的输出是**路径流**，真正的双耳渲染归宿主音频引擎；
//! 但"没有一个能听的产物"就没人能验收音效 —— 音效师要拿它**对表**（听一听这间屋子对不对），
//! 集成方要拿它**回归**（同一场景两次渲染逐位一致）。所以这一层只做两件事：
//!
//! 1. **逐带分频**：干信号按 [`SPLIT_HZ`] 的两级一阶低通切成低/中/高 —— `低 + 中 + 高 ≡ 原信号`
//!    （数学上完美重构：中带 = 两级低通之差、高带 = 原信号 − 二级低通）⇒ 三带同参时渲染结果与
//!    "根本不切"只差 f32 舍入（≤1e-5 相对，判据钉住这条）；
//! 2. **直叠**：每条路径按 `bands` 缩放对应那一带的干信号副本、按 `delay` 对齐、叠到输出。
//!
//! ## 两条口径（**别猜，就照这个**）
//!
//! - **幅度 = `sqrt(能量)`**：[`SoundPath::bands`] 是**能量**增益（直达 = `1/d²` 这种）⇒ 渲染时
//!   逐带取平方根变**幅度**（球面扩散下 `1/d²` 的能量对应 `1/d` 的幅度，近处不会响得过分）。
//! - **不乘** [`SoundPath::gain`]：它就是中频带（`bands[BANDS/2]`），乘了等于中频带**计能两次**。
//!
//! ## 边界（如实登记）
//!
//! 单声道（**不做** HRTF/双耳、不做多普勒、不做空气非线性）；时延量化到**整数采样**（48 kHz 下
//! ≈ 21 µs，远低于可闻的时延分辨）；**没有混响尾巴** —— 本层只渲染显式路径（直达/一次反射/
//! 绕射/散射/折射/穿孔/透射），多次反射仍在路线图外。
//!
//! ## 确定性
//!
//! 采样率与时长都是定值、滤波器是一阶 IIR（定状态、逐样本）、时延取整数采样、全程 `f32`、
//! 伪随机激励是**定种 LCG** ⇒ 同输入两跑**逐位相同**（判据 `tests/a6_audio.rs`）。

use crate::{SoundPath, BANDS};
use std::fs;

/// 分频交叉点（Hz）：低|中 与 中|高。
///
/// ⚠️ 本层**不给绝对频率**（同 [`crate::BANDS`] 的口径）；这两个数是**渲染器自己的**口径，
/// 宿主想按自己的扬声器改，请自己渲染（别改这里去凑）。
pub const SPLIT_HZ: [f32; 2] = [500.0, 4000.0];

/// 渲染参数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AudioConfig {
    /// 采样率（Hz）。
    pub sample_rate: u32,
    /// 渲染时长（秒）—— 本层没有混响尾巴，所以"最后一条显式路径 + 干信号长度"就够。
    pub seconds: f32,
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            sample_rate: 48_000,
            seconds: 1.0,
        }
    }
}

impl AudioConfig {
    /// 采样点数（秒 → 点，四舍五入；下限 1 ⇒ 永远给得出东西）。
    pub fn samples(&self) -> usize {
        let n = (self.sample_rate as f32 * self.seconds).round();
        if n < 1.0 {
            1
        } else {
            n as usize
        }
    }
}

/// 秒 → 采样序号（**四舍五入** ⇒ 确定性量化；见模块头注的"时延量化到整数采样"）。
fn at(seconds: f32, sample_rate: u32) -> usize {
    let n = (seconds * sample_rate as f32).round();
    if n <= 0.0 {
        0
    } else {
        n as usize
    }
}

/// 一段**单声道**波形（`samples` 是线性幅度，量级 ≈ `[-1, 1]`；见 [`Clip::peak`]）。
#[derive(Clone, Debug, PartialEq)]
pub struct Clip {
    /// 采样率（Hz）。
    pub sample_rate: u32,
    /// 逐采样幅度。
    pub samples: Vec<f32>,
}

impl Clip {
    /// 时长（秒）。
    pub fn seconds(&self) -> f32 {
        if self.sample_rate == 0 {
            0.0
        } else {
            self.samples.len() as f32 / self.sample_rate as f32
        }
    }

    /// 峰值**绝对值**（归一化/写盘前看它会不会削顶）。
    pub fn peak(&self) -> f32 {
        self.samples.iter().fold(0.0f32, |m, &x| m.max(x.abs()))
    }

    /// 整体 RMS（听感响度的粗代理）。
    pub fn rms(&self) -> f32 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let sum: f32 = self.samples.iter().map(|&x| x * x).sum();
        (sum / self.samples.len() as f32).sqrt()
    }

    /// 整体缩放（**就地**；`0` 或负数不做事，免得静音被翻相位）。
    pub fn scale(&mut self, g: f32) {
        if g <= 0.0 {
            return;
        }
        for x in self.samples.iter_mut() {
            *x *= g;
        }
    }
}

/// 一阶低通系数：`a = 1 − exp(−2π·fc/fs)`（`fc ≥ fs/2` 时夹到 1 ⇒ 全通）。
fn one_pole(sample_rate: u32, fc: f32) -> f32 {
    if sample_rate == 0 {
        return 1.0;
    }
    let a = 1.0 - (-6.283_185_5 * fc / sample_rate as f32).exp();
    a.clamp(0.0, 1.0)
}

/// 三带分频：两级一阶低通，`低 + 中 + 高 ≡ 原信号`（完美重构）。
pub fn split3(dry: &[f32], sample_rate: u32) -> [Vec<f32>; BANDS] {
    let a1 = one_pole(sample_rate, SPLIT_HZ[0]);
    let a2 = one_pole(sample_rate, SPLIT_HZ[1]);
    let mut low = vec![0.0f32; dry.len()];
    let mut mid = vec![0.0f32; dry.len()];
    let mut high = vec![0.0f32; dry.len()];
    let (mut y1, mut y2) = (0.0f32, 0.0f32);
    for (i, &x) in dry.iter().enumerate() {
        y1 += a1 * (x - y1);
        y2 += a2 * (x - y2);
        low[i] = y1;
        mid[i] = y2 - y1;
        high[i] = x - y2;
    }
    [low, mid, high]
}

/// 把一批路径**叠进**输出缓冲（就地累加）。
///
/// `t0_seconds` = 这批路径的**发生时刻**（例如"第 3 秒那声撞击"）；`dry` 是那一声的干信号。
/// 越界的部分直接丢（输出缓冲就是这么长；想听更长的尾巴就把缓冲开大）。
pub fn mix_into(
    out: &mut [f32],
    paths: &[SoundPath],
    dry: &[f32],
    t0_seconds: f32,
    sample_rate: u32,
) {
    if out.is_empty() || dry.is_empty() || paths.is_empty() {
        return;
    }
    let bands = split3(dry, sample_rate);
    let base = at(t0_seconds, sample_rate);
    for p in paths {
        // 三带都 0 ⇒ 这条路径没能量（别白白扫一遍）。
        if p.bands.iter().all(|&c| c <= 0.0) {
            continue;
        }
        let start = base + at(p.delay, sample_rate);
        if start >= out.len() {
            continue;
        }
        for (b, band) in bands.iter().enumerate() {
            let coef = p.bands[b];
            if coef <= 0.0 {
                continue;
            }
            // 能量 → 幅度（模块头注的口径）。
            let amp = coef.sqrt();
            for (i, &x) in band.iter().enumerate() {
                let j = start + i;
                if j >= out.len() {
                    break;
                }
                out[j] += amp * x;
            }
        }
    }
}

/// 一批路径的完整渲染（从 0 秒开始，长度 = [`AudioConfig::samples`]）。
pub fn render(paths: &[SoundPath], dry: &[f32], cfg: &AudioConfig) -> Clip {
    let mut samples = vec![0.0f32; cfg.samples()];
    mix_into(&mut samples, paths, dry, 0.0, cfg.sample_rate);
    Clip {
        sample_rate: cfg.sample_rate,
        samples,
    }
}

/// **冲激响应**：干信号取单位冲激（对表/判据用：尖峰位置 = 时延、尖峰高 = 逐带增益的平方根）。
pub fn impulse_response(paths: &[SoundPath], cfg: &AudioConfig) -> Clip {
    let mut dry = vec![0.0f32; cfg.samples()];
    dry[0] = 1.0;
    render(paths, &dry, cfg)
}

/// 一记**确定性的"拍手"**：指数衰减的伪随机噪声（定种 LCG ⇒ 逐位可复现），峰值归一到 1。
///
/// 它就是"手边没有素材时先听一间屋子"的默认激励：宽带、短促、时间上够尖 ⇒ 时延与回响都听得出来。
pub fn clap(sample_rate: u32, seconds: f32) -> Vec<f32> {
    let n = AudioConfig {
        sample_rate,
        seconds,
    }
    .samples();
    let mut out = vec![0.0f32; n];
    let mut s: u32 = 0x1234_5678;
    for (i, v) in out.iter_mut().enumerate() {
        s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (s >> 8) as f32 / 8_388_608.0 - 1.0; // u24 → [-1, 1)
        let t = i as f32 / sample_rate as f32;
        *v = noise * (-t * 35.0).exp();
    }
    let peak = out.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
    if peak > 0.0 {
        for v in out.iter_mut() {
            *v /= peak;
        }
    }
    out
}

/// 写 **16 bit PCM 单声道 WAV**（RIFF；零依赖，一次落盘）。
pub fn write_wav(path: &str, clip: &Clip) -> Result<(), String> {
    fs::write(path, wav_bytes(clip)).map_err(|e| format!("写不了 {path}：{e}"))
}

/// WAV 字节（`RIFF/WAVE/fmt /data` 四块 = 44 字节头 + PCM16 数据）—— 与 [`write_wav`] 同一口径，
/// 分开是为了判据能直接看字节。
pub fn wav_bytes(clip: &Clip) -> Vec<u8> {
    let data_len = (clip.samples.len() * 2) as u32;
    let mut b = Vec::with_capacity(44 + clip.samples.len() * 2);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data_len).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes()); // 1 = PCM
    b.extend_from_slice(&1u16.to_le_bytes()); // 单声道
    b.extend_from_slice(&clip.sample_rate.to_le_bytes());
    b.extend_from_slice(&(clip.sample_rate * 2).to_le_bytes()); // 字节率
    b.extend_from_slice(&2u16.to_le_bytes()); // 块对齐
    b.extend_from_slice(&16u16.to_le_bytes()); // 位深
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data_len.to_le_bytes());
    for &x in &clip.samples {
        // ⚠️ 定标取 **32768**（不是 32767）：读回来除的也是 32768 ⇒ **没有系统性增益误差**，
        // 只剩 ±0.5 LSB 的量化（32767 会在往返里多出一层 1/32768 ≈ 3e-5 的比例误差）。
        let v = (x.clamp(-1.0, 1.0) * 32_768.0)
            .round()
            .clamp(-32_768.0, 32_767.0);
        b.extend_from_slice(&(v as i16).to_le_bytes());
    }
    b
}

/// 读 WAV（PCM16 / IEEE float32；**多声道降混**成单声道；其它格式报错 —— 不猜）。
pub fn read_wav(path: &str) -> Result<Clip, String> {
    let bytes = fs::read(path).map_err(|e| format!("读不了 {path}：{e}"))?;
    parse_wav(&bytes)
}

/// 小端读 `u16`（越界给 0 —— 只有解析头时才用，长度已经在外面查过）。
fn u16le(b: &[u8], i: usize) -> u16 {
    match (b.get(i), b.get(i + 1)) {
        (Some(&a), Some(&c)) => u16::from_le_bytes([a, c]),
        _ => 0,
    }
}

/// 小端读 `u32`（同 [`u16le`]）。
fn u32le(b: &[u8], i: usize) -> u32 {
    match (b.get(i), b.get(i + 1), b.get(i + 2), b.get(i + 3)) {
        (Some(&a), Some(&c), Some(&d), Some(&e)) => u32::from_le_bytes([a, c, d, e]),
        _ => 0,
    }
}

/// 解析 WAV 字节：`(格式, 通道, 采样率, 位深)` 来自 `fmt ` 块、样本来自 `data` 块。
pub fn parse_wav(b: &[u8]) -> Result<Clip, String> {
    if b.len() < 12 || &b[0..4] != b"RIFF" || &b[8..12] != b"WAVE" {
        return Err("不是 RIFF/WAVE（本层只认 WAV）".to_string());
    }
    let mut fmt: Option<(u16, u16, u32, u16)> = None;
    let mut data: Option<&[u8]> = None;
    let mut pos = 12usize;
    while pos + 8 <= b.len() {
        let size = u32le(b, pos + 4) as usize;
        let start = pos + 8;
        let end = start.saturating_add(size).min(b.len());
        if &b[pos..pos + 4] == b"fmt " && size >= 16 {
            fmt = Some((
                u16le(b, start),
                u16le(b, start + 2),
                u32le(b, start + 4),
                u16le(b, start + 14),
            ));
        } else if &b[pos..pos + 4] == b"data" {
            data = Some(&b[start..end]);
        }
        pos = end + (size & 1); // 块按偶数字节对齐
    }
    let (format, channels, sample_rate, bits) = fmt.ok_or("缺 `fmt ` 块")?;
    let data = data.ok_or("缺 `data` 块")?;
    let ch = channels.max(1) as usize;
    let frames = decode_frames(data, format, bits, ch)?.ok_or_else(|| {
        format!("不支持的 WAV 格式（format={format} / {bits} bit）—— 只认 PCM16 与 float32")
    })?;
    let mut samples = Vec::with_capacity(frames.len() / ch);
    for f in frames.chunks(ch) {
        let sum: f32 = f.iter().sum();
        samples.push(sum / ch as f32); // 多声道降混
    }
    Ok(Clip {
        sample_rate,
        samples,
    })
}

/// 样本解码：`Ok(Some(逐采样值))` = 认；`Ok(None)` = 格式不认识（调用方决定怎么报）。
fn decode_frames(
    data: &[u8],
    format: u16,
    bits: u16,
    ch: usize,
) -> Result<Option<Vec<f32>>, String> {
    let mut out = Vec::new();
    match (format, bits) {
        // PCM 16 bit（本层自己写出来的就是它）。
        (1, 16) => {
            out.reserve(data.len() / 2);
            // ⚠️ 常量块大小一律用 `as_chunks`（本仓 CI 的 clippy 有 `chunks_exact_to_as_chunks`）。
            for c in data.as_chunks::<2>().0 {
                out.push(i16::from_le_bytes(*c) as f32 / 32_768.0);
            }
        }
        // PCM 8 bit（无符号）。
        (1, 8) => {
            for &x in data {
                out.push((x as f32 - 128.0) / 128.0);
            }
        }
        // IEEE float32。
        (3, 32) => {
            out.reserve(data.len() / 4);
            for c in data.as_chunks::<4>().0 {
                out.push(f32::from_le_bytes(*c));
            }
        }
        _ => return Ok(None),
    }
    if ch > 1 {
        // 样本数必须是整帧；余下的半个帧丢掉（写坏的文件不该把降混搞成错位）。
        out.truncate(out.len() / ch * ch);
    }
    Ok(Some(out))
}
