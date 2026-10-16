//! `audio` 子命令：声学路径渲染成能听的 WAV（A6 + A7 混响；纯搬移自 `lib.rs`）。

use crate::acoustic::{acoustic_cfg, medium, room};
use crate::scene::scene;
use vxl_phys::Vec3;
use vxl_phys_acoustics::audio as aco_audio;
use vxl_phys_acoustics::reverb as aco_reverb;
use vxl_phys_acoustics::{AcousticConfig, AcousticGeometry, AcousticsExt, Listener, SoundPath};

/// `audio`：把声学路径流渲染成**能听的 WAV**（A6）。
///
/// 时间轴按**物理步长**排（`w.config.dt`，本仓固定 60 Hz）：第 `t` 个 tick 的那批路径从
/// `t·dt` 秒开始叠 ⇒ 听到的是"这一串撞击在房间里"的声音。听者固定站在墙外侧（与 `acoustic`
/// 同一位姿），声源逐 tick 从 `impacts()` 来（会动）。
///
/// 干信号默认是**确定性的一记闷响**（[`aco_audio::knock`]，低通过的，避免"电锯味"）；给了 `干信号.wav` 就用它
/// （PCM16 / float32，**多声道降混**）。输出是 16 bit PCM 单声道 WAV。
///
/// ⚠️ 写盘前按峰值**归一**（只在峰值 >1 时缩，比例打印出来）：多声撞击会在时间轴上叠起来，
/// 不管一下就会削顶 —— 削了顶的产物对表没意义。相对电平不受影响（整体一个因子）。
///
/// ⚠️ **只认 WAV**：mp4/m4a/mp3 这类**压缩容器本层不解码**（引解码器就要背编解码器与许可
/// 那一摊子，且与"确定性优先"无关）—— 看见这些扩展名直接给一条能抄的 `ffmpeg` 命令。
pub(super) fn audio_cmd(
    name: &str,
    ticks: usize,
    out: Option<&str>,
    dry_path: Option<&str>,
    f: AudioFlags,
) -> Result<String, String> {
    let Some(out) = out else {
        return Err("audio 需要输出路径：`audio <场景> [ticks] <出.wav> [干信号.wav]`".to_string());
    };
    let mut w = scene(name)?;
    let (dt, rate) = (w.config.dt, aco_audio::AudioConfig::default().sample_rate);
    let dry = match dry_path {
        Some(p) => read_dry(p)?,
        None => aco_audio::knock(rate, 0.18),
    };
    let room = room();
    let field = medium();
    let geo = room.geom(Some(&field));
    let cfg = acoustic_cfg(false, f.order2);
    let rv = audio_reverb(&geo, &cfg, f.reverb);
    // 默认听者在墙**外侧**（只能听到门洞/绕射/透射）；`--inside` 挪到声源那间屋里（直达 + 反射 + 尾巴）。
    let listener = Listener {
        pos: if f.inside {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 3.0)
        },
        radius: 0.5,
    };
    // ⚠️ 尾巴按**干信号自己的长度**留（至少 2 s）：最后一声撞击之后，那条干信号得整段装得下 ——
    // 干信号比尾巴长（比如喂一段 10 s 的素材）时，只按 ticks 算会把尾部**截掉**（本层没有混响
    // 尾巴，只有显式路径，所以"干信号多长"就是答案；见 A6 模块头注）。
    let dry_secs = dry.len() as f32 / rate as f32;
    let seconds = ticks as f32 * dt + dry_secs.max(2.0) + tail_seconds(&rv) + 0.05;
    // ⚠️ 先算成变量再转 `usize`（`usize-gate` 认「括号里先乘再转 usize」那个写法，虽然这里是浮点）。
    let frames = seconds * rate as f32;
    let mut samples = vec![0.0f32; frames.max(1.0) as usize];
    // 混响尾巴要**整条干信号轨**（卷积线性 ⇒ 摊好一次卷，比逐事件调快得多，见 `mix_reverb_tail`）。
    let mut track = vec![0.0f32; samples.len()];
    let (mut events, mut count) = (0usize, 0usize);
    for t in 0..ticks {
        w.step();
        let mut tick: Vec<SoundPath> = Vec::new();
        w.acoustic_cues(&geo, &listener, &cfg, &mut tick);
        if tick.is_empty() {
            continue;
        }
        events += 1;
        count += tick.len();
        aco_audio::mix_into(&mut samples, &tick, &dry, t as f32 * dt, rate);
        aco_audio::place_into(&mut track, &dry, t as f32 * dt, rate);
    }
    aco_audio::mix_reverb_tail(&mut samples, &track, &rv, rate);
    let mut clip = aco_audio::Clip {
        sample_rate: rate,
        samples,
    };
    let peak = clip.peak();
    let gain = if peak > 1.0 { 1.0 / peak } else { 1.0 };
    clip.scale(gain);
    aco_audio::write_wav(out, &clip)?;
    Ok(format!(
        "已写入 {out}：{:.2} s / {} 点 / {rate} Hz / {events} 声事件（{count} 条路径）/ \
         峰值 {peak:.4}（归一 ×{gain:.4}）/ 混响 {}\n",
        clip.seconds(),
        clip.samples.len(),
        reverb_text(&rv)
    ))
}

/// 读干信号：**只认 WAV**；压缩容器（mp4/m4a/mp3…）**不解码**（引解码器要背编解码器与许可那一摊子，
/// 且与确定性优先的纪律无关），但"读不了"不该让人去猜 ⇒ 直接把能抄的转码命令打出来。
fn read_dry(path: &str) -> Result<Vec<f32>, String> {
    let ext = path.rsplit('.').next().unwrap_or("").to_ascii_lowercase();
    if COMPRESSED.contains(&ext.as_str()) {
        return Err(format!(
            "干信号只认 WAV：`{path}` 看着是 .{ext} 这种压缩容器，本层**不解码**。先转一道再喂：\n  \
             ffmpeg -i \"{path}\" -vn -ac 1 -ar 48000 -c:a pcm_s16le dry.wav\n  \
             vxl-phys audio <场景> [ticks] <出.wav> dry.wav"
        ));
    }
    Ok(aco_audio::read_wav(path)?.samples)
}

/// `audio` 的三个开关（`--reverb` / `--order2` / `--inside`）：收口成一个结构体 —— 形参再多一个就撞
/// `args-gate` 的软阈值（>7 棘轮）。
#[derive(Clone, Copy, Debug)]
pub(super) struct AudioFlags {
    pub(super) reverb: bool,
    pub(super) order2: bool,
    pub(super) inside: bool,
}

/// `audio` 的混响参数（A7）：`--reverb` 才开。演示房间是**声明的半开放几何**（地板 + 带门洞的墙 +
/// 天花板，四面敞开）⇒ 体积量不出来 ⇒ 用**声明的包络**：听者那间屋 `x∈[-4,4] y∈[0,6] z∈[1.5,4]`
/// （`V = 120 m³`、`S = 118 m²`）。建出来的面（天花板 / 墙背面）按几何表算吸声，没建出来的面按
/// **零吸声**（保守 ⇒ 尾巴偏长）；想要别的房间感就改这两个数。
fn audio_reverb(
    geo: &AcousticGeometry<'_>,
    cfg: &AcousticConfig,
    on: bool,
) -> aco_reverb::ReverbParams {
    if !on {
        return aco_reverb::ReverbParams::default();
    }
    aco_reverb::reverb_params_from(geo, cfg, 8.0 * 6.0 * 2.5, 118.0)
}

/// 开了混响就多留 1.5×RT60（中带，`BANDS/2`）：统计尾巴得衰减到听不见。
fn tail_seconds(rv: &aco_reverb::ReverbParams) -> f32 {
    if rv.is_on() {
        rv.rt60[1] * 1.5
    } else {
        0.0
    }
}

/// 汇总行里的混响部分（关了写"关"）—— 挪出来是为了压 `audio_cmd` 的圈复杂度。
fn reverb_text(rv: &aco_reverb::ReverbParams) -> String {
    if rv.is_on() {
        format!(
            "RT60 {:.2}/{:.2}/{:.2} s（Sabine）",
            rv.rt60[0], rv.rt60[1], rv.rt60[2]
        )
    } else {
        "关".to_string()
    }
}

/// 干信号里**压缩容器**的扩展名：`audio` 见到它们会直接报转码命令（本层只认 WAV，见 `audio_cmd`）。
const COMPRESSED: [&str; 14] = [
    "mp4", "m4a", "mov", "mkv", "webm", "avi", "mp3", "aac", "ogg", "oga", "opus", "flac", "wma",
    "wmv",
];
