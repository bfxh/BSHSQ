//! **A6「离线音频渲染」判据**（`docs/SURVEY-ACOUSTIC-RAYTRACING.md` §8 第 3 条）：
//! cue 流 → **能听的波形**。这里钉六件事：
//!
//! 1. **尖峰 = 时延**（±1 采样）、**尖峰高 = `sqrt(能量)`**（模块头注那条口径，不是能量本身）；
//! 2. **叠法**与手算参照逐采样一致（时延量化 + 逐带幅度 + 就地累加；参照在测试侧独立写一遍）；
//! 3. **分带是行为判据**（判据只谈"低频走低带、高频走高带"这类**单调**性质，不绑具体 Hz 的
//!    频响数值 —— 与 [`BANDS`] 的口径一致）；
//! 4. **首达摆位**：干信号是一记拍手，输出的首达就该落在 `min(delay)` 上（时延真的摆在时间轴上）；
//! 5. **WAV 读写往返**（PCM16 量化误差 ≤ 2e-5）+ 多声道降混 + 非 WAV 报错（不猜）；
//! 6. **真实场景端到端**：体素地面砸出的冲击 → 绕射/反射路径 → 渲染出**有声且有限**的波形；
//!    同输入两跑**逐位相同**（含 WAV 字节），且渲染前后 `state_hash` 不变（渲染层只读）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::audio::{self, AudioConfig, Clip};
use vxl_phys_acoustics::{
    AcousticConfig, AcousticGeometry, AcousticsExt, Edge, Listener, PathKind, SoundPath, BANDS,
};

const FS: u32 = 48_000;

/// 一条手工路径（`bands` 就是**能量**增益；`gain` 取中频带 —— 与引擎的口径一致）。
fn path(delay: f32, bands: [f32; BANDS], kind: PathKind) -> SoundPath {
    SoundPath {
        source: 0,
        delay,
        bands,
        gain: bands[BANDS / 2],
        dir: [0.0, 0.0, 1.0],
        kind,
    }
}

/// 只有第 `b` 带有能量的路径。
fn one_band(b: usize) -> [f32; BANDS] {
    let mut bands = [0.0f32; BANDS];
    bands[b] = 1.0;
    bands
}

/// 一段正弦（测试侧的干信号素材）。
fn sine(freq: f32, seconds: f32) -> Vec<f32> {
    let n = (FS as f32 * seconds) as usize;
    (0..n)
        .map(|i| (6.283_185_5 * freq * i as f32 / FS as f32).sin())
        .collect()
}

fn rms(x: &[f32]) -> f32 {
    if x.is_empty() {
        return 0.0;
    }
    (x.iter().map(|&v| v * v).sum::<f32>() / x.len() as f32).sqrt()
}

/// 峰值位置（IR 的尖峰判据用它）。
fn peak_index(x: &[f32]) -> usize {
    let mut bi = 0usize;
    let mut bv = 0.0f32;
    for (i, &v) in x.iter().enumerate() {
        if v.abs() > bv {
            bv = v.abs();
            bi = i;
        }
    }
    bi
}

/// 第一个超过 `gate` 的采样位置（首达判据用它）。
fn first_above(x: &[f32], gate: f32) -> usize {
    for (i, &v) in x.iter().enumerate() {
        if v.abs() > gate {
            return i;
        }
    }
    x.len()
}

/// ① **尖峰 = 时延、尖峰高 = `sqrt(能量)`**：两条路径（能量 0.25 / 0.0625 ⇒ 幅度 0.5 / 0.25）。
#[test]
fn ir_taps_land_on_delays_with_sqrt_energy_amplitude() {
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.06,
    };
    let paths = [
        path(0.010, [0.25; BANDS], PathKind::Direct),
        path(0.030, [0.0625; BANDS], PathKind::Reflected),
    ];
    let ir = audio::impulse_response(&paths, &cfg);
    assert_eq!(
        peak_index(&ir.samples),
        480,
        "最强尖峰该落在 0.010 s（480 点）"
    );
    for (idx, amp) in [(480usize, 0.5f32), (1440, 0.25)] {
        let got = ir.samples[idx];
        assert!(
            (got - amp).abs() <= amp * 1e-3,
            "第 {idx} 点的尖峰高该是 sqrt(能量) = {amp}（实际 {got}）"
        );
    }
    // 三带同参 ⇒ 分带器的**和**就是原信号：尖峰之外没有别的东西（尾巴是分带器的指数衰减，
    // 幅度已经掉到 1% 以下）。
    assert!(
        ir.samples[480 + 200] < 0.5 * 0.01,
        "尖峰后面只该剩分带器的尾巴（实际 {}）",
        ir.samples[480 + 200]
    );
}

/// ② **叠法**：多路径 + 多带 ⇒ 与测试侧**独立手算**的逐采样累加一致。
///
/// 这条钉的是叠法（时延**量化**到采样、幅度取 `sqrt(能量)`、就地累加、越界丢弃）；分带器本身
/// 由 ③ 的**行为**判据覆盖 ⇒ 参照里直接调 [`audio::split3`] 是有意为之（不然这条会把分带器的
/// 实现也抄一遍，变成自证）。
#[test]
fn mixing_law_matches_a_hand_summed_reference() {
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.05,
    };
    let dry = sine(700.0, 0.02);
    let paths = [
        path(0.0, [1.0, 0.5, 0.25], PathKind::Direct),
        path(0.0123, [0.04, 0.01, 0.0], PathKind::Transmitted),
    ];
    let got = audio::render(&paths, &dry, &cfg);
    let bands = audio::split3(&dry, FS);
    let mut want = vec![0.0f32; cfg.samples()];
    for p in paths.iter() {
        let start = (p.delay * FS as f32).round() as usize;
        for (b, band) in bands.iter().enumerate() {
            for (i, &x) in band.iter().enumerate() {
                let j = start + i;
                if j < want.len() {
                    want[j] += p.bands[b].sqrt() * x;
                }
            }
        }
    }
    for (i, (&g, &w)) in got.samples.iter().zip(want.iter()).enumerate() {
        assert!((g - w).abs() <= 1e-6, "第 {i} 个采样：{g} vs 手算 {w}");
    }
}

/// ③ **逐带选择性**（行为口径）：低频该走低带、2 kHz 该走中带、高频该走高带。
#[test]
fn band_selectivity_is_monotone() {
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.2,
    };
    let skip = FS as usize / 20; // 跳过 50 ms 的滤波器瞬态
    let level = |b: usize, f: f32| -> f32 {
        let dry = sine(f, cfg.seconds);
        let clip = audio::render(&[path(0.0, one_band(b), PathKind::Direct)], &dry, &cfg);
        rms(&clip.samples[skip..])
    };
    let (l, m, h) = (level(0, 60.0), level(1, 60.0), level(2, 60.0));
    assert!(
        l > m && m > h && l >= 10.0 * h,
        "60 Hz 该走低带（低 {l} / 中 {m} / 高 {h}）"
    );
    // 中带是"两级低通之间"的宽带 ⇒ 2 kHz 只要**最响**（两端的口径见 60 Hz / 12 kHz 那两条，
    // 它们才是"高频更闷"这类单调性质的硬判据）。
    let (l, m, h) = (level(0, 2_000.0), level(1, 2_000.0), level(2, 2_000.0));
    assert!(
        m > l && m > 1.5 * h,
        "2 kHz 该走中带（低 {l} / 中 {m} / 高 {h}）"
    );
    let (l, m, h) = (level(0, 12_000.0), level(1, 12_000.0), level(2, 12_000.0));
    assert!(
        h > m && m > l && h >= 10.0 * l,
        "12 kHz 该走高带（低 {l} / 中 {m} / 高 {h}）"
    );
}

/// ④ **首达摆位**：干信号是一记"拍手" ⇒ 输出的首个显著样本落在 `delay` 上；更远的那条更晚更弱。
#[test]
fn first_arrival_lands_at_the_direct_delay() {
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.2,
    };
    let dry = audio::clap(FS, 0.05);
    let gate = 0.2 * dry.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
    let onset = first_above(&dry, gate);
    let near = audio::render(&[path(0.02, [1.0; BANDS], PathKind::Direct)], &dry, &cfg);
    let want = onset + (0.02 * FS as f32).round() as usize;
    let got = first_above(&near.samples, gate);
    assert!(
        got.abs_diff(want) <= 4,
        "首达该在 {want}（拍手首达 {onset} + 0.02 s），实际 {got}"
    );
    let far = audio::render(&[path(0.05, [0.25; BANDS], PathKind::Direct)], &dry, &cfg);
    assert!(
        first_above(&far.samples, gate) > got + 1_000,
        "更远的那条必须更晚到"
    );
    assert!(far.peak() < near.peak(), "更远的那条必须更弱（球面扩散）");
}

/// ⑤ **WAV 往返**：写出的字节头自检 + 读回来逐采样 ≤ 2e-5（16 bit 量化）+ 多声道降混 + 报错。
#[test]
fn wav_round_trip_and_downmix() {
    let clip = Clip {
        sample_rate: FS,
        samples: sine(440.0, 0.01),
    };
    let bytes = audio::wav_bytes(&clip);
    assert_eq!(
        bytes.len(),
        44 + clip.samples.len() * 2,
        "44 字节头 + PCM16"
    );
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"WAVE");
    assert_eq!(&bytes[36..40], b"data");
    let parsed = audio::parse_wav(&bytes);
    assert!(
        parsed.is_ok(),
        "自己写出来的 WAV 读不回来：{:?}",
        parsed.as_ref().err()
    );
    let back = match parsed {
        Ok(c) => c,
        Err(_) => return,
    };
    assert_eq!(back.sample_rate, FS);
    assert_eq!(back.samples.len(), clip.samples.len());
    for (i, (a, b)) in clip.samples.iter().zip(back.samples.iter()).enumerate() {
        assert!(
            (a - b).abs() <= 2e-5,
            "第 {i} 个采样量化误差过大：{a} vs {b}"
        );
    }
    // 手写一段**立体声 float32**（左 +1 / 右 −1 ⇒ 等幅反相）：降混必须给 0，且不报错。
    let stereo = stereo_f32_wav(8);
    let parsed = audio::parse_wav(&stereo);
    assert!(
        parsed.is_ok(),
        "float32 立体声读不回来：{:?}",
        parsed.as_ref().err()
    );
    let mixed = match parsed {
        Ok(c) => c,
        Err(_) => return,
    };
    assert_eq!(mixed.samples.len(), 8, "8 帧立体声 ⇒ 8 个降混采样");
    assert!(
        mixed.samples.iter().all(|&x| x == 0.0),
        "两声道等幅反相 ⇒ 降混为 0（实际 {:?}）",
        mixed.samples
    );
    assert!(
        audio::parse_wav(b"definitely not a wav file").is_err(),
        "不是 WAV 就该报错（不猜）"
    );
}

/// 手写 8 帧、左 +1 / 右 −1 的 **float32 立体声** WAV 字节（判降混用；本层写不出 float32 ⇒ 手工造）。
fn stereo_f32_wav(frames: usize) -> Vec<u8> {
    let data_len = (frames * 2 * 4) as u32;
    let mut b = Vec::new();
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data_len).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&3u16.to_le_bytes()); // 3 = IEEE float
    b.extend_from_slice(&2u16.to_le_bytes()); // 立体声
    b.extend_from_slice(&FS.to_le_bytes());
    b.extend_from_slice(&(FS * 2 * 4).to_le_bytes());
    b.extend_from_slice(&8u16.to_le_bytes()); // 块对齐 = 2 通道 × 4 字节
    b.extend_from_slice(&32u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data_len.to_le_bytes());
    for _ in 0..frames {
        b.extend_from_slice(&1.0f32.to_le_bytes());
        b.extend_from_slice(&(-1.0f32).to_le_bytes());
    }
    b
}

/// 声学几何的**声明房间**（与 `a4_diffraction.rs` 同款的最小版）：地板 + 挡板 + 挡板顶上的棱。
struct Room {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    edges: Vec<Edge>,
}

impl Room {
    fn new() -> Self {
        let verts = vec![
            Vec3::new(-4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, 4.0),
            Vec3::new(-4.0, 0.0, 4.0),
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ];
        let tris = vec![[0u32, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]];
        Self {
            verts,
            tris,
            absorb: vec![[0.25f32; BANDS]; 4],
            trans: vec![[0.0f32; BANDS]; 4],
            edges: vec![Edge {
                a: Vec3::new(-1.0, 1.0, 1.0),
                b: Vec3::new(1.0, 1.0, 1.0),
                coeff: [1.0f32; BANDS],
            }],
        }
    }

    fn geom(&self) -> AcousticGeometry<'_> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            scattering: &[],
            portals: &[],
            edges: &self.edges,
            medium: None,
        }
    }
}

/// 体素地面 + 一发落体冲击（声源 = `World::impacts()` 的接触点）。
fn world_with_impact() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.0, -0.25, -1.0), 0.25, 8, 1, 8);
    vol.fill_box(Vec3::new(-1.0, -0.25, -1.0), Vec3::new(1.0, 0.0, 1.0));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(0.0, -6.0, 0.0);
    for _ in 0..40 {
        w.step();
        if !w.impacts().is_empty() {
            break;
        }
    }
    w
}

/// ⑥ **真实场景端到端**：引擎产出的路径 ⇒ 渲得出声、首达摆位对、两跑逐位一致、只读。
#[test]
fn real_scene_renders_audible_and_deterministic() {
    let mut w = world_with_impact();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let room = Room::new();
    let geo = room.geom();
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.3,
    };
    let dry = audio::clap(FS, 0.05);
    let before = w.state_hash();
    // 两个位姿各判一遍：**墙后**（直达被挡，只剩绕射那条腿）与**墙前**（直达 + 反射）。
    // ⚠️ 墙后拿不到镜面反射是**几何正确**的结果（薄墙的镜面反射要求声源与听者在同一侧）⇒
    // 判据按"这个几何真正该有的条数"给，而不是一律"≥2"。
    for (label, pos, want) in [
        ("墙后（绕射）", Vec3::new(0.0, 0.5, 2.0), 1usize),
        ("墙前（直达 + 反射）", Vec3::new(0.0, 0.6, -2.0), 2usize),
    ] {
        let listener = Listener { pos, radius: 0.5 };
        let mut paths: Vec<SoundPath> = Vec::new();
        let st = w.acoustic_cues(&geo, &listener, &AcousticConfig::default(), &mut paths);
        assert!(
            st.paths() >= want,
            "{label}：至少 {want} 条路径（实际 {}；{st:?}）",
            st.paths()
        );
        let a = audio::render(&paths, &dry, &cfg);
        let b = audio::render(&paths, &dry, &cfg);
        assert_eq!(a.samples, b.samples, "{label}：同输入两跑必须逐位相同");
        assert_eq!(
            audio::wav_bytes(&a),
            audio::wav_bytes(&b),
            "{label}：WAV 字节也必须逐位相同"
        );
        let peak = a.peak();
        assert!(
            peak > 0.0 && peak.is_finite() && peak <= 8.0,
            "{label}：渲染结果要有声且有限（peak {peak}）"
        );
        // 首达不会早于几何给的时延（能量不能凭空早到），也不该拖后
        let min_delay = paths.iter().fold(f32::INFINITY, |m, p| m.min(p.delay));
        let onset = first_above(&a.samples, 0.2 * peak);
        let floor_idx = (min_delay * FS as f32).floor() as usize;
        assert!(
            onset + 4 >= floor_idx,
            "{label}：首达 {onset} 早于几何给的最短时延 {floor_idx}"
        );
        assert!(
            onset < floor_idx + FS as usize / 20,
            "{label}：首达不该拖后 50 ms 以上（{onset} vs {floor_idx}）"
        );
    }
    assert_eq!(w.state_hash(), before, "渲染层必须只读（state_hash 不变）");
}
