//! **A7 判据**：统计混响尾巴（Sabine 逐带 RT60 + 确定性能量包络跟随）。
//!
//! 参照全部**手算**：一个 `4 × 3 × 2.5 m` 的鞋盒 ⇒ `V = 30 m³`、
//! `S = 2·(4·3 + 4·2.5 + 3·2.5) = 59 m²`；均匀吸声 `ᾱ = 0.2` ⇒
//! `RT60 = 0.161·30/(59·0.2) = 0.40932 s`、`onset = 4V/(S·c) = 5.931 ms`、`mix = 0.35·0.8 = 0.28`。
//!
//! 六段：① 鞋盒的 Sabine 绝对值；② **逐带** `RT60 ∝ 1/α`（`α = [0.1, 0.2, 0.4]` ⇒ 4:2:1）；
//! ③ 吸声是**面积加权**的（半边 0.1、半边 0.5 ⇒ `ᾱ = 0.3`）；④ 不闭合 ⇒ 没有尾巴（开场景没有"房间"）；
//! ⑤ 渲染：冲激的尾巴在 `RT60` 处掉 **60 dB**（±6 dB）、`onset` 之前一个采样都不动；
//! ⑥ `mix = 0` ⇒ 与"不加尾巴"**逐位**相同 + 同输入两跑**逐位**相同 + 单调性。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::audio::{self, AudioConfig};
use vxl_phys_acoustics::reverb::{reverb_params, reverb_params_from, ReverbParams, MIX};
use vxl_phys_acoustics::{AcousticConfig, AcousticGeometry, AcousticsExt, Listener, BANDS};

const FS: u32 = 48_000;
const SPEED: f32 = 343.0;

/// 鞋盒 `4 × 3 × 2.5`（原点在角落）：12 个三角、逐面吸声由 `alpha` 给。
struct Box6 {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
}

impl Box6 {
    /// `alpha_of(face)`：6 张面按加入顺序 `0=y⁻ 1=y⁺ 2=x⁻ 3=x⁺ 4=z⁻ 5=z⁺`，每面 2 个三角。
    ///
    /// ⚠️ **共享 8 个角点**（不是每面各建 4 个顶点）：`reverb` 的"闭合"判定是按**索引**配对的
    /// —— 各面各建顶点的话，几何看着是盒子、索引上却是一个个独立的四边形 ✗。
    fn new(alpha_of: &dyn Fn(usize) -> [f32; BANDS]) -> Self {
        let (x, y, z) = (4.0f32, 3.0f32, 2.5f32);
        let verts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(x, 0.0, 0.0),
            Vec3::new(x, y, 0.0),
            Vec3::new(0.0, y, 0.0),
            Vec3::new(0.0, 0.0, z),
            Vec3::new(x, 0.0, z),
            Vec3::new(x, y, z),
            Vec3::new(0.0, y, z),
        ];
        // ⚠️ 绕序一律**朝外**（闭合 + 定向的前提；`detect_edges` 也靠它）。
        let faces: [[u32; 3]; 12] = [
            [0, 1, 5],
            [0, 5, 4], // y⁻
            [3, 7, 6],
            [3, 6, 2], // y⁺
            [0, 4, 7],
            [0, 7, 3], // x⁻
            [1, 2, 6],
            [1, 6, 5], // x⁺
            [0, 3, 2],
            [0, 2, 1], // z⁻
            [4, 5, 6],
            [4, 6, 7], // z⁺
        ];
        let mut absorb = Vec::with_capacity(12);
        for f in 0..6 {
            absorb.extend([alpha_of(f); 2]);
        }
        Self {
            verts,
            tris: faces.to_vec(),
            absorb,
        }
    }

    fn geom(&self) -> AcousticGeometry<'_> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &[],
            scattering: &[],
            portals: &[],
            edges: &[],
            medium: None,
        }
    }
}

fn cfg() -> AcousticConfig {
    AcousticConfig::default()
}

fn flat(a: f32) -> [f32; BANDS] {
    [a; BANDS]
}

fn close(got: f32, want: f32, tag: &str) {
    assert!(
        (got - want).abs() <= want.abs() * 1e-4,
        "{tag}：实测 {got} vs 手算 {want}"
    );
}

/// ① 鞋盒 + 均匀吸声 0.2：`RT60 = 0.40932 s`、`onset = 5.931 ms`、`mix = 0.28`。
#[test]
fn shoebox_sabine_matches_hand_values() {
    let b = Box6::new(&|_| flat(0.2));
    let rv = reverb_params(&b.geom(), &cfg());
    for bnd in 0..BANDS {
        close(rv.rt60[bnd], 0.409_319_8, "RT60 = 0.161·30/(59·0.2)");
        close(rv.mix[bnd], MIX * 0.8, "mix = 0.35·(1−0.2)");
    }
    close(rv.onset, 4.0 * 30.0 / (59.0 * SPEED), "onset = 4V/(S·c)");
    assert!(rv.is_on(), "封闭鞋盒必须有尾巴");
}

/// ② 逐带：`α = [0.1, 0.2, 0.4]` ⇒ `RT60` 之比 4 : 2 : 1（`0.161·30/(59·0.1) = 0.8186395 s`）。
#[test]
fn per_band_rt60_is_inverse_alpha() {
    let b = Box6::new(&|_| [0.1, 0.2, 0.4]);
    let rv = reverb_params(&b.geom(), &cfg());
    close(rv.rt60[0], 0.818_639_5, "α=0.1 的 RT60");
    close(rv.rt60[1], 0.409_319_8, "α=0.2 的 RT60（= 上一半）");
    close(rv.rt60[2], 0.204_659_9, "α=0.4 的 RT60（= 再一半）");
    assert!(
        rv.mix[0] > rv.mix[1] && rv.mix[1] > rv.mix[2],
        "越吸声尾巴越弱"
    );
}

/// ③ 吸声必须**按面积加权**：半边 0.1、半边 0.5 ⇒ `ᾱ = 0.3` ⇒ `RT60 = 0.161·30/(59·0.3)`。
#[test]
fn absorption_is_area_weighted() {
    let b = Box6::new(&|f| if f % 2 == 0 { flat(0.1) } else { flat(0.5) });
    let rv = reverb_params(&b.geom(), &cfg());
    // 每对相对面面积相等（12 / 10 / 7.5 m² 各一对）⇒ 一半面积 0.1、一半 0.5 ⇒ ᾱ = 0.3。
    close(
        rv.rt60[1],
        0.161 * 30.0 / (59.0 * 0.3),
        "面积加权后的 ᾱ = 0.3",
    );
}

/// ④ 开场景（拆掉一面墙）⇒ 不闭合 ⇒ **没有尾巴**（体积没有意义，`mix` 全 0）。
#[test]
fn open_geometry_has_no_tail() {
    let mut b = Box6::new(&|_| flat(0.2));
    b.tris.truncate(b.tris.len() - 2); // 去掉 z⁺ 那面（两块三角）
    b.absorb.truncate(b.absorb.len() - 2);
    let rv = reverb_params(&b.geom(), &cfg());
    assert!(!rv.is_on(), "不闭合的几何不该有统计尾巴");
    assert_eq!(rv.mix, [0.0; BANDS], "没尾巴就是全 0（不是小值）");
}

/// ⑤ 渲染：冲激的尾巴在 `RT60` 处掉 60 dB（±6 dB）、`onset` 之前**一个采样都不动**。
#[test]
fn rendered_tail_decays_by_sixty_db() {
    let b = Box6::new(&|_| flat(0.2));
    let rv = reverb_params(&b.geom(), &cfg());
    let secs = 1.2;
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: secs,
    };
    let n = cfg.samples();
    let mut dry = vec![0.0f32; n];
    dry[0] = 1.0;
    let mut wet = vec![0.0f32; n];
    audio::mix_reverb_into(&mut wet, &dry, &rv, 0.0, FS);
    let start = (rv.onset * FS as f32) as usize;
    assert!(
        wet[..start].iter().all(|v| *v == 0.0),
        "onset（{} 采样）之前不该有尾巴",
        start
    );
    assert!(
        wet[start + 8..].iter().any(|v| v.abs() > 1e-3),
        "onset 之后要有尾巴"
    );
    // ⚠️ 窗要**同宽、中心正好相隔 RT60**：窗内指数衰减带来的形状修正在两个窗里相同、比值里消掉
    // ⇒ 能量密度比严格是 `keep^RT60 = 1e-3`（−60 dB）。第一版写成"开头 50 ms vs 结尾 50 ms"，
    // 两个窗的**中心**只差 `RT60 − 25 ms` ⇒ 实测 −52.5 dB —— 那是判据算错了，不是合成错。
    let win = (FS as f32 * 0.05) as usize;
    let rt = (rv.rt60[1] * FS as f32) as usize;
    let density = |from: usize| -> f32 {
        wet[from..from + win].iter().map(|v| v * v).sum::<f32>() / win as f32
    };
    let db = 10.0 * (density(start + rt) / density(start)).log10();
    assert!(
        (db + 60.0).abs() <= 3.0,
        "相隔一个 RT60 的两个同宽窗，能量密度该低 ~60 dB（实测 {db:.1} dB）"
    );
}

/// ⑥ 边界与确定性：`mix = 0` ⇒ 与不加尾巴逐位相同；同输入两跑逐位相同；吸声大 ⇒ 同一时刻更弱。
#[test]
fn boundaries_determinism_and_monotonicity() {
    let cfg = AudioConfig {
        sample_rate: FS,
        seconds: 0.6,
    };
    let n = cfg.samples();
    let mut dry = vec![0.0f32; n];
    dry[0] = 1.0;
    let off = ReverbParams {
        rt60: [0.4; BANDS],
        mix: [0.0; BANDS],
        onset: 0.0,
    };
    let mut a = vec![0.0f32; n];
    let mut b = vec![0.0f32; n];
    audio::mix_reverb_into(&mut a, &dry, &off, 0.0, FS);
    audio::mix_reverb_into(&mut b, &dry, &off, 0.0, FS);
    assert_eq!(a, b, "同输入两跑必须逐位一致");
    assert!(a.iter().all(|v| *v == 0.0), "mix = 0 ⇒ 一个采样都不加");
    // 吸声大（RT60 短）⇒ 同一时刻的尾巴更弱
    let long = ReverbParams {
        rt60: [0.8; BANDS],
        mix: [0.28; BANDS],
        onset: 0.0,
    };
    let short = ReverbParams {
        rt60: [0.2; BANDS],
        mix: [0.28; BANDS],
        onset: 0.0,
    };
    let mut l = vec![0.0f32; n];
    let mut s = vec![0.0f32; n];
    audio::mix_reverb_into(&mut l, &dry, &long, 0.0, FS);
    audio::mix_reverb_into(&mut s, &dry, &short, 0.0, FS);
    let at = (0.15 * FS as f32) as usize;
    // 归一化（`sqrt(1−keep)`）之后，同一点的比值不再随 RT60 无限增长：0.15 s 处约 3.5×，
    // 留 2.5× 的余量（判据只谈"长的更响"这个单调性）。
    assert!(
        l[at].abs() > s[at].abs() * 2.5,
        "RT60 长的那条在 0.15 s 处该显著更响（{} vs {}）",
        l[at],
        s[at]
    );
}

/// ⑦ **声明包络**：三角网不闭合时，作者给的 `V`/`S` 顶上——吸声仍按几何表**面积加权**，
/// 而声明表面积里**没建出来的面按零吸声**（口径写在 `reverb` 头注里）。
///
/// 手算：拆掉 `z⁺`（12 m²）后剩下 47 m² 都按 0.2 ⇒ `ᾱ = 47·0.2/59 = 0.1593220` ⇒
/// `RT60 = 0.161·30/(59·0.1593220) = 0.5138298 s`。
#[test]
fn declared_envelope_covers_open_meshes() {
    let mut open = Box6::new(&|_| flat(0.2));
    open.tris.truncate(open.tris.len() - 2);
    open.absorb.truncate(open.absorb.len() - 2);
    let geo = open.geom();
    assert!(
        !reverb_params(&geo, &cfg()).is_on(),
        "不闭合时几何自己量不出房间（先钉住这条前提，否则下面没意义）"
    );
    let rv = reverb_params_from(&geo, &cfg(), 30.0, 59.0);
    assert!(rv.is_on(), "声明了包络就该有尾巴");
    close(rv.rt60[1], 0.513_829_8, "声明包络下的 RT60");
    close(rv.onset, 4.0 * 30.0 / (59.0 * SPEED), "声明包络下的 onset");
    // 体积/表面积非正 ⇒ 关（不给"半间屋"这类含糊输入留口子）
    assert!(!reverb_params_from(&geo, &cfg(), 0.0, 59.0).is_on());
    assert!(!reverb_params_from(&geo, &cfg(), 30.0, -1.0).is_on());
}

/// 零干扰（与 A1–A6 同款）：算混响参数不碰 `World`。
#[test]
fn reverb_params_are_read_only() {
    let b = Box6::new(&|_| flat(0.2));
    let mut w = World::new(PhysConfig::default());
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let before = w.state_hash();
    let rv = reverb_params(&b.geom(), &cfg());
    let mut paths = Vec::new();
    let listener = Listener {
        pos: Vec3::new(0.5, 1.0, 0.5),
        radius: 0.4,
    };
    let _ = w.acoustic_cues(&b.geom(), &listener, &cfg(), &mut paths);
    assert!(rv.is_on(), "封闭鞋盒要有尾巴（否则这条判据空跑）");
    assert_eq!(w.state_hash(), before, "声学层必须只读（state_hash 不变）");
}
