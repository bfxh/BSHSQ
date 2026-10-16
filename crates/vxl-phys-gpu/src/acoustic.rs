//! **声学核（GPU 档 M1/M2/M3/M4）**：一批 `(声源, 听者)` 对，每对一个独立工作项。
//!
//! 与 CPU 端 `vxl-phys-acoustics` 的对应那几条**同口径**（口径逐条写在 `src/acoustic.wgsl`
//! 的头注里）：M1 直达（遮挡走 Möller–Trumbore，端点共面不计；`dist/speed` 与 `1/dist²`）、
//! M2 一次镜面反射（镜像源法，逐对取最强那条）、M3 绕射（棱上 Fermat 点 +
//! `coeff·(1+cosθ)/2/(d₁·d₂)`，只在阴影里产出）、M4 散射（面当二级声源，Lambertian：
//! `s·cos_in·cos_out·A/(d₁²·d₂²)`，同侧 + 两腿不挡，取最强那个面）、M5 介质积分（水平层表 +
//! 定步数中点采样：`Σ len/c` 与 `Π exp(-α_b·len)`）。这里要钉的是
//! "同一批输入 ⇒ 与 CPU 参考**逐对一致**"（浮点容差、拓扑逐位），探针见
//! `tests/acoustic_*_probe.rs`。
//!
//! ⚠️ **CI 不跑**：本仓既有口径是 `cargo test --workspace --exclude vxl-phys-gpu`（headless 运行器
//! 无 GPU）。有适配器的机器上按探针跑。

use vxl_phys_core::Vec3;
use wgpu::util::DeviceExt;

use crate::pipeline::{dispatch, ent, f32_bytes, mk_layout, mk_pipe, Kind};

/// **M6 二阶反射**的宿主入口：单独一个子模块 —— 这个文件已经贴着本仓 god 门（单文件 ≤ 800 行）
/// 的边，新腿就落在子模块里（顺手把"改到就减"落一步）。
mod reflect2;

pub use reflect2::{reflected2_on_adapter, Reflect2Out};

/// 一个 `(声源, 听者)` 对的读数（与 WGSL 输出缓冲同序：`src_i * n_lst + lst_i`）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairOut {
    /// 直达是否被挡。
    pub blocked: bool,
    /// 时延（秒）。
    pub delay: f32,
    /// 能量增益（`1/d²`）。
    pub gain: f32,
}

/// 一个 `(声源, 听者)` 对的**反射**读数（两次归约：有/无 + 最强那条的时延/增益/三角序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReflectOut {
    /// 是否找到一次镜面反射。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`。
    pub delay: f32,
    /// 能量增益 ＝ `(1-a)/(d₁+d₂)²`。
    pub gain: f32,
    /// 最强那条反射所在三角的序号（没有则为 `-1`）。
    pub tri: f32,
}

/// 一个 `(声源, 听者)` 对的**绕射**读数（有/无 + 最强那条棱的时延/增益/棱序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffractOut {
    /// 是否找到绕射路径（阴影里且某条棱的两条腿都通视）。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`（介质积分还没上卡 ⇒ 只有几何行程）。
    pub delay: f32,
    /// 能量增益 ＝ `coeff·(1+cosθ)/2/(d₁·d₂)`。
    pub gain: f32,
    /// 最强那条棱的序号（没有则为 `-1`）。
    pub edge: f32,
}

/// 一个 `(声源, 听者)` 对的**散射**读数（有/无 + 最强那个面的时延/增益/三角序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScatterOut {
    /// 是否找到散射路径（面与声源/听者**同侧**、两个余弦 > 0、两条腿都不被挡）。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`（到面中心的两段直线；介质积分还没上卡）。
    pub delay: f32,
    /// 能量增益 ＝ `s·cos_in·cos_out·A/(d₁²·d₂²)`（`A = |cross|/2`；**单频带**）。
    pub gain: f32,
    /// 最强那个面的三角序号（没有则为 `-1`）。
    pub tri: f32,
}

/// 一批输入（几何/材质/棱都是**平铺切片** ⇒ 直接就是卡的布局）。
///
/// 打成结构体不是为了好看：`diffract` 这条本来要 7 个形参，撞本仓 `args-gate` 的软阈值
/// （>7 棘轮、7 实测也算违规）⇒ 收口成一个"场景入参"。
pub struct SceneIn<'a> {
    /// 声源位置。
    pub srcs: &'a [Vec3],
    /// 听者位置。
    pub dsts: &'a [Vec3],
    /// 三角面（每 3 个 `Vec3` 一面）。
    pub tris: &'a [[Vec3; 3]],
    /// 逐三角吸收（只有反射核读它；空表 ⇒ 全 0）。
    pub absorb: &'a [f32],
    /// 逐三角**散射系数**（只有散射核读它；**单频带**口径，空表 ⇒ 全 0 = 纯镜面）。
    pub scatter: &'a [f32],
    /// 衍射棱（每条两个端点；空表 ⇒ 无棱可绕）。
    pub edges: &'a [[Vec3; 2]],
    /// 逐棱绕射系数（**单频带口径**：频带表还没上卡）——与 [`SceneIn::edges`] **一一对应**。
    /// 波长 / 楔角**不在** `SceneIn` 里：见 [`diffracted_wedge_on_adapter`] 的 `band_hz` / `wedges`。
    pub edge_coeff: &'a [f32],
    /// 声速（m/s）。
    pub speed: f32,
}

/// `Params` 的宿主侧镜像（**字段顺序与 `acoustic.wgsl` 的 `struct Params` 一一对应**：32 B）。
///
/// 打成结构体不是为了好看：`n_slabs` / `medium_steps` 上来之后形参到 7 个，撞本仓 `args-gate`
/// 的软阈值 ⇒ 收口成一份"参数镜像"（顺便把布局写在一处，改字段只改这里）。
#[derive(Clone, Copy, Debug, Default)]
struct Params {
    n_src: u32,
    n_lst: u32,
    n_tris: u32,
    speed: f32,
    n_edges: u32,
    n_slabs: u32,
    medium_steps: u32,
    /// **A9a 波长哨兵**（m）：`≤ 0` ⇒ 绕射退回旧口径 `(1+cosθ)/2`（**逐位不变**）；
    /// `> 0` ⇒ 真 Fresnel 刀刃系数 + A9b 楔角修正。
    /// （这一格原为 pad —— 仍是 8 字段 ⇒ 32 B，撞 16 的倍数，对齐不变。）
    lambda: f32,
}

/// 序列化成 WGSL uniform 的字节（32 B；顺序 = 字段声明顺序）。
fn params_bytes(p: &Params) -> Vec<u8> {
    let mut b = Vec::with_capacity(32);
    for v in [p.n_src, p.n_lst, p.n_tris] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(&p.speed.to_le_bytes());
    for v in [p.n_edges, p.n_slabs, p.medium_steps] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(&p.lambda.to_le_bytes());
    b
}

/// 一个**水平层**（介质）：`y ∈ [y_lo, y_hi]` 内声速 `c`、逐带吸收 `alpha`（1/m）。
///
/// `alpha` 是 **3 个频带**，与 `vxl-phys-acoustics::BANDS` **同值**（GPU 档不引那个 crate：
/// 依赖面只留 `wgpu` + `vxl-phys-core`，所以这个 3 是**显式约定**，改那边要同步这里）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slab {
    pub y_lo: f32,
    pub y_hi: f32,
    pub c: f32,
    pub alpha: [f32; 3],
}

/// 一段（直线）路径的**介质积分**读数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MediumOut {
    /// 走完这段的**用时**（秒）＝ `Σ len_i / c_i`（逐采样点）。
    pub time: f32,
    /// 逐带**衰减因子**＝ `Π exp(-α_b·len)`（1 = 不衰减；3 带同 [`Slab::alpha`]）。
    pub atten: [f32; 3],
}

/// 平铺 `vec4`：每个 `Vec3` 占 4 个 f32（第 4 个补 0）⇒ 与 WGSL 的 `array<vec4<f32>>` 对齐。
fn vec4_bytes(v: &[Vec3]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(v.len() * 4);
    for p in v {
        vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
    }
    f32_bytes(&vals)
}

/// 三角平铺：每个三角 3 个 `vec4`（同 `vec4_bytes` 的补零口径）。
fn tris_bytes(tris: &[[Vec3; 3]]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(tris.len() * 12);
    for t in tris {
        for p in t {
            vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
        }
    }
    f32_bytes(&vals)
}

/// 棱平铺：每条 2 个 `vec4`（`a`、`b`；同 `vec4_bytes` 的补零口径）。
fn edges_bytes(edges: &[[Vec3; 2]]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(edges.len() * 8);
    for e in edges {
        for p in e {
            vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
        }
    }
    f32_bytes(&vals)
}

/// **A9b 逐棱楔角**（与 [`SceneIn::edges`] **一一对应**）：`interior_angle` = 楔体**内角** Ω（弧度），
/// `face0` = 楔体**某一面**上**垂直于棱**的方向（定 `φ = 0` 的基准）。
/// `Ω ≤ 0`（或整表空）⇒ 该棱按**刀口**算（`n = 2`、`fac = 1`）。
///
/// 口径与 CPU `vxl-phys-acoustics::utd::Wedge` 相同；GPU 档不引那个 crate ⇒ 这里是**约定镜像**。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WedgeIn {
    pub interior_angle: f32,
    pub face0: Vec3,
}

/// 楔角平铺：每条 1 个 `vec4`（`Ω|face0.x|face0.y|face0.z`）。
fn wedge_bytes(wedges: &[WedgeIn]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(wedges.len() * 4);
    for w in wedges {
        vals.extend_from_slice(&[w.interior_angle, w.face0.x, w.face0.y, w.face0.z]);
    }
    f32_bytes(&vals)
}

/// 空楔角表时的哑表（`Ω = 0` ⇒ 核内按刀口算；只为让空表也有一张非零长的缓冲）。
const ZERO_WEDGE: WedgeIn = WedgeIn {
    interior_angle: 0.0,
    face0: Vec3::ZERO,
};

/// 场景缓冲（两个入口共用：只有输出不同）。
struct Buffers {
    pb: wgpu::Buffer,
    sb: wgpu::Buffer,
    db: wgpu::Buffer,
    tb: wgpu::Buffer,
    ab: wgpu::Buffer,
    kb: wgpu::Buffer,
    eb: wgpu::Buffer,
    cb: wgpu::Buffer,
    wb: wgpu::Buffer,
    ob: wgpu::Buffer,
    rb: wgpu::Buffer,
    n_pairs: usize,
}

/// 上传一批输入（几何/材质表都是平铺切片 ⇒ 直接就是卡的布局）。
///
/// `wedges` 与 `sc.edges` 一一对应（空表 ⇒ 全按刀口）；`band_hz ≤ 0` ⇒ `lambda = 0`
/// ⇒ 绕射走**旧口径**（逐位不变）。
fn upload(
    device: &wgpu::Device,
    sc: &SceneIn<'_>,
    wedges: &[WedgeIn],
    band_hz: f32,
) -> (Buffers, u64) {
    // 与 CPU `utd::wavelength` 同式：`hz ≤ 0 ⇒ 0`，否则 `max(c,1e-3)/hz`。
    let lambda = if band_hz <= 0.0 {
        0.0
    } else {
        sc.speed.max(1e-3) / band_hz
    };
    let n_pairs = sc.srcs.len() * sc.dsts.len();
    let mk_storage = |label: &str, bytes: &[u8]| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        })
    };
    let pb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("acoustic.params"),
        contents: &params_bytes(&Params {
            n_src: sc.srcs.len() as u32,
            n_lst: sc.dsts.len() as u32,
            n_tris: sc.tris.len() as u32,
            speed: sc.speed,
            n_edges: sc.edges.len() as u32,
            lambda,
            ..Params::default()
        }),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let sb = mk_storage("acoustic.srcs", &vec4_bytes(sc.srcs));
    let db = mk_storage("acoustic.dsts", &vec4_bytes(sc.dsts));
    let tb = mk_storage("acoustic.tris", &tris_bytes(sc.tris));
    // 这几张表都可能为空（直达不吃吸收、场景可能没有棱）⇒ 统一补**非零长**的哑缓冲：
    // 零长 storage 绑定不合法。哑值都是 0 ⇒ 对读数没有影响。
    //
    // ⚠️ 逐三角那两张表（吸收/散射）空表时按**三角数**补零：核内是 `absorb[t]` / `scatter_coef[t]`
    // 直接索引 ⇒ 只补 1 个元素的话，"空表 + 多个三角"就是**越界读**（不是静默出错那么轻）。
    let zeros = |n: usize| vec![0.0f32; n.max(1)];
    let ab = mk_storage(
        "acoustic.absorb",
        &f32_bytes(if_empty(sc.absorb, &zeros(sc.tris.len()))),
    );
    let kb = mk_storage(
        "acoustic.scatter",
        &f32_bytes(if_empty(sc.scatter, &zeros(sc.tris.len()))),
    );
    let eb = mk_storage(
        "acoustic.edges",
        &edges_bytes(if_empty(sc.edges, &[ZERO_EDGE])),
    );
    let cb = mk_storage(
        "acoustic.edge_coeff",
        &f32_bytes(if_empty(sc.edge_coeff, &[0.0])),
    );
    let wb = mk_storage(
        "acoustic.edge_wedge",
        &wedge_bytes(if_empty(wedges, &[ZERO_WEDGE])),
    );
    let out_bytes = (n_pairs * 16) as u64;
    let ob = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("acoustic.out"),
        size: out_bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let rb = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("acoustic.rb"),
        size: out_bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    (
        Buffers {
            pb,
            sb,
            db,
            tb,
            ab,
            kb,
            eb,
            cb,
            wb,
            ob,
            rb,
            n_pairs,
        },
        out_bytes,
    )
}

/// 一根"哑棱"（全零；只为让空棱表也有一张非零长的缓冲）。
const ZERO_EDGE: [Vec3; 2] = [Vec3::ZERO; 2];

/// 空切片 ⇒ 换成哑表（零长 storage 绑定不合法）。
fn if_empty<'a, T: Copy>(v: &'a [T], dummy: &'a [T]) -> &'a [T] {
    if v.is_empty() {
        dummy
    } else {
        v
    }
}

/// 跑指定入口并回读：每对 4 个 f32（`[kind, delay, gain, extra]`）。
fn run_entry(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    b: &Buffers,
    entry: &str,
) -> Vec<[f32; 4]> {
    let out_bytes = (b.n_pairs * 16) as u64;
    let bgl = mk_layout(
        device,
        "acoustic.bgl",
        &[
            (0, Kind::Uniform),
            (1, Kind::Ro),
            (2, Kind::Ro),
            (3, Kind::Ro),
            (4, Kind::Rw),
            (5, Kind::Ro),
            (6, Kind::Ro),
            (7, Kind::Ro),
            (8, Kind::Ro),
            (10, Kind::Ro),
        ],
    );
    // ⚠️ 这个变量必须叫 `shader`（词汇门只豁免这一种写法，见 `scripts/vocab_scan.sh` 头部）。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("acoustic.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("acoustic.wgsl").into()),
    });
    let pipe = mk_pipe(device, &bgl, &shader, "acoustic.pipe", entry);
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("acoustic.bg"),
        layout: &bgl,
        entries: &[
            ent(0, &b.pb),
            ent(1, &b.sb),
            ent(2, &b.db),
            ent(3, &b.tb),
            ent(4, &b.ob),
            ent(5, &b.ab),
            ent(6, &b.eb),
            ent(7, &b.cb),
            ent(8, &b.kb),
            ent(10, &b.wb),
        ],
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("acoustic.enc"),
    });
    dispatch(&mut enc, &pipe, &bg, (b.n_pairs as u32).div_ceil(64).max(1));
    enc.copy_buffer_to_buffer(&b.ob, 0, &b.rb, 0, out_bytes);
    queue.submit(Some(enc.finish()));
    read_rows(device, &b.rb, b.n_pairs)
}

/// 回读 `n` 行（每行 4 个 `f32`）—— M1–M4 与 M5 两个入口共用。
fn read_rows(device: &wgpu::Device, rb: &wgpu::Buffer, n: usize) -> Vec<[f32; 4]> {
    let slice = rb.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        tx.send(r).ok();
    });
    device
        .poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        })
        .ok();
    rx.recv().ok();
    let data = crate::mapped::mapped_view(slice);
    let mut rows = Vec::with_capacity(n);
    for k in 0..n {
        let o = k * 16;
        let f = |j: usize| {
            f32::from_le_bytes([
                data[o + j * 4],
                data[o + j * 4 + 1],
                data[o + j * 4 + 2],
                data[o + j * 4 + 3],
            ])
        };
        rows.push([f(0), f(1), f(2), f(3)]);
    }
    drop(data);
    rb.unmap();
    rows
}

/// 在指定适配器上跑一趟**直达核**（M1）：返回 `(适配器名, 逐对读数)`。
///
/// 对序是 `src_i * n_lst + lst_i`（与 `acoustic.wgsl` 的展平口径一致）。
pub fn direct_paths_on_adapter(
    adapter_index: usize,
    srcs: &[Vec3],
    dsts: &[Vec3],
    tris: &[[Vec3; 3]],
    speed: f32,
) -> Result<(String, Vec<PairOut>), String> {
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if srcs.is_empty() || dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb: &[],
        scatter: &[],
        edges: &[],
        edge_coeff: &[],
        speed,
    };
    let (b, _) = upload(&device, &sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "direct") {
        out.push(PairOut {
            blocked: row[3] > 0.5,
            delay: row[1],
            gain: row[2],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**一次镜面反射核**（M2）：逐对取**最强那条**反射（平局按三角序号取小）。
///
/// `absorb` 逐三角给（`0` = 全反射；缺了按空表 ⇒ 当 0 算）。
pub fn reflected_on_adapter(
    adapter_index: usize,
    srcs: &[Vec3],
    dsts: &[Vec3],
    tris: &[[Vec3; 3]],
    absorb: &[f32],
    speed: f32,
) -> Result<(String, Vec<ReflectOut>), String> {
    // 逐三角表与三角表**一一对应**（空表 = 全 0，由 `upload` 补零；给短了就是越界读）。
    if !absorb.is_empty() && absorb.len() != tris.len() {
        return Err(format!(
            "吸收表与三角表必须一一对应（{} vs {}）",
            absorb.len(),
            tris.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if srcs.is_empty() || dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb,
        scatter: &[],
        edges: &[],
        edge_coeff: &[],
        speed,
    };
    let (b, _) = upload(&device, &sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "reflect") {
        out.push(ReflectOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            tri: row[3],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**绕射核**（M3）：**只在阴影里**产出，逐对取**最强那条**棱
/// （平局按棱序号取小 ⇒ 确定）。
///
/// 这是 [`diffracted_wedge_on_adapter`] 的**旧口径薄壳**：不给波长 ⇒ `lambda = 0`
/// ⇒ 增益 = `coeff·(1+cosθ)/2 /(d₁·d₂)`、时延 = `(d₁+d₂)/speed`（既有探针**逐位不变**）。
pub fn diffracted_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
) -> Result<(String, Vec<DiffractOut>), String> {
    diffracted_wedge_on_adapter(adapter_index, sc, &[], 0.0)
}

/// 在指定适配器上跑一趟**绕射核**（M3），并上 **A9a 真刀刃 Fresnel + A9b 楔角修正**。
///
/// `wedges` 与 [`SceneIn::edges`] **一一对应**（空表 ⇒ 全按刀口）；`band_hz > 0` ⇒ 卡上取
/// `lambda = max(speed,1e-3)/band_hz`，增益 = `coeff·|E_d/E₀|²·fac /(d₁·d₂)`
/// （`fac` = 楔角修正因子，`n = 2` ⇒ `1.0`）；`band_hz ≤ 0` ⇒ 退回旧口径 `(1+cosθ)/2`。
pub fn diffracted_wedge_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
    wedges: &[WedgeIn],
    band_hz: f32,
) -> Result<(String, Vec<DiffractOut>), String> {
    // 三张表逐条对应：核内是 `edge_coeff[e]` / `edge_wedge[e]` 直接索引 ⇒ 给短了就是**越界读**
    // （哑表只兜空表）。
    if sc.edges.len() != sc.edge_coeff.len() {
        return Err(format!(
            "棱表与逐棱系数必须一一对应（{} vs {}）",
            sc.edges.len(),
            sc.edge_coeff.len()
        ));
    }
    if !wedges.is_empty() && wedges.len() != sc.edges.len() {
        return Err(format!(
            "楔角表与棱表必须一一对应（{} vs {}）",
            wedges.len(),
            sc.edges.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if sc.srcs.is_empty() || sc.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let (b, _) = upload(&device, sc, wedges, band_hz);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "diffract") {
        out.push(DiffractOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            edge: row[3],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**散射核**（M4）：逐对取**最强那个面**（平局按三角序号取小 ⇒ 确定）。
///
/// 口径与 CPU 的 `push_scatter` 同式（面中心当二级声源、Lambertian 工程口径、
/// 同侧 + 两腿不挡）：`gain = s·cos_in·cos_out·A/(d₁²·d₂²)`。`scatter` 是**逐三角**系数
/// （单频带；频带表还没上卡），空表 ⇒ 全 0 ⇒ 一对也产不出（纯镜面场景的语义）。
pub fn scattered_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
) -> Result<(String, Vec<ScatterOut>), String> {
    // 同 `reflected_on_adapter`：逐三角表要么空（= 全 0）、要么与三角表等长。
    if !sc.scatter.is_empty() && sc.scatter.len() != sc.tris.len() {
        return Err(format!(
            "散射表与三角表必须一一对应（{} vs {}）",
            sc.scatter.len(),
            sc.tris.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if sc.srcs.is_empty() || sc.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let (b, _) = upload(&device, sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "scatter") {
        out.push(ScatterOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            tri: row[3],
        });
    }
    Ok((name, out))
}

/// **介质积分的入参**（M5）：这条腿不吃几何/材质表/棱，只吃"一批直线段 + 一张水平层表"。
pub struct MediumIn<'a> {
    /// 段起点（与 `dsts` 成对：对序 = `srcs[i] * n_lst + dsts[i]`）。
    pub srcs: &'a [Vec3],
    /// 段终点。
    pub dsts: &'a [Vec3],
    /// 水平层表（按 `y` 升序、不重叠；**空表 = 纯空气**）。
    pub slabs: &'a [Slab],
    /// 每段的**定步数**（`0` 按 1 算）—— 定步数、无早退 ⇒ 逐位可复现。
    pub steps: u32,
    /// 空气声速（层表没覆盖到的采样点用它）。
    pub speed: f32,
}

/// 层表平铺：每条 2 个 `vec4`（`y_lo|y_hi|c|α_低`、`α_中|α_高|0|0`）。
fn slabs_bytes(slabs: &[Slab]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(slabs.len() * 8);
    for s in slabs {
        vals.extend_from_slice(&[s.y_lo, s.y_hi, s.c, s.alpha[0]]);
        vals.extend_from_slice(&[s.alpha[1], s.alpha[2], 0.0, 0.0]);
    }
    f32_bytes(&vals)
}

/// 空层表时的哑表（`n_slabs = 0` ⇒ 核内一次都不读它；只为让绑定非零长）。
const ZERO_SLAB: Slab = Slab {
    y_lo: 0.0,
    y_hi: 0.0,
    c: 0.0,
    alpha: [0.0; 3],
};

/// 介质核的缓冲（与 M1–M4 那套**不共用**：绑定布局不一样 ⇒ 各建自己的）。
struct MediumBuffers {
    pb: wgpu::Buffer,
    sb: wgpu::Buffer,
    db: wgpu::Buffer,
    lb: wgpu::Buffer,
    ob: wgpu::Buffer,
    rb: wgpu::Buffer,
    n_pairs: usize,
}

/// 上传介质核的输入。
fn upload_medium(device: &wgpu::Device, mi: &MediumIn<'_>) -> MediumBuffers {
    let n_pairs = mi.srcs.len() * mi.dsts.len();
    let mk_storage = |label: &str, bytes: &[u8]| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        })
    };
    let out_bytes = (n_pairs * 16) as u64;
    MediumBuffers {
        pb: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("medium.params"),
            contents: &params_bytes(&Params {
                n_src: mi.srcs.len() as u32,
                n_lst: mi.dsts.len() as u32,
                speed: mi.speed,
                n_slabs: mi.slabs.len() as u32,
                medium_steps: mi.steps,
                ..Params::default()
            }),
            usage: wgpu::BufferUsages::UNIFORM,
        }),
        sb: mk_storage("medium.srcs", &vec4_bytes(mi.srcs)),
        db: mk_storage("medium.dsts", &vec4_bytes(mi.dsts)),
        lb: mk_storage(
            "medium.slabs",
            &slabs_bytes(if_empty(mi.slabs, &[ZERO_SLAB])),
        ),
        ob: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("medium.out"),
            size: out_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        }),
        rb: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("medium.rb"),
            size: out_bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        }),
        n_pairs,
    }
}

/// 跑 `medium` 入口并回读（自有绑定布局：`0` 参数 / `1` 起点 / `2` 终点 / `9` 层表 / `4` 输出）。
fn run_medium_entry(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    b: &MediumBuffers,
) -> Vec<[f32; 4]> {
    let out_bytes = (b.n_pairs * 16) as u64;
    let bgl = mk_layout(
        device,
        "medium.bgl",
        &[
            (0, Kind::Uniform),
            (1, Kind::Ro),
            (2, Kind::Ro),
            (9, Kind::Ro),
            (4, Kind::Rw),
        ],
    );
    // ⚠️ 这个变量名必须是 `shader`（词汇门只豁免这一种写法，见 `scripts/vocab_scan.sh` 头部）。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("acoustic.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("acoustic.wgsl").into()),
    });
    let pipe = mk_pipe(device, &bgl, &shader, "medium.pipe", "medium");
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("medium.bg"),
        layout: &bgl,
        entries: &[
            ent(0, &b.pb),
            ent(1, &b.sb),
            ent(2, &b.db),
            ent(9, &b.lb),
            ent(4, &b.ob),
        ],
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("medium.enc"),
    });
    dispatch(&mut enc, &pipe, &bg, (b.n_pairs as u32).div_ceil(64).max(1));
    enc.copy_buffer_to_buffer(&b.ob, 0, &b.rb, 0, out_bytes);
    queue.submit(Some(enc.finish()));
    read_rows(device, &b.rb, b.n_pairs)
}

/// 在指定适配器上跑一趟**介质积分核**（M5）：直线段上按定步数中点采样，返回 `(用时, 逐带衰减)`。
///
/// 折线（折射的折线）不在这里：宿主**逐段调用**后把时延相加、衰减相乘 —— 两者都是可分量 ⇒
/// 与"一次算完整条折线"等价（判据 `tests/acoustic_medium_probe.rs` 钉了这条组合性）。
pub fn medium_on_adapter(
    adapter_index: usize,
    mi: &MediumIn<'_>,
) -> Result<(String, Vec<MediumOut>), String> {
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if mi.srcs.is_empty() || mi.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let b = upload_medium(&device, mi);
    let out = run_medium_entry(&device, &queue, &b)
        .into_iter()
        .map(|r| MediumOut {
            time: r[0],
            atten: [r[1], r[2], r[3]],
        })
        .collect();
    Ok((name, out))
}
