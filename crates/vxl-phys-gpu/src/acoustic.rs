//! **声学核（GPU 档 M1/M2/M3/M4）**：一批 `(声源, 听者)` 对，每对一个独立工作项。
//!
//! 与 CPU 端 `vxl-phys-acoustics` 的对应那几条**同口径**（口径逐条写在 `src/acoustic.wgsl`
//! 的头注里）：M1 直达（遮挡走 Möller–Trumbore，端点共面不计；`dist/speed` 与 `1/dist²`）、
//! M2 一次镜面反射（镜像源法，逐对取最强那条）、M3 绕射（棱上 Fermat 点 +
//! `coeff·(1+cosθ)/2/(d₁·d₂)`，只在阴影里产出）、M4 散射（面当二级声源，Lambertian：
//! `s·cos_in·cos_out·A/(d₁²·d₂²)`，同侧 + 两腿不挡，取最强那个面）。介质积分仍留 CPU 侧 ——
//! 这里要钉的是
//! "同一批输入 ⇒ 与 CPU 参考**逐对一致**"（浮点容差、拓扑逐位），探针见
//! `tests/acoustic_*_probe.rs`。
//!
//! ⚠️ **CI 不跑**：本仓既有口径是 `cargo test --workspace --exclude vxl-phys-gpu`（headless 运行器
//! 无 GPU）。有适配器的机器上按探针跑。

use vxl_phys_core::Vec3;
use wgpu::util::DeviceExt;

use crate::pipeline::{dispatch, ent, f32_bytes, mk_layout, mk_pipe, Kind};

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
    pub edge_coeff: &'a [f32],
    /// 声速（m/s）。
    pub speed: f32,
}

/// `Params`（与 `acoustic.wgsl` 的 `struct Params` 逐字段对齐：32 B）。
fn params_bytes(n_src: u32, n_lst: u32, n_tris: u32, speed: f32, n_edges: u32) -> Vec<u8> {
    let mut b = Vec::with_capacity(32);
    for v in [n_src, n_lst, n_tris] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(&speed.to_le_bytes());
    // ⚠️ uniform 结构体必须补到 16 的倍数（少一个 pad 整块错位）。
    b.extend_from_slice(&n_edges.to_le_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b
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
    ob: wgpu::Buffer,
    rb: wgpu::Buffer,
    n_pairs: usize,
}

/// 上传一批输入（几何/材质表都是平铺切片 ⇒ 直接就是卡的布局）。
fn upload(device: &wgpu::Device, sc: &SceneIn<'_>) -> (Buffers, u64) {
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
        contents: &params_bytes(
            sc.srcs.len() as u32,
            sc.dsts.len() as u32,
            sc.tris.len() as u32,
            sc.speed,
            sc.edges.len() as u32,
        ),
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
        ],
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("acoustic.enc"),
    });
    dispatch(&mut enc, &pipe, &bg, (b.n_pairs as u32).div_ceil(64).max(1));
    enc.copy_buffer_to_buffer(&b.ob, 0, &b.rb, 0, out_bytes);
    queue.submit(Some(enc.finish()));
    let slice = b.rb.slice(..);
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
    let mut rows = Vec::with_capacity(b.n_pairs);
    for k in 0..b.n_pairs {
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
    b.rb.unmap();
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
    let (b, _) = upload(&device, &sc);
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
    let (b, _) = upload(&device, &sc);
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
/// 输入口径见 [`SceneIn`]：`edges` 是棱的两个端点、`edge_coeff` 是**单频带**绕射系数
/// （频带表还没上卡）；介质衰减也不在内（M5 未做）⇒ 增益 = `coeff·D(θ)/(d₁·d₂)`、
/// 时延 = `(d₁+d₂)/speed`。
pub fn diffracted_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
) -> Result<(String, Vec<DiffractOut>), String> {
    // 两张表逐条对应：核内是 `edge_coeff[e]` 直接索引 ⇒ 给短了就是**越界读**（哑表只兜空表）。
    if sc.edges.len() != sc.edge_coeff.len() {
        return Err(format!(
            "棱表与逐棱系数必须一一对应（{} vs {}）",
            sc.edges.len(),
            sc.edge_coeff.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if sc.srcs.is_empty() || sc.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let (b, _) = upload(&device, sc);
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
    let (b, _) = upload(&device, sc);
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
