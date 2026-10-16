//! **声学核（M1–M5）的"卡资源怎么建 / 字节怎么排"**：`acoustic.rs` 的子模块 —— 参数镜像与
//! 五张表的字节化、场景/介质两套缓冲的建置与上传、两个入口的绑定组与派发。
//!
//! **为什么拆出来**：`acoustic.rs` 受本仓 god 门**文件行数硬阈**（单文件 ≤ 800 行）管，而这一片
//! 是"`wgpu` 资源怎么建"的**设备关注点**，与留在 `acoustic.rs` 的对外 API
//! （[`super::direct_paths_on_adapter`] / [`super::reflected_on_adapter`] /
//! [`super::diffracted_wedge_on_adapter`] / [`super::scattered_on_adapter`] /
//! [`super::medium_on_adapter`] 与 [`super::SceneIn`] / [`super::MediumIn`]）职责不同 ——
//! 前者只跟 `Device`/`wgpu::Buffer` 打交道，后者才是宿主看得见的读数与入参。
//!
//! **纯搬移**：算式、注释文字、顺序一字未改；可见性只从私有提到 `pub(super)`
//! （**不是**裸 `pub`，`acoustic` 的对外 API 面一行没变）。`include_str!("../acoustic.wgsl")` 按**本文件**
//! 所在目录解析 ⇒ 搬到子目录后写成 `"../acoustic.wgsl"`（纯搬移里唯一改动的一行，已在
//! `run_entry` / `run_medium_entry` 两处登记）。同 `bbox/pipes.rs`、`pipeline/construct.rs`、
//! `acoustic/reflect2.rs` 的先例。

use super::{MediumIn, SceneIn, Slab, WedgeIn};
use crate::pipeline::{dispatch, ent, f32_bytes, mk_layout, mk_pipe, Kind};
use vxl_phys_core::Vec3;
use wgpu::util::DeviceExt;

/// `Params` 的宿主侧镜像（**字段顺序与 `acoustic.wgsl` 的 `struct Params` 一一对应**：32 B）。
///
/// 打成结构体不是为了好看：`n_slabs` / `medium_steps` 上来之后形参到 7 个，撞本仓 `args-gate`
/// 的软阈值 ⇒ 收口成一份"参数镜像"（顺便把布局写在一处，改字段只改这里）。
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Params {
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
pub(super) fn params_bytes(p: &Params) -> Vec<u8> {
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
/// 平铺 `vec4`：每个 `Vec3` 占 4 个 f32（第 4 个补 0）⇒ 与 WGSL 的 `array<vec4<f32>>` 对齐。
pub(super) fn vec4_bytes(v: &[Vec3]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(v.len() * 4);
    for p in v {
        vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
    }
    f32_bytes(&vals)
}
/// 三角平铺：每个三角 3 个 `vec4`（同 `vec4_bytes` 的补零口径）。
pub(super) fn tris_bytes(tris: &[[Vec3; 3]]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(tris.len() * 12);
    for t in tris {
        for p in t {
            vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
        }
    }
    f32_bytes(&vals)
}
/// 棱平铺：每条 2 个 `vec4`（`a`、`b`；同 `vec4_bytes` 的补零口径）。
pub(super) fn edges_bytes(edges: &[[Vec3; 2]]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(edges.len() * 8);
    for e in edges {
        for p in e {
            vals.extend_from_slice(&[p.x, p.y, p.z, 0.0]);
        }
    }
    f32_bytes(&vals)
}
/// 楔角平铺：每条 1 个 `vec4`（`Ω|face0.x|face0.y|face0.z`）。
pub(super) fn wedge_bytes(wedges: &[WedgeIn]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(wedges.len() * 4);
    for w in wedges {
        vals.extend_from_slice(&[w.interior_angle, w.face0.x, w.face0.y, w.face0.z]);
    }
    f32_bytes(&vals)
}
/// 空楔角表时的哑表（`Ω = 0` ⇒ 核内按刀口算；只为让空表也有一张非零长的缓冲）。
pub(super) const ZERO_WEDGE: WedgeIn = WedgeIn {
    interior_angle: 0.0,
    face0: Vec3::ZERO,
};
/// 场景缓冲（两个入口共用：只有输出不同）。
pub(super) struct Buffers {
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
    pub(super) n_pairs: usize,
}
/// 上传一批输入（几何/材质表都是平铺切片 ⇒ 直接就是卡的布局）。
///
/// `wedges` 与 `sc.edges` 一一对应（空表 ⇒ 全按刀口）；`band_hz ≤ 0` ⇒ `lambda = 0`
/// ⇒ 绕射走**旧口径**（逐位不变）。
pub(super) fn upload(
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
pub(super) const ZERO_EDGE: [Vec3; 2] = [Vec3::ZERO; 2];

/// 空切片 ⇒ 换成哑表（零长 storage 绑定不合法）。
pub(super) fn if_empty<'a, T: Copy>(v: &'a [T], dummy: &'a [T]) -> &'a [T] {
    if v.is_empty() {
        dummy
    } else {
        v
    }
}
/// 跑指定入口并回读：每对 4 个 f32（`[kind, delay, gain, extra]`）。
pub(super) fn run_entry(
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
        source: wgpu::ShaderSource::Wgsl(include_str!("../acoustic.wgsl").into()),
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
pub(super) fn read_rows(device: &wgpu::Device, rb: &wgpu::Buffer, n: usize) -> Vec<[f32; 4]> {
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
/// 层表平铺：每条 2 个 `vec4`（`y_lo|y_hi|c|α_低`、`α_中|α_高|0|0`）。
pub(super) fn slabs_bytes(slabs: &[Slab]) -> Vec<u8> {
    let mut vals = Vec::with_capacity(slabs.len() * 8);
    for s in slabs {
        vals.extend_from_slice(&[s.y_lo, s.y_hi, s.c, s.alpha[0]]);
        vals.extend_from_slice(&[s.alpha[1], s.alpha[2], 0.0, 0.0]);
    }
    f32_bytes(&vals)
}
/// 空层表时的哑表（`n_slabs = 0` ⇒ 核内一次都不读它；只为让绑定非零长）。
pub(super) const ZERO_SLAB: Slab = Slab {
    y_lo: 0.0,
    y_hi: 0.0,
    c: 0.0,
    alpha: [0.0; 3],
};
/// 介质核的缓冲（与 M1–M4 那套**不共用**：绑定布局不一样 ⇒ 各建自己的）。
pub(super) struct MediumBuffers {
    pb: wgpu::Buffer,
    sb: wgpu::Buffer,
    db: wgpu::Buffer,
    lb: wgpu::Buffer,
    ob: wgpu::Buffer,
    rb: wgpu::Buffer,
    pub(super) n_pairs: usize,
}
/// 上传介质核的输入。
pub(super) fn upload_medium(device: &wgpu::Device, mi: &MediumIn<'_>) -> MediumBuffers {
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
pub(super) fn run_medium_entry(
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
        source: wgpu::ShaderSource::Wgsl(include_str!("../acoustic.wgsl").into()),
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
