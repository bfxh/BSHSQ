//! **声学「直达路径」核（GPU 档 M1）**：一批 `(声源, 听者)` 对，每对一个独立工作项。
//!
//! 与 CPU 端 `vxl-phys-acoustics` 的直达那条**同口径**：遮挡走 Möller–Trumbore（端点共面不计）、
//! 时延 = `dist / speed`、增益 = `1 / dist²`。反射/绕射/散射/介质积分仍留 CPU 侧 —— 这里要钉的是
//! "同一批输入 ⇒ 与 CPU 参考**逐对一致**"（浮点容差、拓扑逐位），探针见
//! `tests/acoustic_direct_probe.rs`。
//!
//! ⚠️ **CI 不跑**：本仓既有口径是 `cargo test --workspace --exclude vxl-phys-gpu`（headless 运行器
//! 无 GPU）。有适配器的机器上按探针跑；这里给出的是**第一个可对拍的设备内核**。

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

/// `Params`（与 `acoustic.wgsl` 的 `struct Params` 逐字段对齐：16 B）。
fn params_bytes(n_src: u32, n_lst: u32, n_tris: u32, speed: f32) -> Vec<u8> {
    let mut b = Vec::with_capacity(16);
    for v in [n_src, n_lst, n_tris] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(&speed.to_le_bytes());
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

/// 在指定适配器上跑一趟直达核：返回 `(适配器名, 逐对读数)`。
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
    let n_pairs = srcs.len() * dsts.len();
    if n_pairs == 0 {
        return Ok((name, Vec::new()));
    }
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
            srcs.len() as u32,
            dsts.len() as u32,
            tris.len() as u32,
            speed,
        ),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let sb = mk_storage("acoustic.srcs", &vec4_bytes(srcs));
    let db = mk_storage("acoustic.dsts", &vec4_bytes(dsts));
    let tb = mk_storage("acoustic.tris", &tris_bytes(tris));
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
    let bgl = mk_layout(
        &device,
        "acoustic.bgl",
        &[
            (0, Kind::Uniform),
            (1, Kind::Ro),
            (2, Kind::Ro),
            (3, Kind::Ro),
            (4, Kind::Rw),
        ],
    );
    // ⚠️ 这个变量必须叫 `shader`（词汇门只豁免这一种写法，见 `scripts/vocab_scan.sh` 头部）。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("acoustic.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("acoustic.wgsl").into()),
    });
    let pipe = mk_pipe(&device, &bgl, &shader, "acoustic.direct", "direct");
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("acoustic.bg"),
        layout: &bgl,
        entries: &[
            ent(0, &pb),
            ent(1, &sb),
            ent(2, &db),
            ent(3, &tb),
            ent(4, &ob),
        ],
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("acoustic.enc"),
    });
    dispatch(&mut enc, &pipe, &bg, (n_pairs as u32).div_ceil(64).max(1));
    enc.copy_buffer_to_buffer(&ob, 0, &rb, 0, out_bytes);
    queue.submit(Some(enc.finish()));
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
    let mut out = Vec::with_capacity(n_pairs);
    for k in 0..n_pairs {
        let o = k * 16;
        let f = |j: usize| {
            f32::from_le_bytes([
                data[o + j * 4],
                data[o + j * 4 + 1],
                data[o + j * 4 + 2],
                data[o + j * 4 + 3],
            ])
        };
        out.push(PairOut {
            blocked: f(3) > 0.5,
            delay: f(1),
            gain: f(2),
        });
    }
    drop(data);
    rb.unmap();
    Ok((name, out))
}
