//! **包围盒归约的设备侧建置与解码**：`bbox.rs` 的子模块 —— 管线/绑定组**建一次**的那两段
//! （探针 `make_probe` 与常驻阶段 `BboxStage::new` 共用），外加"有序位模式 ↔ 角点"的解码工具。
//!
//! **为什么拆出来**：`bbox.rs` 受本仓 god 门**文件行数硬阈**（单文件 ≤ 800 行）管，而这一片
//! 是"`wgpu` 资源怎么建"的**设备关注点**，与留在 `bbox.rs` 的对外 API（[`super::BoxOut`] /
//! [`super::host_box`] / [`super::ordered_to_f32`] / [`super::BboxStage`]）职责不同 —— 前者只跟
//! `Device`/`wgpu::Buffer` 打交道，后者才是宿主看得见的读数与参考实现。
//!
//! **纯搬移**：算式、注释、顺序一字未改（唯一例外是 `include_str!` 的路径：它按**本文件**所在
//! 目录解析，搬到子目录后须写成 `"../bbox.wgsl"`）；可见性只从私有提到 `pub(super)`
//! （**不是**裸 `pub`，对外 API 面一行没变）。同 `pipeline/construct.rs`、`acoustic/reflect2.rs` 的先例。

use super::ordered_to_f32;
use crate::pipeline::ent;
use wgpu::util::DeviceExt;

/// 归约的一次性建置产物（绑定组 + 管线；`box_on_adapter` 只读引用）。
pub(super) struct BboxPipes {
    pub(super) bg: wgpu::BindGroup,
    pub(super) reduce: wgpu::ComputePipeline,
    pub(super) setup: wgpu::ComputePipeline,
}

/// 一次性建置：着色器 / 绑定布局 / 绑定组 / 两条管线（不进出稳态计时）。
pub(super) fn make_bbox_pipes(
    device: &wgpu::Device,
    pos_b: &wgpu::Buffer,
    bb_b: &wgpu::Buffer,
    rp_b: &wgpu::Buffer,
    box_out_b: &wgpu::Buffer,
    sp_b: &wgpu::Buffer,
) -> BboxPipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("bbox.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("../bbox.wgsl").into()),
    });
    // 5 个槽位：0 pos(read) | 1 bb(rw，原子) | 2 rp(uniform) | 3 box_out(rw) | 4 sp(uniform)
    let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..5u32)
        .map(|binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if binding == 2 || binding == 4 {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: binding == 0,
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("bbox.bgl"),
        entries: &entries,
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("bbox.bg"),
        layout: &bgl,
        entries: &[
            ent(0, pos_b),
            ent(1, bb_b),
            ent(2, rp_b),
            ent(3, box_out_b),
            ent(4, sp_b),
        ],
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("bbox.pl"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });
    let p_reduce = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("bbox.reduce"),
        layout: Some(&pl),
        module: &shader,
        entry_point: Some("reduce"),
        compilation_options: Default::default(),
        cache: None,
    });
    let p_setup = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("bbox.setup"),
        layout: Some(&pl),
        module: &shader,
        entry_point: Some("box_setup"),
        compilation_options: Default::default(),
        cache: None,
    });

    BboxPipes {
        bg,
        reduce: p_reduce,
        setup: p_setup,
    }
}

pub(super) fn f32_bytes(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::with_capacity(v.len() * 4);
    for x in v {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// `reduce` 的工作组大小（与 `bbox.wgsl` 的 `WG` 一致）。
pub(super) const WG: u32 = 64;
/// `bb` 的中性元：`[+∞, +∞, +∞, −∞, −∞, −∞]` **在有序映射下**的位模式
/// （⚠️ `+∞` 映射后是 `0xFF800000`：有序映射把正数的符号位翻成 1）。
pub(super) const BB_INIT: [u32; 6] = [
    0xFF80_0000,
    0xFF80_0000,
    0xFF80_0000,
    0x007F_FFFF,
    0x007F_FFFF,
    0x007F_FFFF,
];

/// `bb` 的中性元字节（`BB_INIT` 的 LE 展开）。
pub(super) fn bb_init_bytes() -> Vec<u8> {
    let mut b = Vec::with_capacity(24);
    for x in BB_INIT {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// 探针的一次性建置（`box_on_adapter` 的第一段）：`pos`/`bb`/`box_out`/`sp`/`rp` 与回读缓冲 + 管线。
/// 返回 `(bb, box_out, readback, pipes)`——`pos` 与两个 uniform 只被 bind group 持有。
pub(super) fn make_probe(
    device: &wgpu::Device,
    pos_flat: &[f32],
    n: u32,
    ngroups: u32,
    h: f32,
    max_bins: usize,
) -> (wgpu::Buffer, wgpu::Buffer, wgpu::Buffer, BboxPipes) {
    let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.pos"),
        contents: &f32_bytes(pos_flat),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let bb_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.bb"),
        contents: &bb_init_bytes(),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
    });
    let rp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.rp"),
        contents: &{
            let mut b = Vec::with_capacity(16);
            for x in [n, ngroups, 0, 0] {
                b.extend_from_slice(&x.to_le_bytes());
            }
            b
        },
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let box_out_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bbox.box_out"),
        size: 48, // 8 槽（uniform 形）+ 3 槽紧凑 `hi`（近域过滤用的真 AABB，见 `bbox.wgsl` 头注）
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let sp_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("bbox.sp"),
        contents: &f32_bytes(&[h, max_bins as f32, 0.0, 0.0]),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bbox.readback"),
        size: 72, // bb（24 B）+ box_out（48 B）
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let pipes = make_bbox_pipes(device, &pos_b, &bb_b, &rp_b, &box_out_b, &sp_b);
    (bb_b, box_out_b, readback, pipes)
}

/// `bb` 的有序 u32 → 角点 `(min, max)`。
pub(super) fn bb_to_corners(bb: [u32; 6]) -> ([f32; 3], [f32; 3]) {
    (
        [
            ordered_to_f32(bb[0]),
            ordered_to_f32(bb[1]),
            ordered_to_f32(bb[2]),
        ],
        [
            ordered_to_f32(bb[3]),
            ordered_to_f32(bb[4]),
            ordered_to_f32(bb[5]),
        ],
    )
}

/// 回读字节 → `bb`（6 个有序 u32）。
pub(super) fn decode_bb(data: &[u8]) -> [u32; 6] {
    let mut bb = [0u32; 6];
    for (k, slot) in bb.iter_mut().enumerate() {
        let o = k * 4;
        *slot = u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
    }
    bb
}
