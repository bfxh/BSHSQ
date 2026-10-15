//! **建包的四段**（`Packet::build` 的骨料）：缓冲清单 / 四组 uniform / 绑定布局 + 计算管线 / 绑定组。
//! 2026-10-16 从 `pipeline.rs` 拆出——那个文件在同一片里已放过三张表，god 门是棘轮、只能"搬进新文件"
//! 才净缩得下来（新文件只判硬阈）。
//!
//! 纯搬移：四段之间只经**持有结构体**（[`Bufs`]/[`Params`]/[`Pipes`]）交接，`build` 按序借用、不落字段；
//! `Binds` 的构造（`make_bind_groups`）早已在 `two_class.rs`，这里只留结构体本身。

use super::{bind_layout, two_class};
use super::{f32_bytes, mk_layout, mk_pipe, Kind, PacketCfg};
use wgpu::util::DeviceExt;

/// `Packet` 的**缓冲清单**（`new` 的第一段：建 + 上传初值）。
pub(crate) struct Bufs {
    pub pos_b: wgpu::Buffer,
    pub vel_b: wgpu::Buffer,
    pub pmass_b: wgpu::Buffer,
    pub press_b: wgpu::Buffer,
    pub dens_b: wgpu::Buffer,
    pub out_b: wgpu::Buffer,
    pub bins_b: wgpu::Buffer,
    pub counts_b: wgpu::Buffer,
    pub start_b: wgpu::Buffer,
    pub items_b: wgpu::Buffer,
    pub cursor_b: wgpu::Buffer,
    pub overflow_b: wgpu::Buffer,
    pub readback_b: wgpu::Buffer,
}

pub(crate) fn make_buffers(
    device: &wgpu::Device,
    n: u32,
    cap_total: u32,
    pos_flat: &[f32],
    vel_flat: &[f32],
    pmass: &[f32],
) -> Bufs {
    let storage = |label: &str, size: u64, extra: wgpu::BufferUsages| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST
                | extra,
            mapped_at_creation: false,
        })
    };
    let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("p.pos"),
        contents: &f32_bytes(pos_flat),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
    });
    let vel_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("p.vel"),
        contents: &f32_bytes(vel_flat),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
    });
    let pmass_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("p.pmass"),
        contents: &f32_bytes(pmass),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let press_b = storage("p.press", (n as u64) * 4, wgpu::BufferUsages::empty());
    let dens_b = storage("p.dens", (n as u64) * 4, wgpu::BufferUsages::empty());
    let out_b = storage("p.out", (n as u64) * 24, wgpu::BufferUsages::empty());
    let bins_b = storage("p.bins", (n as u64) * 4, wgpu::BufferUsages::empty());
    // ⚠️ 这三张**按格数**的缓冲是一次性分配的（与 CPU 侧 `Vec` 会自动增长不同）：
    // `cap_total` = 分配额度，`refresh_box` 按它夹"总格数预算"⇒ 箱子跟随时不会越界写。
    // **两类档**：每张 = `2 × stride`，`stride = align8(total+1)`（与 `grid.wgsl::table_stride`
    // 同式 —— 类 1 表以**切片**绑给相位核，偏移须满足 `min_storage_buffer_offset_alignment`）。
    let tbl = ((cap_total + 1).div_ceil(8) * 8) as u64 * 4;
    let counts_b = storage("p.counts", 2 * tbl, wgpu::BufferUsages::empty());
    // ⚠️ `start_b` **不能**用上面的 `storage`（`mapped_at_creation: false`）：类 1 表区
    // （`[tbl, 2tbl)`）在纯流体档**永不写入**，而相位核经 `cell_start[tstride + ci]` 读它（§28.2）
    // ⇒ "从未写入"区域的读数是驱动相关的 ⇒ 必须是**确定性的零**（`mapped_at_creation` 规格保证
    // 全零映射，立即 unmap 即可；2b 档每子步由第二遍网格重写该区）。
    let start_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("p.start"),
        size: 2 * tbl,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: true,
    });
    start_b.unmap();
    let items_b = storage("p.items", (n as u64) * 4, wgpu::BufferUsages::empty());
    let cursor_b = storage("p.cursor", 2 * tbl, wgpu::BufferUsages::empty());
    let overflow_b = storage("p.overflow", 4, wgpu::BufferUsages::empty());
    let readback_b = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("p.readback"),
        size: (n as u64) * 24,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    Bufs {
        pos_b,
        vel_b,
        pmass_b,
        press_b,
        dens_b,
        out_b,
        bins_b,
        counts_b,
        start_b,
        items_b,
        cursor_b,
        overflow_b,
        readback_b,
    }
}

/// `Packet` 的四组 uniform（`new` 的第二段：字节布局 + 建缓冲）。
pub(crate) struct Params {
    pub grid_params_b: wgpu::Buffer,
    /// **类 1（边界）的网格 uniform**（两遍分派用；`n_fluid == n` 时不跑）。
    pub grid2_params_b: wgpu::Buffer,
    pub phase_params_b: wgpu::Buffer,
    pub eos_params_b: wgpu::Buffer,
    pub int_params_b: wgpu::Buffer,
}

pub(crate) fn make_params(device: &wgpu::Device, cfg: &PacketCfg, n: u32, total: u32) -> Params {
    // —— 四组 uniform ——
    // 网格 uniform 两套（类 0 / 类 1）：**两遍都用 `n_fluid = cfg.n_fluid`**、只差 `class_lo`。
    let grid_params_b = two_class::grid_params_buf(device, cfg, n, total, 0);
    let grid2_params_b = two_class::grid_params_buf(device, cfg, n, total, cfg.n_fluid);
    let phase_params = {
        let mut b = Vec::with_capacity(80);
        for x in cfg.gmin {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in [
            cfg.inv,
            cfg.h2,
            cfg.k6,
            cfg.w0,
            cfg.mass,
            cfg.ks,
            cfg.h,
            cfg.alpha_c,
            0.0,
        ] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        for x in cfg.gravity {
            b.extend_from_slice(&x.to_le_bytes());
        }
        // 末段首字段是相位核读的 **`n_fluid`**（不是总粒子数）：`density.wgsl`/`force.wgsl` 用它
        // 区分"流体邻居 ⇒ 计入 sum"与"边界邻居 ⇒ 计入 sum_b"（Akinci 口径，与 CPU 同式）。
        // ⚠️ 首版这里写的是 `n`（全量）⇒ 边界粒子上卡后全被当流体 ⇒ 一 tick 就炸（动能比 289）。
        for x in [cfg.n_fluid, cfg.dims[0], cfg.dims[1], cfg.dims[2], n] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let eos_params = {
        let mut b = Vec::with_capacity(32);
        for x in [cfg.b_tait, cfg.rho0, cfg.gamma] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b.extend_from_slice(&u32::from(cfg.clamp_neg).to_le_bytes());
        for x in [n, 0, 0, 0] {
            b.extend_from_slice(&x.to_le_bytes());
        }
        b
    };
    let uniform = |label: &str, bytes: &[u8]| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        })
    };
    let phase_params_b = uniform("p.phase_params", &phase_params);
    let eos_params_b = uniform("p.eos_params", &eos_params);
    let int_params_b = uniform(
        "p.int_params",
        &[0u8; 32], // 每子步 `write_buffer` 更新
    );

    Params {
        grid_params_b,
        grid2_params_b,
        phase_params_b,
        eos_params_b,
        int_params_b,
    }
}

/// `Packet` 的绑定布局 + 计算管线（`new` 的第三段）。
pub(crate) struct Pipes {
    pub bl_grid: wgpu::BindGroupLayout,
    pub bl_dens: wgpu::BindGroupLayout,
    pub bl_force: wgpu::BindGroupLayout,
    pub bl_eos: wgpu::BindGroupLayout,
    pub bl_int: wgpu::BindGroupLayout,
    pub p_bin: wgpu::ComputePipeline,
    pub p_scan: wgpu::ComputePipeline,
    pub p_place: wgpu::ComputePipeline,
    pub p_canon: wgpu::ComputePipeline,
    pub p_dens: wgpu::ComputePipeline,
    pub p_eos: wgpu::ComputePipeline,
    pub p_force: wgpu::ComputePipeline,
    pub p_int: wgpu::ComputePipeline,
}

/// 建一个着色器模块（`make_pipelines` 里 5 次同一形状；提出去是为了让那个函数**变短**——
/// god 门是棘轮：文件长一行的代价是"最长函数必须更短"）。
fn sh_module(device: &wgpu::Device, label: &str, src: &str) -> wgpu::ShaderModule {
    device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(label),
        source: wgpu::ShaderSource::Wgsl(src.into()),
    })
}

pub(crate) fn make_pipelines(device: &wgpu::Device) -> Pipes {
    // —— 五个着色器模块 + 八条管线 ——
    let sh = |label: &str, src: &str| sh_module(device, label, src);
    let m_grid = sh("grid.wgsl", include_str!("../grid.wgsl"));
    let m_dens = sh("density.wgsl", include_str!("../density.wgsl"));
    let m_force = sh("force.wgsl", include_str!("../force.wgsl"));
    let m_eos = sh("eos.wgsl", include_str!("../eos.wgsl"));
    let m_int = sh("integrate.wgsl", include_str!("../integrate.wgsl"));

    // 网格：0 uniform + 1(pos,只读) + 2..=7(读写)
    let bl_grid = mk_layout(device, "p.bl_grid", &bind_layout::spec("urwwwwww"));
    // 密度：0 uniform | 1 pos(只读) | 3 pmass(只读) | 5/6 格表(只读) | 7 dens(读写)
    //（类 1 表不单独绑定：与网格同一张 `start_b`，基址在核里按运行时 dims 现算，§28.2）
    let bl_dens = mk_layout(device, "p.bl_dens", &bind_layout::spec("ur_r_rrw"));
    // 力：0 uniform | 1..=7 只读（含 dens）| 8 out(读写)
    let bl_force = mk_layout(
        device,
        "p.bl_force",
        &[
            (0, Kind::Uniform),
            (1, Kind::Ro),
            (2, Kind::Ro),
            (3, Kind::Ro),
            (4, Kind::Ro),
            (5, Kind::Ro),
            (6, Kind::Ro),
            (7, Kind::Ro),
            (8, Kind::Rw),
        ],
    );
    let bl_eos = mk_layout(
        device,
        "p.bl_eos",
        &[(0, Kind::Uniform), (1, Kind::Ro), (2, Kind::Rw)],
    );
    let bl_int = mk_layout(
        device,
        "p.bl_int",
        &[
            (0, Kind::Uniform),
            (1, Kind::Rw),
            (2, Kind::Rw),
            (3, Kind::Ro),
        ],
    );

    let p_bin = mk_pipe(device, &bl_grid, &m_grid, "p.bin_count", "bin_count");
    let p_scan = mk_pipe(device, &bl_grid, &m_grid, "p.scan", "scan");
    let p_place = mk_pipe(device, &bl_grid, &m_grid, "p.place", "place");
    let p_canon = mk_pipe(device, &bl_grid, &m_grid, "p.canon", "canon");
    let p_dens = mk_pipe(device, &bl_dens, &m_dens, "p.density", "density");
    let p_force = mk_pipe(device, &bl_force, &m_force, "p.force", "force");
    let p_eos = mk_pipe(device, &bl_eos, &m_eos, "p.eos", "eos");
    let p_int = mk_pipe(device, &bl_int, &m_int, "p.integrate", "integrate");

    Pipes {
        bl_grid,
        bl_dens,
        bl_force,
        bl_eos,
        bl_int,
        p_bin,
        p_scan,
        p_place,
        p_canon,
        p_dens,
        p_eos,
        p_force,
        p_int,
    }
}

/// `Packet` 的五张 bind group（`new` 的第四段：把缓冲/参数/管线绑起来）。
pub(crate) struct Binds {
    pub bg_grid: [wgpu::BindGroup; 2], // [0]=类0 [1]=类1
    pub bg_dens: wgpu::BindGroup,
    pub bg_force: wgpu::BindGroup,
    pub bg_eos: wgpu::BindGroup,
    pub bg_int: wgpu::BindGroup,
}

// `make_bind_groups` 在 `pipeline/two_class.rs`（棘轮：`pipeline.rs` 857 行超阈 ⇒ 必须净缩）。
// 那里同时给出**类 1 格表的切片绑定**（相位核的槽 9）。
