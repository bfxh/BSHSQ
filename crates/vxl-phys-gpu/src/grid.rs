//! **GPU 网格重建探针**（把 CPU 的计数排序原样搬到卡上）：判据 = **与 CPU 逐位同表**
//! （`start`/`items` 全等）。纪律同 `probe.rs`：口径 A 在网格上可达（只有整数运算 +
//! 一次 reduce ⇒ 不依赖 FMA 收缩）；不绑厂商；提交异步 ⇒ 计时须含同步回读。

use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GridParams {
    pub gmin: [f32; 3],
    pub inv: f32,
    pub nx: u32,
    pub ny: u32,
    pub nz: u32,
    pub n: u32,
    pub total: u32,
    /// 逐格规范化（插入排序）的**段长上限**：超过则不规范化并记 `overflow`（最坏情况护栏）。
    pub cap: u32,
    /// 流体粒子个数（`[0, n_fluid)` = 类 0；**纯流体档 = `n`**）。
    pub n_fluid: u32,
    /// 本遍处理哪一类：`0` = 类 0（流体）、非 `0` = 类 1（边界）。
    pub class_lo: u32,
}

/// 输入：位置（扁平 xyz）+ 箱参数（须与 CPU `rebuild` 那组**完全相同**才谈同表）。
pub struct GridInputs<'a> {
    pub pos_flat: &'a [f32],
}

/// GPU 网格表 + 读数。
pub struct GridOut {
    /// 每格起点（长度 `total + 1`，与 CPU `start` 同义；**类 0** = 纯流体档的全量表）。
    pub start: Vec<u32>,
    /// **类 1（边界）**的每格起点（长度 `total + 1`；单类档 / 类 1 为空时为空 vec）。
    pub start2: Vec<u32>,
    /// 按格分组的粒子索引（长度 `n`；格内 = **粒子索引升序**）。
    pub items: Vec<u32>,
    /// 逐粒的格线性下标（诊断用：CPU 侧可从 `start/items` 反推同一张表）。
    pub bins: Vec<u32>,
    /// 未规范化的格数（**判据是 0**；非 0 ⇒ 表不再与 CPU 同表）。
    pub overflow: u32,
    pub setup_ms: f32,
    pub per_run_ms: f32,
    pub adapter: String,
    pub error: Option<String>,
}

impl GridOut {
    fn err(msg: String) -> Self {
        Self {
            start: Vec::new(),
            start2: Vec::new(),
            items: Vec::new(),
            bins: Vec::new(),
            overflow: 0,
            setup_ms: 0.0,
            per_run_ms: 0.0,
            adapter: String::new(),
            error: Some(msg),
        }
    }
}

/// 网格探针的缓冲 + 回读偏移。
pub(crate) struct GridBuffs {
    pub pos_b: wgpu::Buffer,
    pub params_b: wgpu::Buffer,
    pub bins_b: wgpu::Buffer,
    pub counts_b: wgpu::Buffer,
    pub start_b: wgpu::Buffer,
    pub items_b: wgpu::Buffer,
    pub cursor_b: wgpu::Buffer,
    pub overflow_b: wgpu::Buffer,
    pub readback: wgpu::Buffer,
    pub start_off: u64,
    /// **类 1 表**的回读偏移（`start_off` 之后紧邻）。
    pub start2_off: u64,
    pub items_off: u64,
    pub bins_off: u64,
    pub of_off: u64,
}

pub(crate) fn make_grid_buffs(
    device: &wgpu::Device,
    n: usize,
    total: usize,
    params: GridParams,
    inputs: &GridInputs<'_>,
) -> GridBuffs {
    let params_bytes = params_bytes(&params);
    let mut pos_bytes = Vec::with_capacity(inputs.pos_flat.len() * 4);
    for x in inputs.pos_flat {
        pos_bytes.extend_from_slice(&x.to_le_bytes());
    }

    let pos_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("grid.pos"),
        contents: &pos_bytes,
        usage: wgpu::BufferUsages::STORAGE,
    });
    let params_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("grid.params"),
        contents: &params_bytes,
        // `COPY_DST`：两类档要用 `queue.write_buffer` 换 `class_lo` 再跑第二遍。
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });
    // 需要 `clear_buffer` / `copy_buffer_to_buffer` ⇒ 一律带 COPY_DST；结果要回读 ⇒ 带 COPY_SRC。
    let mk = |label: &str, size: u64| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    // **两类表**（§26.1）：`counts`/`start`/`cursor` 各 `2·(total+1)`（类 0 在前、类 1 在后）
    // ⇒ 纯流体档只用前一半（逐位不变），2b 档两半都用。
    // 表步长 = align8(total+1)（与 `grid.wgsl::table_stride` 同式；切片绑定须 32 B 对齐）。
    let tbl = ((total + 1).div_ceil(8) * 8 * 4) as u64;
    let bins_b = mk("grid.bins", (n * 4) as u64);
    let counts_b = mk("grid.counts", 2 * tbl);
    let start_b = mk("grid.start", 2 * tbl);
    let items_b = mk("grid.items", (n * 4) as u64);
    let cursor_b = mk("grid.cursor", 2 * tbl);
    let overflow_b = mk("grid.overflow", 4);
    // 回读：start(两张表) | items | bins | overflow（每轮尾同步一次，只为校验）
    let start_off = 0u64;
    let start2_off = start_off + tbl;
    let items_off = start2_off + tbl;
    let bins_off = items_off + (n * 4) as u64;
    let of_off = bins_off + (n * 4) as u64;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("grid.readback"),
        size: of_off + 4,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    GridBuffs {
        pos_b,
        params_b,
        bins_b,
        counts_b,
        start_b,
        items_b,
        cursor_b,
        overflow_b,
        readback,
        start_off,
        start2_off,
        items_off,
        bins_off,
        of_off,
    }
}

/// `GridParams` → 48 字节 uniform（唯一的序列化点）。
pub(crate) fn params_bytes(params: &GridParams) -> Vec<u8> {
    let mut b = Vec::with_capacity(48);
    for x in params.gmin {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b.extend_from_slice(&params.inv.to_le_bytes());
    for x in [
        params.nx,
        params.ny,
        params.nz,
        params.n,
        params.total,
        params.cap,
        params.n_fluid,
        params.class_lo,
    ] {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

/// 网格四相位管线 + 绑定组（一次性 setup 的产物；`submit_grid_pass` 里只读引用）。
pub(crate) struct GridPipes {
    bin: wgpu::ComputePipeline,
    scan: wgpu::ComputePipeline,
    place: wgpu::ComputePipeline,
    canon: wgpu::ComputePipeline,
    bg: wgpu::BindGroup,
    /// **类 1 的绑定组**（唯一的差别是 binding 0 指向第二份 params ⇒ 两遍各自一份
    /// uniform，**不走"中途 `write_buffer`"** —— 实测那条路在本题下没生效）。
    bg2: wgpu::BindGroup,
}

pub(crate) fn make_grid_pipes(
    device: &wgpu::Device,
    bufs: &GridBuffs,
    params2_b: &wgpu::Buffer,
) -> GridPipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("grid.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("grid.wgsl").into()),
    });
    // 一份布局覆盖四个入口（除 `pos` 外全部 read_write；见 `grid.wgsl` 头注）
    let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..8u32)
        .map(|binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if binding == 0 {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: binding == 1,
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("grid.bgl"),
        entries: &entries,
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("grid.pl"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });
    let mk_pipe = |label: &str, entry: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(label),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let p_bin = mk_pipe("grid.bin_count", "bin_count");
    let p_scan = mk_pipe("grid.scan", "scan");
    let p_place = mk_pipe("grid.place", "place");
    let p_canon = mk_pipe("grid.canon", "canon");
    fn ent(binding: u32, b: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
        wgpu::BindGroupEntry {
            binding,
            resource: b.as_entire_binding(),
        }
    }
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("grid.bg"),
        layout: &bgl,
        entries: &[
            ent(0, &bufs.params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.bins_b),
            ent(3, &bufs.counts_b),
            ent(4, &bufs.start_b),
            ent(5, &bufs.items_b),
            ent(6, &bufs.cursor_b),
            ent(7, &bufs.overflow_b),
        ],
    });
    let bg2 = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("grid.bg2"),
        layout: &bgl,
        entries: &[
            ent(0, params2_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.bins_b),
            ent(3, &bufs.counts_b),
            ent(4, &bufs.start_b),
            ent(5, &bufs.items_b),
            ent(6, &bufs.cursor_b),
            ent(7, &bufs.overflow_b),
        ],
    });

    GridPipes {
        bin: p_bin,
        scan: p_scan,
        place: p_place,
        canon: p_canon,
        bg,
        bg2,
    }
}

pub(crate) fn parse_grid(
    n: usize,
    total: usize,
    bufs: &GridBuffs,
    data: &[u8],
) -> (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, u32) {
    let u32_at = |i: usize| -> u32 {
        let c = &data[i * 4..i * 4 + 4];
        u32::from_le_bytes([c[0], c[1], c[2], c[3]])
    };
    let s_base = (bufs.start_off / 4) as usize;
    let s2_base = (bufs.start2_off / 4) as usize;
    let i_base = (bufs.items_off / 4) as usize;
    let b_base = (bufs.bins_off / 4) as usize;
    let start: Vec<u32> = (0..total + 1).map(|k| u32_at(s_base + k)).collect();
    let start2: Vec<u32> = (0..total + 1).map(|k| u32_at(s2_base + k)).collect();
    let items: Vec<u32> = (0..n).map(|k| u32_at(i_base + k)).collect();
    let bins: Vec<u32> = (0..n).map(|k| u32_at(b_base + k)).collect();
    let overflow = u32_at(bufs.of_off as usize / 4);

    (start, start2, items, bins, overflow)
}

/// 回读两张表 + `items`/`bins`/`overflow`（两类两遍**共用**一段回读）。
fn read_back_grid(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    bufs: &GridBuffs,
    n: usize,
    total: usize,
) -> (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, u32) {
    // 表步长 = align8(total+1)（与 `grid.wgsl::table_stride` 同式；切片绑定须 32 B 对齐）。
    let tbl = ((total + 1).div_ceil(8) * 8 * 4) as u64;
    {
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("grid.bufs.readback"),
        });
        enc.copy_buffer_to_buffer(&bufs.start_b, 0, &bufs.readback, bufs.start_off, tbl);
        // 类 1 表在**对齐步长**处（两段拷贝：readback 里仍按紧凑布局排）。
        let half = bufs.start_b.size() / 2;
        enc.copy_buffer_to_buffer(
            &bufs.start_b,
            half,
            &bufs.readback,
            bufs.start2_off,
            ((total as u64) + 1) * 4,
        );
        enc.copy_buffer_to_buffer(
            &bufs.items_b,
            0,
            &bufs.readback,
            bufs.items_off,
            (n * 4) as u64,
        );
        enc.copy_buffer_to_buffer(
            &bufs.bins_b,
            0,
            &bufs.readback,
            bufs.bins_off,
            (n * 4) as u64,
        );
        enc.copy_buffer_to_buffer(&bufs.overflow_b, 0, &bufs.readback, bufs.of_off, 4);
        queue.submit(Some(enc.finish()));
    }
    let slice = bufs.readback.slice(..);
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
    let out = parse_grid(n, total, bufs, &data);
    // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
    drop(data);
    bufs.readback.unmap();
    out
}

// 9 参（`device`/`queue`/`bufs`/`pipes` + 4 个分派量 + `clear`/`second` 两个开关）：
// 纯转发、无状态 —— 拆结构体只会加一层壳（先例：本仓 `probe.rs` 的长签名）。
#[allow(clippy::too_many_arguments)]
fn submit_grid_pass(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    bufs: &GridBuffs,
    pipes: &GridPipes,
    groups_n: u32,
    groups_total: u32,
    total: usize,
    clear: bool,
    second: bool,
) {
    let bg = if second { &pipes.bg2 } else { &pipes.bg };
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("grid.enc"),
    });
    // ① 清零（counts / overflow）：`clear_buffer` 是**确定**的（不是"未定义内容"）。
    // ⚠️ **两类档只在第一遍清**（第二遍再清会把类 0 的计数表抹掉）。
    if clear {
        enc.clear_buffer(&bufs.counts_b, 0, None);
        enc.clear_buffer(&bufs.overflow_b, 0, None);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.bin_count"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.bin);
        cp.set_bind_group(0, bg, &[]);
        let (gx, gy) = crate::probe::split_2d(groups_n);
        cp.dispatch_workgroups(gx, gy, 1);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.scan"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.scan);
        cp.set_bind_group(0, bg, &[]);
        cp.dispatch_workgroups(1, 1, 1);
    }
    enc.copy_buffer_to_buffer(
        &bufs.start_b,
        0,
        &bufs.cursor_b,
        0,
        ((total + 1) * 4) as u64,
    );
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.place"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.place);
        cp.set_bind_group(0, bg, &[]);
        let (gx, gy) = crate::probe::split_2d(groups_n);
        cp.dispatch_workgroups(gx, gy, 1);
    }
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("grid.canon"),
            timestamp_writes: None,
        });
        cp.set_pipeline(&pipes.canon);
        cp.set_bind_group(0, bg, &[]);
        let (gx, gy) = crate::probe::split_2d(groups_total);
        cp.dispatch_workgroups(gx, gy, 1);
    }
    queue.submit(Some(enc.finish()));
}

/// 在**指定适配器序号**上跑「网格重建」（四入口一条链、缓冲/管线复用、`repeats` 轮）。
pub fn grid_on_adapter(
    adapter_index: usize,
    inputs: &GridInputs<'_>,
    params: GridParams,
    repeats: usize,
) -> GridOut {
    let t0 = std::time::Instant::now();
    let (adapter_name, device, queue) = match crate::probe::device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => return GridOut::err(e),
    };
    let n = params.n as usize;
    let total = params.total as usize;
    // **纯流体档 = 类 0 覆盖全量粒子**（`n_fluid = n`、`class_lo = 0`）⇒ 表仍在 `[0, total+1)`、
    // 基址 0 ⇒ 与单类版**逐位相同**（"不动默认档"的关键）。
    let mut params = params;
    params.n_fluid = params.n;
    params.class_lo = 0;

    let bufs = make_grid_buffs(&device, n, total, params, inputs);
    let pipes = make_grid_pipes(&device, &bufs, &bufs.params_b);
    let setup_ms = (t0.elapsed().as_secs_f64() * 1e3) as f32;
    let reps = repeats.max(1);
    let wg = 64u32;
    let groups_n = (n as u32).div_ceil(wg);
    let groups_total = (total as u32).div_ceil(wg);
    let t_run = std::time::Instant::now();
    submit_grid_pass(
        &device,
        &queue,
        &bufs,
        &pipes,
        groups_n,
        groups_total,
        total,
        true,
        false,
    );
    let (start, start2, items, bins, overflow) = read_back_grid(&device, &queue, &bufs, n, total);
    let per_run_ms = (t_run.elapsed().as_secs_f64() * 1e3) as f32 / reps as f32;
    GridOut {
        start,
        start2,
        items,
        bins,
        overflow,
        setup_ms,
        per_run_ms,
        adapter: adapter_name,
        error: None,
    }
}

/// **两类档**：同一份输入上跑**两遍**四入口（类 0 = 流体、类 1 = 边界）⇒ 两张表 + 共享
/// `items`（槽位 = 流体块 ‖ 边界块）。⚠️ 清零只给第一遍；要求 `1 ≤ n_fluid < n`。
pub fn grid_two_class_on_adapter(
    adapter_index: usize,
    inputs: &GridInputs<'_>,
    params: GridParams,
    n_fluid: u32,
) -> GridOut {
    let t0 = std::time::Instant::now();
    let (adapter_name, device, queue) = match crate::probe::device_for(adapter_index) {
        Ok(v) => v,
        Err(e) => return GridOut::err(e),
    };
    let n = params.n as usize;
    let total = params.total as usize;
    let n_fluid = n_fluid.clamp(1, params.n - 1);
    let mut p_f = params;
    p_f.n_fluid = n_fluid;
    p_f.class_lo = 0;
    // 类 1 的 params 走**第二份 uniform 缓冲**（不在两遍之间写缓冲 —— 实测那条路没生效）。
    let mut p_b = p_f;
    p_b.class_lo = n_fluid;
    let params2_b = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("grid.params2"),
        contents: &params_bytes(&p_b),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });

    let bufs = make_grid_buffs(&device, n, total, p_f, inputs);
    let pipes = make_grid_pipes(&device, &bufs, &params2_b);
    let setup_ms = (t0.elapsed().as_secs_f64() * 1e3) as f32;
    let wg = 64u32;
    // ⚠️ **分派要覆盖到 `cls_hi`**（核里索引的是**全局粒子号**、范围检查在核内做）：
    // 类 0 覆盖 `[0, n_fluid)`、类 1 要覆盖到 `n`（只派 `n − n_fluid` 个 id 的话
    // 全部落在 `[n_fluid, n)` 之外 ⇒ **一遍空转**（本轮实测踩到：表全 0）。
    let groups_f = n_fluid.div_ceil(wg);
    let groups_b = params.n.div_ceil(wg);
    let groups_total = (total as u32).div_ceil(wg);
    let t_run = std::time::Instant::now();
    // 类 0（流体）：清零 + 四入口
    submit_grid_pass(
        &device,
        &queue,
        &bufs,
        &pipes,
        groups_f,
        groups_total,
        total,
        true,
        false,
    );
    // 类 1（边界）：**不清零**；用 `bg2`（binding 0 = 类 1 的 params）再跑四入口
    submit_grid_pass(
        &device,
        &queue,
        &bufs,
        &pipes,
        groups_b,
        groups_total,
        total,
        false,
        true,
    );
    let (start, start2, items, bins, overflow) = read_back_grid(&device, &queue, &bufs, n, total);
    let per_run_ms = (t_run.elapsed().as_secs_f64() * 1e3) as f32;
    GridOut {
        start,
        start2,
        items,
        bins,
        overflow,
        setup_ms,
        per_run_ms,
        adapter: adapter_name,
        error: None,
    }
}
