//! **求解卡上档的后端**（`PLAN-gpu.md` §30 B 片：B1 差量上传 + B3 构建 + B4 Jacobi 接进 `World`）。
//!
//! 与窄相档（`crate::narrow`）的分工差别：那边把**几何求交**搬上卡（实测单对慢 4–6× ⇒ 负优化），
//! 这边搬**构建 + 解算**（§3.8 的账：构建占 solve 的 54–69%，solve 占 tick 的 80%）。
//! 窄相/宽相/分岛/休眠都留在主机（见 `vxl_phys_solver::tier_phase`）。
//!
//! **一次 `solve_run` = 一次提交**：散写变动记录 → `make_iw` → `build` → `sweeps × (sweep, apply)`
//! → 速度表回读。常驻状态（体表/世界逆惯量/冲量）留在卡上；速度表每趟整表进出（主机在两趟之间
//! 还要做位置积分与重力，拿不到就接不下去）。
//!
//! ⚠️ **口径**：Jacobi + 无暖启动 + 无锚点回收 ⇒ 与默认档**必然不同**（`PLAN-gpu.md` B6 另立质量
//! 判据）。默认路径一行都不走这里，本档只在显式注册后生效。

use crate::probe::device_for;

// 布局常量**只有一份**：定义在 `vxl_phys_core::solve_tier`，这里重导出 ⇒ 探针/门面与核同源。
use vxl_phys_core::solve_tier::diff_records;
pub use vxl_phys_core::solve_tier::{
    SolveTierBackend, SolveTierIn, SolveTierParams, BD_WORDS, CON_WORDS, CT_WORDS, IMP_WORDS,
    IW_WORDS, VEL_WORDS,
};

/// 一排线程数（64）。
const WG: u32 = 64;
/// 诊断字数：`[0]` = 体号越界、`[1]` = 有效质量退化（**两个都必须 0**，非 0 ⇒ 表写坏了）。
pub const DIAG_WORDS: usize = 2;
/// 护栏：容量上限（超了报错，让调用方回退 CPU）。
const MAX_ITEMS: u32 = 1 << 22;

/// **分项计时**（进程级累加，µs）——同 `narrow::prof` 的理由：档被 `Box` 进了 `World`，
/// 探针拿不到对象 ⇒ 只有在后端内部累加、任何探针都能读，才能回答"整 tick 多出来的钱在哪一项"。
///
/// `[0]` 差量比对+增量打包 `[1]` `write_buffer`（上传全部表）`[2]` 编码+提交+`poll(Wait)`
/// `[3]` 映射+解码 `[4]` 调用次数。
pub mod prof {
    use std::sync::atomic::{AtomicU64, Ordering};

    pub static W: [AtomicU64; 5] = [
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
    ];

    /// 累加一项（µs）。
    pub fn add(i: usize, us: u64) {
        W[i].fetch_add(us, Ordering::Relaxed);
    }

    /// 读一项（µs）。
    pub fn us(i: usize) -> u64 {
        W[i].load(Ordering::Relaxed)
    }

    /// 清零（探针在量某一条链**之前**调）。
    pub fn reset() {
        for a in &W {
            a.store(0, Ordering::Relaxed);
        }
    }
}

/// `solve_run` 的分项读数（µs + 趟数）。
pub fn prof_snapshot() -> [u64; 5] {
    let mut o = [0u64; 5];
    for (i, v) in o.iter_mut().enumerate() {
        *v = prof::us(i);
    }
    o
}

struct Buffers {
    par: wgpu::Buffer,
    bd: wgpu::Buffer,
    iw: wgpu::Buffer,
    vel: wgpu::Buffer,
    ct: wgpu::Buffer,
    con: wgpu::Buffer,
    imp: wgpu::Buffer,
    dimp: wgpu::Buffer,
    off: wgpu::Buffer,
    items: wgpu::Buffer,
    upd_idx: wgpu::Buffer,
    upd_rec: wgpu::Buffer,
    diag: wgpu::Buffer,
    /// 回读靶：速度表 + 尾部 `DIAG_WORDS` 个字。
    rb: wgpu::Buffer,
}

/// **卡上求解档**：设备/管线/常驻缓冲在此，`solve_run` 每趟只重写输入表并回读速度。
pub struct SolveTier {
    adapter: String,
    device: wgpu::Device,
    queue: wgpu::Queue,
    bg: wgpu::BindGroup,
    /// `[0]` `clear_diag` `[1]` `scatter_records` `[2]` `make_iw` `[3]` `build`
    /// `[4]` `contact_sweep` `[5]` `body_apply`。
    pipes: [wgpu::ComputePipeline; 6],
    bufs: Buffers,
    cap_bodies: u32,
    cap_contacts: u32,
    /// **上一份 `bd`**（B1 差量的基线）。`None` = 首趟全量。
    cache: std::sync::Mutex<Option<Vec<f32>>>,
}

fn f32_bytes(v: &[f32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}
fn u32_bytes(v: &[u32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

/// `n` 个元素需要多少 workgroup（`(x, y)` 两维，单维顶 65535）。
fn groups(n: u32) -> (u32, u32) {
    let g = n.div_ceil(WG);
    if g <= 65535 {
        (g, 1)
    } else {
        (65535, g.div_ceil(65535))
    }
}

fn make_bufs(device: &wgpu::Device, cap_b: u32, cap_c: u32) -> Buffers {
    let mk = |label: &str, words: u64, src: bool| {
        let usage = if src {
            wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC
        } else {
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST
        };
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: (words * 4).max(16),
            usage,
            mapped_at_creation: false,
        })
    };
    let b64 = cap_b as u64;
    let c64 = cap_c as u64;
    Buffers {
        par: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("solve.par"),
            size: 64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
        bd: mk("solve.bd", b64 * BD_WORDS as u64, false),
        iw: mk("solve.iw", b64 * IW_WORDS as u64, false),
        // 速度表要能被 `copy_buffer_to_buffer` 出去 ⇒ 带 COPY_SRC。
        vel: mk("solve.vel", b64 * VEL_WORDS as u64, true),
        ct: mk("solve.ct", c64 * CT_WORDS as u64, false),
        con: mk("solve.con", c64 * CON_WORDS as u64, false),
        imp: mk("solve.imp", c64 * IMP_WORDS as u64, false),
        dimp: mk("solve.dimp", c64 * IMP_WORDS as u64, false),
        off: mk("solve.off", b64 + 1, false),
        items: mk("solve.items", c64 * 2, false),
        upd_idx: mk("solve.upd_idx", b64, false),
        upd_rec: mk("solve.upd_rec", b64 * BD_WORDS as u64, false),
        // 诊断：核里 `atomicAdd` ⇒ 必须 STORAGE(read_write)；回读时再拷进 `rb` 尾部。
        diag: mk("solve.diag", DIAG_WORDS as u64, true),
        rb: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("solve.rb"),
            size: ((b64 * VEL_WORDS as u64 + DIAG_WORDS as u64) * 4).max(16),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        }),
    }
}

fn build_pipeline(
    device: &wgpu::Device,
    bufs: &Buffers,
) -> (wgpu::BindGroup, [wgpu::ComputePipeline; 6]) {
    let ro = wgpu::BufferBindingType::Storage { read_only: true };
    let rw = wgpu::BufferBindingType::Storage { read_only: false };
    // 绑定序 = `solve_tier.wgsl` 的 0..12（9 是 uniform 参数块）。
    let types = [
        rw,
        rw,
        rw,
        ro,
        rw,
        rw,
        rw,
        ro,
        ro,
        wgpu::BufferBindingType::Uniform,
        ro,
        ro,
        rw,
    ];
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("solve.bgl"),
        entries: &types
            .iter()
            .enumerate()
            .map(|(b, ty)| wgpu::BindGroupLayoutEntry {
                binding: b as u32,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: *ty,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>(),
    });
    // ⚠️ 这个变量**必须叫 `shader`**：词汇门只豁免计算管线描述符里那一字段取形参 `shader`
    // 的写法（见 `scripts/vocab_scan.sh` 头部）。
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("solve_tier.wgsl"),
        source: wgpu::ShaderSource::Wgsl(include_str!("solve_tier.wgsl").into()),
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("solve.pl"),
        bind_group_layouts: &[Some(&layout)],
        immediate_size: 0,
    });
    let entries = [
        "clear_diag",
        "scatter_records",
        "make_iw",
        "build",
        "contact_sweep",
        "body_apply",
    ];
    let mk_pipe = |e: &'static str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(e),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(e),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let pipes: [wgpu::ComputePipeline; 6] = [
        mk_pipe(entries[0]),
        mk_pipe(entries[1]),
        mk_pipe(entries[2]),
        mk_pipe(entries[3]),
        mk_pipe(entries[4]),
        mk_pipe(entries[5]),
    ];
    let mut bgs = Vec::with_capacity(13);
    for (binding, buf) in [
        (0u32, &bufs.bd),
        (1, &bufs.iw),
        (2, &bufs.vel),
        (3, &bufs.ct),
        (4, &bufs.con),
        (5, &bufs.imp),
        (6, &bufs.dimp),
        (7, &bufs.off),
        (8, &bufs.items),
        (9, &bufs.par),
        (10, &bufs.upd_idx),
        (11, &bufs.upd_rec),
        (12, &bufs.diag),
    ] {
        bgs.push(wgpu::BindGroupEntry {
            binding,
            resource: buf.as_entire_binding(),
        });
    }
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("solve.bg"),
        layout: &layout,
        entries: &bgs,
    });
    (bg, pipes)
}

impl SolveTier {
    /// 在**指定适配器序号**上建档。`cap_bodies`/`cap_contacts` = 常驻缓冲容量，
    /// 必须 ≥ 后续每趟的规模（超了本档报错，调用方回退 CPU）。
    pub fn new(adapter_index: usize, cap_bodies: u32, cap_contacts: u32) -> Result<Self, String> {
        if cap_bodies == 0
            || cap_contacts == 0
            || cap_bodies > MAX_ITEMS
            || cap_contacts > MAX_ITEMS
        {
            return Err(format!(
                "容量不合法：cap_bodies={cap_bodies}、cap_contacts={cap_contacts}（要 1..={MAX_ITEMS}）"
            ));
        }
        let (adapter, device, queue) = device_for(adapter_index)?;
        let bufs = make_bufs(&device, cap_bodies, cap_contacts);
        let (bg, pipes) = build_pipeline(&device, &bufs);
        Ok(Self {
            adapter,
            device,
            queue,
            bg,
            pipes,
            bufs,
            cap_bodies,
            cap_contacts,
            cache: std::sync::Mutex::new(None),
        })
    }

    /// 适配器名（诊断）。
    pub fn adapter(&self) -> &str {
        &self.adapter
    }

    /// 一趟的规模校验（长度整条 + 不超容量）；`Err` ⇒ 调用方整趟回退 CPU。
    fn check(&self, inp: &SolveTierIn) -> Result<(u32, u32), String> {
        if !inp.bd.len().is_multiple_of(BD_WORDS) || !inp.ct.len().is_multiple_of(CT_WORDS) {
            return Err(format!(
                "表长度不是整条：bd={}、ct={}",
                inp.bd.len(),
                inp.ct.len()
            ));
        }
        // **上传边界**：三张表必须全为有限值。否则回读侧只能说"第 N 字非有限"，分不清
        // 是主机表带进来的还是核里算出来的；这里能直接报出**哪张表的哪条记录**。
        for (name, table, words) in [
            ("体表 bd", inp.bd, BD_WORDS),
            ("速度表 vel", inp.vel, VEL_WORDS),
            ("接触表 ct", inp.ct, CT_WORDS),
        ] {
            if let Some(i) = table.iter().position(|v| !v.is_finite()) {
                return Err(format!(
                    "{name} 第 {i} 字（记录 {}）非有限 ⇒ 拒绝上传",
                    i / words
                ));
            }
        }
        let n_b = (inp.bd.len() / BD_WORDS) as u32;
        let n_c = (inp.ct.len() / CT_WORDS) as u32;
        if n_b > self.cap_bodies || n_c > self.cap_contacts {
            return Err(format!(
                "超容量：体 {n_b}/{}、接触 {n_c}/{}（调用方应回退 CPU）",
                self.cap_bodies, self.cap_contacts
            ));
        }
        if inp.off.len() as u32 != n_b + 1 || inp.items.len() as u32 > self.cap_contacts * 2 {
            return Err(format!(
                "CSR 不匹配：off={}（应 {}）、items={}",
                inp.off.len(),
                n_b + 1,
                inp.items.len()
            ));
        }
        if inp.vel.len() as u32 != n_b * VEL_WORDS as u32 {
            return Err(format!("速度表长度不符：{}", inp.vel.len()));
        }
        Ok((n_b, n_c))
    }

    /// 参数块：`[0..3]` = `n_c`/`n_b`/`n_upd`/`sweeps`，`[4..]` = 标量（`to_words` 的口径）。
    fn write_params(&self, n_c: u32, n_b: u32, n_upd: u32, p: &SolveTierParams) {
        let mut w = p.to_words();
        w[0] = n_c;
        w[1] = n_b;
        w[2] = n_upd;
        let bytes: Vec<u8> = w.iter().flat_map(|x| x.to_le_bytes()).collect();
        self.queue.write_buffer(&self.bufs.par, 0, &bytes);
    }
}

impl SolveTierBackend for SolveTier {
    fn solve_run(&self, inp: &SolveTierIn) -> Result<Vec<f32>, String> {
        let (n_b, n_c) = self.check(inp)?;
        let t0 = std::time::Instant::now();
        // B1：与上一份 `bd` 逐记录比 ⇒ 只散写变动的（首趟 = 全量）。
        let mut guard = self
            .cache
            .lock()
            .map_err(|_| "差量基线锁被毒化".to_string())?;
        let changed = diff_records(&mut guard, inp.bd);
        drop(guard);
        let mut idx: Vec<u8> = Vec::with_capacity(changed.len() * 4);
        let mut rec: Vec<u8> = Vec::with_capacity(changed.len() * BD_WORDS * 4);
        for &i in &changed {
            idx.extend_from_slice(&i.to_le_bytes());
            let o = i as usize * BD_WORDS;
            rec.extend_from_slice(&f32_bytes(&inp.bd[o..o + BD_WORDS]));
        }
        prof::add(0, t0.elapsed().as_micros() as u64);
        // 上传：体表增量 + 速度整表 + 接触整表 + CSR + 参数。
        let t1 = std::time::Instant::now();
        put(&self.queue, &self.bufs.upd_idx, &idx);
        put(&self.queue, &self.bufs.upd_rec, &rec);
        put(&self.queue, &self.bufs.vel, &f32_bytes(inp.vel));
        put(&self.queue, &self.bufs.ct, &f32_bytes(inp.ct));
        put(&self.queue, &self.bufs.off, &u32_bytes(inp.off));
        put(&self.queue, &self.bufs.items, &u32_bytes(inp.items));
        self.write_params(n_c, n_b, changed.len() as u32, &inp.params);
        prof::add(1, t1.elapsed().as_micros() as u64);
        // 一次提交：清零诊断 → 散写 → 逆惯量 → 构建 → `sweeps` 扫 → 回读拷贝。
        let t2 = std::time::Instant::now();
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("solve"),
            });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("solve.tick"),
                timestamp_writes: None,
            });
            cp.set_bind_group(0, &self.bg, &[]);
            let mut go = |slot: usize, n: u32| {
                cp.set_pipeline(&self.pipes[slot]);
                let (x, y) = groups(n);
                cp.dispatch_workgroups(x, y, 1);
            };
            go(0, WG);
            go(1, changed.len() as u32);
            go(2, n_b);
            go(3, n_c);
            for _ in 0..inp.params.sweeps.max(1) {
                go(4, n_c);
                go(5, n_b);
            }
        }
        let vel_words = (n_b as usize) * VEL_WORDS;
        enc.copy_buffer_to_buffer(&self.bufs.vel, 0, &self.bufs.rb, 0, (vel_words * 4) as u64);
        enc.copy_buffer_to_buffer(
            &self.bufs.diag,
            0,
            &self.bufs.rb,
            (vel_words * 4) as u64,
            (DIAG_WORDS * 4) as u64,
        );
        self.queue.submit(Some(enc.finish()));
        prof::add(2, t2.elapsed().as_micros() as u64);
        // 回读（同步：主机在两趟之间要做位置积分 ⇒ 这一趟必须拿到速度）。
        let t3 = std::time::Instant::now();
        let slice = self.bufs.rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .map_err(|e| format!("等待求解提交失败：{e}"))?;
        rx.recv()
            .map_err(|_| "回读通道断了".to_string())
            .and_then(|r| r.map_err(|e| format!("映射回读失败：{e}")))?;
        let (out, d0, d1) = {
            let view = crate::mapped::mapped_view(slice);
            let (ch, _) = view.as_chunks::<4>();
            let f = |w: usize| u32::from_le_bytes(ch[vel_words + w]);
            (
                ch[..vel_words]
                    .iter()
                    .map(|b| f32::from_le_bytes(*b))
                    .collect::<Vec<f32>>(),
                f(0),
                f(1),
            )
        };
        self.bufs.rb.unmap();
        prof::add(3, t3.elapsed().as_micros() as u64);
        prof::add(4, 1);
        if d0 != 0 || d1 != 0 {
            // 诊断非零 = 体号越界 / 有效质量退化 ⇒ 表被写坏，速度结果**不可信**，整趟作废。
            // 差量基线顺手清掉：下一趟必须全量重传（卡上这份体表已经不匹配了）。
            if let Ok(mut g) = self.cache.lock() {
                *g = None;
            }
            return Err(format!("卡上诊断非零：体号越界={d0}、有效质量退化={d1}"));
        }
        Ok(out)
    }

    fn tier_name(&self) -> &str {
        &self.adapter
    }
}

/// 写一张表；**空表跳过**（这趟没有变动 ⇒ 卡上保持原样，别拿 0 字节去碰 `write_buffer`）。
fn put(queue: &wgpu::Queue, buf: &wgpu::Buffer, bytes: &[u8]) {
    if !bytes.is_empty() {
        queue.write_buffer(buf, 0, bytes);
    }
}
