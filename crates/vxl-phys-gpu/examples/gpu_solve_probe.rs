//! **GPU 求解吞吐探针**（`PLAN-gpu.md` §17.2 第三片的前置测量；不是求解器接线）。
//!
//! 只回答一个数：10 万接触 × 1 万体的 **Jacobi 一扫**（接触段 + 体累加段）在卡上多久，
//! 对照主机**单线程、同式、同 CSR 序**的实现。判据**预先钉死**（写进 `M1-RULING §3.17`）：
//! - GPU ≤ **1.0 ms/扫** ⇒ 12 扫/帧 ≈ 12 ms（主机 ~120 ms）⇒ 值得继续；
//! - **1–3 ms/扫** ⇒ 边缘；≥ **5 ms/扫** ⇒ 停手。
//!
//! ⚠️ **不主张质量**：Jacobi ≠ 默认档的顺序冲量；世界逆惯量用**对角近似**。口径 B（比 `max|Δv|`）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_solve_probe -- [contacts] [bodies] [sweeps] [--adapter K]`
use std::time::Instant;

use vxl_phys_gpu::mapped::mapped_view;

const WB: usize = 10; // 每体 f32：lin3 · ang3 · inv_mass · inv_i_diag3 · pad1
const WC: usize = 15; // 每接触 f32：a,b(体号) · n3 · ra3 · rb3 · depth · mu · kn · kt

struct Params {
    n_c: u32,
    n_b: u32,
    beta: f32,
    slop: f32,
    dt: f32,
    _p0: f32,
    _p1: f32,
    _p2: f32,
}

struct Data {
    bv: Vec<f32>,
    ct: Vec<f32>,
    off: Vec<u32>,
    items: Vec<u32>,
    n_b: usize,
    n_c: usize,
}

struct Rng(u64);

impl Rng {
    fn f(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }
    fn unit(&mut self) -> [f32; 3] {
        let mut v = [
            self.f() * 2.0 - 1.0,
            self.f() * 2.0 - 1.0,
            self.f() * 2.0 - 1.0,
        ];
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-6);
        for x in &mut v {
            *x /= l;
        }
        v
    }
}

fn build(n_b: usize, n_c: usize) -> Data {
    let mut r = Rng(0x9E37_79B9_7F4A_7C15);
    let mut bv = Vec::with_capacity(n_b * WB);
    for _ in 0..n_b {
        let im = 1.0 / (5.0 + r.f() * 20.0);
        bv.extend_from_slice(&[
            r.f() * 0.1 - 0.05,
            r.f() * 0.1 - 0.05,
            r.f() * 0.1 - 0.05,
            r.f() * 0.1 - 0.05,
            r.f() * 0.1 - 0.05,
            r.f() * 0.1 - 0.05,
            im,
            im * 30.0,
            im * 30.0,
            im * 30.0,
        ]);
    }
    let mut ct = Vec::with_capacity(n_c * WC);
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); n_b];
    for i in 0..n_c {
        let a = (r.f() * n_b as f32) as usize % n_b;
        let mut b = (r.f() * n_b as f32) as usize % n_b;
        if b == a {
            b = (b + 1) % n_b;
        }
        let n = r.unit();
        let ra = [r.f() * 0.2 - 0.1, r.f() * 0.2 - 0.1, r.f() * 0.1 - 0.05];
        let rb = [r.f() * 0.2 - 0.1, r.f() * 0.2 - 0.1, r.f() * 0.1 - 0.05];
        let k = 0.6 + r.f() * 1.4;
        // 体号当 f32 存（< 2^24 精确）。**不要**用 from_bits：0..1e4 是非规格化数，卡上可能被冲刷成 0。
        ct.extend_from_slice(&[
            a as f32,
            b as f32,
            n[0],
            n[1],
            n[2],
            ra[0],
            ra[1],
            ra[2],
            rb[0],
            rb[1],
            rb[2],
            r.f() * 0.015 - 0.01, // depth（负 = 穿透）
            0.5,                  // mu
            k,
            k,
        ]);
        lists[a].push(i as u32);
        lists[b].push(i as u32);
    }
    let mut off = Vec::with_capacity(n_b + 1);
    let mut items = Vec::with_capacity(n_c * 2);
    off.push(0u32);
    for l in &lists {
        items.extend_from_slice(l); // 按接触序升序（探针夹具天然满足）
        off.push(items.len() as u32);
    }
    Data {
        bv,
        ct,
        off,
        items,
        n_b,
        n_c,
    }
}

fn tangents(n: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let mut r = [1.0f32, 0.0, 0.0];
    if n[0].abs() > 0.9 {
        r = [0.0, 1.0, 0.0];
    }
    let mut t1 = [
        n[1] * r[2] - n[2] * r[1],
        n[2] * r[0] - n[0] * r[2],
        n[0] * r[1] - n[1] * r[0],
    ];
    let l = (t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2])
        .sqrt()
        .max(1e-6);
    for x in &mut t1 {
        *x /= l;
    }
    let t2 = [
        n[1] * t1[2] - n[2] * t1[1],
        n[2] * t1[0] - n[0] * t1[2],
        n[0] * t1[1] - n[1] * t1[0],
    ];
    (t1, t2)
}

/// 主机接触段（只读位姿/速度）。
fn cpu_contact(d: &Data, imp: &mut [f32], dimp: &mut [f32], p: &Params) {
    for i in 0..d.n_c {
        let o = i * WC;
        let ia = d.ct[o] as usize;
        let ib = d.ct[o + 1] as usize;
        let n = [d.ct[o + 2], d.ct[o + 3], d.ct[o + 4]];
        let ra = [d.ct[o + 5], d.ct[o + 6], d.ct[o + 7]];
        let rb = [d.ct[o + 8], d.ct[o + 9], d.ct[o + 10]];
        let (depth, mu, kn, kt) = (d.ct[o + 11], d.ct[o + 12], d.ct[o + 13], d.ct[o + 14]);
        let vel = |b: usize, r: [f32; 3]| {
            let q = b * WB;
            [
                d.bv[q] + d.bv[q + 4] * r[2] - d.bv[q + 5] * r[1],
                d.bv[q + 1] + d.bv[q + 5] * r[0] - d.bv[q + 3] * r[2],
                d.bv[q + 2] + d.bv[q + 3] * r[1] - d.bv[q + 4] * r[0],
            ]
        };
        let va = vel(ia, ra);
        let vb = vel(ib, rb);
        let vr = [vb[0] - va[0], vb[1] - va[1], vb[2] - va[2]];
        let io = i * 3;
        let vn = vr[0] * n[0] + vr[1] * n[1] + vr[2] * n[2];
        let bias = (-(p.beta / p.dt) * (depth - p.slop)).max(0.0);
        let pn0 = imp[io];
        let pn = (pn0 + (-vn - bias) * kn).clamp(0.0, 1.0e9);
        imp[io] = pn;
        dimp[io] = pn - pn0;
        let (t1, t2) = tangents(n);
        let lim = mu * pn;
        let d1 = vr[0] * t1[0] + vr[1] * t1[1] + vr[2] * t1[2];
        let p1_0 = imp[io + 1];
        let p1 = (p1_0 - d1 * kt).clamp(-lim, lim);
        imp[io + 1] = p1;
        dimp[io + 1] = p1 - p1_0;
        let d2 = vr[0] * t2[0] + vr[1] * t2[1] + vr[2] * t2[2];
        let p2_0 = imp[io + 2];
        let p2 = (p2_0 - d2 * kt).clamp(-lim, lim);
        imp[io + 2] = p2;
        dimp[io + 2] = p2 - p2_0;
    }
}

/// 主机体累加段（CSR 升序）。
fn cpu_apply(d: &mut Data, dimp: &[f32]) {
    for b in 0..d.n_b {
        let q = b * WB;
        let im = d.bv[q + 6];
        if im <= 0.0 {
            continue;
        }
        let (s, e) = (d.off[b] as usize, d.off[b + 1] as usize);
        let (mut dp, mut dq) = ([0.0f32; 3], [0.0f32; 3]);
        for k in s..e {
            let ci = d.items[k] as usize;
            let o = ci * WC;
            let ia = d.ct[o] as usize;
            let ib = d.ct[o + 1] as usize;
            let n = [d.ct[o + 2], d.ct[o + 3], d.ct[o + 4]];
            let ra = [d.ct[o + 5], d.ct[o + 6], d.ct[o + 7]];
            let rb = [d.ct[o + 8], d.ct[o + 9], d.ct[o + 10]];
            let (t1, t2) = tangents(n);
            let io = ci * 3;
            let j = [
                n[0] * dimp[io] + t1[0] * dimp[io + 1] + t2[0] * dimp[io + 2],
                n[1] * dimp[io] + t1[1] * dimp[io + 1] + t2[1] * dimp[io + 2],
                n[2] * dimp[io] + t1[2] * dimp[io + 1] + t2[2] * dimp[io + 2],
            ];
            let (sgn, r) = if b == ia {
                (-1.0f32, ra)
            } else if b == ib {
                (1.0, rb)
            } else {
                continue;
            };
            for c in 0..3 {
                dp[c] += sgn * j[c];
            }
            dq[0] += sgn * (r[1] * j[2] - r[2] * j[1]);
            dq[1] += sgn * (r[2] * j[0] - r[0] * j[2]);
            dq[2] += sgn * (r[0] * j[1] - r[1] * j[0]);
        }
        for c in 0..3 {
            d.bv[q + c] += dp[c] * im;
            d.bv[q + 3 + c] += dq[c] * d.bv[q + 7 + c];
        }
    }
}

fn bytes_f32(v: &[f32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}
fn bytes_u32(v: &[u32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

struct Gpu {
    bv: wgpu::Buffer,
    bg: wgpu::BindGroup,
    p_contact: wgpu::ComputePipeline,
    p_apply: wgpu::ComputePipeline,
    n_c: u32,
    n_b: u32,
}

fn mk_buf(device: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.max(4),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

fn groups(n: u32) -> (u32, u32) {
    let g = n.div_ceil(64);
    if g <= 65535 {
        (g, 1)
    } else {
        (65535, g.div_ceil(65535))
    }
}

/// 建缓冲 + 两条流水线 + 绑定组，并上传初值。
fn setup(device: &wgpu::Device, queue: &wgpu::Queue, data: &Data, bv0: &[f32], p: &Params) -> Gpu {
    let (n_b, n_c) = (data.n_b, data.n_c);
    let bv = mk_buf(device, "bv", (n_b * WB * 4) as u64);
    let ct = mk_buf(device, "ct", (n_c * WC * 4) as u64);
    let imp = mk_buf(device, "imp", (n_c * 3 * 4) as u64);
    let dimp = mk_buf(device, "dimp", (n_c * 3 * 4) as u64);
    let off = mk_buf(device, "off", ((n_b + 1) * 4) as u64);
    let items = mk_buf(device, "items", (n_c * 2 * 4) as u64);
    let par = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("params"),
        size: 32,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    queue.write_buffer(&bv, 0, &bytes_f32(bv0));
    queue.write_buffer(&ct, 0, &bytes_f32(&data.ct));
    let zero = vec![0.0f32; n_c * 3];
    queue.write_buffer(&imp, 0, &bytes_f32(&zero));
    queue.write_buffer(&off, 0, &bytes_u32(&data.off));
    queue.write_buffer(&items, 0, &bytes_u32(&data.items));
    let pbytes: Vec<u8> = [
        p.n_c.to_le_bytes().as_slice(),
        p.n_b.to_le_bytes().as_slice(),
        p.beta.to_le_bytes().as_slice(),
        p.slop.to_le_bytes().as_slice(),
        p.dt.to_le_bytes().as_slice(),
        p._p0.to_le_bytes().as_slice(),
        p._p1.to_le_bytes().as_slice(),
        p._p2.to_le_bytes().as_slice(),
    ]
    .concat();
    queue.write_buffer(&par, 0, &pbytes);

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("solve"),
        source: wgpu::ShaderSource::Wgsl(include_str!("gpu_solve_probe.wgsl").into()),
    });
    // 显式布局：两个入口用到的绑定集合不同（接触段不读 off/items），自动布局会各建一套。
    let storage = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("solve-bgl"),
        entries: &[
            storage(0, false),
            storage(1, true),
            storage(2, false),
            storage(3, false),
            storage(4, true),
            storage(5, true),
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("solve-pl"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });
    let pipe = |entry: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(entry),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let mut entries = Vec::with_capacity(7);
    for (binding, b) in [
        (0u32, &bv),
        (1, &ct),
        (2, &imp),
        (3, &dimp),
        (4, &off),
        (5, &items),
        (6, &par),
    ] {
        entries.push(wgpu::BindGroupEntry {
            binding,
            resource: b.as_entire_binding(),
        });
    }
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("solve-bg"),
        layout: &bgl,
        entries: &entries,
    });
    Gpu {
        bv,
        bg,
        p_contact: pipe("contact_sweep"),
        p_apply: pipe("body_apply"),
        n_c: n_c as u32,
        n_b: n_b as u32,
    }
}

/// 跑 `warmup + sweeps` 扫，只给后 `sweeps` 扫计时（提交 + 同步一起算）。
fn run(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    g: &Gpu,
    warmup: u32,
    sweeps: u32,
) -> (u64, u64) {
    let (gc, gcy) = groups(g.n_c);
    let (gb, gby) = groups(g.n_b);
    let one = || {
        let mut enc =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("sweep"),
                timestamp_writes: None,
            });
            cp.set_bind_group(0, &g.bg, &[]);
            cp.set_pipeline(&g.p_contact);
            cp.dispatch_workgroups(gc, gcy, 1);
            cp.set_pipeline(&g.p_apply);
            cp.dispatch_workgroups(gb, gby, 1);
        }
        queue.submit(Some(enc.finish()));
        device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
    };
    let t_setup = Instant::now();
    for _ in 0..warmup {
        one();
    }
    let setup_us = t_setup.elapsed().as_micros() as u64;
    let t1 = Instant::now();
    for _ in 0..sweeps.max(1) {
        one();
    }
    (
        t1.elapsed().as_micros() as u64 / sweeps.max(1) as u64,
        setup_us,
    )
}

/// 读回体表并解成 f32。
fn readback_bv(device: &wgpu::Device, queue: &wgpu::Queue, g: &Gpu, n_b: usize) -> Vec<f32> {
    let bytes = (n_b * WB * 4) as u64;
    let stage = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("read"),
        size: bytes,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    enc.copy_buffer_to_buffer(&g.bv, 0, &stage, 0, bytes);
    queue.submit(Some(enc.finish()));
    let slice = stage.slice(..);
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
    let view = mapped_view(slice);
    let (chunks, _) = view.as_chunks::<4>();
    chunks
        .iter()
        .map(|c| f32::from_bits(u32::from_le_bytes(*c)))
        .collect()
}

fn verdict(gpu_us: u64) -> &'static str {
    if gpu_us <= 1000 {
        "✅ ≤1.0 ms/扫 ⇒ 值得继续"
    } else if gpu_us <= 3000 {
        "⚠️ 1–3 ms/扫 ⇒ 边缘"
    } else if gpu_us >= 5000 {
        "❌ ≥5 ms/扫 ⇒ 停手"
    } else {
        "⚠️ 3–5 ms/扫 ⇒ 偏负"
    }
}

fn main() {
    let mut it = std::env::args().skip(1);
    let n_c: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    let n_b: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(10_000);
    let sweeps: u32 = it.next().and_then(|s| s.parse().ok()).unwrap_or(12);
    let rest: Vec<String> = it.by_ref().collect();
    let mut adapter = 0usize;
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    let p = Params {
        n_c: n_c as u32,
        n_b: n_b as u32,
        beta: 0.2,
        slop: 0.005,
        dt: 1.0 / 240.0,
        _p0: 0.0,
        _p1: 0.0,
        _p2: 0.0,
    };
    let mut data = build(n_b, n_c);
    // 两链必须吃**同一初值**：CPU 链会原地推进 `data.bv`。
    let bv0 = data.bv.clone();
    let mut imp_c = vec![0.0f32; n_c * 3];
    let mut dimp_c = vec![0.0f32; n_c * 3];
    for _ in 0..2 {
        cpu_contact(&data, &mut imp_c, &mut dimp_c, &p);
        cpu_apply(&mut data, &dimp_c);
    }
    let t0 = Instant::now();
    for _ in 0..sweeps {
        cpu_contact(&data, &mut imp_c, &mut dimp_c, &p);
        cpu_apply(&mut data, &dimp_c);
    }
    let cpu_us = t0.elapsed().as_micros() as u64 / sweeps.max(1) as u64;

    let (name, device, queue) = match vxl_phys_gpu::probe::device_for(adapter) {
        Ok(t) => t,
        Err(e) => {
            println!("⚠️ 没起 GPU 设备（{e}）——本探针需要适配器");
            return;
        }
    };
    println!("adapter: {name}");
    let g = setup(&device, &queue, &data, &bv0, &p);
    let (gpu_us, setup_us) = run(&device, &queue, &g, 2, sweeps);
    let gv = readback_bv(&device, &queue, &g, n_b);
    let mut max_d = 0.0f32;
    for (i, v) in gv.iter().enumerate().take(n_b * WB) {
        max_d = max_d.max((*v - data.bv[i]).abs());
    }
    println!(
        "N={n_c} 接触 / {n_b} 体 · {sweeps} 扫 · 体表 {} KB / 接触表 {} KB",
        n_b * WB * 4 / 1024,
        n_c * WC * 4 / 1024
    );
    println!(
        "CPU 单线程：{cpu_us} us/扫（{sweeps} 扫合计 {} us）",
        cpu_us * sweeps as u64
    );
    println!("GPU（接触+累加，含提交+同步）：{gpu_us} us/扫 · setup {setup_us} us");
    println!("口径 B：max|Δv| = {max_d:.3e}（GPU vs CPU 同式参考）");
    println!(
        "12 扫/帧：CPU ≈ {} ms vs GPU ≈ {:.1} ms",
        (cpu_us * 12) / 1000,
        (gpu_us * 12) as f32 / 1000.0
    );
    println!("判据：{}", verdict(gpu_us));
}
