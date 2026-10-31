//! **B3 构建内核探针**：每体世界逆惯量 + 每接触四个有效质量项，在卡上要多久（口径 B 对拍）。
//!
//! `PLAN-gpu §30`（B 全链路状态常驻）的第一刀：构建在 CPU 占 solve 的 54–69%，是 B 的唯一未知项。
//! 本探针与 `helpers::contact_mass` / `contact_mass_cross` **同式**，只量吞吐 + 校验一致性。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_build_probe -- [contacts] [bodies]`
use std::time::Instant;

use vxl_phys_core::{Mat3, Quat, Vec3};
use vxl_phys_gpu::mapped::mapped_view;

const WB: usize = 11; // pos3 · quat4 · inv_mass · local_inv_i3
const WI: usize = 9; // 世界逆惯量（列主序）
const WC: usize = 9; // a,b · point3 · normal3 · pad
const WO: usize = 10; // ra3 · rb3 · kn kt1 kt2 kcross

struct Fixture {
    bd: Vec<f32>,
    iw: Vec<f32>,
    ct: Vec<f32>,
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
}

fn build_fixture(n_b: usize, n_c: usize) -> Fixture {
    let mut r = Rng(0xABCD_1234_5678_9F01);
    let mut bd = Vec::with_capacity(n_b * WB);
    let mut iw = vec![0.0f32; n_b * WI];
    for b in 0..n_b {
        let im = 1.0 / (5.0 + r.f() * 20.0);
        let loc = [im * 30.0, im * 25.0, im * 20.0];
        let q = Quat::from_axis_angle(
            Vec3::new(r.f() * 2.0 - 1.0, r.f() * 2.0 - 1.0, r.f() * 2.0 - 1.0).normalize(),
            r.f() * 3.0,
        );
        bd.extend_from_slice(&[
            r.f() * 10.0 - 5.0,
            r.f() * 10.0 - 5.0,
            r.f() * 10.0 - 5.0,
            q.x,
            q.y,
            q.z,
            q.w,
            im,
            loc[0],
            loc[1],
            loc[2],
        ]);
        let m = Mat3::world_inv_inertia(q, Vec3::new(loc[0], loc[1], loc[2]));
        let (c0, c1, c2) = (
            m.mul_vec3(Vec3::X),
            m.mul_vec3(Vec3::Y),
            m.mul_vec3(Vec3::Z),
        );
        let o = b * WI;
        iw[o..o + 9].copy_from_slice(&[c0.x, c0.y, c0.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z]);
    }
    let mut ct = Vec::with_capacity(n_c * WC);
    for _ in 0..n_c {
        let a = (r.f() * n_b as f32) as usize % n_b;
        let mut b = (r.f() * n_b as f32) as usize % n_b;
        if b == a {
            b = (b + 1) % n_b;
        }
        let n = Vec3::new(r.f() * 2.0 - 1.0, r.f() * 2.0 - 1.0, r.f() * 2.0 - 1.0).normalize();
        ct.extend_from_slice(&[
            a as f32,
            b as f32,
            r.f() * 10.0 - 5.0,
            r.f() * 10.0 - 5.0,
            r.f() * 10.0 - 5.0,
            n.x,
            n.y,
            n.z,
            0.0,
        ]);
    }
    Fixture {
        bd,
        iw,
        ct,
        n_b,
        n_c,
    }
}

/// 列主序 `M·v`（与卡上 `mat3x3 * vec3` 同序；`iw` 由 `Mat3::world_inv_inertia` 列出）。
fn mv(iw: &[f32], b: usize, v: Vec3) -> Vec3 {
    let o = b * WI;
    Vec3::new(
        iw[o] * v.x + iw[o + 3] * v.y + iw[o + 6] * v.z,
        iw[o + 1] * v.x + iw[o + 4] * v.y + iw[o + 7] * v.z,
        iw[o + 2] * v.x + iw[o + 5] * v.y + iw[o + 8] * v.z,
    )
}

fn tangents(n: Vec3) -> (Vec3, Vec3) {
    let r = if n.x.abs() > 0.9 { Vec3::Y } else { Vec3::X };
    let t1 = n.cross(r).normalize();
    (t1, n.cross(t1))
}

#[allow(clippy::too_many_arguments)]
fn contact_mass(
    iw: &[f32],
    ia: usize,
    ib: usize,
    im_a: f32,
    im_b: f32,
    ra: Vec3,
    rb: Vec3,
    d: Vec3,
) -> f32 {
    let mut k = im_a + im_b;
    if im_a > 0.0 {
        let w = mv(iw, ia, ra.cross(d));
        k += w.cross(ra).dot(d);
    }
    if im_b > 0.0 {
        let w = mv(iw, ib, rb.cross(d));
        k += w.cross(rb).dot(d);
    }
    if k > 1e-12 {
        1.0 / k
    } else {
        0.0
    }
}

/// 主机参考：一次 `make_iw` + 一次 `build`。
fn cpu_ref(f: &Fixture) -> Vec<f32> {
    let mut out = vec![0.0f32; f.n_c * WO];
    for i in 0..f.n_c {
        let o = i * WC;
        let ia = f.ct[o] as usize;
        let ib = f.ct[o + 1] as usize;
        let pt = Vec3::new(f.ct[o + 2], f.ct[o + 3], f.ct[o + 4]);
        let n = Vec3::new(f.ct[o + 5], f.ct[o + 6], f.ct[o + 7]);
        let pa = Vec3::new(f.bd[ia * WB], f.bd[ia * WB + 1], f.bd[ia * WB + 2]);
        let pb = Vec3::new(f.bd[ib * WB], f.bd[ib * WB + 1], f.bd[ib * WB + 2]);
        let im_a = f.bd[ia * WB + 7];
        let im_b = f.bd[ib * WB + 7];
        let ra = pt - pa;
        let rb = pt - pb;
        let (t1, t2) = tangents(n);
        let kn = contact_mass(&f.iw, ia, ib, im_a, im_b, ra, rb, n);
        let kt1 = contact_mass(&f.iw, ia, ib, im_a, im_b, ra, rb, t1);
        let kt2 = contact_mass(&f.iw, ia, ib, im_a, im_b, ra, rb, t2);
        let mut kc = (im_a + im_b) * t1.dot(t2);
        if im_a > 0.0 {
            kc += ra.cross(t1).dot(mv(&f.iw, ia, ra.cross(t2)));
        }
        if im_b > 0.0 {
            kc += rb.cross(t1).dot(mv(&f.iw, ib, rb.cross(t2)));
        }
        let w = i * WO;
        out[w] = ra.x;
        out[w + 1] = ra.y;
        out[w + 2] = ra.z;
        out[w + 3] = rb.x;
        out[w + 4] = rb.y;
        out[w + 5] = rb.z;
        out[w + 6] = kn;
        out[w + 7] = kt1;
        out[w + 8] = kt2;
        out[w + 9] = kc;
    }
    out
}

fn bytes(v: &[f32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

fn mk_buf(device: &wgpu::Device, label: &str, n: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: (n * 4).max(4) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

struct Gpu {
    iw: wgpu::Buffer,
    out: wgpu::Buffer,
    bg: wgpu::BindGroup,
    p_iw: wgpu::ComputePipeline,
    p_build: wgpu::ComputePipeline,
    n_b: u32,
    n_c: u32,
}

fn setup(device: &wgpu::Device, queue: &wgpu::Queue, f: &Fixture) -> Gpu {
    let (n_b, n_c) = (f.n_b, f.n_c);
    let bd = mk_buf(device, "bd", n_b * WB);
    let iw = mk_buf(device, "iw", n_b * WI);
    let ct = mk_buf(device, "ct", n_c * WC);
    let out = mk_buf(device, "out", n_c * WO);
    queue.write_buffer(&bd, 0, &bytes(&f.bd));
    queue.write_buffer(&ct, 0, &bytes(&f.ct));
    let mut pbytes: Vec<u8> = Vec::with_capacity(16);
    pbytes.extend_from_slice(&(n_c as u32).to_le_bytes());
    pbytes.extend_from_slice(&(n_b as u32).to_le_bytes());
    pbytes.extend_from_slice(&[0u8; 8]);
    let par = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("params"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    queue.write_buffer(&par, 0, &pbytes);

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("build"),
        source: wgpu::ShaderSource::Wgsl(include_str!("gpu_build_probe.wgsl").into()),
    });
    let st = |b: u32, ro: bool| wgpu::BindGroupLayoutEntry {
        binding: b,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: ro },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("build-bgl"),
        entries: &[
            st(0, true),
            st(1, false),
            st(2, true),
            st(3, false),
            wgpu::BindGroupLayoutEntry {
                binding: 4,
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
        label: Some("build-pl"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });
    let pipe = |e: &str| {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(e),
            layout: Some(&pl),
            module: &shader,
            entry_point: Some(e),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("build-bg"),
        layout: &bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: bd.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: iw.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: ct.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: out.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: par.as_entire_binding(),
            },
        ],
    });
    Gpu {
        iw,
        out,
        bg,
        p_iw: pipe("make_iw"),
        p_build: pipe("build"),
        n_b: n_b as u32,
        n_c: n_c as u32,
    }
}

fn groups(n: u32) -> (u32, u32) {
    let g = n.div_ceil(64);
    if g <= 65535 {
        (g, 1)
    } else {
        (65535, g.div_ceil(65535))
    }
}

/// 跑 1 次预热 + 10 次计时（提交 + 同步一起算）。
fn run(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    g: &Gpu,
    p: &wgpu::ComputePipeline,
    n: u32,
) -> u64 {
    let (gx, gy) = groups(n);
    let one = || {
        let mut enc =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("pass"),
                timestamp_writes: None,
            });
            cp.set_bind_group(0, &g.bg, &[]);
            cp.set_pipeline(p);
            cp.dispatch_workgroups(gx, gy, 1);
        }
        queue.submit(Some(enc.finish()));
        device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .ok();
    };
    one();
    let t = Instant::now();
    for _ in 0..10 {
        one();
    }
    t.elapsed().as_micros() as u64 / 10
}

fn readback(device: &wgpu::Device, queue: &wgpu::Queue, buf: &wgpu::Buffer, n: usize) -> Vec<f32> {
    let stage = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("read"),
        size: (n * 4) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    enc.copy_buffer_to_buffer(buf, 0, &stage, 0, (n * 4) as u64);
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
    let (ch, _) = view.as_chunks::<4>();
    ch.iter()
        .map(|c| f32::from_bits(u32::from_le_bytes(*c)))
        .collect()
}

fn main() {
    let mut it = std::env::args().skip(1);
    let n_c: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    let n_b: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(10_000);
    let f = build_fixture(n_b, n_c);

    let t0 = Instant::now();
    let cpu_out = cpu_ref(&f);
    let cpu_us = t0.elapsed().as_micros() as u64;

    let (name, device, queue) = match vxl_phys_gpu::probe::device_for(0) {
        Ok(t) => t,
        Err(e) => {
            println!("⚠️ 没起 GPU 设备（{e}）");
            return;
        }
    };
    println!("adapter: {name}");
    let g = setup(&device, &queue, &f);
    let iw_us = run(&device, &queue, &g, &g.p_iw, g.n_b);
    let build_us = run(&device, &queue, &g, &g.p_build, g.n_c);

    let gpu_out = readback(&device, &queue, &g.out, n_c * WO);
    let giw = readback(&device, &queue, &g.iw, n_b * WI);
    let mut iw_abs = 0.0f32;
    for (gv, c) in giw.iter().zip(f.iw.iter()) {
        iw_abs = iw_abs.max((gv - c).abs());
    }
    let mut col = [0.0f32; WO];
    let mut max_abs = 0.0f32;
    for (k, gv) in gpu_out.iter().enumerate() {
        let d = (gv - cpu_out[k]).abs();
        col[k % WO] = col[k % WO].max(d);
        max_abs = max_abs.max(d);
    }
    println!("N={n_c} 接触 / {n_b} 体：CPU 全链 {cpu_us} us");
    println!("GPU：make_iw {iw_us} us · build {build_us} us（各 10 次均值，含提交+同步）");
    println!("世界逆惯量 M：max|Δ| {iw_abs:.3e}");
    println!(
        "逐列 max|Δ|：ra {:?} · rb {:?} · kn {:.2e} kt1 {:.2e} kt2 {:.2e} kc {:.2e}",
        &col[0..3],
        &col[3..6],
        col[6],
        col[7],
        col[8],
        col[9]
    );
    println!("口径 B：max|Δ| {max_abs:.3e}");
    println!(
        "折算：4 子步/帧 ⇒ 构建 ≈ {:.2} ms/tick（CPU 侧对应段 ~64 ms）",
        (iw_us + build_us) as f32 * 4.0 / 1000.0
    );
}
