//! **端到端（B1+B3+B4）在真实场景上的第一读数**：主机跑出真实流形 → 卡上 `make_iw + build + 12×Jacobi`
//! → 与主机同式参考对拍（口径 B）并计时。
//!
//! 场景 = `m1pile`（25×20²=10k 体）；窄相仍走主机（B2 未做），本探针只验**构建+求解**这条链。
//! ⚠️ 不主张质量：Jacobi ≠ 默认档顺序 GS；世界逆惯量在体累加里用**对角**近似（与 CPU 参考同式）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_resident_probe -- [ticks]`
use std::time::Instant;

use vxl_phys::{BodyBuildExt, DiagExt, HeightField, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_core::{Mat3, Vec3 as V3};
use vxl_phys_gpu::mapped::mapped_view;

const SIDE: usize = 20;
const LAYERS: usize = 25;
const SPACING: f32 = 0.52;
const WB: usize = 11;
const WI: usize = 9;
const WC: usize = 9;
const WCON: usize = 10;
const WV: usize = 6;
const SWEEPS: usize = 12;

fn build_scene(ticks: u32) -> World {
    let cfg = PhysConfig {
        velocity_iterations: 16,
        substeps: 4,
        threads: 8,
        shock_iterations: 2,
        stabilization_iterations: 2,
        wake_gate_k: 8,
        settled_hold_iterations: 4,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_heightfield(HeightField::flat(-8.0, -8.0, 17, 17, 1.0, 0.0));
    let off = (SIDE as f32 - 1.0) * 0.5 * SPACING;
    for layer in 0..LAYERS {
        let y = 0.25 + layer as f32 * SPACING;
        for row in 0..SIDE {
            for col in 0..SIDE {
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(col as f32 * SPACING - off, y, row as f32 * SPACING - off),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    for _ in 0..ticks {
        w.step();
    }
    w
}

struct Data {
    bd: Vec<f32>,
    iw: Vec<f32>,
    ct: Vec<f32>,
    vel: Vec<f32>,
    off: Vec<u32>,
    items: Vec<u32>,
    n_b: usize,
    n_c: usize,
}

fn extract(w: &World) -> Data {
    let n_b = w.bodies.len();
    let mut bd = Vec::with_capacity(n_b * WB);
    let mut iw = vec![0.0f32; n_b * WI];
    let mut vel = Vec::with_capacity(n_b * WV);
    for b in 0..n_b {
        let q = w.bodies.rot(b);
        let li = w.bodies.local_inv_inertia[b];
        bd.extend_from_slice(&[
            w.bodies.position[b].x,
            w.bodies.position[b].y,
            w.bodies.position[b].z,
            q.x,
            q.y,
            q.z,
            q.w,
            w.bodies.inv_mass[b],
            li.x,
            li.y,
            li.z,
        ]);
        let m = Mat3::world_inv_inertia(q, li);
        let (c0, c1, c2) = (m.mul_vec3(V3::X), m.mul_vec3(V3::Y), m.mul_vec3(V3::Z));
        let o = b * WI;
        iw[o..o + 9].copy_from_slice(&[c0.x, c0.y, c0.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z]);
        let lv = w.bodies.linvel[b];
        let av = w.bodies.angvel(b);
        vel.extend_from_slice(&[lv.x, lv.y, lv.z, av.x, av.y, av.z]);
    }
    let ms = w.manifolds();
    let mut ct = Vec::with_capacity(ms.len() * WC);
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); n_b];
    let mut ci = 0u32;
    for m in ms {
        let p = m
            .points
            .iter()
            .next()
            .map(|p| p.point)
            .unwrap_or(Vec3::ZERO);
        ct.extend_from_slice(&[
            m.a as f32, m.b as f32, p.x, p.y, p.z, m.normal.x, m.normal.y, m.normal.z,
            0.5, // mu
        ]);
        lists[m.a as usize].push(ci);
        lists[m.b as usize].push(ci);
        ci += 1;
    }
    let mut off = vec![0u32];
    let mut items = Vec::with_capacity(ct.len() / WC * 2);
    for l in &lists {
        items.extend_from_slice(l);
        off.push(items.len() as u32);
    }
    Data {
        bd,
        iw,
        ct,
        vel,
        off,
        items,
        n_b,
        n_c: ci as usize,
    }
}

fn mv(iw: &[f32], b: usize, v: V3) -> V3 {
    let o = b * WI;
    V3::new(
        iw[o] * v.x + iw[o + 3] * v.y + iw[o + 6] * v.z,
        iw[o + 1] * v.x + iw[o + 4] * v.y + iw[o + 7] * v.z,
        iw[o + 2] * v.x + iw[o + 5] * v.y + iw[o + 8] * v.z,
    )
}

#[allow(clippy::too_many_arguments)]
fn cmass(iw: &[f32], ia: usize, ib: usize, im_a: f32, im_b: f32, ra: V3, rb: V3, d: V3) -> f32 {
    let mut k = im_a + im_b;
    if im_a > 0.0 {
        k += mv(iw, ia, ra.cross(d)).cross(ra).dot(d);
    }
    if im_b > 0.0 {
        k += mv(iw, ib, rb.cross(d)).cross(rb).dot(d);
    }
    if k > 1e-12 {
        1.0 / k
    } else {
        0.0
    }
}

fn tangents(n: V3) -> (V3, V3) {
    let r = if n.x.abs() > 0.9 { V3::Y } else { V3::X };
    let t1 = n.cross(r).normalize();
    (t1, n.cross(t1))
}

/// 主机参考整链：构建 + `SWEEPS` 扫。
fn cpu_ref(d: &Data, con: &mut Vec<f32>, vel: &mut [f32]) {
    cpu_build(d, con);
    // 累计冲量**跨扫保留**（与卡上 `imp` 常驻一致）；放函数外只分配一次。
    let mut imp = vec![0.0f32; d.n_c * 3];
    let mut dimp = vec![0.0f32; d.n_c * 3];
    for _ in 0..SWEEPS {
        cpu_sweep(d, con, &mut imp, &mut dimp, vel);
    }
}

/// 主机参考的构建段。
fn cpu_build(d: &Data, con: &mut Vec<f32>) {
    con.resize(d.n_c * WCON, 0.0);
    for i in 0..d.n_c {
        let o = i * WC;
        let ia = d.ct[o] as usize;
        let ib = d.ct[o + 1] as usize;
        let pt = V3::new(d.ct[o + 2], d.ct[o + 3], d.ct[o + 4]);
        let n = V3::new(d.ct[o + 5], d.ct[o + 6], d.ct[o + 7]);
        let ra = pt - V3::new(d.bd[ia * WB], d.bd[ia * WB + 1], d.bd[ia * WB + 2]);
        let rb = pt - V3::new(d.bd[ib * WB], d.bd[ib * WB + 1], d.bd[ib * WB + 2]);
        let (im_a, im_b) = (d.bd[ia * WB + 7], d.bd[ib * WB + 7]);
        let (t1, t2) = tangents(n);
        let kn = cmass(&d.iw, ia, ib, im_a, im_b, ra, rb, n);
        let kt1 = cmass(&d.iw, ia, ib, im_a, im_b, ra, rb, t1);
        let kt2 = cmass(&d.iw, ia, ib, im_a, im_b, ra, rb, t2);
        let w = i * WCON;
        con[w..w + 6].copy_from_slice(&[ra.x, ra.y, ra.z, rb.x, rb.y, rb.z]);
        con[w + 6] = kn;
        con[w + 7] = kt1;
        con[w + 8] = kt2;
        con[w + 9] = 0.0;
    }
}

/// 主机参考的扫描段（`SWEEPS` 次 Jacobi，与卡上同式同序）。
fn cpu_sweep(d: &Data, con: &[f32], imp: &mut [f32], dimp: &mut [f32], vel: &mut [f32]) {
    for i in 0..d.n_c {
        let o = i * WC;
        let ia = d.ct[o] as usize;
        let ib = d.ct[o + 1] as usize;
        let n = V3::new(d.ct[o + 5], d.ct[o + 6], d.ct[o + 7]);
        let mu = d.ct[o + 8];
        let w = i * WCON;
        let ra = V3::new(con[w], con[w + 1], con[w + 2]);
        let rb = V3::new(con[w + 3], con[w + 4], con[w + 5]);
        let kn = con[w + 6];
        let kt = 0.5 * (con[w + 7] + con[w + 8]);
        let vat = |b: usize, r: V3| {
            let q = b * WV;
            V3::new(vel[q], vel[q + 1], vel[q + 2])
                + V3::new(vel[q + 3], vel[q + 4], vel[q + 5]).cross(r)
        };
        let vr = vat(ib, rb) - vat(ia, ra);
        let io = i * 3;
        let pn0 = imp[io];
        let pn = (pn0 - vr.dot(n) * kn).clamp(0.0, 1.0e9);
        imp[io] = pn;
        dimp[io] = pn - pn0;
        let (t1, t2) = tangents(n);
        let lim = mu * pn;
        let p1_0 = imp[io + 1];
        let p1 = (p1_0 - vr.dot(t1) * kt).clamp(-lim, lim);
        imp[io + 1] = p1;
        dimp[io + 1] = p1 - p1_0;
        let p2_0 = imp[io + 2];
        let p2 = (p2_0 - vr.dot(t2) * kt).clamp(-lim, lim);
        imp[io + 2] = p2;
        dimp[io + 2] = p2 - p2_0;
    }
    for b in 0..d.n_b {
        let im = d.bd[b * WB + 7];
        if im <= 0.0 {
            continue;
        }
        let (s, e) = (d.off[b] as usize, d.off[b + 1] as usize);
        let (mut dp, mut dq) = (V3::ZERO, V3::ZERO);
        for k in s..e {
            let ci = d.items[k] as usize;
            let o = ci * WC;
            let ia = d.ct[o] as usize;
            let ib = d.ct[o + 1] as usize;
            let n = V3::new(d.ct[o + 5], d.ct[o + 6], d.ct[o + 7]);
            let w = ci * WCON;
            let ra = V3::new(con[w], con[w + 1], con[w + 2]);
            let rb = V3::new(con[w + 3], con[w + 4], con[w + 5]);
            let (t1, t2) = tangents(n);
            let io = ci * 3;
            let j = n * dimp[io] + t1 * dimp[io + 1] + t2 * dimp[io + 2];
            if b == ia {
                dp -= j;
                dq -= ra.cross(j);
            } else if b == ib {
                dp += j;
                dq += rb.cross(j);
            }
        }
        let vq = b * WV;
        let lv = V3::new(vel[vq], vel[vq + 1], vel[vq + 2]) + dp * im;
        let o2 = b * WI;
        let ii = V3::new(d.iw[o2], d.iw[o2 + 4], d.iw[o2 + 8]);
        let av = V3::new(vel[vq + 3], vel[vq + 4], vel[vq + 5]) + dq.mul_per_elem(ii);
        vel[vq..vq + 3].copy_from_slice(&[lv.x, lv.y, lv.z]);
        vel[vq + 3..vq + 6].copy_from_slice(&[av.x, av.y, av.z]);
    }
}

fn bytes(v: &[f32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}
fn bytes_u32(v: &[u32]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

struct Gpu {
    bd: wgpu::Buffer,
    vel: wgpu::Buffer,
    con: wgpu::Buffer,
    imp: wgpu::Buffer,
    bg: wgpu::BindGroup,
    p_iw: wgpu::ComputePipeline,
    p_build: wgpu::ComputePipeline,
    p_sweep: wgpu::ComputePipeline,
    p_apply: wgpu::ComputePipeline,
    scatter: wgpu::ComputePipeline,
    par: wgpu::Buffer,
    upd_idx: wgpu::Buffer,
    upd_rec: wgpu::Buffer,
    n_b: u32,
    n_c: u32,
}

fn groups(n: u32) -> (u32, u32) {
    let g = n.div_ceil(64);
    if g <= 65535 {
        (g, 1)
    } else {
        (65535, g.div_ceil(65535))
    }
}

struct Bufs {
    bd: wgpu::Buffer,
    iw: wgpu::Buffer,
    vel: wgpu::Buffer,
    ct: wgpu::Buffer,
    con: wgpu::Buffer,
    imp: wgpu::Buffer,
    dimp: wgpu::Buffer,
    off: wgpu::Buffer,
    items: wgpu::Buffer,
    par: wgpu::Buffer,
    upd_idx: wgpu::Buffer,
    upd_rec: wgpu::Buffer,
}

fn mk_bufs(device: &wgpu::Device, d: &Data) -> Bufs {
    let mk = |label: &str, n: usize| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: (n * 4).max(4) as u64,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        })
    };
    Bufs {
        bd: mk("bd", d.n_b * WB),
        iw: mk("iw", d.n_b * WI),
        vel: mk("vel", d.n_b * WV),
        ct: mk("ct", d.n_c * WC),
        con: mk("con", d.n_c * WCON),
        imp: mk("imp", d.n_c * 3),
        dimp: mk("dimp", d.n_c * 3),
        off: mk("off", d.n_b + 1),
        items: mk("items", d.n_c * 2),
        par: device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
        upd_idx: mk("upd_idx", d.n_b),
        upd_rec: mk("upd_rec", d.n_b * WB),
    }
}

struct Pipes {
    bg: wgpu::BindGroup,
    scatter: wgpu::ComputePipeline,
    iw: wgpu::ComputePipeline,
    build: wgpu::ComputePipeline,
    sweep: wgpu::ComputePipeline,
    apply: wgpu::ComputePipeline,
}

fn make_pipes(device: &wgpu::Device, b: &Bufs) -> Pipes {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("resident"),
        source: wgpu::ShaderSource::Wgsl(include_str!("gpu_resident_probe.wgsl").into()),
    });
    let st = |i: u32, ro: bool| wgpu::BindGroupLayoutEntry {
        binding: i,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: ro },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let mut entries = vec![
        st(0, false),
        st(1, false),
        st(2, false),
        st(3, true),
        st(4, false),
        st(5, false),
        st(6, false),
        st(7, true),
        st(8, true),
        st(10, true),
        st(11, true),
    ];
    entries.push(wgpu::BindGroupLayoutEntry {
        binding: 9,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    });
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("resident-bgl"),
        entries: &entries,
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("resident-pl"),
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
    let mut bgs = Vec::with_capacity(10);
    for (binding, buf) in [
        (0u32, &b.bd),
        (1, &b.iw),
        (2, &b.vel),
        (3, &b.ct),
        (4, &b.con),
        (5, &b.imp),
        (6, &b.dimp),
        (7, &b.off),
        (8, &b.items),
        (9, &b.par),
        (10, &b.upd_idx),
        (11, &b.upd_rec),
    ] {
        bgs.push(wgpu::BindGroupEntry {
            binding,
            resource: buf.as_entire_binding(),
        });
    }
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("resident-bg"),
        layout: &bgl,
        entries: &bgs,
    });
    Pipes {
        bg,
        scatter: pipe("scatter_records"),
        iw: pipe("make_iw"),
        build: pipe("build"),
        sweep: pipe("contact_sweep"),
        apply: pipe("body_apply"),
    }
}

fn setup(device: &wgpu::Device, queue: &wgpu::Queue, d: &Data) -> (Gpu, u64) {
    let b = mk_bufs(device, d);
    let t = Instant::now();
    queue.write_buffer(&b.bd, 0, &bytes(&d.bd));
    queue.write_buffer(&b.ct, 0, &bytes(&d.ct));
    queue.write_buffer(&b.vel, 0, &bytes(&d.vel));
    queue.write_buffer(&b.off, 0, &bytes_u32(&d.off));
    queue.write_buffer(&b.items, 0, &bytes_u32(&d.items));
    let mut pb: Vec<u8> = Vec::with_capacity(32);
    pb.extend_from_slice(&(d.n_c as u32).to_le_bytes());
    pb.extend_from_slice(&(d.n_b as u32).to_le_bytes());
    pb.extend_from_slice(&0.2f32.to_le_bytes());
    pb.extend_from_slice(&0.005f32.to_le_bytes());
    pb.extend_from_slice(&(1.0f32 / 240.0).to_le_bytes());
    pb.extend_from_slice(&[0u8; 12]);
    queue.write_buffer(&b.par, 0, &pb);
    let up_us = t.elapsed().as_micros() as u64;
    let p = make_pipes(device, &b);
    let g = Gpu {
        bd: b.bd,
        vel: b.vel,
        con: b.con,
        imp: b.imp,
        bg: p.bg,
        p_iw: p.iw,
        p_build: p.build,
        p_sweep: p.sweep,
        p_apply: p.apply,
        scatter: p.scatter,
        par: b.par,
        upd_idx: b.upd_idx,
        upd_rec: b.upd_rec,
        n_b: d.n_b as u32,
        n_c: d.n_c as u32,
    };
    (g, up_us)
}

/// 一 tick 一提交：`make_iw + build + 12×(sweep, apply)`。
fn chain(device: &wgpu::Device, queue: &wgpu::Queue, g: &Gpu) {
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    {
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("tick"),
            timestamp_writes: None,
        });
        cp.set_bind_group(0, &g.bg, &[]);
        let mut go = |p: &wgpu::ComputePipeline, n: u32| {
            cp.set_pipeline(p);
            let (x, y) = groups(n);
            cp.dispatch_workgroups(x, y, 1);
        };
        go(&g.p_iw, g.n_b);
        go(&g.p_build, g.n_c);
        for _ in 0..SWEEPS {
            go(&g.p_sweep, g.n_c);
            go(&g.p_apply, g.n_b);
        }
    }
    queue.submit(Some(enc.finish()));
    device
        .poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        })
        .ok();
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

/// B1：差量上传 + 散写。判据 = 同步后**卡上体表与主机逐字节相同**（max|Δ| 必须 0）。
fn delta_test(device: &wgpu::Device, queue: &wgpu::Queue, g: &Gpu, d: &Data) -> (u64, usize, f32) {
    let k = d.n_b / 4;
    let mut bd2 = d.bd.clone();
    let mut idx: Vec<u8> = Vec::with_capacity(k * 4);
    let mut rec: Vec<f32> = Vec::with_capacity(k * WB);
    for t in 0..k {
        let b = (t * 4) % d.n_b;
        idx.extend_from_slice(&(b as u32).to_le_bytes());
        for c in 0..WB {
            let mut v = d.bd[b * WB + c];
            if c < 3 {
                v += 0.01; // 位姿微动（散写是否生效一眼可见）
            }
            rec.push(v);
            bd2[b * WB + c] = v;
        }
    }
    let mut pb: Vec<u8> = Vec::with_capacity(32);
    pb.extend_from_slice(&(d.n_c as u32).to_le_bytes());
    pb.extend_from_slice(&(d.n_b as u32).to_le_bytes());
    pb.extend_from_slice(&0.2f32.to_le_bytes());
    pb.extend_from_slice(&0.005f32.to_le_bytes());
    pb.extend_from_slice(&(1.0f32 / 240.0).to_le_bytes());
    pb.extend_from_slice(&(k as u32).to_le_bytes());
    pb.extend_from_slice(&[0u8; 8]);

    let t = Instant::now();
    queue.write_buffer(&g.upd_idx, 0, &idx);
    queue.write_buffer(&g.upd_rec, 0, &bytes(&rec));
    queue.write_buffer(&g.par, 0, &pb);
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    {
        let (x, y) = groups(k as u32);
        let mut cp = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("delta"),
            timestamp_writes: None,
        });
        cp.set_bind_group(0, &g.bg, &[]);
        cp.set_pipeline(&g.scatter);
        cp.dispatch_workgroups(x, y, 1);
    }
    queue.submit(Some(enc.finish()));
    device
        .poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: None,
        })
        .ok();
    let us = t.elapsed().as_micros() as u64;

    let back = readback(device, queue, &g.bd, d.n_b * WB);
    let mut diff = 0.0f32;
    for (i, v) in back.iter().enumerate().take(d.n_b * WB) {
        diff = diff.max((v - bd2[i]).abs());
    }
    (us, k, diff)
}

fn main() {
    let ticks: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(40);
    let w = build_scene(ticks);
    let d = extract(&w);
    println!("场景 m1pile @ {ticks} tick：体 {} · 流形 {}", d.n_b, d.n_c);

    let mut con_c = Vec::new();
    let mut vel_c = d.vel.clone();
    let t0 = Instant::now();
    cpu_ref(&d, &mut con_c, &mut vel_c);
    let cpu_us = t0.elapsed().as_micros() as u64;

    let (name, device, queue) = match vxl_phys_gpu::probe::device_for(0) {
        Ok(t) => t,
        Err(e) => {
            println!("⚠️ 没起 GPU 设备（{e}）");
            return;
        }
    };
    println!("adapter: {name}");
    let (g, up_us) = setup(&device, &queue, &d);

    // 计时：1 次预热 + 10 次（会反复推进状态，只用于计时）。
    chain(&device, &queue, &g);
    let t = Instant::now();
    for _ in 0..10 {
        chain(&device, &queue, &g);
    }
    let chain_us = t.elapsed().as_micros() as u64 / 10;

    // 对拍前复位（重传速度、清零累计冲量），再跑**正好一次**。
    queue.write_buffer(&g.vel, 0, &bytes(&d.vel));
    queue.write_buffer(&g.imp, 0, &vec![0u8; d.n_c * 3 * 4]);
    chain(&device, &queue, &g);

    let giw = readback(&device, &queue, &g.con, d.n_c * WCON);
    let mut dcon = 0.0f32;
    for (i, x) in giw.iter().enumerate().take(d.n_c * WCON) {
        dcon = dcon.max((x - con_c[i]).abs());
    }
    let gv = readback(&device, &queue, &g.vel, d.n_b * WV);
    let mut dv = 0.0f32;
    for (i, x) in gv.iter().enumerate().take(d.n_b * WV) {
        dv = dv.max((x - vel_c[i]).abs());
    }
    let (d_us, d_k, d_diff) = delta_test(&device, &queue, &g, &d);
    println!(
        "上传（全量体表+接触表）：{up_us} us（{} KB）",
        (d.n_b * WB + d.n_c * WC) * 4 / 1024
    );
    println!(
        "B1 差量：{d_k} 体（{} KB，2 次 write + 散写核）= {d_us} us · 同步后 max|Δ| {d_diff:.0e}（判据 = 0）",
        d_k * (WB * 4 + 4) / 1024
    );
    println!("主机参考（构建 + {SWEEPS} 扫 Jacobi）：{cpu_us} us");
    println!("卡上全链（make_iw + build + {SWEEPS}×(sweep+apply)，一次提交）：{chain_us} us");
    println!("一致性：构建产物 max|Δ| {dcon:.3e} · 末态 max|Δv| {dv:.3e}（口径 B）");
    println!(
        "折算 4 子步/帧：卡上 ≈ {:.1} ms/tick（主机构建+求解 ≈ {:.1} ms/tick）",
        chain_us as f32 * 4.0 / 1000.0,
        cpu_us as f32 * 4.0 / 1000.0
    );
}
