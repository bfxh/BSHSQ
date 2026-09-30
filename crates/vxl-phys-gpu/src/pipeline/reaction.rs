//! reaction：边界粒子反作用的**卡上聚合**（`PLAN-gpu.md` §13.2 的"剩半步"）。
//!
//! CPU 侧的口径（`vxl-phys-fluid/src/fluid_boundary.rs::aggregate_reactions`）：逐粒 `bforce`
//! 按**段表**（`spans`）聚合成每体 `(力, 绕体原点的力矩)`，求和序 = 段序 × 段内索引序。
//! 本档把同一步搬到卡上（`reduce.wgsl`），于是耦合回路要回读的不再是"边界段"（20k × 24 B
//! ≈ 480 KB/tick），而是**每体 6 个 f32**（5 体 = 120 B）。
//!
//! **判据**（§13.2 三条，实现里逐条对应）：
//! - **求和序**：一个体 = 一个 workgroup、段内升序串行 ⇒ 与 CPU 的 `for k in start..end` 同序
//!   （不用原子加：原子浮点加不定序 ⇒ 比口径 B 更差）；
//! - **符号/参照系**：核里 `d = p_k − origin`、`τ = Σ d × f_k`，与 CPU 逐字同式；
//! - **回读量**：每体 6 个 f32（本档的 `aggregate` 就回读这个）。

use super::*;

use vxl_phys_core::Vec3;

/// 段表条目在卡上的跨度（字节）：8 个 32 位槽 —— `origin.xyz | start | end | 3 个填充`。
/// 与 `reduce.wgsl` 的 `struct Span` 布局一致（vec3 后接 u32 按 WGSL 规则落在偏移 12）。
const SPAN_STRIDE: usize = 32;

/// 反作用聚合阶段：管线建一次；段表/输出缓冲按**体数**变化时重建（体数不变则复用）。
pub struct ReactionStage {
    pipe: wgpu::ComputePipeline,
    /// **C2 累加入口**（`reduce_add`）：向 `react_b` 就地累加；`pipe`（覆写入口）留作快照/自洽。
    pipe_add: wgpu::ComputePipeline,
    bgl: wgpu::BindGroupLayout,
    spans_b: Option<wgpu::Buffer>,
    react_b: Option<wgpu::Buffer>,
    rb: Option<wgpu::Buffer>,
    n_bodies: usize,
    /// `begin_tick` 清账用的全零字节（与 `react_b` 同尺寸，prepare 时重建 ⇒ 热路径零分配）。
    zeros: Vec<u8>,
}

impl ReactionStage {
    /// 建管线（`reduce.wgsl` 的 `reduce`/`reduce_add` 两入口共用一张布局）。缓冲留到 `aggregate` 按体数分配。
    pub fn new(pkt: &Packet) -> Self {
        let sh = pkt
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("reduce.wgsl"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../reduce.wgsl").into()),
            });
        // 0 pos(只读) | 1 out(只读) | 2 spans(只读) | 3 react(读写) —— 无 uniform（段表长度由
        // 缓冲字节数给出 ⇒ `arrayLength`），所以这张布局只有 storage 槽。
        let bgl = mk_layout(
            &pkt.device,
            "p.bl_reduce",
            &[(0, Kind::Ro), (1, Kind::Ro), (2, Kind::Ro), (3, Kind::Rw)],
        );
        let pipe = mk_pipe(&pkt.device, &bgl, &sh, "p.reduce", "reduce");
        let pipe_add = mk_pipe(&pkt.device, &bgl, &sh, "p.reduce_add", "reduce_add");
        Self {
            pipe,
            pipe_add,
            bgl,
            spans_b: None,
            react_b: None,
            rb: None,
            n_bodies: 0,
            zeros: Vec::new(),
        }
    }

    /// 段表/输出缓冲按体数（重）建；体数不变时复用（耦合回路每 tick 调用也不重建）。
    fn prepare(&mut self, pkt: &Packet, n: usize) {
        if self.n_bodies == n && self.spans_b.is_some() {
            return;
        }
        let mk = |label: &str, size: u64, usage: wgpu::BufferUsages| {
            pkt.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        self.spans_b = Some(mk(
            "r.spans",
            (n * SPAN_STRIDE) as u64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        ));
        self.react_b = Some(mk(
            "r.react",
            (n * 24) as u64,
            // COPY_DST：C2 的 `begin_tick` 每 tick 用 `write_buffer` 清账（缺它第二个 tick 即崩）。
            wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
        ));
        self.rb = Some(mk(
            "r.rb",
            (n * 24) as u64,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        ));
        self.zeros = vec![0u8; n * 24];
        self.n_bodies = n;
    }

    /// 段表上传（每体 32 B）：`(体 id, 体原点, 起始, 结束)`——体 id 只留在主机侧做还原，
    /// 卡上只读 `origin/start/end`（体 id 对求和没有贡献）。
    fn upload_spans(&self, pkt: &Packet, spans: &[(u32, Vec3, u32, u32)]) {
        let mut raw = Vec::with_capacity(spans.len() * SPAN_STRIDE);
        for &(_, origin, start, end) in spans {
            for c in [origin.x, origin.y, origin.z] {
                raw.extend_from_slice(&c.to_le_bytes());
            }
            for v in [start, end, 0u32, 0u32, 0u32] {
                raw.extend_from_slice(&v.to_le_bytes());
            }
        }
        if let Some(b) = self.spans_b.as_ref() {
            pkt.queue.write_buffer(b, 0, &raw);
        }
    }

    /// 绑定组（三处同一张）：`pos | out | spans | react`。
    fn make_bg(
        &self,
        pkt: &Packet,
        spans_b: &wgpu::Buffer,
        react_b: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        pkt.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("p.bg_reduce"),
            layout: &self.bgl,
            entries: &[
                ent(0, &pkt.pos_b),
                ent(1, &pkt.out_b),
                ent(2, spans_b),
                ent(3, react_b),
            ],
        })
    }

    /// **快照聚合**（覆写入口 `reduce`）：`out` 的边界段 → 每体 `(体 id, 力, 绕体原点的力矩)`。
    ///
    /// 返回序 = 传入 `spans` 的段序（与 CPU `boundary_reactions()` 同序 ⇒ 可直接对拍）。
    /// 读的是**当前** `out`（= 末子步快照口径）——C2 起耦合回路的 tick 平均走
    /// [`Self::tick_average`]，本方法留给 report 的自洽腿（同一份 `out` 换序求和核对）。
    pub fn aggregate(
        &mut self,
        pkt: &Packet,
        spans: &[(u32, Vec3, u32, u32)],
    ) -> Vec<(u32, Vec3, Vec3)> {
        if spans.is_empty() {
            return Vec::new();
        }
        self.prepare(pkt, spans.len());
        self.upload_spans(pkt, spans);
        let (Some(spans_b), Some(react_b)) = (self.spans_b.as_ref(), self.react_b.as_ref()) else {
            // `prepare` 之后不可能走到这里（缓冲同生同灭）——留个明确出口而不是 unwrap。
            return Vec::new();
        };
        let bg = self.make_bg(pkt, spans_b, react_b);
        let mut enc = pkt
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("p.reduce.enc"),
            });
        // 一个体一个 workgroup（段内并行会改求和序 ⇒ 只有 gid.x < 体数 的那条线程干活）。
        dispatch(&mut enc, &self.pipe, &bg, spans.len() as u32);
        self.encode_copy(&mut enc);
        pkt.queue.submit(Some(enc.finish()));
        self.download(pkt, spans)
    }

    /// **回读并解码** `rb`（`aggregate`/`read_tick` 共用）：映射 → 每体 `(体 id, 力, 力矩)`。
    /// 段序 = 传入 `spans` 的段序。**不除子步数**（时间加权归调用方）。
    fn download(
        &mut self,
        pkt: &Packet,
        spans: &[(u32, Vec3, u32, u32)],
    ) -> Vec<(u32, Vec3, Vec3)> {
        let Some(rb) = self.rb.as_ref() else {
            return Vec::new();
        };
        pkt.poll_wait().ok();
        let slice = rb.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        pkt.poll_wait().ok();
        rx.recv().ok();
        let data = slice.get_mapped_range();
        // 按索引解码（**别用 `chunks_exact`**：CI 的 clippy 比本机新，会判
        // "using `chunks_exact` with a constant chunk size" ⇒ 门红）。
        let g = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
        let mut out = Vec::with_capacity(spans.len());
        for (b, &(body, _, _, _)) in spans.iter().enumerate() {
            let o = b * 24;
            out.push((
                body,
                Vec3::new(g(o), g(o + 4), g(o + 8)),
                Vec3::new(g(o + 12), g(o + 16), g(o + 20)),
            ));
        }
        // 映射出的范围要在 `unmap` 前先释放（顺序不能反）。
        drop(data);
        rb.unmap();
        out
    }

    /// **tick 开头清账**（C2 累加口径）：`react_b` 写零。`queue.write_buffer` 排在**下一次提交
    /// 之前**生效 ⇒ 必先于本 tick 的第一个子步；首 tick 缓冲未建时跳过（WebGPU 建缓冲即全零
    /// ⇒ 同样正确）。
    pub(crate) fn begin_tick(&self, pkt: &Packet) {
        if let Some(b) = self.react_b.as_ref() {
            pkt.queue.write_buffer(b, 0, &self.zeros);
        }
    }

    /// **每子步一次**（C2）：派 `reduce_add`（向 `react_b` 累加本子步的每体聚合）。
    /// 编进调用方的 encoder（与子步同一提交，不多付提交）；段表随调用上传（幂等、量小）。
    pub(crate) fn encode_accumulate(
        &mut self,
        pkt: &Packet,
        enc: &mut wgpu::CommandEncoder,
        spans: &[(u32, Vec3, u32, u32)],
    ) {
        if spans.is_empty() {
            return;
        }
        self.prepare(pkt, spans.len());
        self.upload_spans(pkt, spans);
        let (Some(spans_b), Some(react_b)) = (self.spans_b.as_ref(), self.react_b.as_ref()) else {
            return;
        };
        let bg = self.make_bg(pkt, spans_b, react_b);
        dispatch(enc, &self.pipe_add, &bg, spans.len() as u32);
    }

    /// 把 `react_b`（= Σ_s）拷进回读缓冲（编进调用方的 encoder；`aggregate` 编进自己的那趟）。
    pub(crate) fn encode_copy(&self, enc: &mut wgpu::CommandEncoder) {
        if let (Some(react_b), Some(rb)) = (self.react_b.as_ref(), self.rb.as_ref()) {
            enc.copy_buffer_to_buffer(react_b, 0, rb, 0, (self.n_bodies * 24) as u64);
        }
    }

    /// **tick 末读账**（配 `encode_copy`，整 tick 只此一次同步）：每体 `(体 id, Σ_s 力, Σ_s 力矩)`。
    pub(crate) fn read_tick(
        &mut self,
        pkt: &Packet,
        spans: &[(u32, Vec3, u32, u32)],
    ) -> Vec<(u32, Vec3, Vec3)> {
        if spans.is_empty() {
            return Vec::new();
        }
        self.download(pkt, spans)
    }

    /// **C2 的整个 tick**（步进器/探针共用）：清账 →（每子步：子步命令链 + `reduce_add` 累加）→
    /// 末子步回拷 → 一次回读 → ÷ 子步数。返回每体 **tick 平均力**，与 CPU
    /// `boundary_reactions()` 同口径（`PLAN-COUPLING.md` §5 / D1(b)）。
    /// 逐子步提交：`recompute_box` 档的 uniform 必须"每子步刷新 ⇒ 立即消费"（整批一提交会让
    /// 所有子步读到最后一口箱子）；固定箱档的提交开销 µs 级、耦合 tick ≥ 数 ms ⇒ 可忽略。
    pub fn tick_average(
        &mut self,
        pk: &mut Packet,
        pc: &PacketCfg,
        substeps: usize,
        walls: Option<&WallStage>,
        spans: &[(u32, Vec3, u32, u32)],
    ) -> Vec<(u32, Vec3, Vec3)> {
        let sub = substeps.max(1);
        let dt_sub = (1.0f32 / 60.0) / sub as f32;
        self.begin_tick(pk);
        for si in 0..sub {
            if pc.recompute_box {
                pk.refresh_box();
            }
            let mut enc = pk
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
            pk.encode_substep(&mut enc, pc, dt_sub, 0b111_1111, walls);
            self.encode_accumulate(pk, &mut enc, spans);
            if si + 1 == sub {
                self.encode_copy(&mut enc);
            }
            pk.queue.submit(Some(enc.finish()));
        }
        let inv = 1.0 / sub as f32;
        self.read_tick(pk, spans)
            .into_iter()
            .map(|(b, f, t)| (b, f * inv, t * inv))
            .collect()
    }

    /// **验收报告**（`gpu_tick_probe --tank` 用）：GPU 逐粒 vs CPU 逐粒 + GPU 每体 tick 平均 vs
    /// CPU `breact`，外加一条"同一批项换序求和"的自洽核对。
    ///
    /// 参数取**朴素类型**（本 crate 对 `vxl-phys-fluid` 只有 `[dev-dependencies]` ⇒ 不引用它的
    /// 类型）；CPU 侧直接喂 `boundary_forces()` / `boundary_reactions()` / `boundary_spans()`。
    /// **C2**：每体一腿改收 `gpu_react`（**tick 平均**，调用方从 [`Self::tick_average`] 的累加
    /// 回路取得——单份 `out` 里只有末子步快照，报告自身重造不出平均值）；自洽腿仍是快照口径
    /// （`aggregate` 覆写重派一次，与逐粒同取末子步 ⇒ 换序只该差舍入）。
    pub fn report(
        &mut self,
        pkt: &Packet,
        n_fluid: u32,
        cpu_force: &[Vec3],
        cpu_react: &[(u32, Vec3, Vec3)],
        spans: &[(u32, Vec3, u32, u32)],
        gpu_react: &[(u32, Vec3, Vec3)],
    ) -> String {
        let gf = pkt.read_boundary_forces(n_fluid);
        if gf.len() < cpu_force.len() * 6 {
            return format!(
                "  ├ 反作用：GPU 回读不足（{} < {}）——跳过报数\n",
                gf.len(),
                cpu_force.len() * 6
            );
        }
        // ① 逐粒（与 §13.2 的验收口径同一组数；两侧都是**末子步快照**口径）
        let (mut mx, mut scale) = (0.0f32, 0.0f32);
        let (mut sc, mut sg) = (Vec3::ZERO, Vec3::ZERO);
        for (k, b) in cpu_force.iter().enumerate() {
            let g = Vec3::new(gf[k * 6], gf[k * 6 + 1], gf[k * 6 + 2]);
            mx = mx.max((*b - g).length());
            scale += b.length();
            sc += *b;
            sg += g;
        }
        let mut s = format!(
            "  ├ 反作用【逐粒】{} 粒边界：max |ΔF| = {mx:.3e} N（相对 Σ|F_cpu| = {:.2e}）| \
             ΣF：CPU {:.4e} N vs GPU {:.4e} N（差 {:.2e}）\n",
            cpu_force.len(),
            mx / scale.max(1e-30),
            sc.length(),
            sg.length(),
            (sc - sg).length()
        );
        // ② 每体（**tick 平均**：调用方从 C2 累加回路取得）——相对量用 CPU 侧的 Σ|F|、Σ|τ| 标定
        let (mut mf, mut mt) = (0.0f32, 0.0f32);
        let (mut sf, mut st) = (0.0f32, 0.0f32);
        let mut rows = String::new();
        for (i, &(body, f, tau)) in cpu_react.iter().enumerate() {
            let (g, gt) = match gpu_react.get(i) {
                Some(&(_, g, gt)) => (g, gt),
                None => (Vec3::ZERO, Vec3::ZERO),
            };
            mf = mf.max((f - g).length());
            mt = mt.max((tau - gt).length());
            sf += f.length();
            st += tau.length();
            if i < 8 {
                rows.push_str(&format!(
                    "  │   体 {body}：|ΔF| {:.3e} N / |Δτ| {:.3e}\n",
                    (f - g).length(),
                    (tau - gt).length()
                ));
            }
        }
        s.push_str(&format!(
            "  ├ 反作用【每体·tick 平均】{} 体：max |ΔF| = {mf:.3e} N（相对 Σ|F_cpu| = {:.2e}）\
             / max |Δτ| = {mt:.3e}（相对 Σ|τ_cpu| = {:.2e}）\n",
            cpu_react.len(),
            mf / sf.max(1e-30),
            mt / st.max(1e-30)
        ));
        s.push_str(&rows);
        // ③ 自洽（**末子步快照**口径，②③不同口径别混）：`aggregate` 覆写重派一次，
        // 与逐粒同取当前 `out` ⇒ 同一批项换序求和，只该差舍入。
        let gb = self.aggregate(pkt, spans);
        let (mut sb, mut sp) = (Vec3::ZERO, Vec3::ZERO);
        for &(_, g, _) in gb.iter() {
            sb += g;
        }
        for k in 0..cpu_force.len() {
            sp += Vec3::new(gf[k * 6], gf[k * 6 + 1], gf[k * 6 + 2]);
        }
        s.push_str(&format!(
            "  └ 自洽：Σ_体 F_gpu(末子步) − Σ_粒 F_gpu = {:.3e} N（同一批项换序求和 ⇒ 只该差舍入）\n",
            (sb - sp).length()
        ));
        s
    }
}
