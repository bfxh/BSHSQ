//! **提交/会话族**（`Packet` 的骨料）：把"跑 tick / 子步 / 回读等待"这一族方法从
//! `impl Packet` 搬出来，2026-10-16 起落在本文件。为什么拆：
//! - **god 门棘轮**：`Packet` 是「字段 > 24 且 方法 > 24」的双超登记项，方法数**只准减** ⇒
//!   要把它从 25 降到 19，只能"搬进新文件"（新文件只判硬阈）；
//!   且以后**新增一个提交档/会话方法，不必再动 `Packet`**。
//! - **职责分离**：`Packet` 是"建一次、常驻"的设备资源持有者；提交/会话是"按 tick 调"的编排——
//!   二者生命周期不同 ⇒ 单独成档。
//!
//! 纯搬移：7 个方法签名逐字未改；方法体里 `self.X` → `self.pk.X`（`Packet` 上的字段与
//! `poll_wait`/`refresh_box`/`encode_substep`），入口只有 [`Packet::submission`] 一个。

use super::{Packet, PacketCfg, TickMs, WallStage};

/// **`Packet` 的提交/会话视图**：借一份 `&mut Packet`，把"跑 tick / 子步 / 回读等待"这一族方法
/// 收在这里。由 [`Packet::submission`] 构造；同一时刻只该借出一个（借 `&mut` ⇒ 天然互斥）。
pub struct Submission<'a> {
    pk: &'a mut Packet,
}

impl Packet {
    /// **借出提交/会话视图**：跑 tick / 子步 / 回读等待都经这里（见 [`Submission`]）。唯一入口——
    /// 让"新增一个提交档/会话方法"不必再动 `Packet` 本身（god 门棘轮）。
    pub fn submission(&mut self) -> Submission<'_> {
        Submission { pk: self }
    }
}

impl<'a> Submission<'a> {
    /// 跑 `ticks` 个 tick（每 tick `substeps` 个子步，子步 `dt = 1/(60·substeps)`）。
    /// `tick_readback = true` 时**每 tick** 回读一次状态（模拟耦合接口），否则只在末尾同步一次。
    pub fn run(
        &mut self,
        cfg: &PacketCfg,
        ticks: usize,
        substeps: usize,
        tick_readback: bool,
    ) -> TickMs {
        self.run_stages(cfg, 0b111_1111, ticks, substeps, tick_readback, false)
    }

    /// **管线档回读**（`PLAN-gpu` §19.1 ③）：同 `run(.., tick_readback = true)`，但**只发起不等待**
    /// ——末子步的 `copy_buffer_to_buffer` + `submit` 发出去就返回，**不调 `poll_wait`**（含末尾那次）；
    /// 宿主在真要读之前调 `readback_pending_wait()` 一次等完 ⇒ **回读的等待与下一 tick 的计算重叠**
    /// （渲染/耦合用"上一帧状态"是行业惯例）。默认档走 `run`（同步）⇒ 判据与哈希不受影响。
    pub fn run_deferred(&mut self, cfg: &PacketCfg, ticks: usize, substeps: usize) -> TickMs {
        self.run_stages(cfg, 0b111_1111, ticks, substeps, true, true)
    }

    /// **消融计时**（诊断）：只跑 `stages` 打开的相位，返回每 tick 毫秒。
    /// 位：0 分箱 / 1 扫描+拷贝+占位 / 2 规范化 / 3 密度 / 4 EOS / 5 力 / 6 积分
    /// ——用来把"整 tick 为什么这么慢"拆开看。
    pub fn run_stages(
        &mut self,
        cfg: &PacketCfg,
        stages: u32,
        ticks: usize,
        substeps: usize,
        tick_readback: bool,
        defer: bool,
    ) -> TickMs {
        self.run_stages_with(cfg, stages, ticks, substeps, tick_readback, None, defer)
    }

    /// **跑一个子步**（`dt_sub` 由调用方给；`cfg.recompute_box` 时先刷箱子）——给"**逐子步**喂表/换档"
    /// 这类接线与诊断用（`run_stages_with` 把子步批在一次提交里，调用方插不进手）。
    /// 语义与 `run_stages_with(.., substeps = 1)` 相同，**只有 dt 不同**（那个固定 1/60）。
    pub fn run_substep(
        &mut self,
        cfg: &PacketCfg,
        dt_sub: f32,
        stages: u32,
        walls: Option<&WallStage>,
    ) -> TickMs {
        let mut out = TickMs::default();
        if cfg.recompute_box {
            out.box_ms += self.pk.refresh_box();
        }
        let mut enc = self
            .pk
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        self.pk.encode_substep(&mut enc, cfg, dt_sub, stages, walls);
        self.pk.queue.submit(Some(enc.finish()));
        out
    }

    /// 同 [`run_stages`](Submission::run_stages)，但带**壁面镜像鬼影**档（`walls`）：每子步在密度之后、
    /// EOS 之前多分派一趟 `wall_ghost.wgsl`（`WallStage::upload` 先给表）。
    #[allow(clippy::too_many_arguments)] // 8 参：配置 + 相位掩码 + tick/子步 + 回读 + 壁面档 + 管线档
    pub fn run_stages_with(
        &mut self,
        cfg: &PacketCfg,
        stages: u32,
        ticks: usize,
        substeps: usize,
        tick_readback: bool,
        walls: Option<&WallStage>,
        // 只发起回读不等待（`run_deferred` 传 true；见该方法注）。
        defer: bool,
    ) -> TickMs {
        let dt_tick = 1.0 / 60.0;
        let dt_sub = dt_tick / substeps as f32;
        let mut out = TickMs::default();
        // ⚠️ **这里不做"预热"**：预热会顺带把状态多推进一个 tick。踩过——漂移表里表现为
        // "整块刚性位移 + Δv 恰好 = g·dt"（看着像混沌/接线错，其实是被测对象被提前推进了）。
        // 要摊掉首轮编译/首触成本，请由**调用方**显式跑一次丢弃计时的 `run`。
        let t0 = std::time::Instant::now();
        for _ in 0..ticks {
            if cfg.recompute_box {
                // **每子步重算箱子**（与 CPU 的 `substep()` 同频）：代价 = 每子步一次提交 + 回读往返，
                // 单独记进 `out.box_ms` 以便读数里能看见这笔开销。
                for si in 0..substeps {
                    out.box_ms += self.pk.refresh_box();
                    let mut enc = self
                        .pk
                        .device
                        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
                    self.pk.encode_substep(&mut enc, cfg, dt_sub, stages, walls);
                    if tick_readback && si + 1 == substeps {
                        enc.copy_buffer_to_buffer(
                            &self.pk.pos_b,
                            0,
                            &self.pk.readback_b,
                            0,
                            (self.pk.n as u64) * 12,
                        );
                    }
                    self.pk.queue.submit(Some(enc.finish()));
                }
                if tick_readback && !defer {
                    self.pk.poll_wait().ok();
                }
                continue;
            }
            let mut enc = self
                .pk
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
            for _ in 0..substeps {
                self.pk.encode_substep(&mut enc, cfg, dt_sub, stages, walls);
            }
            if tick_readback {
                enc.copy_buffer_to_buffer(
                    &self.pk.pos_b,
                    0,
                    &self.pk.readback_b,
                    0,
                    (self.pk.n as u64) * 12,
                );
            }
            self.pk.queue.submit(Some(enc.finish()));
            if tick_readback && !defer {
                self.pk.poll_wait().ok();
            }
        }
        if !defer {
            self.pk.poll_wait().ok();
        }
        out.total = (t0.elapsed().as_secs_f64() * 1e3) as f32;
        out.per_tick = out.total / ticks.max(1) as f32;
        out
    }

    /// **回读的"延迟等待"**（`PLAN-gpu` §19.1 ③ 的原语）：配合 `run_deferred(..)` 使用时，那次调用
    /// 只**发起**（末子步拷贝 + `submit`）就返回、不阻塞；宿主在**真要读之前**（下一 tick 的计算
    /// 已发出之后）调本方法把待完成的 GPU 工作一次等完 ⇒ **回读的等待与下一 tick 的计算重叠**
    /// （渲染/耦合用"上一帧状态"是行业惯例）。默认档走 `run`（同步）⇒ 不涉及本方法。
    pub fn readback_pending_wait(&self) {
        self.pk.poll_wait().ok();
    }

    /// 量一次"状态回读 + 同步"的成本（耦合接口的代价）。
    /// 状态回读计时：**拷贝恒为全量 `pos`**，但**映射区间可指定**（`map_bytes`）——这是
    /// `PLAN-gpu` §19.1 ① 的判别量：耗时随映射区间**线性下降** ⇒ 驱动按**映射区间**拷贝
    /// （生产上可只映射真正需要的部分）；**映射 1 MB 仍花全量时间** ⇒ 是**整块 flush**
    /// （驱动行为，只能换回读路径）。传 `u64::MAX` = 全量（原口径）。
    pub fn measure_readback_ms(&self, map_bytes: u64) -> f32 {
        let total = (self.pk.n as u64) * 12;
        let map = map_bytes.clamp(1, total);
        let t = std::time::Instant::now();
        let mut enc = self
            .pk
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.pk.pos_b, 0, &self.pk.readback_b, 0, total);
        self.pk.queue.submit(Some(enc.finish()));
        self.pk.poll_wait().ok();
        let slice = self.pk.readback_b.slice(..map);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).ok();
        });
        self.pk.poll_wait().ok();
        rx.recv().ok();
        crate::mapped::mapped_view(slice);
        self.pk.readback_b.unmap();
        (t.elapsed().as_secs_f64() * 1e3) as f32
    }
}
