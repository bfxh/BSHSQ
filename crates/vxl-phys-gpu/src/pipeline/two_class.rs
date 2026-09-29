//! **两类格表档（管线侧）**：`PLAN-gpu.md` §26.3 的 ①② 步 —— 相位核的**「每格两段」**结构
//! 与类 1（边界）格表的绑定。
//!
//! **本片的范围（行为逐位不变）**：段结构与绑定先落地；**第二遍网格**（类 1 的 uniform +
//! 第二次分派）**尚未接**（§26.3.2 记了它的 device-lost 与二分计划）。因此当前 `bstart` 绑到
//! `start_b` 的**后半段**（建缓冲时的零、无人写）⇒ 边界段 `[n_fluid+0, n_fluid+0)` **恒空**
//! ⇒ 核的枚举序与改动前**逐项相同**（段内仍逐项判类，见 `density.wgsl`/`force.wgsl`）。
//!
//! **为什么本文件独立**：`pipeline.rs` 已 857 行（超 800 硬阈、靠祖父条款）⇒ 棘轮要求
//! 「任何增长都配同文件最长函数下降」⇒ 只能把 `make_bind_groups` 整块搬进来（净缩）+ 新逻辑
//! 写在这边。
//!
//! **对齐**：类 1 表以**切片**绑定 ⇒ 偏移须满足 `min_storage_buffer_offset_alignment`（本机 32 B）
//! ⇒ 表步长 = `align8(total+1)`（与 `grid.wgsl::table_stride` 同式）；三张按格数的表各
//! `2 × stride × 4` 字节，前段类 0、后段类 1。
use super::*;

/// 类 1 表的**切片视图**（相位核的槽 9）：偏移/长度取 `start_b.size()/2`（缓冲 = `2 × 对齐步长`）
/// ⇒ 不必知道 `total`。**必须切片**：绑整段会读到类 0 的错区（实测症状：`sorted_copies_bitwise`
/// 红 —— 边界段取到类 0 的界 ⇒ 幻影邻居）。
pub(crate) fn class1_entry(bufs: &Bufs) -> wgpu::BindGroupEntry<'_> {
    let half = bufs.start_b.size() / 2;
    wgpu::BindGroupEntry {
        binding: 9,
        resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &bufs.start_b,
            offset: half,
            size: std::num::NonZeroU64::new(half),
        }),
    }
}

/// `Packet` 的六张 bind group（`new` 的第四段；从 `pipeline.rs` 搬来 + 两类档的槽 9 新增）。
pub(crate) fn make_bind_groups(
    device: &wgpu::Device,
    bufs: &Bufs,
    prm: &Params,
    pipes: &Pipes,
) -> Binds {
    let bg_grid = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_grid"),
        layout: &pipes.bl_grid,
        entries: &[
            ent(0, &prm.grid_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.bins_b),
            ent(3, &bufs.counts_b),
            ent(4, &bufs.start_b),
            ent(5, &bufs.items_b),
            ent(6, &bufs.cursor_b),
            ent(7, &bufs.overflow_b),
        ],
    });
    let bg_dens = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_dens"),
        layout: &pipes.bl_dens,
        entries: &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.pos_b),
            ent(3, &bufs.pmass_b),
            ent(5, &bufs.start_b),
            ent(6, &bufs.items_b),
            ent(7, &bufs.dens_b),
            class1_entry(bufs),
        ],
    });
    let bg_force = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_force"),
        layout: &pipes.bl_force,
        entries: &[
            ent(0, &prm.phase_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.vel_b),
            ent(3, &bufs.pmass_b),
            ent(4, &bufs.press_b),
            ent(5, &bufs.start_b),
            ent(6, &bufs.items_b),
            ent(7, &bufs.dens_b),
            ent(8, &bufs.out_b),
            class1_entry(bufs),
        ],
    });
    let bg_eos = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_eos"),
        layout: &pipes.bl_eos,
        entries: &[
            ent(0, &prm.eos_params_b),
            ent(1, &bufs.dens_b),
            ent(2, &bufs.press_b),
        ],
    });
    let bg_int = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("p.bg_int"),
        layout: &pipes.bl_int,
        entries: &[
            ent(0, &prm.int_params_b),
            ent(1, &bufs.pos_b),
            ent(2, &bufs.vel_b),
            ent(3, &bufs.out_b),
        ],
    });

    Binds {
        bg_grid,
        bg_dens,
        bg_force,
        bg_eos,
        bg_int,
    }
}

/// 格序副本档的 **EOS 绑定组**（从 `sorted.rs` 搬来：那边受棘轮 —— 本片要净缩它，而本文件是
/// 新文件、只判硬阈）。`bg_eos` 与两类档无关（只碰 dens/press），纯搬移。
pub(crate) fn sorted_eos_bind(
    device: &wgpu::Device,
    pipes: &Pipes,
    eos_params_b: &wgpu::Buffer,
    dens_c: &wgpu::Buffer,
    press_c: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("s.bg_eos"),
        layout: &pipes.bl_eos,
        entries: &[ent(0, eos_params_b), ent(1, dens_c), ent(2, press_c)],
    })
}
