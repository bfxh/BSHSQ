//! **两类格表档（管线侧）**：`PLAN-gpu.md` §26.3 —— 网格**两遍**（类 0 = 流体 / 类 1 = 边界）
//! + 相位核的**「每格两段」**结构。
//!
//! **为什么**：格序副本档（`sort_copies`）对 2b 打不开 —— 空间排序把两类混在一条副本里，
//! 核里 `j < n_fluid` 的**标签**判断失效。两块方案：副本按「流体块 ‖ 边界块」摆 ⇒ 副本下标
//! < `n_fluid` 恒是流体 ⇒ 标签重新成立；每格两段的枚举序与平铺档逐条相同（§26 已证）⇒ 逐位不变。
//!
//! **两遍的 uniform 语义**（§26.3.1）：两遍都用 `n_fluid = cfg.n_fluid`、只差 `class_lo`
//! （类 0 写 0、类 1 写 `cfg.n_fluid` ⇒ 核里判为边界类）⇒ 类 0 处理 `[0, n_fluid)`、
//! 类 1 处理 `[n_fluid, n)`。`n_fluid == n` 时**跳过第二遍**。
//! ⚠️ **箱刷新（`recompute_box`）必须同步两份 uniform**（`refresh_box` 里已补）——否则两遍用
//! 不同的盒/`total` ⇒ 表几何不一致。
//!
//! **类 1 表怎么被相位核看见**：不单独绑定 —— 相位核（density/force）从**同一张** `start_b`
//! 的 `cell_start` 上按 `tstride = align8(运行时 total + 1)` 现算基址（与网格两遍同式同源）。
//! ⚠️ §28.2 的教训：早先由主机按**建包时的 `cap_total`** 做静态切片绑定，`cap_total ≠ 运行时
//! total`（箱跟随档必然发生）时切片与核里的偏移不同源 ⇒ 读进从未写入的区域 ⇒ 未初始化垃圾 ⇒
//! 死循环（device lost）或边界段静默全空（物理错误）。
use super::{dispatch, ent, Binds, Bufs, Packet, PacketCfg, Params, Pipes};
use wgpu::util::DeviceExt;

/// 网格 uniform（48 B）：`gmin(3f32) | inv | nx | ny | nz | n | total | cap | n_fluid | class_lo`。
/// `class_lo = 0` ⇒ 类 0（`[0, n_fluid)`）；非 0 ⇒ 类 1（`[n_fluid, n)`）。
pub(crate) fn grid_params_buf(
    device: &wgpu::Device,
    cfg: &PacketCfg,
    n: u32,
    total: u32,
    class_lo: u32,
) -> wgpu::Buffer {
    let mut b = Vec::with_capacity(48);
    for x in cfg.gmin {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b.extend_from_slice(&cfg.inv.to_le_bytes());
    for x in [
        cfg.dims[0],
        cfg.dims[1],
        cfg.dims[2],
        n,
        total,
        cfg.cap,
        cfg.n_fluid,
        class_lo,
    ] {
        b.extend_from_slice(&x.to_le_bytes());
    }
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("p.grid_params"),
        contents: &b,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    })
}

/// `Packet` 的六张 bind group（`new` 的第四段；从 `pipeline.rs` 搬来 + 两类档的槽 9 新增）。
pub(crate) fn make_bind_groups(
    device: &wgpu::Device,
    bufs: &Bufs,
    prm: &Params,
    pipes: &Pipes,
) -> Binds {
    let grid_group = |label: &str, params: &wgpu::Buffer| {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(label),
            layout: &pipes.bl_grid,
            entries: &[
                ent(0, params),
                ent(1, &bufs.pos_b),
                ent(2, &bufs.bins_b),
                ent(3, &bufs.counts_b),
                ent(4, &bufs.start_b),
                ent(5, &bufs.items_b),
                ent(6, &bufs.cursor_b),
                ent(7, &bufs.overflow_b),
            ],
        })
    };
    // 类 0 / 类 1 各一份（唯一差别 = binding 0 的 params）—— 两遍之间**不写缓冲**
    // （§26.2 坑②：中途 `write_buffer` 换 params 实测不生效）。
    let bg0 = grid_group("p.bg_grid", &prm.grid_params_b);
    let bg1 = grid_group("p.bg_grid2", &prm.grid2_params_b);
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
        bg_grid: [bg0, bg1],
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

/// **网格四入口**（从 `encode_substep` 搬来 + 两类档的两遍）：`n_fluid < n` 时**再跑一遍**
/// （绑定组换 `bg_grid[1]` ⇒ 类 1 的 uniform）。**自由函数而非方法**：`Packet` 是登记债务
/// （字段/方法只准减），加方法即红。
///
/// ⚠️ 两条实测纪律：① **清零只做一次**（第二遍再清会抹掉类 0 的计数表）；② 分派沿用
/// `groups_n`（= `n.div_ceil(64)`，覆盖到 `n`）—— 核里的索引是**全局粒子号**、范围检查在核内做
/// ⇒ 只派 `n − n_fluid` 个 id 会**整遍空转**。
pub(crate) fn encode_grid(
    pk: &Packet,
    enc: &mut wgpu::CommandEncoder,
    cfg: &PacketCfg,
    stages: u32,
) {
    let two = cfg.n_fluid < cfg.n;
    if stages & 0b000_0001 != 0 {
        enc.clear_buffer(&pk.counts_b, 0, None);
        enc.clear_buffer(&pk.overflow_b, 0, None);
        dispatch(enc, &pk.p_bin, &pk.bg_grid[0], pk.groups_n);
        if two {
            dispatch(enc, &pk.p_bin, &pk.bg_grid[1], pk.groups_n);
        }
    }
    if stages & 0b000_0010 != 0 {
        dispatch(enc, &pk.p_scan, &pk.bg_grid[0], 1);
        // 游标由 `scan` 自己写（见 `grid.wgsl`）⇒ 这里不再 `copy_buffer_to_buffer`。
        dispatch(enc, &pk.p_place, &pk.bg_grid[0], pk.groups_n);
        if two {
            dispatch(enc, &pk.p_scan, &pk.bg_grid[1], 1);
            dispatch(enc, &pk.p_place, &pk.bg_grid[1], pk.groups_n);
        }
    }
    if stages & 0b000_0100 != 0 {
        dispatch(enc, &pk.p_canon, &pk.bg_grid[0], pk.groups_total);
        if two {
            dispatch(enc, &pk.p_canon, &pk.bg_grid[1], pk.groups_total);
        }
    }
}
