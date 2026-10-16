//! **声学核（GPU 档 M1/M2/M3/M4）**：一批 `(声源, 听者)` 对，每对一个独立工作项。
//!
//! 与 CPU 端 `vxl-phys-acoustics` 的对应那几条**同口径**（口径逐条写在 `src/acoustic.wgsl`
//! 的头注里）：M1 直达（遮挡走 Möller–Trumbore，端点共面不计；`dist/speed` 与 `1/dist²`）、
//! M2 一次镜面反射（镜像源法，逐对取最强那条）、M3 绕射（棱上 Fermat 点 +
//! `coeff·(1+cosθ)/2/(d₁·d₂)`，只在阴影里产出）、M4 散射（面当二级声源，Lambertian：
//! `s·cos_in·cos_out·A/(d₁²·d₂²)`，同侧 + 两腿不挡，取最强那个面）、M5 介质积分（水平层表 +
//! 定步数中点采样：`Σ len/c` 与 `Π exp(-α_b·len)`）。这里要钉的是
//! "同一批输入 ⇒ 与 CPU 参考**逐对一致**"（浮点容差、拓扑逐位），探针见
//! `tests/acoustic_*_probe.rs`。
//!
//! ⚠️ **CI 不跑**：本仓既有口径是 `cargo test --workspace --exclude vxl-phys-gpu`（headless 运行器
//! 无 GPU）。有适配器的机器上按探针跑。

use vxl_phys_core::Vec3;

/// **M6 二阶反射**的宿主入口：单独一个子模块 —— 这个文件已经贴着本仓 god 门（单文件 ≤ 800 行）
/// 的边，新腿就落在子模块里（顺手把"改到就减"落一步）。
mod reflect2;

// ── 设备侧建置与字节化 = 子模块 `acoustic/buffers.rs`
// 参数镜像、五张表的字节化、场景/介质两套缓冲的建置与上传、两个入口的绑定组与派发，
// 2026-10-17 从本文件**纯搬移**进子模块：那一片是设备关注点，本文件留对外 API
// （`*_on_adapter` 入口与 `SceneIn` / `MediumIn` / 各 `*Out`）。子模块里的可见性是
// `pub(super)`（对外面一行没变）；`run_entry` 同时被 `reflect2` 用 ⇒ 在这里 `use` 引入以便续出去。
mod buffers;
use self::buffers::{run_entry, run_medium_entry, upload, upload_medium};

pub use reflect2::{reflected2_on_adapter, Reflect2Out};

/// 一个 `(声源, 听者)` 对的读数（与 WGSL 输出缓冲同序：`src_i * n_lst + lst_i`）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairOut {
    /// 直达是否被挡。
    pub blocked: bool,
    /// 时延（秒）。
    pub delay: f32,
    /// 能量增益（`1/d²`）。
    pub gain: f32,
}

/// 一个 `(声源, 听者)` 对的**反射**读数（两次归约：有/无 + 最强那条的时延/增益/三角序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReflectOut {
    /// 是否找到一次镜面反射。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`。
    pub delay: f32,
    /// 能量增益 ＝ `(1-a)/(d₁+d₂)²`。
    pub gain: f32,
    /// 最强那条反射所在三角的序号（没有则为 `-1`）。
    pub tri: f32,
}

/// 一个 `(声源, 听者)` 对的**绕射**读数（有/无 + 最强那条棱的时延/增益/棱序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffractOut {
    /// 是否找到绕射路径（阴影里且某条棱的两条腿都通视）。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`（介质积分还没上卡 ⇒ 只有几何行程）。
    pub delay: f32,
    /// 能量增益 ＝ `coeff·(1+cosθ)/2/(d₁·d₂)`。
    pub gain: f32,
    /// 最强那条棱的序号（没有则为 `-1`）。
    pub edge: f32,
}

/// 一个 `(声源, 听者)` 对的**散射**读数（有/无 + 最强那个面的时延/增益/三角序号）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScatterOut {
    /// 是否找到散射路径（面与声源/听者**同侧**、两个余弦 > 0、两条腿都不被挡）。
    pub hit: bool,
    /// 时延（秒）＝ `(d₁+d₂)/speed`（到面中心的两段直线；介质积分还没上卡）。
    pub delay: f32,
    /// 能量增益 ＝ `s·cos_in·cos_out·A/(d₁²·d₂²)`（`A = |cross|/2`；**单频带**）。
    pub gain: f32,
    /// 最强那个面的三角序号（没有则为 `-1`）。
    pub tri: f32,
}

/// 一批输入（几何/材质/棱都是**平铺切片** ⇒ 直接就是卡的布局）。
///
/// 打成结构体不是为了好看：`diffract` 这条本来要 7 个形参，撞本仓 `args-gate` 的软阈值
/// （>7 棘轮、7 实测也算违规）⇒ 收口成一个"场景入参"。
pub struct SceneIn<'a> {
    /// 声源位置。
    pub srcs: &'a [Vec3],
    /// 听者位置。
    pub dsts: &'a [Vec3],
    /// 三角面（每 3 个 `Vec3` 一面）。
    pub tris: &'a [[Vec3; 3]],
    /// 逐三角吸收（只有反射核读它；空表 ⇒ 全 0）。
    pub absorb: &'a [f32],
    /// 逐三角**散射系数**（只有散射核读它；**单频带**口径，空表 ⇒ 全 0 = 纯镜面）。
    pub scatter: &'a [f32],
    /// 衍射棱（每条两个端点；空表 ⇒ 无棱可绕）。
    pub edges: &'a [[Vec3; 2]],
    /// 逐棱绕射系数（**单频带口径**：频带表还没上卡）——与 [`SceneIn::edges`] **一一对应**。
    /// 波长 / 楔角**不在** `SceneIn` 里：见 [`diffracted_wedge_on_adapter`] 的 `band_hz` / `wedges`。
    pub edge_coeff: &'a [f32],
    /// 声速（m/s）。
    pub speed: f32,
}

/// 一个**水平层**（介质）：`y ∈ [y_lo, y_hi]` 内声速 `c`、逐带吸收 `alpha`（1/m）。
///
/// `alpha` 是 **3 个频带**，与 `vxl-phys-acoustics::BANDS` **同值**（GPU 档不引那个 crate：
/// 依赖面只留 `wgpu` + `vxl-phys-core`，所以这个 3 是**显式约定**，改那边要同步这里）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slab {
    pub y_lo: f32,
    pub y_hi: f32,
    pub c: f32,
    pub alpha: [f32; 3],
}

/// 一段（直线）路径的**介质积分**读数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MediumOut {
    /// 走完这段的**用时**（秒）＝ `Σ len_i / c_i`（逐采样点）。
    pub time: f32,
    /// 逐带**衰减因子**＝ `Π exp(-α_b·len)`（1 = 不衰减；3 带同 [`Slab::alpha`]）。
    pub atten: [f32; 3],
}

/// **A9b 逐棱楔角**（与 [`SceneIn::edges`] **一一对应**）：`interior_angle` = 楔体**内角** Ω（弧度），
/// `face0` = 楔体**某一面**上**垂直于棱**的方向（定 `φ = 0` 的基准）。
/// `Ω ≤ 0`（或整表空）⇒ 该棱按**刀口**算（`n = 2`、`fac = 1`）。
///
/// 口径与 CPU `vxl-phys-acoustics::utd::Wedge` 相同；GPU 档不引那个 crate ⇒ 这里是**约定镜像**。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WedgeIn {
    pub interior_angle: f32,
    pub face0: Vec3,
}

/// 在指定适配器上跑一趟**直达核**（M1）：返回 `(适配器名, 逐对读数)`。
///
/// 对序是 `src_i * n_lst + lst_i`（与 `acoustic.wgsl` 的展平口径一致）。
pub fn direct_paths_on_adapter(
    adapter_index: usize,
    srcs: &[Vec3],
    dsts: &[Vec3],
    tris: &[[Vec3; 3]],
    speed: f32,
) -> Result<(String, Vec<PairOut>), String> {
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if srcs.is_empty() || dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb: &[],
        scatter: &[],
        edges: &[],
        edge_coeff: &[],
        speed,
    };
    let (b, _) = upload(&device, &sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "direct") {
        out.push(PairOut {
            blocked: row[3] > 0.5,
            delay: row[1],
            gain: row[2],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**一次镜面反射核**（M2）：逐对取**最强那条**反射（平局按三角序号取小）。
///
/// `absorb` 逐三角给（`0` = 全反射；缺了按空表 ⇒ 当 0 算）。
pub fn reflected_on_adapter(
    adapter_index: usize,
    srcs: &[Vec3],
    dsts: &[Vec3],
    tris: &[[Vec3; 3]],
    absorb: &[f32],
    speed: f32,
) -> Result<(String, Vec<ReflectOut>), String> {
    // 逐三角表与三角表**一一对应**（空表 = 全 0，由 `upload` 补零；给短了就是越界读）。
    if !absorb.is_empty() && absorb.len() != tris.len() {
        return Err(format!(
            "吸收表与三角表必须一一对应（{} vs {}）",
            absorb.len(),
            tris.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if srcs.is_empty() || dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb,
        scatter: &[],
        edges: &[],
        edge_coeff: &[],
        speed,
    };
    let (b, _) = upload(&device, &sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "reflect") {
        out.push(ReflectOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            tri: row[3],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**绕射核**（M3）：**只在阴影里**产出，逐对取**最强那条**棱
/// （平局按棱序号取小 ⇒ 确定）。
///
/// 这是 [`diffracted_wedge_on_adapter`] 的**旧口径薄壳**：不给波长 ⇒ `lambda = 0`
/// ⇒ 增益 = `coeff·(1+cosθ)/2 /(d₁·d₂)`、时延 = `(d₁+d₂)/speed`（既有探针**逐位不变**）。
pub fn diffracted_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
) -> Result<(String, Vec<DiffractOut>), String> {
    diffracted_wedge_on_adapter(adapter_index, sc, &[], 0.0)
}

/// 在指定适配器上跑一趟**绕射核**（M3），并上 **A9a 真刀刃 Fresnel + A9b 楔角修正**。
///
/// `wedges` 与 [`SceneIn::edges`] **一一对应**（空表 ⇒ 全按刀口）；`band_hz > 0` ⇒ 卡上取
/// `lambda = max(speed,1e-3)/band_hz`，增益 = `coeff·|E_d/E₀|²·fac /(d₁·d₂)`
/// （`fac` = 楔角修正因子，`n = 2` ⇒ `1.0`）；`band_hz ≤ 0` ⇒ 退回旧口径 `(1+cosθ)/2`。
pub fn diffracted_wedge_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
    wedges: &[WedgeIn],
    band_hz: f32,
) -> Result<(String, Vec<DiffractOut>), String> {
    // 三张表逐条对应：核内是 `edge_coeff[e]` / `edge_wedge[e]` 直接索引 ⇒ 给短了就是**越界读**
    // （哑表只兜空表）。
    if sc.edges.len() != sc.edge_coeff.len() {
        return Err(format!(
            "棱表与逐棱系数必须一一对应（{} vs {}）",
            sc.edges.len(),
            sc.edge_coeff.len()
        ));
    }
    if !wedges.is_empty() && wedges.len() != sc.edges.len() {
        return Err(format!(
            "楔角表与棱表必须一一对应（{} vs {}）",
            wedges.len(),
            sc.edges.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if sc.srcs.is_empty() || sc.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let (b, _) = upload(&device, sc, wedges, band_hz);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "diffract") {
        out.push(DiffractOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            edge: row[3],
        });
    }
    Ok((name, out))
}

/// 在指定适配器上跑一趟**散射核**（M4）：逐对取**最强那个面**（平局按三角序号取小 ⇒ 确定）。
///
/// 口径与 CPU 的 `push_scatter` 同式（面中心当二级声源、Lambertian 工程口径、
/// 同侧 + 两腿不挡）：`gain = s·cos_in·cos_out·A/(d₁²·d₂²)`。`scatter` 是**逐三角**系数
/// （单频带；频带表还没上卡），空表 ⇒ 全 0 ⇒ 一对也产不出（纯镜面场景的语义）。
pub fn scattered_on_adapter(
    adapter_index: usize,
    sc: &SceneIn<'_>,
) -> Result<(String, Vec<ScatterOut>), String> {
    // 同 `reflected_on_adapter`：逐三角表要么空（= 全 0）、要么与三角表等长。
    if !sc.scatter.is_empty() && sc.scatter.len() != sc.tris.len() {
        return Err(format!(
            "散射表与三角表必须一一对应（{} vs {}）",
            sc.scatter.len(),
            sc.tris.len()
        ));
    }
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if sc.srcs.is_empty() || sc.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let (b, _) = upload(&device, sc, &[], 0.0);
    let mut out = Vec::with_capacity(b.n_pairs);
    for row in run_entry(&device, &queue, &b, "scatter") {
        out.push(ScatterOut {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            tri: row[3],
        });
    }
    Ok((name, out))
}

/// **介质积分的入参**（M5）：这条腿不吃几何/材质表/棱，只吃"一批直线段 + 一张水平层表"。
pub struct MediumIn<'a> {
    /// 段起点（与 `dsts` 成对：对序 = `srcs[i] * n_lst + dsts[i]`）。
    pub srcs: &'a [Vec3],
    /// 段终点。
    pub dsts: &'a [Vec3],
    /// 水平层表（按 `y` 升序、不重叠；**空表 = 纯空气**）。
    pub slabs: &'a [Slab],
    /// 每段的**定步数**（`0` 按 1 算）—— 定步数、无早退 ⇒ 逐位可复现。
    pub steps: u32,
    /// 空气声速（层表没覆盖到的采样点用它）。
    pub speed: f32,
}

/// 在指定适配器上跑一趟**介质积分核**（M5）：直线段上按定步数中点采样，返回 `(用时, 逐带衰减)`。
///
/// 折线（折射的折线）不在这里：宿主**逐段调用**后把时延相加、衰减相乘 —— 两者都是可分量 ⇒
/// 与"一次算完整条折线"等价（判据 `tests/acoustic_medium_probe.rs` 钉了这条组合性）。
pub fn medium_on_adapter(
    adapter_index: usize,
    mi: &MediumIn<'_>,
) -> Result<(String, Vec<MediumOut>), String> {
    let (name, device, queue) = crate::probe::device_for(adapter_index)?;
    if mi.srcs.is_empty() || mi.dsts.is_empty() {
        return Ok((name, Vec::new()));
    }
    let b = upload_medium(&device, mi);
    let out = run_medium_entry(&device, &queue, &b)
        .into_iter()
        .map(|r| MediumOut {
            time: r[0],
            atten: [r[1], r[2], r[3]],
        })
        .collect();
    Ok((name, out))
}
