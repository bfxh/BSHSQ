//! SAT 扫描的 SSE2 内核（x86_64 基线 ISA，**无需运行时特性探测**）——**逐位透明**。
//!
//! 只并行「4 条轴」，轴内算术与标量参考 `sat_scan_scalar` 一字不差：
//! - 点积结合序显式 `((x·x + y·y) + z·z)`（与 `Vec3::dot` 的左结合一致）；
//! - `|v|` 用符号位掩码（bit-exact），`sqrt` 不在此模块使用；
//! - **退化轴**（`length_squared < 0.5`）、`sep > skin` 早退、`sep > best` 严格择优
//!   三项规则都在**标量侧按轴序**逐 lane 复刻 ⇒ 与标量实现逐位同结果。
//!
//! 非 x86_64（如 CI 的 aarch64 交叉）自动走标量路径；两条路径的四段哈希
//! （`m0_gates`/`determinism`/status/stress）必须逐位一致——这是跨平台哈希门的要求。
//!
//! SAFETY：`unsafe` 全部限于 `core::arch::x86_64` 的 `_mm_*` 内建调用；
//! `_mm_loadu_ps` 的输入都是长度 4 的**本地数组**（未对齐读取安全），无裸指针运算、
//! 无生命周期擦除、无别名假设；SSE2 是 x86_64 基线 ISA ⇒ 不需要 `target_feature`
//! 门（调用点因此无需 `unsafe fn` 的前置条件）。
#![allow(unsafe_code)]

use vxl_phys_core::Vec3;

/// 标量参考实现（语义基准）：非 x86_64 走它；x86_64 上由测试用它和 SIMD 逐位对照
/// （故 x86_64 的发布构建里它「未使用」，非死代码）。
#[cfg_attr(target_arch = "x86_64", allow(dead_code))]
#[allow(clippy::too_many_arguments)] // 内核签名（轴表 + 两盒的 half/axes/center + skin）
/// 返回 `None` = 存在分离轴（拒）或全轴退化；`Some((best_sep, n, idx))` 同原实现。
pub(crate) fn sat_scan_scalar(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    let mut best = f32::MIN;
    let mut best_n = Vec3::ZERO;
    let mut best_idx = usize::MAX;
    for (idx, &n0) in axes.iter().enumerate() {
        if n0.length_squared() < 0.5 {
            continue;
        }
        let ra =
            ha.x * aa[0].dot(n0).abs() + ha.y * aa[1].dot(n0).abs() + ha.z * aa[2].dot(n0).abs();
        let rb =
            hb.x * ab[0].dot(n0).abs() + hb.y * ab[1].dot(n0).abs() + hb.z * ab[2].dot(n0).abs();
        let ca = pa.dot(n0);
        let cb = pb.dot(n0);
        let (min_a, max_a) = (ca - ra, ca + ra);
        let (min_b, max_b) = (cb - rb, cb + rb);
        let sep1 = min_b - max_a;
        let sep2 = min_a - max_b;
        let (sep, n) = if sep1 >= sep2 {
            (sep1, n0)
        } else {
            (sep2, -n0)
        };
        if sep > skin {
            return None;
        }
        if sep > best {
            best = sep;
            best_n = n;
            best_idx = idx;
        }
    }
    if best == f32::MIN {
        None
    } else {
        Some((best, best_n, best_idx))
    }
}

/// SIMD 路径分发：x86_64 走 SSE2 内核，其余走标量（结果逐位相同）。
#[allow(clippy::too_many_arguments)]
#[cfg(target_arch = "x86_64")]
pub(crate) fn sat_scan(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    sat_scan_sse2(axes, ha, aa, pa, hb, ab, pb, skin)
}

#[cfg(not(target_arch = "x86_64"))]
pub(crate) fn sat_scan(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    sat_scan_scalar(axes, ha, aa, pa, hb, ab, pb, skin)
}

#[cfg(target_arch = "x86_64")]
#[allow(clippy::too_many_arguments)]
fn sat_scan_sse2(
    axes: &[Vec3],
    ha: Vec3,
    aa: &[Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: &[Vec3; 3],
    pb: Vec3,
    skin: f32,
) -> Option<(f32, Vec3, usize)> {
    use core::arch::x86_64::*;
    let mut best = f32::MIN;
    let mut best_n = Vec3::ZERO;
    let mut best_idx = usize::MAX;
    let n_all = axes.len();
    let mut i = 0usize;
    while i < n_all {
        let m = (n_all - i).min(4);
        // 装 4 条轴的分量行；尾组复制最后一条占位（其 lane 在标量侧不被消费）。
        let mut cx = [0f32; 4];
        let mut cy = [0f32; 4];
        let mut cz = [0f32; 4];
        for k in 0..4 {
            let a = axes[i + k.min(m - 1)];
            cx[k] = a.x;
            cy[k] = a.y;
            cz[k] = a.z;
        }
        let mut sep_arr = [0f32; 4];
        let flags: i32; // bit k: 1 = 该 lane 用 −n（sep2 更大；下面 unsafe 块内赋值）
        let degen: i32; // bit k: 1 = 该 lane 退化（标量侧 continue）
                        // SAFETY: 本块（含内部 `dot_lane`）只调用 x86_64 基线 SSE2 内建——函数在
                        // `#[cfg(target_arch = "x86_64")]` 下编译，故无需运行时特性探测；指针入参只有两处：
                        // `_mm_loadu_ps` 读 `cx/cy/cz`、`_mm_storeu_ps` 写 `sep_arr`，都是长度 4 的本地
                        // `f32` 数组（16 字节未对齐访问仍在界内），其余内建只消费 `__m128` 寄存器值：
                        // 无裸指针运算、无生命周期擦除、无别名假设。
        unsafe {
            let nx = _mm_loadu_ps(cx.as_ptr());
            let ny = _mm_loadu_ps(cy.as_ptr());
            let nz = _mm_loadu_ps(cz.as_ptr());
            let signmask = _mm_set1_ps(-0.0);
            // 退化判定：length_squared < 0.5（结合序同 Vec3::dot）
            let ls = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(nx, nx), _mm_mul_ps(ny, ny)),
                _mm_mul_ps(nz, nz),
            );
            degen = _mm_movemask_ps(_mm_cmplt_ps(ls, _mm_set1_ps(0.5)));
            // lane 版「标量向量 · 4 条轴」：v.x·nx + v.y·ny + v.z·nz（同左结合序）
            #[inline(always)]
            unsafe fn dot_lane(v: Vec3, nx: __m128, ny: __m128, nz: __m128) -> __m128 {
                _mm_add_ps(
                    _mm_add_ps(
                        _mm_mul_ps(_mm_set1_ps(v.x), nx),
                        _mm_mul_ps(_mm_set1_ps(v.y), ny),
                    ),
                    _mm_mul_ps(_mm_set1_ps(v.z), nz),
                )
            }
            let absf = |v: __m128| _mm_andnot_ps(signmask, v);
            let ra = _mm_add_ps(
                _mm_add_ps(
                    _mm_mul_ps(_mm_set1_ps(ha.x), absf(dot_lane(aa[0], nx, ny, nz))),
                    _mm_mul_ps(_mm_set1_ps(ha.y), absf(dot_lane(aa[1], nx, ny, nz))),
                ),
                _mm_mul_ps(_mm_set1_ps(ha.z), absf(dot_lane(aa[2], nx, ny, nz))),
            );
            let rb = _mm_add_ps(
                _mm_add_ps(
                    _mm_mul_ps(_mm_set1_ps(hb.x), absf(dot_lane(ab[0], nx, ny, nz))),
                    _mm_mul_ps(_mm_set1_ps(hb.y), absf(dot_lane(ab[1], nx, ny, nz))),
                ),
                _mm_mul_ps(_mm_set1_ps(hb.z), absf(dot_lane(ab[2], nx, ny, nz))),
            );
            let ca = dot_lane(pa, nx, ny, nz);
            let cb = dot_lane(pb, nx, ny, nz);
            let sep1 = _mm_sub_ps(_mm_sub_ps(cb, rb), _mm_add_ps(ca, ra)); // min_b − max_a
            let sep2 = _mm_sub_ps(_mm_sub_ps(ca, ra), _mm_add_ps(cb, rb)); // min_a − max_b
                                                                           // **同一谓词 + 位选择**（审计 F-12）：标量是 `sep1 >= sep2 ? sep1 : sep2`，
                                                                           // 这里让**同一个** `ge` 既定 `flags`（法线方向）又定 `sep`（值）。旧写法
                                                                           // `_mm_max_ps(sep2, sep1)` 在 NaN 时按 Intel 语义返回第二操作数，而标量的 `>=`
                                                                           // 对 NaN 判假会取 `sep2` ⇒ 两条路径各有各的算术 ⇒ 分类可分叉。平局（含带符号零）由
                                                                           // 同一个 `>=` 定夺 ⇒ 与标量逐位同；判据见 tests 的平局与极值两条（后者是**扩覆盖**）。
            let ge = _mm_cmpge_ps(sep1, sep2);
            flags = _mm_movemask_ps(ge) ^ 0b1111;
            let sep = _mm_or_ps(_mm_and_ps(ge, sep1), _mm_andnot_ps(ge, sep2));
            _mm_storeu_ps(sep_arr.as_mut_ptr(), sep);
        }
        // 标量侧：按轴序复刻三条规则（退化跳过 / 早退 / 严格择优）。
        for k in 0..m {
            if degen & (1 << k) != 0 {
                continue;
            }
            let s = sep_arr[k];
            if s > skin {
                return None;
            }
            if s > best {
                best = s;
                best_idx = i + k;
                let n0 = axes[i + k];
                best_n = if flags & (1 << k) != 0 { -n0 } else { n0 };
            }
        }
        i += m;
    }
    if best == f32::MIN {
        None
    } else {
        Some((best, best_n, best_idx))
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests;
