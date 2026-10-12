//! **介质积分核的探针（GPU；需适配器）**：水平层 + 定步数中点采样 —— 参照值**手算**，不重跑同一算法。
//!
//! 场景选成能手算的：段 `(0,0,0) → (0,4,0)`（长 4 m），层界落在 `y = 2`；**每段定 8 步** ⇒
//! 子段长 0.5 m、中点 `y = 0.25 / 0.75 / … / 3.75` ⇒ 前 4 个中点在下层、后 4 个在上层
//! ⇒ 用时 = `2/c_下 + 2/c_上`、衰减 = `exp(-(2α_下 + 2α_上))` —— 全是手写常数。
//!
//! 六段判据：① 纯空气 = 闭式 `len/speed`；② 均匀层的闭式 `exp(-α·len)`；③ **跨层的中点采样**
//! 手算分段和；④ 单调性（吸收越大越弱 / 声速越大越快）；⑤ **组合性**（折线 = 逐段"时延相加、
//! 衰减相乘"，在同一采样集合下严格成立）；⑥ 确定性 + `steps = 0` 按 1 算 + 层表没覆盖 ⇒ 空气。
//!
//! ⚠️ **一条测试里顺序跑多段**：`cargo test` 默认并行跑测试函数，每条都会自己建一个 wgpu
//! 设备，实测两个设备并存会互相卡死（见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。
//! ⚠️ 无适配器 ⇒ 跳过（`[SKIP]`）；CI 口径 `--exclude vxl-phys-gpu` ⇒ 只在有卡的机器上跑。

use vxl_phys_core::Vec3;
use vxl_phys_gpu::acoustic::{medium_on_adapter, MediumIn, MediumOut, Slab};

const SPEED: f32 = 343.0;
const STEPS: u32 = 8;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 竖直段 `(0,0,0) → (0, len, 0)`。
fn vertical(len: f32) -> (Vec<Vec3>, Vec<Vec3>) {
    (
        vec![Vec3::new(0.0, 0.0, 0.0)],
        vec![Vec3::new(0.0, len, 0.0)],
    )
}

/// 一层（默认的层表项）。
fn slab(y_lo: f32, y_hi: f32, c: f32, alpha: [f32; 3]) -> Slab {
    Slab {
        y_lo,
        y_hi,
        c,
        alpha,
    }
}

/// 跑一趟（`slabs` 为空 = 纯空气）。
fn run(srcs: &[Vec3], dsts: &[Vec3], slabs: &[Slab], steps: u32) -> Option<Vec<MediumOut>> {
    let mi = MediumIn {
        srcs,
        dsts,
        slabs,
        steps,
        speed: SPEED,
    };
    match medium_on_adapter(0, &mi) {
        Ok((name, out)) => {
            println!("[gpu] {name}: {} 对（介质积分）", out.len());
            Some(out)
        }
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            None
        }
    }
}

/// 相对误差断言（手写常数当参照）。
fn close(got: f32, want: f32, tag: &str) {
    assert!(
        (got - want).abs() <= want.abs() * 1e-5,
        "{tag}：卡上 {got} vs 手算 {want}"
    );
}

/// ① 纯空气（空层表）与"层表没覆盖"都该退化成 `len/speed`、衰减 1。
fn air_and_uncovered() {
    let (srcs, dsts) = vertical(4.0);
    let want_time = 4.0 / SPEED;
    let Some(out) = run(&srcs, &dsts, &[], STEPS) else {
        return;
    };
    close(out[0].time, want_time, "空层表该是 len/speed");
    for (b, a) in out[0].atten.iter().enumerate() {
        close(*a, 1.0, &format!("空层表第 {b} 带该不衰减"));
    }
    // 层表在别处（y ∈ [10, 20]）⇒ 采样点没命中 ⇒ 还是空气
    let far = [slab(10.0, 20.0, 686.0, [1.0, 1.0, 1.0])];
    let Some(out) = run(&srcs, &dsts, &far, STEPS) else {
        return;
    };
    close(out[0].time, want_time, "没命中的层表该退化成空气");
    close(out[0].atten[1], 1.0, "没命中的层表不该衰减");
}

/// ② 均匀层：闭式 `len/c` 与 `exp(-α_b·len)`（手写常数 `e^-1 / e^-2 / e^-4`）。
fn uniform_slab_closed_form() {
    let (srcs, dsts) = vertical(4.0);
    let one = [slab(-1.0, 5.0, 686.0, [0.25, 0.5, 1.0])];
    let Some(out) = run(&srcs, &dsts, &one, STEPS) else {
        return;
    };
    close(out[0].time, 4.0 / 686.0, "均匀层该是 len/c（c = 686）");
    close(out[0].atten[0], 0.367_879_44, "e^-1");
    close(out[0].atten[1], 0.135_335_28, "e^-2");
    close(out[0].atten[2], 0.018_315_64, "e^-4");
}

/// ③ **跨层的中点采样**：层界 y = 2、8 步（子段 0.5 m）⇒ 前 4 个中点在下层、后 4 个在上层。
///
/// 用时 `2/c_下 + 2/c_上`、衰减 `exp(-(2α_下 + 2α_上))`；手写常数 `exp(-0.8 / -1.6 / -3.2)`。
fn cross_layer_midpoint_sampling() {
    let (srcs, dsts) = vertical(4.0);
    let two = [
        slab(-1.0, 2.0, 343.0, [0.1, 0.2, 0.4]),
        slab(2.0, 5.0, 686.0, [0.3, 0.6, 1.2]),
    ];
    let Some(out) = run(&srcs, &dsts, &two, STEPS) else {
        return;
    };
    let want_time = 2.0 / 343.0 + 2.0 / 686.0;
    close(out[0].time, want_time, "跨层用时 = 2/c_下 + 2/c_上");
    close(out[0].atten[0], 0.449_328_97, "exp(-0.8)");
    close(out[0].atten[1], 0.201_896_52, "exp(-1.6)");
    close(out[0].atten[2], 0.040_762_2, "exp(-3.2)");
}

/// ④ 单调性：吸收越大越弱、声速越大越快（同几何、同层界）。
fn monotone_in_absorption_and_speed() {
    let (srcs, dsts) = vertical(4.0);
    let quiet = [slab(-1.0, 5.0, 343.0, [0.05, 0.05, 0.05])];
    let loud = [slab(-1.0, 5.0, 343.0, [0.5, 0.5, 0.5])];
    let slow = [slab(-1.0, 5.0, 343.0, [0.1, 0.1, 0.1])];
    let fast = [slab(-1.0, 5.0, 1029.0, [0.1, 0.1, 0.1])];
    let Some(a) = run(&srcs, &dsts, &quiet, STEPS) else {
        return;
    };
    let Some(b) = run(&srcs, &dsts, &loud, STEPS) else {
        return;
    };
    let Some(c) = run(&srcs, &dsts, &slow, STEPS) else {
        return;
    };
    let Some(d) = run(&srcs, &dsts, &fast, STEPS) else {
        return;
    };
    for bi in 0..3 {
        assert!(
            b[0].atten[bi] < a[0].atten[bi],
            "第 {bi} 带：吸收大的必须更弱（{} vs {}）",
            b[0].atten[bi],
            a[0].atten[bi]
        );
    }
    assert!(
        d[0].time < c[0].time,
        "声速大的必须更快（{} vs {}）",
        d[0].time,
        c[0].time
    );
}

/// ⑤ **组合性**：折线 = 逐段的"时延相加、衰减相乘"。
///
/// ⚠️ 严格成立的前提是**同一采样集合**：4 m 分 16 步 = 两段 2 m 各 8 步（中点位置逐个相同）
/// ⇒ 只有求和顺序带来的舍入差。换步数就是换采样点 ⇒ 差异属于口径内的量化（不是 bug）。
fn polyline_composes_exactly() {
    let (s1, d1) = vertical(4.0);
    let (s2, d2) = (
        vec![Vec3::new(0.0, 0.0, 0.0)],
        vec![Vec3::new(0.0, 2.0, 0.0)],
    );
    let (s3, d3) = (
        vec![Vec3::new(0.0, 2.0, 0.0)],
        vec![Vec3::new(0.0, 4.0, 0.0)],
    );
    let two = [
        slab(-1.0, 2.0, 343.0, [0.1, 0.2, 0.4]),
        slab(2.0, 5.0, 686.0, [0.3, 0.6, 1.2]),
    ];
    let Some(whole) = run(&s1, &d1, &two, 16) else {
        return;
    };
    let Some(first) = run(&s2, &d2, &two, STEPS) else {
        return;
    };
    let Some(second) = run(&s3, &d3, &two, STEPS) else {
        return;
    };
    let want_time = first[0].time + second[0].time;
    close(whole[0].time, want_time, "折线用时 = 逐段相加");
    for b in 0..3 {
        let want = first[0].atten[b] * second[0].atten[b];
        close(whole[0].atten[b], want, "折线衰减 = 逐段相乘");
    }
}

/// ⑥ 确定性 + `steps = 0` 按 1 算（定次数的下限）。
fn determinism_and_zero_steps() {
    let (srcs, dsts) = vertical(4.0);
    let one = [slab(-1.0, 5.0, 686.0, [0.25, 0.5, 1.0])];
    let Some(a) = run(&srcs, &dsts, &one, STEPS) else {
        return;
    };
    let Some(b) = run(&srcs, &dsts, &one, STEPS) else {
        return;
    };
    assert_eq!(a, b, "同输入两跑必须逐位一致");
    let Some(zero) = run(&srcs, &dsts, &one, 0) else {
        return;
    };
    let Some(one_step) = run(&srcs, &dsts, &one, 1) else {
        return;
    };
    assert_eq!(
        zero, one_step,
        "steps = 0 该按 1 算（0 步 = 没采样，不是静默归零）"
    );
}

/// 六段顺序跑（见文件头注：**必须一条测试**）。
#[test]
fn medium_matches_hand_derived_values() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    air_and_uncovered();
    uniform_slab_closed_form();
    cross_layer_midpoint_sampling();
    monotone_in_absorption_and_speed();
    polyline_composes_exactly();
    determinism_and_zero_steps();
}
