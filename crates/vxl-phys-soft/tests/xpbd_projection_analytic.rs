//! **XPBD 投影的解析对拍**（《LSSMJ-CICD…v5》§15.4 的 XPBD 耦合合同里，上一片只到
//! `partially` 的那两条：顺应度随 `h²` 的尺度律、`lambda` 每子步清零）。
//!
//! **为什么能精确测**：`Rope::substep` 是 predict → project → contacts → write_back 打包的，
//! 没有"只跑投影"的公开入口。但**位置读数**上的干扰项可以逐个消掉：
//! - `gravity = 0` + 初速 `0` ⇒ 预测步 `x += v·h` 是**零位移**；
//! - 无 provider 且 `radius < 0` ⇒ 接触段整段跳过；
//! - `write_back` 只改 `vel`、**不改 `pos`** ⇒ 对位置残差无影响。
//!   ⇒ `substeps = 1` 时，一次 `step` 对**位置**的效果就是**恰好一次约束投影**。
//!
//! **单约束**用 `Rope`（2 节点 = 1 条距离约束）。`Rope` 与布料共用同一套 XPBD 口径
//! ——`rope.rs` 头注：`α̃ = α/h²`、`Δλ = (−C − α̃·λ)/(w₁+w₂+α̃)`、`λ` **每子步清零**。
//!
//! **解析**（`λ₀ = 0` 起步的一次投影）：`C₁ = C₀ · α̃/(w + α̃)`，其中 `α̃ = α/h²`、`w = w₁ + w₂`。
//! 于是两条合同各有一条**精确**判据：
//! ① 换 `h` 后 `C₁` 仍等于解析式 ⇒ 钉住 `α̃` 的 `h²` 尺度（写成 `α/h` 会对不上）；
//! ② 连续两个 tick 的**残差比例相同** ⇒ 钉住 `λ` 每子步清零（累积会让第二个比例偏离）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::Rope;

/// 两节点绳：弦长 `chord`、总长 `total_len` ⇒ `rest_len = total_len`、初始应变 `C₀ = chord − rest`。
///
/// `Rope::span` 默认把**首尾钉住**（`inv_mass = 0`）⇒ 本测试要测约束本身，所以解钉成两根自由粒子。
fn two_node(chord: f32, total_len: f32, alpha: f32) -> Rope {
    let mut r = Rope::span(Vec3::ZERO, Vec3::new(chord, 0.0, 0.0), 2, total_len, -1.0);
    r.inv_mass[0] = 1.0;
    r.inv_mass[1] = 1.0;
    r.compliance = alpha;
    r.substeps = 1; // ⇒ h = dt ⇒ 一次投影
    r.chain_iterations = 1;
    r.radius = -1.0; // 关接触
    r
}

fn cur_len(r: &Rope) -> f32 {
    (r.pos[1] - r.pos[0]).length()
}

/// ① **顺应度随 `h²` 缩放**：一次投影后的残差 = `C₀·α̃/(w + α̃)`（`α̃ = α/h²`），换 `h` 仍对得上。
#[test]
fn single_projection_matches_analytic_alpha_over_h_squared() {
    let cases = [
        (1e-5f32, 1.0f32 / 60.0),
        (1e-5, 1.0 / 240.0), // h 缩小 4 倍 ⇒ α̃ 放大 16 倍
        (3e-4, 1.0 / 60.0),
        (1e-6, 1.0 / 30.0),
    ];
    for (alpha, dt) in cases {
        let mut r = two_node(1.2, 1.0, alpha);
        let w = r.inv_mass[0] + r.inv_mass[1];
        let c0 = cur_len(&r) - r.rest_len;
        r.step(dt, Vec3::ZERO, &NoProviders, 0, &[]);
        let c1 = cur_len(&r) - r.rest_len;
        let a_tilde = alpha / (dt * dt); // substeps = 1 ⇒ h = dt
        let want = c0 * a_tilde / (w + a_tilde);
        let rel = (c1 - want).abs() / want.abs();
        println!("α={alpha:.0e} dt={dt:.6}：C₁ 实测 {c1:.9}｜解析 {want:.9}｜相对差 {rel:.2e}");
        // 容差 1%：`C₁` 是**小量**（~1e-4），由两个 ~1.0 的长度相减而来 ⇒ f32 灾难性抵消会把
        // 相对误差放大到 ~1e-3（实测 8.4e-4 那一档就是它）。而"把 α/h² 写成 α/h"会让
        // 结果差**一个数量级以上**（ratio 0.0177 vs 3e-4）⇒ 1% 完全够分辨。
        assert!(
            rel < 1e-2,
            "一次投影与解析式不符（相对差 {rel:.2e}）⇒ 查 a_tilde 是否为 α/h²"
        );
    }
}

/// ② **`lambda` 每子步清零**：连续两个 tick（各一次投影）的**残差比例相同**。
///
/// 若 `λ` 跨子步累积，第二个 tick 的 `Δλ = (−C − α̃·λ)/(w + α̃)` 会因为 `λ ≠ 0` 而偏离同一个比例。
#[test]
fn lambda_resets_each_substep_so_projection_ratio_is_constant() {
    let (alpha, dt) = (1e-5f32, 1.0f32 / 60.0);
    let mut r = two_node(1.2, 1.0, alpha);
    let w = r.inv_mass[0] + r.inv_mass[1];
    let a_tilde = alpha / (dt * dt);
    let ratio = a_tilde / (w + a_tilde); // 解析：C_{k+1} = C_k · ratio
    let c0 = cur_len(&r) - r.rest_len;
    r.step(dt, Vec3::ZERO, &NoProviders, 0, &[]);
    let c1 = cur_len(&r) - r.rest_len;
    // ⚠️ **第二个 tick 前必须清 `vel`**：`write_back` 把第 1 次投影的位移换算成了速度
    // （`v = (x − x_prev)/h`），不清零的话第二个 tick 的**预测步**会带着它继续缩短 ⇒ 过冲、
    // 残差**变号**（实测比例 −0.96 而不是 +0.018）。清掉 `vel` 才剩"纯投影"。
    r.vel[0] = Vec3::ZERO;
    r.vel[1] = Vec3::ZERO;
    r.step(dt, Vec3::ZERO, &NoProviders, 0, &[]);
    let c2 = cur_len(&r) - r.rest_len;
    let (r1, r2) = (c1 / c0, c2 / c1);
    println!("残差比例：第 1 步 {r1:.9}｜第 2 步 {r2:.9}｜解析 {ratio:.9}");
    // 容差 1%（同上一条：`C` 是小量、减法抵消会放大相对误差）。反过来说，若 `λ` **真的**跨子步
    // 累积，第二个比例会偏 `~w/α̃ ≈ 55 倍`（量级差），1% 完全够分辨。
    assert!(
        (r1 - ratio).abs() / ratio < 1e-2,
        "第 1 步比例 {r1:.9} 与解析 {ratio:.9} 不符"
    );
    assert!(
        (r2 - ratio).abs() / ratio < 1e-2,
        "第 2 步比例 {r2:.9} 偏离解析 {ratio:.9} ⇒ lambda 没有每子步清零"
    );
}
