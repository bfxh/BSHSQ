//! **根因锁**（`PLAN-gpu.md` §3.25）：卡上 `tangents` 的归一化必须零安全。
//!
//! 内建 `normalize` 对零长度给 NaN，而 `body_apply` 的
//! `j = n*dn + tf[0]*dt1 + tf[1]*dt2` 里 `NaN * 0.0` 仍是 NaN ⇒ 退化法向会把 NaN 灌进
//! 速度表，**哪怕这一趟的切向冲量是 0**。主机 `Vec3::normalize` 的回退是零向量
//! （`vxl-phys-core/src/math.rs`），卡上必须同口径。
//!
//! 无适配器 ⇒ 跳过（与其它 GPU 判据同口径：CI 的口径是 `--exclude vxl-phys-gpu`）。
//! 两个用例放**同一条** `#[test]`：同一块卡上并发跑「建缓冲 + 提交 + 回读」会挂死。

use vxl_phys_core::solve_tier::{self as st, ContactRow, SolveTierBackend};
use vxl_phys_core::Vec3;
use vxl_phys_gpu::solve_tier::SolveTier;

/// 两个动体（1 kg、单位四元数、各轴逆惯量 0.1），相对速度沿 ±Y。
/// `bd` = `pos3 · quat4 · inv_mass · local_inv_i3`（与 `st::pack_bodies` 同序）。
fn two_bodies() -> (Vec<f32>, Vec<f32>) {
    let bd = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 1.0,
        1.0, 0.1, 0.1, 0.1,
    ];
    let vel = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0];
    (bd.to_vec(), vel.to_vec())
}

fn params() -> st::SolveTierParams {
    st::SolveTierParams {
        dt: 1.0 / 60.0,
        inv_dt: 60.0,
        erp_dyn: 0.2,
        erp_static: 0.2,
        cfm_dyn: 1.0,
        cfm_static: 1.0,
        slop: 0.005,
        max_corr: 0.5,
        e_threshold: 1.0,
        bias: 1.0,
        sweeps: 4,
    }
}

/// 一条接触（体 0 / 体 1，点在两体之间，`mu = 0.5`、`e = 0`、深度 0.01 = 已穿透）。
fn one_contact(n: Vec3) -> Vec<f32> {
    let mut ct = Vec::new();
    st::pack_contact(
        &mut ct,
        &ContactRow {
            a: 0,
            b: 1,
            point: Vec3::new(0.0, 0.01, 0.0),
            normal: n,
            depth: 0.01,
            mu: 0.5,
            e: 0.0,
        },
    );
    ct
}

/// 参考轴选择规则与核内 `tangents` 逐字一致 ⇒ 用它判断本例是否**真的**退化。
fn cross_with_refr(n: Vec3) -> Vec3 {
    let refr = if n.y.abs() >= 0.9 { Vec3::X } else { Vec3::Y };
    n.cross(refr)
}

/// 一趟卡上求解，返回 `(速度表, 失败说明)`；空串 = 这趟跑通了。
/// 不把 `Result` 摊给调用方：判红一律走 `assert`，一个 `panic!` 都不留（todo 门数的是宏）。
fn run(n: Vec3) -> (Vec<f32>, String) {
    let (bd, vel) = two_bodies();
    let ct = one_contact(n);
    let (off, items) = st::build_csr(&ct, 2);
    let tier = match SolveTier::new(0, 64, 64) {
        Ok(t) => t,
        Err(e) => return (Vec::new(), format!("建档失败：{e}")),
    };
    match tier.solve_run(&st::SolveTierIn {
        bd: &bd,
        vel: &vel,
        ct: &ct,
        off: &off,
        items: &items,
        params: params(),
    }) {
        Ok(out) => (out, String::new()),
        Err(e) => (Vec::new(), e),
    }
}

#[test]
fn degenerate_normal_never_writes_nan() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    let (_, vel) = two_bodies();
    // ① 零法向：没有可推的方向 ⇒ 速度表必须**逐字不动**（NaN 一进表立刻红）。
    let zero = Vec3::ZERO;
    assert!(
        cross_with_refr(zero).length() <= 1e-12,
        "零法向：用例没走到退化支 ⇒ 判据空过"
    );
    let (out, err) = run(zero);
    assert!(err.is_empty(), "零法向：整趟作废 ⇒ {err}");
    assert_eq!(out, vel, "零法向：卡上不该推出任何速度（逐字相同是硬口径）");
    // ② 短法向 `(0, 0.5, 0)`：`|n.y| < 0.9` ⇒ 参考轴选 Y ⇒ `cross = 0`，同样退化，但法向
    //    仍有方向 ⇒ 必须算出**有限**速度，且确实把两个体顶开。
    let short = Vec3::new(0.0, 0.5, 0.0);
    assert!(
        cross_with_refr(short).length() <= 1e-12,
        "短法向：用例没走到退化支 ⇒ 判据空过"
    );
    let (out, err) = run(short);
    assert!(err.is_empty(), "短法向：整趟作废 ⇒ {err}");
    let bad = out.iter().position(|v| !v.is_finite());
    assert!(bad.is_none(), "短法向：速度表第 {bad:?} 字非有限");
    assert_ne!(
        out, vel,
        "短法向：沿 +Y 的相对速度该被顶开 ⇒ 表却一个字没变"
    );
}
