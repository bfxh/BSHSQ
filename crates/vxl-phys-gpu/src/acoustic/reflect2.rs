//! **M6：二阶镜面反射（两跳镜像展开）**的宿主入口 —— `声源 →(面 A) →(面 B) → 听者`。
//!
//! 单开一个入口（而不是把 M2 加宽）：卡上每个核各回"每对一行"，由宿主在合成时决定怎么用 ——
//! 这样 M2（[`super::reflected_on_adapter`]）的既有读数**逐位不变**，既有探针不用动。
//! 单开一个**文件**：`acoustic.rs` 已经贴着本仓 god 门（单文件 ≤ 800 行）的边。
//!
//! ⚠️ **卡上不做预算截断**（与 M1–M5 同口径：卡只负责"给定场景逐对取最强"，预算/阶数归宿主）
//! ⇒ 与 CPU `reflection_order = 2`（A8）**逐对一致的前提**是 CPU 侧 `max_reflections ≥
//! tris.len()`（不截断才同解）；CPU 的 `dropped_budget` 在卡上没有对应信号。每对 `O(n_tris³)`
//! ⇒ 只对离线、小场景有意义（见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。

use super::{run_entry, upload, SceneIn};
use vxl_phys_core::Vec3;

/// 一个 `(声源, 听者)` 对的**二阶（两跳）反射**读数（逐对取最强那条两跳链）。
///
/// 与 [`super::ReflectOut`] **分成两条**（不是把 M2 加宽）：卡上每个入口各产出"一行一列"，由宿主
/// 在合成时决定怎么用 —— 这样 M2 的既有读数**逐位不变**，既有探针不用动。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reflect2Out {
    /// 是否找到一条成立的两跳镜面反射。
    pub hit: bool,
    /// 时延（秒）＝ 三段腿（`s→p₁`、`p₁→p₂`、`p₂→l`）的总程长 `/ speed`。
    pub delay: f32,
    /// 能量增益 ＝ `(1-a₁)(1-a₂)/(总程长)²`（两跳的面各打折一次）。
    pub gain: f32,
    /// **第一跳**所在三角的序号（没有则为 `-1`）；第二跳的序号不在这行里。
    pub face: f32,
}

/// 在指定适配器上跑一趟**二阶反射核**（`acoustic.wgsl` 的 `reflect2` 入口）。
///
/// `absorb` 逐三角给（`0` = 全反射；`1` = 该面打折到 0 ⇒ 它自己抢不到"最强"那一位，
/// 于是拿它当**只遮不反**的挡板用）。逐对取**最强那条**两跳链，平局按 `(面 A, 面 B)` 序号
/// 取小 ⇒ 确定。口径与前提见本文件头注。
pub fn reflected2_on_adapter(
    adapter_index: usize,
    srcs: &[Vec3],
    dsts: &[Vec3],
    tris: &[[Vec3; 3]],
    absorb: &[f32],
    speed: f32,
) -> Result<(String, Vec<Reflect2Out>), String> {
    // 与 `reflected_on_adapter` 同款守卫：空表 = 全 0（由 `upload` 补零），给短了就是越界读。
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
    for row in run_entry(&device, &queue, &b, "reflect2") {
        out.push(Reflect2Out {
            hit: row[0] > 0.5,
            delay: row[1],
            gain: row[2],
            face: row[3],
        });
    }
    Ok((name, out))
}
