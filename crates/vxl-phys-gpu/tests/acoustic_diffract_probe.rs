//! **绕射核的探针（GPU；需适配器）**：一把"刀口"（屏障顶上的棱）—— 参照值是**手算的几何**，
//! 与卡上那 48 次二分**不是同一份实现**。
//!
//! 场景：屏障在 `z = 1`、`x ∈ [-1,1]`、`y ∈ [0,1]`，顶部一条棱 `(-1,1,1)→(1,1,1)`；声源与听者
//! 分居两侧、连线被屏障挡死（阴影里）。这个几何**对称** ⇒ Fermat 点落在棱的中点 `(0,1,1)`
//! （`f' = cos∠(dir,P−src) + cos∠(dir,P−dst)` 在 `x = 0` 处两项等值反向 ⇒ 驻点，且在棱内部
//! ⇒ 就是最小值），折线长 `= 2·√4.5`、`cosθ = 4/4.5` ⇒ 时延与增益都有闭式。
//! 另用**沿棱暴力扫 20001 点**交叉验证"这确实是最小值"（与 CPU 的 `tests/a4_diffraction.rs`
//! 同路子，但那边是被测实现自己的测试，这里是卡上内核的独立参照）。
//!
//! ⚠️ **一条测试里顺序跑多段**：`cargo test` 默认并行跑测试函数，每条都会自己建一个 wgpu
//! 设备，实测两个设备并存会互相卡死（见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。分段就是
//! 为了让每个函数留在 god 门的 120 行以内。
//! ⚠️ 无适配器 ⇒ 跳过（打印 `[SKIP]`）；CI 口径 `cargo test --workspace --exclude vxl-phys-gpu`
//! ⇒ 这条只在有卡的机器上跑。

use vxl_phys_core::Vec3;
use vxl_phys_gpu::acoustic::{diffracted_on_adapter, DiffractOut, SceneIn};

const SPEED: f32 = 343.0;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 屏障：`z = 1`、`x ∈ [-1,1]`、`y ∈ [0,1]`（两块三角；顶边就是那条衍射棱）。
fn barrier() -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
        ],
        [
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ],
    ]
}

/// 顶棱（屏障的顶边）。
fn top_edge() -> Vec<[Vec3; 2]> {
    vec![[Vec3::new(-1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0)]]
}

/// 竖棱（`x = 1`、`z = 1`、`y ∈ [0,1]`）—— 第 4 段用它当"更弱的那条"。
fn weak_edge() -> [Vec3; 2] {
    [Vec3::new(1.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 1.0)]
}

/// 挡板：`z = 2`、`x ∈ [-2,2]`、`y ∈ [0,3]`（两块三角）—— 第 5 段用它挡死绕射的第二条腿。
fn wall() -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-2.0, 0.0, 2.0),
            Vec3::new(2.0, 0.0, 2.0),
            Vec3::new(2.0, 3.0, 2.0),
        ],
        [
            Vec3::new(-2.0, 0.0, 2.0),
            Vec3::new(2.0, 3.0, 2.0),
            Vec3::new(-2.0, 3.0, 2.0),
        ],
    ]
}

/// **独立参照**：沿棱暴力扫 `20001` 点求 `min(|src−P| + |P−dst|)`（与卡上的二分不是同一份实现）。
fn brute_force_min(a: Vec3, b: Vec3, src: Vec3, dst: Vec3) -> f32 {
    const N: usize = 20001;
    let mut best = f32::INFINITY;
    for i in 0..=N {
        let u = i as f32 / N as f32;
        let p = a + (b - a) * u;
        let len = (p - src).length() + (p - dst).length();
        if len < best {
            best = len;
        }
    }
    best
}

/// **手算参照**：棱上的点**由几何推出来**（对称 ⇒ 驻点在棱的中点），再按口径算 `(时延, 增益)`。
///
/// 口径（`coeff · D(θ) / (d₁·d₂)`，`D = (1+cosθ)/2`）与内核同式，但**棱上的点不吃卡上那 48 次
/// 二分** —— 所以这条对拍钉的是"卡上找对了那个点"，不是"同一份公式抄了两遍"。
fn hand_at(p: Vec3, src: Vec3, dst: Vec3, coeff: f32) -> (f32, f32) {
    let v1 = p - src;
    let v2 = dst - p;
    let (d1, d2) = (v1.length(), v2.length());
    let cos_t = (v1.dot(v2) / (d1 * d2)).clamp(-1.0, 1.0);
    ((d1 + d2) / SPEED, coeff * 0.5 * (1.0 + cos_t) / (d1 * d2))
}

/// 跑一趟：核起不来（设备/驱动问题）⇒ 打 `[SKIP]` 并给 `None`（与既有两条探针同款）。
fn try_run(sc: &SceneIn<'_>) -> Option<(String, Vec<DiffractOut>)> {
    match diffracted_on_adapter(0, sc) {
        Ok(v) => Some(v),
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            None
        }
    }
}

/// 第 1–3 段：阴影里、单棱 ⇒ 闭式对拍 + 同输入两跑逐位一致 + 暴力扫棱确认是最小值。
fn shadow_stages(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let (s, d) = (srcs[0], dsts[0]);
    let foot = Vec3::new(0.0, 1.0, 1.0);
    let (want_delay, want_gain) = hand_at(foot, s, d, 1.0);
    let coeff = [1.0f32];
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb: &[],
        scatter: &[],
        edges: top,
        edge_coeff: &coeff,
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（刀口绕射）", got.len());
    assert_eq!(got.len(), 1, "一对声源/听者 ⇒ 一行读数");
    let r = got[0];
    assert!(r.hit, "屏障后方必须给出绕射路径（拐角后仍可闻）");
    assert_eq!(
        r.edge, 0.0,
        "只有一条棱 ⇒ 棱序号必须是 0（实际 {}）",
        r.edge
    );
    assert!(
        (r.delay - want_delay).abs() <= want_delay * 1e-4,
        "时延（卡上 {} vs 手算 {want_delay}）",
        r.delay
    );
    assert!(
        (r.gain - want_gain).abs() <= want_gain * 1e-4,
        "增益（卡上 {} vs 手算 {want_gain}）",
        r.gain
    );
    let Some((_, again)) = try_run(&sc) else {
        return;
    };
    assert_eq!(again, got, "同输入两跑必须逐位一致");
    let brute = brute_force_min(top[0][0], top[0][1], s, d);
    let card = r.delay * SPEED;
    assert!(
        (card - brute).abs() <= brute * 1e-4,
        "折线长应为棱上的最小值（卡上 {card}，暴力扫 {brute}）"
    );
    assert!(
        (brute - (s - foot).length() - (d - foot).length()).abs() <= brute * 1e-4,
        "暴力扫的最小值应当就在手算那个点上（暴力扫 {brute}）"
    );
}

/// 第 4 段：两条棱、**弱的那条排在前面** ⇒ 选的必须是最强那条（棱序号 1）。
fn strongest_edge_stage(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let (s, d) = (srcs[0], dsts[0]);
    let (_, want_gain) = hand_at(Vec3::new(0.0, 1.0, 1.0), s, d, 1.0);
    let weak_gain = hand_at(Vec3::new(1.0, 0.5, 1.0), s, d, 0.25).1;
    assert!(
        weak_gain < want_gain * 0.5,
        "这条判据要有意义：弱棱必须真的弱（{weak_gain} vs {want_gain}）"
    );
    let both = vec![weak_edge(), top[0]];
    let coeff = vec![0.25f32, 1.0];
    let sc = SceneIn {
        srcs,
        dsts,
        tris,
        absorb: &[],
        scatter: &[],
        edges: &both,
        edge_coeff: &coeff,
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（两条棱）", got.len());
    assert!(got[0].hit, "两条棱的场景也该有绕射");
    assert_eq!(got[0].edge, 1.0, "必须取最强那条（实际 {}）", got[0].edge);
    assert!(
        (got[0].gain - want_gain).abs() <= want_gain * 1e-4,
        "取到的那条就是顶棱 ⇒ 增益应等于单棱时的值（{} vs {want_gain}）",
        got[0].gain
    );
}

/// 第 5–6 段：两条**否证** —— 绕射的第二条腿被挡死 ⇒ 不产出；听者抬到屏障顶以上（直达通视）
/// ⇒ 也不产出（不与直达重复计能）。
fn blocked_and_lit_stages(tris: &[[Vec3; 3]], top: &[[Vec3; 2]], srcs: &[Vec3], dsts: &[Vec3]) {
    let coeff = [1.0f32];
    let mut blocked = tris.to_vec();
    blocked.extend(wall());
    let sc = SceneIn {
        srcs,
        dsts,
        tris: &blocked,
        absorb: &[],
        scatter: &[],
        edges: top,
        edge_coeff: &coeff,
        speed: SPEED,
    };
    let Some((name, dead)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（第二腿被挡）", dead.len());
    assert!(
        !dead[0].hit,
        "绕射的腿被挡死 ⇒ 不得产出绕射路径（实际 {:?}）",
        dead[0]
    );
    assert_eq!(dead[0].edge, -1.0, "没产出时棱序号必须是 -1");
    let high = vec![Vec3::new(0.5, 2.5, 3.0)];
    let sc = SceneIn {
        srcs,
        dsts: &high,
        tris,
        absorb: &[],
        scatter: &[],
        edges: top,
        edge_coeff: &coeff,
        speed: SPEED,
    };
    let Some((name, lit)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（阴影外）", lit.len());
    assert!(
        !lit[0].hit && lit[0].gain == 0.0,
        "不在阴影里就不该产出绕射（实际 {:?}）",
        lit[0]
    );
}

/// 六段顺序跑（见文件头注：**必须一条测试**）。
#[test]
fn diffraction_matches_hand_geometry_and_respects_shadow_rules() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    let tris = barrier();
    let top = top_edge();
    let srcs = vec![Vec3::new(-0.5, 0.5, -1.0)];
    let dsts = vec![Vec3::new(0.5, 0.5, 3.0)];
    shadow_stages(&tris, &top, &srcs, &dsts);
    strongest_edge_stage(&tris, &top, &srcs, &dsts);
    blocked_and_lit_stages(&tris, &top, &srcs, &dsts);
    // 两张表给短了必须**报错**（核内是逐棱直接索引 ⇒ 不能把越界读留给设备）
    let bad = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &[],
        edges: &top,
        edge_coeff: &[],
        speed: SPEED,
    };
    assert!(
        diffracted_on_adapter(0, &bad).is_err(),
        "棱表 1 条、系数空 ⇒ 必须报错（不给设备越界读的机会）"
    );
}
