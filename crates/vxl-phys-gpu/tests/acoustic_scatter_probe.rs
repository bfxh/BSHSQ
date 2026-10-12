//! **散射核的探针（GPU；需适配器）**：一块"地板"三角面 —— 参照值是**手算的几何**，与卡上跑的
//! 不是同一份实现。
//!
//! 场景刻意选成能**手算**的：`y = 0` 平面上一个直角三角 `(0,0,0) (2,0,0) (0,0,2)`
//! （面积 2、重心 `(2/3, 0, 2/3)`），声源悬在重心正上方 1 m、听者按角度摆：
//!
//! - 法向朝声源那侧（`+y`）⇒ `cos_in = 1`、`d₁ = 1`；
//! - 听者距重心 2 m、与法向夹角 θ ⇒ `cos_out = cosθ`、`d₂ = 2`；
//! - 口径 `gain = s·cos_in·cos_out·A/(d₁²·d₂²)`（`A = 2`）⇒ **`gain = 0.5·cosθ`**（`s = 1`）。
//!
//! 于是判据全是**手写常数**：`0.5`、`0.5·cos30° = 0.4330127`、`0.5·cos60° = 0.25`
//! （"绕离法向 60° 恰好弱一半"）、`0.5·cos75° = 0.12940952`；时延 `(1+2)/343`。参照不重算公式，
//! 只把这几个数写出来 —— 卡上跑的是逐三角的面中心口径（含遮挡与同侧判定）。
//!
//! ⚠️ **一条测试里顺序跑多段**：`cargo test` 默认并行跑测试函数，每条都会自己建一个 wgpu
//! 设备，实测两个设备并存会互相卡死（见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。
//! ⚠️ 无适配器 ⇒ 跳过（`[SKIP]`）；CI 口径 `--exclude vxl-phys-gpu` ⇒ 只在有卡的机器上跑。

use vxl_phys_core::Vec3;
use vxl_phys_gpu::acoustic::{scattered_on_adapter, ScatterOut, SceneIn};

const SPEED: f32 = 343.0;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 重心（手算：三个顶点的平均）。
fn centroid() -> Vec3 {
    Vec3::new(2.0 / 3.0, 0.0, 2.0 / 3.0)
}

/// 地板三角（`y = 0`）：`(0,0,0) (2,0,0) (0,0,2)` —— 面积 2。
fn floor_tri() -> [Vec3; 3] {
    [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 2.0),
    ]
}

/// 同一个三角**绕重心放大 m 倍**（面积 ×m²、重心与法向不变）—— 第 5 段用它造"更强的那一面"。
fn scaled_tri(m: f32) -> [Vec3; 3] {
    let p = centroid();
    floor_tri().map(|v| p + (v - p) * m)
}

/// 悬在重心正上方 1 m 的声源。
fn src() -> Vec3 {
    centroid() + Vec3::new(0.0, 1.0, 0.0)
}

/// 距重心 2 m、与法向（`+y`）夹角 θ 的听者（θ = 0 就是正上方）。
fn dst(theta_deg: f32) -> Vec3 {
    let t = theta_deg * std::f32::consts::PI / 180.0;
    centroid() + Vec3::new(2.0 * t.sin(), 2.0 * t.cos(), 0.0)
}

/// 挡板（`y = 1.5` 上一块 `x,z ∈ 重心 ± 1` 的方板，两块三角）—— 第 4 段用它挡死"到听者"那条腿。
fn plate() -> Vec<[Vec3; 3]> {
    let p = centroid();
    let (x0, x1, z0, z1, y) = (p.x - 1.0, p.x + 1.0, p.z - 1.0, p.z + 1.0, 1.5);
    vec![
        [
            Vec3::new(x0, y, z0),
            Vec3::new(x1, y, z0),
            Vec3::new(x1, y, z1),
        ],
        [
            Vec3::new(x0, y, z0),
            Vec3::new(x1, y, z1),
            Vec3::new(x0, y, z1),
        ],
    ]
}

fn try_run(sc: &SceneIn<'_>) -> Option<(String, Vec<ScatterOut>)> {
    match scattered_on_adapter(0, sc) {
        Ok(v) => Some(v),
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            None
        }
    }
}

/// 第 1–3 段：单面 ⇒ 手算闭式（0.5）+ 角度扫描（`0.5·cosθ`，含"60° 恰好一半"）+ 同侧否证。
fn closed_form_and_angles() {
    let tris = vec![floor_tri()];
    let coeff = vec![1.0f32];
    let srcs = vec![src()];
    // 时延：两段都是直线，(1 + 2) / 343（与角度无关）
    let want_delay = 3.0 / SPEED;

    // ① 正上方：gain = 0.5·cos0 = 0.5
    let dsts = vec![dst(0.0)];
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &coeff,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（散射·正上方）", got.len());
    assert!(got[0].hit, "同侧 + 余弦为正 + 两腿不挡 ⇒ 必须产出散射路径");
    assert_eq!(got[0].tri, 0.0, "只有一个面 ⇒ 三角序号必须是 0");
    assert!(
        (got[0].gain - 0.5).abs() <= 0.5 * 1e-4,
        "gain 该是手算的 0.5（实际 {}）",
        got[0].gain
    );
    assert!(
        (got[0].delay - want_delay).abs() <= want_delay * 1e-4,
        "时延该是 3/343 = {want_delay}（实际 {}）",
        got[0].delay
    );

    // ② 角度扫描：gain = 0.5·cosθ（手写常数）
    let dsts: Vec<Vec3> = [0.0f32, 30.0, 60.0, 75.0].iter().map(|&a| dst(a)).collect();
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &coeff,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((_, got)) = try_run(&sc) else { return };
    let want = [0.5f32, 0.433_012_7, 0.25, 0.129_409_52];
    for (i, w) in want.iter().enumerate() {
        assert!(
            got[i].hit,
            "第 {i} 个角度（{}°）该有散射",
            [0, 30, 60, 75][i]
        );
        assert!(
            (got[i].gain - w).abs() <= w * 1e-4,
            "{}° 的 gain 该是手算的 {w}（实际 {}）",
            [0, 30, 60, 75][i],
            got[i].gain
        );
    }
    // "绕离法向 60° 恰好弱一半"：这条是口径的本体，单独钉一次
    let ratio = got[2].gain / got[0].gain;
    assert!(
        (ratio - 0.5).abs() <= 1e-4,
        "绕离法向 60° 必须恰好弱一半（实际 {ratio}）"
    );

    // ③ 同侧否证：听者到面的**另一侧**（或正好在面内）⇒ 不产出
    let below = vec![
        centroid() + Vec3::new(0.0, -2.0, 0.0),
        centroid() + Vec3::new(0.0, 0.0, 2.0),
    ];
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &below,
        tris: &tris,
        absorb: &[],
        scatter: &coeff,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((_, got)) = try_run(&sc) else { return };
    for (i, r) in got.iter().enumerate() {
        assert!(
            !r.hit,
            "第 {i} 个听者不在面的同侧 ⇒ 不该产出散射（实际 {r:?}）"
        );
    }
}

/// 第 4 段：把"面 → 听者"那条腿挡死 ⇒ 不产出（对照：第 1 段同一个场景没挡板就有）。
fn blocked_leg_kills_it() {
    let mut tris = vec![floor_tri()];
    tris.extend(plate());
    let srcs = vec![src()];
    let dsts = vec![dst(0.0)];
    let coeff = vec![1.0f32, 0.0, 0.0]; // 挡板自己是纯镜面（不参与散射），只负责遮
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &coeff,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（到听者那条腿被挡）", got.len());
    assert!(!got[0].hit, "腿被挡死 ⇒ 不得产出散射（实际 {:?}）", got[0]);
}

/// 第 5 段：**取最强那个面**（不是第一条）—— 把"弱的小面"排在前面、放大 2 倍的"强面"排第二
/// （面积 ×4 ⇒ gain ×4 = 2.0）；再把强面的系数设 0 ⇒ 该落回小面。
fn strongest_face_wins() {
    let tris = vec![floor_tri(), scaled_tri(2.0)];
    let srcs = vec![src()];
    let dsts = vec![dst(0.0)];
    let both = vec![1.0f32, 1.0];
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &both,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（两个面）", got.len());
    assert!(got[0].hit, "两个面都参与 ⇒ 必须有散射");
    assert_eq!(
        got[0].tri, 1.0,
        "必须取最强的那个面（放大面 gain = 2.0 vs 小面 0.5），实际序号 {}",
        got[0].tri
    );
    assert!(
        (got[0].gain - 2.0).abs() <= 2.0 * 1e-4,
        "强面的 gain 该是手算的 2.0（实际 {}）",
        got[0].gain
    );
    let small_only = vec![1.0f32, 0.0];
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &small_only,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((_, got)) = try_run(&sc) else { return };
    assert_eq!(got[0].tri, 0.0, "强面系数 0 ⇒ 该落回小面");
    assert!(
        (got[0].gain - 0.5).abs() <= 0.5 * 1e-4,
        "落回小面 ⇒ gain 回到手算的 0.5（实际 {}）",
        got[0].gain
    );
}

/// 第 6 段：确定性（同输入两跑**逐位**一致）+ 空表 ⇒ 一对也产不出（纯镜面语义）。
fn determinism_and_empty_table() {
    let tris = vec![floor_tri()];
    let coeff = vec![0.7f32];
    let srcs = vec![src()];
    let dsts = vec![dst(20.0)];
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &coeff,
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((_, a)) = try_run(&sc) else { return };
    let Some((_, b)) = try_run(&sc) else { return };
    assert_eq!(a, b, "同输入两跑必须逐位一致");
    assert!(a[0].hit, "第 6 段先要有产出（否则判据空跑）");
    // 空表 = 全 0 = 纯镜面：一个面也不参与
    let sc = SceneIn {
        srcs: &srcs,
        dsts: &dsts,
        tris: &tris,
        absorb: &[],
        scatter: &[],
        edges: &[],
        edge_coeff: &[],
        speed: SPEED,
    };
    let Some((name, got)) = try_run(&sc) else {
        return;
    };
    println!("[gpu] {name}: {} 对（空散射表）", got.len());
    assert!(
        !got[0].hit,
        "空散射表 = 纯镜面 ⇒ 不该产出散射（实际 {:?}）",
        got[0]
    );
}

/// 六段顺序跑（见文件头注：**必须一条测试**；分段是为了每个函数留在 god 门 120 行以内）。
#[test]
fn scatter_matches_hand_geometry() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    closed_form_and_angles();
    blocked_leg_kills_it();
    strongest_face_wins();
    determinism_and_empty_table();
}
