//! **V2 转换窗口判据**（`docs/PLAN-CONVERSION.md` §4 的 I0 / I5 / I6，外加 I3 效应键冲突）。
//!
//! 场景与 `tests/voxel_mesh_extract_rest.rs` 同源（体素地板 8×2×8、边长 0.5、顶面 y=1.0 +
//! 一只半高 0.5 的盒），这样两条判据量的是**同一条路径**上的不同侧面。
//!
//! - **I0 零代际**：默认关 ⇒ 请求被显式拒绝（`Err(Disabled)`，不是静默跳过）；**开档但不发事件**
//!   ⇒ 与关档逐位同哈希、不多建一个体。
//! - **I5 对拍**：提交前后的流形摘要差在阈值内（阈值与其实测锚在 `world_step/conversion.rs`
//!   的 `RECON_*` 常量；金丝雀也在那里——错位 0.5 m 的影子必红）。
//! - **I6 无穿透突变**：切换是本 tick 末的**位姿不跳**操作 ⇒ 盒的静置高度在切换前后不得跳变。
//! - **I3 效应键**：同一体素域同一 tick 被「挖格」与「转换」双消费 ⇒ 两个方向都 fail-loud。

use vxl_phys::{ConversionError, EffectKey, VoxelConversionExt, World};
use vxl_phys_core::{PhysConfig, Quat, Shape, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

/// 静置步数（与 V1 判据同量级：盒在该窗口内落定）。
const SETTLE: usize = 240;
/// 切换之后的观察窗（I6：不给瞬态留台阶，也不取单点当稳态）。
const POST: usize = 120;
/// I6 静置高度容差（m）。V1 的 I3 首测锚（同款地板）是 体素 1.4995 / 网格 1.4997
/// ⇒ 两表示本身的静置差 ≈ 2e-4 m；这里给 2 cm（≈100× 该锚），只拦"切换把盒弹起来/放下去"。
const REST_JUMP_TOL: f32 = 0.02;

fn floor() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

/// 建场：体素地板 + 落盒。返回 `(world, provider id, 盒的体索引)`。
fn scene(enabled: bool) -> (World, u32, usize) {
    let mut cfg = PhysConfig::default();
    cfg.conversion.enabled = enabled;
    let mut w = World::new(cfg);
    let marker = w.add_voxel(floor());
    let pid = w.provider_id_of(marker).unwrap_or(u32::MAX);
    assert!(pid != u32::MAX, "体素 marker 应带 provider id");
    let box_id = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.25, 1.6, 0.25),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    (w, pid, box_id)
}

fn filled(w: &World, pid: u32) -> usize {
    match w.providers().voxel(pid) {
        Some(v) => v.filled_count(),
        None => usize::MAX,
    }
}

#[test]
fn default_off_refuses_loudly_and_enabled_idle_is_bit_identical() {
    // 关档：请求必须**显式拒绝**（不静默跳过）。
    let (mut off, pid, _) = scene(false);
    for _ in 0..SETTLE {
        off.step();
    }
    let e = off.request_voxel_conversion(pid);
    assert!(
        matches!(e, Err(ConversionError::Disabled)),
        "默认关档的请求必须 fail-loud：{e:?}"
    );
    assert!(off.last_conversion().is_none(), "关档不得产生任何转换结果");

    // 开档但**不发事件**：状态与关档逐位同（开关本身零代际）。
    let (mut on_idle, _, _) = scene(true);
    for _ in 0..SETTLE {
        on_idle.step();
    }
    assert!(
        on_idle.last_conversion().is_none(),
        "无事件不得产生转换结果（pass 必须在空队列上短路）"
    );
    assert_eq!(
        on_idle.bodies.len(),
        2,
        "开档无事件不得建体（体素 marker + 盒 = 2）"
    );

    // 关档重跑一遍取哈希（两跑同配置 ⇒ 同时验证确定性）。
    let (mut off2, _, _) = scene(false);
    for _ in 0..SETTLE {
        off2.step();
    }
    assert_eq!(
        on_idle.state_hash(),
        off2.state_hash(),
        "开档无事件的状态哈希必须与关档逐位相同"
    );
}

#[test]
fn conversion_commits_and_keeps_rest_height() {
    let (mut w, pid, box_id) = scene(true);
    for _ in 0..SETTLE {
        w.step();
    }
    let (pos_before, _) = w.bodies.pose(box_id);
    let y_before = pos_before.y;
    let filled_before = filled(&w, pid);
    assert!(filled_before > 0, "转换前源域应有占据格");

    let req = w.request_voxel_conversion(pid);
    assert!(req.is_ok(), "开档请求应当登记：{req:?}");
    w.step(); // 转换在本 tick 末发生

    let (committed, src_pts, shd_pts, src_dep, shd_dep, dot, mesh, body) = match w.last_conversion()
    {
        Some(r) => (
            r.committed,
            r.source_points,
            r.shadow_points,
            r.source_max_depth,
            r.shadow_max_depth,
            r.min_normal_dot,
            r.mesh,
            r.body,
        ),
        None => (false, 0, 0, 0.0, 0.0, 0.0, None, None),
    };
    assert!(
        committed,
        "对拍应在阈内并提交（实测 {src_pts}/{shd_pts} 点、深度 {src_dep:.5}/{shd_dep:.5}、dot {dot:.5}）"
    );
    assert_eq!(src_pts, shd_pts, "I5：两侧流形点数差必须在阈内");
    assert!(
        (src_dep - shd_dep).abs() <= 0.02,
        "I5：最大穿透深度差必须在阈内（{src_dep} vs {shd_dep}）"
    );
    assert!(dot >= 0.99, "I5：法向摘要必须一致（dot={dot}）");

    // 源表示退场（整体移格），目标表示就位（网格 provider + 静态 marker 体）。
    assert_eq!(filled(&w, pid), 0, "提交后源体素域应当为空");
    let mesh_pid = mesh.unwrap_or(u32::MAX);
    assert!(mesh_pid != u32::MAX, "提交应给出网格 provider id");
    assert!(
        w.providers().mesh(mesh_pid).is_some(),
        "提交后应存在网格 provider"
    );
    assert!(body.is_some(), "提交应给出静态 marker 体");
    assert_ne!(mesh_pid, pid, "网格 provider 必须是新 id（id 空间只增）");

    // I6：投影到"静置高度"这一可观测量上——切换前后不得跳变（窗口 = 后 120 tick，非单点）。
    for _ in 0..POST {
        w.step();
    }
    let (pos_after, _) = w.bodies.pose(box_id);
    let y_after = pos_after.y;
    println!("  I6：切换前 y={y_before:.4} → 切换后（+{POST} tick）y={y_after:.4}");
    assert!(
        (y_after - y_before).abs() <= REST_JUMP_TOL,
        "I6：切换前后静置高度跳变 {:.4} m（超阈 {REST_JUMP_TOL}）",
        (y_after - y_before).abs()
    );
    assert!(
        (y_after - 1.5).abs() <= 0.02,
        "切换后应仍静置在地板上（y≈1.5，实测 {y_after}）"
    );
}

#[test]
fn effect_key_conflict_is_loud_in_both_orders() {
    // 顺序 A：先挖格 → 再请求转换 ⇒ 请求被拒（I3）。
    let (mut a, pid_a, _) = scene(true);
    for _ in 0..60 {
        a.step();
    }
    let dug = a.spawn_box_debris(
        pid_a,
        Vec3::new(-2.0, 0.0, -2.0),
        Vec3::new(2.0, 1.0, 2.0),
        1000.0,
    );
    assert!(dug > 0, "提取应当真的挖到格（否则本判据空过）");
    let e = a.request_voxel_conversion(pid_a);
    assert!(
        matches!(
            e,
            Err(ConversionError::EffectConflict {
                already: EffectKey::VoxelExtraction,
                ..
            })
        ),
        "先挖后转必须 fail-loud：{e:?}"
    );

    // 顺序 B：先请求转换 → 同 tick 再挖格 ⇒ 挖格侧拒绝（域原地不动）。
    let (mut b, pid_b, _) = scene(true);
    for _ in 0..60 {
        b.step();
    }
    let req = b.request_voxel_conversion(pid_b);
    assert!(req.is_ok(), "请求应当登记：{req:?}");
    let before = filled(&b, pid_b);
    assert!(before > 0, "请求阶段不得动几何（只登记意图）");
    let dug2 = b.spawn_box_debris(
        pid_b,
        Vec3::new(-2.0, 0.0, -2.0),
        Vec3::new(2.0, 1.0, 2.0),
        1000.0,
    );
    assert_eq!(
        dug2, 0,
        "已被转换占用的域不得再挖（拒绝，不是静默提 0 个格）"
    );
    assert_eq!(filled(&b, pid_b), before, "拒绝必须是原地不动");
}
