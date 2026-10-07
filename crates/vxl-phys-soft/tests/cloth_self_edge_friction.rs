//! **判据：点-边对的自摩擦**（`cloth_edge_friction.rs`；补齐点-边那支的"无摩擦"边界）。
//!
//! **场景**（两个三角形、共 6 个顶点，最小到只留一对接触）：
//! - 下层三角 `[A, B, C]` 铺在 `y = 0`（`AB` 是我们要测的那条边）；
//! - 上层三角 `[P, Q, R]` 悬在 `y = 0.02`，其中 `P` 正落在 `AB` 中点的**上方 0.02**
//!   ——`d_c = 2r = 0.056` ⇒ 只有 `P × AB` 这一对落在接触带内（`P` 到 `A`/`B` 的心距
//!   ≈ 0.102、到 `C` 更远 ⇒ 点-点**够不到**，这正是点-边的盲区几何）；
//! - **切向驱动**：每 tick 给上层三个顶点重置 `+x` 速度（`v = 0.02 m/s`，120 tick 走
//!   `0.04 m` < 边长 `0.2 m` ⇒ `P` 全程没滑出 `AB` 的 `u ∈ [0,1]` 区间）。
//!
//! ⚠️ **为什么用"每 tick 重置速度"而不是直接改 `pos`**：XPBD 是位置式，`prev` 在每个子步
//! 起点对齐 `pos` ⇒ 直接写 `pos` 的位移不进 `pos − prev`，摩擦读不到（见
//! `cloth_volume.rs` 那片的同类坑）。
//!
//! **判据**：
//! ① **数值健壮**：开摩擦跑完全程不产生非有限值；
//! ② **默认档中性**：不设 `friction`（默认 `0`）与显式设 `0` **逐位一致**。
//!
//! ⚠️ **本片没有集成级的"摩擦显著拖住滑移"判据，如实登记原因**：点-边是**硬投影**——推开
//! 一次就把粒子送到 `len = d_c`，而 XPBD 的位置式推开会被 `write_back` 转成速度 ⇒
//! （`damping = 1.0` 无阻尼时）粒子随即离开接触带 ⇒ **接触只持续一两个子步**。最小场景下
//! 实测位移落在 `5.6e-6` 量级、两档差异已在浮点噪声内（`μ=0` 5.609e-6 vs `μ=0.5` 5.614e-6，
//! 方向都不可靠）。要把它做成强判据得先给场景加**阻尼**或改成**多层布**（`cloth_self_edge.rs`
//! 的折布几何是候选）——属后续片。摩擦的**定量**行为由 `src/cloth_edge_friction.rs` 内的
//! 4 条单元判据守（那里能精确控住 `pos/prev/inv_mass/w/depth`）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const V: f32 = 0.02;
const TICKS: usize = 120;
const R: f32 = 0.028;

/// 上层三个顶点（`P` 在最前，索引 3）。
const UPPER: std::ops::Range<usize> = 3..6;

fn scene(mu: Option<f32>) -> ClothSheet {
    let pos = vec![
        Vec3::new(-0.10, 0.00, 0.0),  // 0 A
        Vec3::new(0.10, 0.00, 0.0),   // 1 B
        Vec3::new(0.00, -0.10, 0.0),  // 2 C
        Vec3::new(0.00, 0.02, 0.0),   // 3 P（落在 AB 上方 d_c 内）
        Vec3::new(0.06, 0.02, 0.08),  // 4 Q
        Vec3::new(-0.06, 0.02, 0.08), // 5 R
    ];
    let tris = vec![[0u32, 1, 2], [3, 4, 5]];
    let mut s = ClothSheet::new(pos, tris, 1000.0, 0.01, Stiffness::Hard);
    s.bend_compliance = f32::INFINITY; // 关弯曲（`project_bend` 对非有限显式短路）
    s.self_contacts.cfg.enabled = true; // 自碰撞总开关
    s.self_contacts.cfg.particle_radius = R;
    s.self_contacts.set_point_edge(true); // 点-边子开关
                                          // **只有 P 自由**：下层当"地面"、上层另两顶点当"支架"都钉住 ⇒ P 的位移全部归因于
                                          // "点-边接触 + 切向驱动"这两项（否则推开会把上层三角形整个顶得旋转起来，接触只持续
                                          // 一两个子步 —— 本轮实测踩过：1 tick 内 P 的 y 就被甩到 0.14 ≫ d_c=0.056）。
    for i in 0..3 {
        s.set_pinned(i, true);
    }
    for i in 4..6 {
        s.set_pinned(i, true);
    }
    if let Some(mu) = mu {
        s.self_contacts.cfg.friction = mu;
    }
    s
}

/// 每 tick 给上层重置切向速度（见文件头注：不能用直接写 `pos` 的驱动）。
fn run(s: &mut ClothSheet, ticks: usize) {
    for _ in 0..ticks {
        for i in UPPER {
            s.vel[i] = Vec3::new(V, 0.0, 0.0);
        }
        // 向下重力 ⇒ P 被持续压在 AB 上（接触不中断，摩擦每子步都有活干）。
        s.step(DT, Vec3::new(0.0, -9.8, 0.0), &NoProviders, 0, &[]);
    }
}

/// ① 开摩擦后数值健壮（不断言位移方向/幅度，原因见文件头注）。
#[test]
fn friction_on_stays_finite() {
    let mut s = scene(Some(0.5));
    run(&mut s, TICKS);
    for i in 0..s.pos.len() {
        assert!(s.pos[i].is_finite(), "粒子 {i} 出现非有限值");
    }
}

/// ② 默认档（不设 `friction`）与显式 `0` 逐位一致。
#[test]
fn default_friction_is_off_and_bitwise_identical() {
    let mut a = scene(None);
    let mut b = scene(Some(0.0));
    run(&mut a, 40);
    run(&mut b, 40);
    for i in 0..a.pos.len() {
        assert_eq!(a.pos[i], b.pos[i], "粒子 {i} 不同：默认档被摩擦碰到了");
        assert!(a.pos[i].is_finite(), "粒子 {i} 出现非有限值");
    }
}
