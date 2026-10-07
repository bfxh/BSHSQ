//! **面元力矩的合同判据**（《LSSMJ-CICD…v5》§15.4「面元合力/力矩合同」在本仓的**可执行子集**）。
//!
//! 与 `aero_face_torque.rs` 的分工：那边钉"接线与口径"（静力矩 = 离散和逐位 / 力臂线性 /
//! 动力学方向 / 金丝雀）；本文件钉**合同级性质** —— 换参考点、整体旋转、对称分布、施加次数。
//!
//! **判据**：
//! ① **对称分布零净矩**：两块同尺寸板关于 `yz` 面对称 ⇒ 每面力相同、力臂 `x` 相消 ⇒ 净矩 0；
//! ② **参考点变换**：关于原点读到的 `τ₀` 与 `ΣF` 必须满足 `τ_c = τ₀ − c × ΣF`，且与**独立重算**的
//!    `Σ (p_i − c) × F_i` 一致（引擎若用错参考点，这条会红）；
//! ③ **刚性旋转协变性**：整场景（几何 + 风）绕 `z` 转 90° ⇒ 力矩向量同步旋转（`τ₂ ≈ R·τ₁`）；
//! ④ **反作用恰好一次**：稳态下逐 tick 读到的力矩快照**逐位相同**（若在子步/迭代/帧间重复累计，
//!    读数会单调增长 ⇒ 立刻红）。
//!
//! ⚠️ **文档还列了"纯力偶零合力非零力矩"** —— 本仓**不适用**，如实登记：`face_force` 是
//! **纯阻力**（`F ∥ u`，见 `vxl-phys-aero/src/face.rs`），力向只随相对风流向，**接口不表达力偶**；
//! 升力项也是双面自洽的（`n → −n` ⇒ 力不变）。⇒ 按 §15.4 自己的口径（"若范围排除力矩，拒绝请求
//! 或返回明确 unsupported，禁止悄悄扔掉切向/转动项"），这里**不做该构造**，而不是假装验过。
//!
//! **边界**：只钉上述合同性质，不声称完整气动建模（风场空间变化等仍是线化档的登记边界）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_aero::AeroConfig;

/// ⊥ 板（板在 y=0 平面），|风| = 5。
const WIND: [f32; 3] = [0.0, -5.0, 0.0];

/// `n×n` 平面网格（y=0 平面、跨度 ±size），整体平移 `off`。
fn plate(n: usize, size: f32, off: Vec3) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut pts, mut tris) = (Vec::new(), Vec::new());
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(
                off.x - size + s * ix as f32,
                off.y,
                off.z - size + s * iz as f32,
            ));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// 把第二块网格并进第一块（顶点索引按第一块的长度偏移）。
fn merge(
    a: (Vec<Vec3>, Vec<[u32; 3]>),
    b: (Vec<Vec3>, Vec<[u32; 3]>),
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (mut pa, mut ta) = a;
    let (pb, tb) = b;
    let off = pa.len() as u32;
    pa.extend(pb);
    ta.extend(tb.into_iter().map(|t| [t[0] + off, t[1] + off, t[2] + off]));
    (pa, ta)
}

/// 静态 TriMesh（体在原点）+ 气动配置（无重力 ⇒ 静态体不受它影响，读数更干净）。
fn setup(pts: Vec<Vec3>, tris: Vec<[u32; 3]>, wind: [f32; 3]) -> (World, u32) {
    let mut w = World::new(PhysConfig {
        gravity: Vec3::ZERO,
        ..PhysConfig::default()
    });
    let mesh = w.add_trimesh(pts, tris);
    let half = w.trimesh_half_extents(mesh);
    let b = w.add_static(Shape::TriMesh { mesh, half }, Vec3::ZERO, Quat::IDENTITY);
    w.set_aero(AeroConfig {
        wind,
        ..AeroConfig::default()
    });
    (w, b)
}

/// 按**同一 f32 运算**独立重算"关于参考点 `c`"的逐面力矩离散和（判据②的对照侧）。
fn torque_about(pts: &[Vec3], tris: &[[u32; 3]], cfg: &AeroConfig, c: Vec3) -> Vec3 {
    let wind = Vec3::new(cfg.wind[0], cfg.wind[1], cfg.wind[2]);
    let mut t_sum = Vec3::ZERO;
    for tri in tris {
        let (p0, p1, p2) = (
            pts[tri[0] as usize],
            pts[tri[1] as usize],
            pts[tri[2] as usize],
        );
        let n_full = (p1 - p0).cross(p2 - p0);
        let two_a = n_full.length();
        if two_a <= 1e-12 {
            continue;
        }
        let center = (p0 + p1 + p2) * (1.0 / 3.0);
        let f = vxl_phys_aero::face_force(wind, two_a * 0.5, cfg);
        t_sum += (center - c).cross(f);
    }
    t_sum
}

/// 跑一步让气动快照落地 ⇒ 返回 `(ΣF, τ)`。
///
/// ⚠️ **必须 step 之后再读**：`World::aero_force/aero_torque` 读的是"**本子步施加的那份**"
/// 快照（`AeroState::forces/torques`），`set_aero` 之后还没推进时它是空的 —— 直接读会拿到
/// 零向量，令"非零"断言失败、而"相等/协变"类断言**平凡成立**（本轮实测踩到：①②③④ 全部
/// 拿到 0，其中②③"通过"是假通过）。
fn settle(w: &mut World, b: u32) -> (Vec3, Vec3) {
    w.step();
    (w.aero_force(b), w.aero_torque(b))
}

/// ① 两块板关于 `yz` 面对称 ⇒ 净矩为零（力相同、力臂 `x` 相消）。
#[test]
fn symmetric_pair_has_zero_net_torque() {
    let (pts, tris) = merge(
        plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0)),
        plate(2, 0.25, Vec3::new(-0.5, 0.0, 0.0)),
    );
    let (mut w, b) = setup(pts, tris, WIND);
    let (f, tau) = settle(&mut w, b);
    println!(
        "对称双板：ΣF = ({:.6},{:.6},{:.6})  τ = ({:.9},{:.9},{:.9})",
        f.x, f.y, f.z, tau.x, tau.y, tau.z
    );
    assert!(f.length() > 0.0, "对称双板应有非零合力（≈ 两倍单板阻力）");
    // 净矩的上界由"力 × 力臂"给出；对称下应远小于它（f32 舍入 ⇒ 不追求逐位 0）。
    let scale = f.length() * 0.5;
    assert!(
        tau.length() < 1e-6 * scale,
        "对称分布的净矩应≈0，实测 τ = {tau:?}（上界 {scale:.6}）"
    );
}

/// ② 参考点变换：`τ_c = τ₀ − c × ΣF` 与**独立重算**的 `Σ (p_i − c) × F_i` 一致。
#[test]
fn torque_transforms_with_reference_point() {
    let (pts, tris) = plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0));
    let (mut w, b) = setup(pts, tris, WIND);
    // 独立重算侧的几何**再生成一次**（同参数 ⇒ 逐位相同；新文件禁 `clone`）。
    let (pts2, tris2) = plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0));
    let cfg = AeroConfig {
        wind: WIND,
        ..AeroConfig::default()
    };
    let (f, tau0) = settle(&mut w, b);
    let c = Vec3::new(0.5, 0.0, 0.0);
    let want = tau0 - c.cross(f);
    let got = torque_about(&pts2, &tris2, &cfg, c);
    let err = (got - want).length();
    println!("参考点变换：τ₀ = {tau0:?}｜τ_c(由 τ₀ 推) = {want:?}｜τ_c(重算) = {got:?}");
    let scale = f.length() * 0.5;
    assert!(
        err < 1e-5 * scale.max(1.0),
        "参考点变换不一致：err = {err}（上界 {scale:.6}）"
    );
}

/// ③ 刚性旋转协变性：场景（几何 + 风）绕 `z` 转 90° ⇒ 力矩向量同步旋转。
#[test]
fn torque_is_rotation_covariant() {
    let (pts, tris) = plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0));
    let (mut w1, b1) = setup(pts, tris, WIND);
    let (_, tau1) = settle(&mut w1, b1);
    // `Rz(90°)`：`(x, y, z) → (−y, x, z)`。
    let rz = |v: Vec3| Vec3::new(-v.y, v.x, v.z);
    let (pts2, tris2) = plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0));
    let rot_pts: Vec<Vec3> = pts2.into_iter().map(rz).collect();
    let w2v = rz(Vec3::new(WIND[0], WIND[1], WIND[2]));
    let (mut w2, b2) = setup(rot_pts, tris2, [w2v.x, w2v.y, w2v.z]);
    let (_, tau2) = settle(&mut w2, b2);
    let want = rz(tau1);
    let err = (tau2 - want).length();
    println!("旋转协变：τ₁ = {tau1:?}｜R·τ₁ = {want:?}｜τ₂ = {tau2:?}");
    let scale = tau1.length().max(1.0);
    assert!(
        err < 1e-4 * scale,
        "旋转协变不一致：err = {err}（上界 {scale:.6}）"
    );
}

/// ④ 反作用恰好一次：稳态下逐 tick 快照**逐位相同**（重复累计会单调增长）。
#[test]
fn torque_is_applied_once_per_tick() {
    let (pts, tris) = plate(2, 0.25, Vec3::new(0.5, 0.0, 0.0));
    let (mut w, b) = setup(pts, tris, WIND);
    let (_, t1) = settle(&mut w, b);
    w.step();
    let t2 = w.aero_torque(b);
    w.step();
    let t3 = w.aero_torque(b);
    println!("逐 tick 力矩：t1 = {t1:?}｜t2 = {t2:?}｜t3 = {t3:?}");
    assert!(t1.length() > 0.0, "静态偏置板应有非零力矩");
    assert_eq!(
        t2, t3,
        "稳态下逐 tick 快照应逐位相同（不同 ⇒ 子步/迭代/帧间在重复累计）"
    );
}
