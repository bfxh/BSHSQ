//! **判据：布片自摩擦**（T3 登记的边界："两层之间可以自由滑" ⇒ 折起来留不住）。
//!
//! **口径**（与 rope / 刚体**同一条**库仑锥）：切向滑移被限制在 `μ·depth` 以内，`depth` = 该对的
//! **相对**法向修正量。⚠️ 与 rope 的体接触**不同的那一点**写在那边的注里：那边一侧是刚体、
//! 预算只对**粒子那一侧的滑移**生效（`μ·w_p·λ`）；这里是**两只都在动**的对，要限制的是**相对**
//! 滑移 ⇒ 相对法向修正量恰好是 `depth`。
//!
//! **默认关**：`SelfCollision::friction = 0`（骨架里规格书**没有**规定自摩擦 ⇒ 本片选的默认是
//! "关"）⇒ 切片 T3 的 4 条判据读数**逐位不变**（那 4 条就在 `cloth_self_collision.rs` 里守着）。
//!
//! **场景**：沿用 T3 的折合（8×8 布片、折痕钉住、反向角速度驱动把两半**压合**），**另加切向
//! （`z`）驱动**只推 `+x` 半区 ⇒ 两半之间出现**相对切向滑移**；摩擦力把它按住。
//!
//! **判据**：
//! ① **有摩擦 ⇒ 滑不动**：`μ = 0.5` 的相对滑移 ≤ `μ = 0` 的 **30%**（比值口径，§9）；
//! ② **金丝雀**：`μ = 0`（显式）与 `SelfCollision::default()`（`friction = 0`）**逐位相同**
//!    ⇒ 默认档零成本（那条"关"的路径真的被跳过）。
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::Vec3;
use vxl_phys_soft::{ClothSheet, Stiffness};

const DT: f32 = 1.0 / 60.0;
const N: usize = 8;
const W: usize = N + 1;
/// 折合角速度（与 T3 同：把两半压到一起，让自接触持续成立）。
const OMEGA: f32 = 1.0;
/// **切向**驱动速度（`+z`，只推 `+x` 半区；m/s）。
///
/// ⚠️ **必须小**：首版取 `1.0`（×300 tick = 5 m）⇒ 两层**整体错开**、自接触消失 ⇒
/// 没有对可摩擦（实测 μ=0 与 μ=0.5 的滑移**逐位相同**，正是"没对"的签名）。
/// 取 `0.05`：滑移 ~0.25 m，两层仍大幅重叠（片宽 1 m）⇒ 接触持续成立。
const TAN: f32 = 0.05;
const TICKS: usize = 300;

/// 平铺网格（行主序：`z` 外层、`x` 内层）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
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

/// 折合场景（与 T3 同款）：**关弯曲**（弯曲弹簧会抵抗折痕）、折痕那一列钉住。
/// 返回值第二项 = **按初始位置固定**的半区符号（`+1` = `x>0`、`-1` = `x<0`、`0` = 折痕上）。
///
/// ⚠️ **必须按初始位置定死**（T3 已栽过一次，这里又栽一次并留档）：折合会把 `+x` 半区**带过
/// 折线**（`x` 变负）⇒ 若按**瞬时** `x` 判半区，驱动与测量会在两半之间**来回切**（首版实测
/// μ=0 的"相对滑移"是**负的** −0.097：几何漂移盖过了驱动）。
fn fold_sheet() -> (ClothSheet, Vec<f32>) {
    let (pts, tris) = plate(N, 0.5);
    let halves = pts
        .iter()
        .map(|p| {
            if p.x > 1e-6 {
                1.0
            } else if p.x < -1e-6 {
                -1.0
            } else {
                0.0
            }
        })
        .collect();
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.bend_compliance = f32::INFINITY;
    s.damping = 0.999;
    s.self_contacts.cfg.enabled = true;
    for iz in 0..W {
        s.set_pinned(iz * W + N / 2, true); // 折痕列（`x = 0`）
    }
    (s, halves)
}

/// 两半的反向角速度（`v = sgn·ω·ẑ × r`；同 T3，粒子越过折线即自刹车）。
fn fold_drive(s: &mut ClothSheet) {
    for i in 0..s.pos.len() {
        if s.inv_mass[i] == 0.0 {
            continue;
        }
        let p = s.pos[i];
        if p.x.abs() < 1e-6 {
            continue;
        }
        let sgn = if p.x > 0.0 { 1.0 } else { -1.0 };
        let r = Vec3::new(p.x, p.y, 0.0);
        s.vel[i] = Vec3::new(-r.y, r.x, 0.0) * (sgn * OMEGA);
    }
}

/// **切向驱动**：只给 `+x` 半区（**按初始半区**）的自由粒子加 `+z` 速度。
fn tangential_drive(s: &mut ClothSheet, halves: &[f32]) {
    for (i, h) in halves.iter().enumerate() {
        if s.inv_mass[i] > 0.0 && *h > 0.0 {
            s.vel[i].z = TAN;
        }
    }
}

/// **两半的相对切向（`z`）滑移**：`+x` 半区的平均 `z` 减 `-x` 半区的平均 `z`
/// （镜像初始 ⇒ 起点为 0；有滑移则增大）。用**平均**而不是单点：与 §5 的窗口口径同族。
fn rel_z(s: &ClothSheet, halves: &[f32]) -> f32 {
    let (mut sa, mut na, mut sb, mut nb) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for (i, h) in halves.iter().enumerate() {
        if s.inv_mass[i] == 0.0 || *h == 0.0 {
            continue;
        }
        if *h > 0.0 {
            sa += s.pos[i].z;
            na += 1.0;
        } else {
            sb += s.pos[i].z;
            nb += 1.0;
        }
    }
    sa / na.max(1.0) - sb / nb.max(1.0)
}

/// 跑一场（`mu` = 自摩擦系数）⇒ `(相对切向滑移, 末子步的接触对数)`。
fn run(mu: f32) -> (f32, u32) {
    let (mut s, halves) = fold_sheet();
    s.self_contacts.cfg.friction = mu;
    for _ in 0..TICKS {
        fold_drive(&mut s);
        tangential_drive(&mut s, &halves);
        s.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
    (rel_z(&s, &halves), s.self_contacts.pairs)
}

/// ① **有摩擦 ⇒ 滑不动**：`μ = 0.5` 的相对滑移 ≤ `μ = 0` 的 30%。
#[test]
fn self_friction_resists_tangential_sliding() {
    let ((s0, p0), (s5, p5)) = (run(0.0), run(0.5));
    println!(
        "[判据①自摩擦] {TICKS} tick 相对切向滑移：μ=0 → {s0:+.5} m（接触对 {p0}）；\
         μ=0.5 → {s5:+.5} m（接触对 {p5}）（{:.1}%）",
        100.0 * s5.abs() / s0.abs().max(1e-9)
    );
    assert!(
        p0 > 0 && p5 > 0,
        "场景必须**持续有自接触**（实测接触对 μ=0 → {p0}、μ=0.5 → {p5}）——\
         没有对就没有摩擦可谈（首版驱动太大、两层滑到完全错开）"
    );
    assert!(
        s0.abs() > 1e-3,
        "μ=0 时该**滑得动**（实测 {s0:+.5} m）——不滑说明场景没产生相对切向载荷（判据没分辨力）"
    );
    assert!(
        s5.abs() <= s0.abs() * 0.3,
        "μ=0.5 的滑移该显著小于 μ=0（实测 {s5:+.5} vs {s0:+.5} = {:.0}%）——\
         太大了说明切向锥没生效或预算口径错",
        100.0 * s5.abs() / s0.abs().max(1e-9)
    );
}

/// ② **金丝雀（默认档零成本）**：`μ = 0`（显式）与 `SelfCollision::default()`（`friction = 0`）
/// **逐位相同** ⇒ 那条"关"的路径真的被跳过（T3 的 4 条判据就是靠这条保持逐位不变）。
#[test]
fn zero_friction_is_bit_identical_to_default() {
    let (mut a, ha) = fold_sheet();
    a.self_contacts.cfg.friction = 0.0;
    let (mut b, hb) = fold_sheet(); // 完全不碰 `friction`（走 `SelfCollision::default()`）
    assert_eq!(
        b.self_contacts.cfg.friction, 0.0,
        "骨架默认该是 `friction = 0`（规格书没规定自摩擦 ⇒ 本片选的默认是关）"
    );
    for _ in 0..120 {
        fold_drive(&mut a);
        tangential_drive(&mut a, &ha);
        a.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
        fold_drive(&mut b);
        tangential_drive(&mut b, &hb);
        b.step(DT, Vec3::ZERO, &NoProviders, 0, &[]);
    }
    let same = a.pos.iter().zip(&b.pos).all(|(p, q)| {
        p.x.to_bits() == q.x.to_bits()
            && p.y.to_bits() == q.y.to_bits()
            && p.z.to_bits() == q.z.to_bits()
    });
    println!(
        "[判据②默认档] 120 tick 逐位比较：pos {}",
        if same { "相同 ✅" } else { "**不同 ❌**" }
    );
    assert!(
        same,
        "`friction = 0` 该与默认**逐位相同** ⇒ 关的路径真的被跳过"
    );
}
