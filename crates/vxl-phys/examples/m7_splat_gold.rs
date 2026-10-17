//! **M7 高斯泼溅金样** —— splat 域此前有 6 条测试，但**没有任何场景金样**。
//!
//! splat 与体素是两条并行的**碰撞代理**：体素给硬边，splat 给软连续面（高斯核的等值面）。
//! 它没有自己的时间推进——`step_particles` 只在"粒子云自己动"时才用，作为碰撞代理时
//! 每 tick 被 `contacts_*` 查询一次，跟刚体走同一条管线。
//!
//! 场景：`y = 0` 面铺 7×7 各向同性核（σ = 0.5、幅值 1.0，间距 0.25 m），
//! 一个 0.7 m 立方从等值面上方 0.5 m 落下，静置 720 tick。
//!
//! **判据**（三项）：
//! - 「1」末态位姿的**位模式哈希**（主判据）—— splat 接触路径上没有任何超越函数，
//!   整条链是 `exp` 之外的四则运算 + 一次高斯取值，跨平台可位精确；
//! - 「2」盒子中心 `y` 与**底面最低角点 `y`** 的两位小数（可读）；
//! - 「3」**结构断言** —— 盒底必须落在等值面附近（`|底 − 等值面| < 0.02`，与
//!   `tests/rest_height_contract.rs` 同一口径 2 cm）。这一条是 2026-09-21 那次
//!   `contacts_point: depth = skin − f → −f` 修复的守卫：旧口径下盒会**悬空约一个 band**。
//!
//! ⚠️ **不要断言"完全静止"**：软场本身有轻微起伏（7×7 高斯叠加不是严格平面），
//! 盒底会在 ±2 cm 内微调。判据给的是 2 cm 容忍 + 位模式锁定末态。
//!
//! 运行：`cargo run --release -p vxl-phys --example m7_splat_gold -- [ticks]`
//! 门禁配方：`720`（见 `scripts/gate_gold.sh`）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_splat::{GaussianSplatField, Splat};

/// 门禁配方。
const GATE_TICKS: usize = 720;

/// 立方半长。
const HALF: f32 = 0.35;

/// 平喷溅场：`y = 0` 面上按 0.25 m 铺 7×7 各向同性核（σ = 0.5、幅值 1.0）。
fn flat_splat() -> GaussianSplatField {
    let mut f = GaussianSplatField::new(0.5);
    for ix in -3..=3 {
        for iz in -3..=3 {
            f.push(Splat::isotropic(
                Vec3::new(ix as f32 * 0.25, 0.0, iz as f32 * 0.25),
                0.5,
                1.0,
            ));
        }
    }
    f
}

/// 该场的**等值面高度**（沿 `(x, ·, z)` 二分 `sdf` 的零点）——独立于接触路径的参考。
fn iso_height(f: &GaussianSplatField, x: f32, z: f32) -> f32 {
    let (mut lo, mut hi) = (-1.0f32, 3.0f32);
    let flo = f.sdf(Vec3::new(x, lo, z));
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        let fm = f.sdf(Vec3::new(x, mid, z));
        if (fm > 0.0) == (flo > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

struct Readings {
    hash: u64,
    center_y: f32,
    bottom_y: f32,
}

struct Frozen {
    hash: u64,
    center_y: f32,
    bottom_y: f32,
}

fn readings(ticks: usize) -> Readings {
    let field = flat_splat();
    let y_ref = iso_height(&field, 0.0, 0.0);
    let mut w = World::new(PhysConfig::default());
    w.add_splat_field(field);
    let i = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(0.0, y_ref + HALF + 0.5, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    for _ in 0..ticks {
        w.step();
    }
    let (pos, rot) = w.bodies.pose(i);
    // 盒的 8 个角点里最低的那个 y。
    let mut bottom = f32::INFINITY;
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let local = Vec3::new(sx * HALF, sy * HALF, sz * HALF);
                let wy = rot.rotate_vec3(local).y + pos.y;
                if wy < bottom {
                    bottom = wy;
                }
            }
        }
    }
    // 末态位姿的 FNV-1a 哈希。
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for c in [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w] {
        for b in c.to_bits().to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100_0000_01b3);
        }
    }
    Readings {
        hash: h,
        center_y: pos.y,
        bottom_y: bottom,
    }
}

fn frozen(ticks: usize) -> Option<Frozen> {
    (ticks == GATE_TICKS).then_some(Frozen {
        hash: 0xdbcf_6eec_4b0e_449a,
        center_y: 1.719_448_2,
        bottom_y: 1.369_448_2,
    })
}

fn main() {
    let ticks: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let r = readings(ticks);
    let field = flat_splat();
    let y_ref = iso_height(&field, 0.0, 0.0);
    println!(
        "  盒心 y={:.9} | 底面 y={:.9} | 等值面 {:.9} | 差 {:+.6} | hash=0x{:016x}",
        r.center_y,
        r.bottom_y,
        y_ref,
        r.bottom_y - y_ref,
        r.hash
    );
    let Some(f) = frozen(ticks) else {
        println!("  （跳过基线判定：{ticks} tick 不是门禁配方 {GATE_TICKS}）");
        return;
    };
    let ok = r.hash == f.hash
        && (r.center_y - f.center_y).abs() < 1e-6
        && (r.bottom_y - f.bottom_y).abs() < 1e-6
        // (3) 结构断言：盒底贴住等值面（±2 cm）。这条守的是 2026-09-21 那次
        //     `contacts_point: depth = skin − f → −f` 的修复 —— 旧口径下盒会悬空约一个 band。
        && (r.bottom_y - y_ref).abs() < 0.02;
    if !ok {
        eprintln!(
            "❌ 金样基线 FAIL（m7_splat_gold / {ticks} tick）\n   \
             期望 hash=0x{:016x} center_y={:.9} bottom_y={:.9} 且 |底面−等值面|<0.02\n   \
             实测 hash=0x{:016x} center_y={:.9} bottom_y={:.9}（底面−等值面={:+.6}）",
            f.hash,
            f.center_y,
            f.bottom_y,
            r.hash,
            r.center_y,
            r.bottom_y,
            r.bottom_y - y_ref,
        );
        std::process::exit(1);
    }
    println!("✅ 金样基线 PASS（m7_splat_gold / {ticks} tick）");
}
