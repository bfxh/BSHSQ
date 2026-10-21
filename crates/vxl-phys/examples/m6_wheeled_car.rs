//! **M6 轮式车金样**（vxl-only 通道）—— `ROUTE` M6 出口判据「帆/浮体/车 金样」里的**车**那一项。
//!
//! 场景：**水平厚板**上停一辆两轮车：车身（`0.30×0.10×0.20` 的盒，密度 `500`）+ 两个**球形轮**
//! （半径 `0.08`，密度 `1200`），轮与车身之间是**转动关节**（自由轴 `= z`，即垂直于滚动方向）。
//! 车身起步带 `−x` 方向初速 `2 m/s` ⇒ 轮靠接触摩擦被带着转、车整辆沿 `−x` 滑行。
//!
//! **为什么轮是球而不是圆柱**：圆柱的解析接触要靠多面化，滚动时接触法线在面与面之间跳变
//! （"打摆"，见 `TECH-SURVEY` A9 与 `tests/rolling_probe.rs`）。球是解析接触、纯滚动有闭式解
//! （`tests/rolling_resistance_contracts.rs` 的 R1：`a = 5g·sinθ/7`），因此用它当轮既稳又可判。
//!
//! **为什么地面是水平的（重要）**：初版是倾斜 `10°` 的坡，本地（Windows）稳定通过，但 CI（Linux）
//! 上跑出不同的位模式而失败。根因是**斜坡要用 `Quat::from_axis_angle` 做 `sin/cos`**，而两个平台的
//! libm 在这些函数上有 ulp 级差异；车是接触驱动的混沌系统，180 tick 足够把 1 ulp 放大到位模式不同。
//! 用 `RUSTFLAGS=-C target-cpu=x86-64` 在本地复跑仍通过 ⇒ 排除 CPU 特性，确认是 libm 差异。
//! 改成水平地面后整条路径只剩加减乘除与开方，跨平台逐位一致。
//!
//! **起步必须贴面**：轮心放在表面高度（`y = r`）处。实测把 `r = 0.08` 的轮从高出表面
//! `0.34 m` 放下，球会在接触前攒够速度而**穿透**厚板 —— 同一个场景里把 `r` 改成 `0.3`、
//! 落点贴近表面就完全正常，所以是高速穿透、不是接触失效。
//!
//! 运行：`cargo run --release -p vxl-phys --example m6_wheeled_car -- [ticks]`
//! 门禁配方：`180`（见 `scripts/gate_gold.sh`）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{BodyBuildExt, Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};

const GATE_TICKS: usize = 180;

/// 轮半径（也是轮心的起始高度 = 贴面）。
const WHEEL_R: f32 = 0.08;
/// 车身初速（`−x` 方向）。
const LAUNCH_SPEED: f32 = 2.0;

/// 水平厚板 + 车身 + 两轮 ⇒ 返回 `(世界, 车身 id, 前轮 id)`。
fn scene() -> (World, u32, u32) {
    let mut w = World::new(PhysConfig::default());
    // 厚板：`half.y = 0.5`（薄板会让贴面起步的轮直接穿透）。
    w.add_static(
        Shape::Box {
            half: Vec3::new(8.0, 0.5, 8.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let x0 = 0.0_f32;
    let car = w.add_dynamic(
        Shape::Box {
            half: Vec3::new(0.15, 0.05, 0.10),
        },
        Vec3::new(x0, WHEEL_R + 0.10, 0.0),
        Quat::IDENTITY,
        500.0,
    );
    w.bodies.linvel[car as usize] = Vec3::new(-LAUNCH_SPEED, 0.0, 0.0);
    let mut front = 0u32;
    for (k, dx) in [-0.10_f32, 0.10].into_iter().enumerate() {
        let wheel = w.add_dynamic(
            Shape::Sphere { radius: WHEEL_R },
            Vec3::new(x0 + dx, WHEEL_R, 0.0),
            Quat::IDENTITY,
            1200.0,
        );
        // 转动关节：自由轴 = z（垂直于滚动方向 `x`）⇒ 轮能绕它滚，车身只被带着平移。
        w.joints.add(
            Joint::new(
                JointKind::Revolute,
                car,
                wheel,
                Vec3::new(dx, -0.10, 0.0),
                Vec3::ZERO,
            )
            .with_axis(Vec3::Z),
        );
        if k == 1 {
            front = wheel;
        }
    }
    (w, car, front)
}

/// 跑 `ticks`，返回 `(车身 x, 车身 y, 前轮 x, 前轮 y)`。
fn run(ticks: usize) -> (f32, f32, f32, f32) {
    let (mut w, car, front) = scene();
    for _ in 0..ticks {
        w.step();
    }
    let p = w.bodies.position[car as usize];
    let q = w.bodies.position[front as usize];
    (p.x, p.y, q.x, q.y)
}

/// 冻结的末态读数。真值来源：`GATE_TICKS = 180` 在 Windows 上实测（两次独立运行逐位相同）。
///
/// ⚠️ 这个场景的判据用**容差**而不是位模式：车是接触驱动的多体系统，同一份代码在
/// Linux（CI）与 Windows 上的末态相差约 `1.6e-5`（实测：`x_car` `0xbe223c42` vs `0xbe221cee`）——
/// 是浮点累积差，不是逻辑错。硬要比位模式会让 CI 永远红。对照：本仓的
/// `m3_collapse` 用 hash、`m5_dam_break` 用浮点读数，都不是位模式。
struct Frozen {
    x_car: f32,
    y_car: f32,
    x_wheel: f32,
    y_wheel: f32,
}

fn frozen() -> Frozen {
    // 用位模式写浮点字面量：十进制写法很容易超过 `f32` 的有效位而触发
    // `clippy::excessive_precision`（CI 的强化档 clippy 把它当错误）。
    Frozen {
        x_car: f32::from_bits(0xbe22_3c42),
        y_car: f32::from_bits(0x3e29_d9b9),
        x_wheel: f32::from_bits(0xbd6d_e8c7),
        y_wheel: f32::from_bits(0x3da4_4ab7),
    }
}

/// 容差（绝对）。**实测跨平台最大差 `1.2e-4`**（`x_car`，Windows 与正式 CI 读数对比）
/// ⇒ 取 `1e-3` 留 8 倍余量。早先按位模式换算估成 `1.6e-5` 是错的，CI 实测差了一个数量级。
const TOL: f32 = 1e-3;

fn main() {
    let ticks: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let (x, y, wx, wy) = run(ticks);
    println!("  车身 x={x:.9} y={y:.9} | 前轮 x={wx:.9} y={wy:.9}");
    println!(
        "  位模式 x_car=0x{:08x} y_car=0x{:08x} x_wheel=0x{:08x} y_wheel=0x{:08x}",
        x.to_bits(),
        y.to_bits(),
        wx.to_bits(),
        wy.to_bits()
    );
    let f = frozen();
    let ok = (x - f.x_car).abs() < TOL
        && (y - f.y_car).abs() < TOL
        && (wx - f.x_wheel).abs() < TOL
        && (wy - f.y_wheel).abs() < TOL
        // (3) 行为：初速朝 `−x`，车身必须真的往那边走了（独立于数值冻结的可读判据）。
        && x < 0.0;
    if !ok {
        eprintln!(
            "❌ 金样基线 FAIL（m6_wheeled_car / {ticks} tick）\n   \
             期望 x_car={:.7} y_car={:.7} x_wheel={:.7} y_wheel={:.7}（容差 {TOL}）且 x_car<0\n   \
             实测 x_car={x:.7} y_car={y:.7} x_wheel={wx:.7} y_wheel={wy:.7}",
            f.x_car, f.y_car, f.x_wheel, f.y_wheel,
        );
        std::process::exit(1);
    }
    println!("✅ 金样基线 PASS（m6_wheeled_car / {ticks} tick）");
}
