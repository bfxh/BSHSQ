//! **M6 帆船金样**（vxl-only 通道）—— `ROUTE` M6 出口判据「帆/浮体/车 金样」里的**帆**那一半
//! （浮体在 `m6_float_mooring.rs`；车还没做）。
//!
//! 场景：体素水槽（腔体同 `m5_float_box`）+ **船体**（长条轻盒，密度 `300`，吃水线附近起步）**帆**（竖直三角面板，`x = 0` 平面内、法线沿 `±x`）+ **桅杆**（把帆**固定**钉在船体上）。
//! 风沿 `+x` 吹帆面 ⇒ 面元气动 `F = ½ρCdA·u·|u|` 推帆 ⇒ 经固定关节带动船体**顺风漂移**。
//!
//! **判据**（四项，全部按 `f32` 位模式逐位，口径同 `m4_cantilever` / `m5_float_box`）：
//! 「1」粒子数（守恒，两档都 512）；「2」顺风档末态船心 `x` / `y`；「3」**零风档**末态船心 `x` / `y`；
//! 「4」**差分**：顺风档的 `x` **严格大于**零风档 —— 这一条才是"风真的推了船"的判据。
//!
//! **为什么判据是差分而不是"零风档 x ≡ 0"**（诚实记录，实测得出）：这个场景里**零风档也会漂**
//! （720 tick 实测 `x = +0.1094`），来源有两处，都不是风 —— (1) 水块是从 `y = 1.05` **铸装**的，
//! 沉降过程本身会推船；(2) 帆把质心抬到船体之上，浮力与重力的偏置产生姿态扰动。把船抬高到
//! 吃水线附近只能让这个伪影改号（`−0.096` → `+0.109`），消不掉。所以两档都冻结成基线，
//! 判据只认**两档之差**；顺风档还有一项独立佐证：720 → 1200 tick 的 `x` 只变 0.0011
//! （`0.2132 → 0.2143`）⇒ 顺风档确实已接近稳态，不是某一瞬的相位。
//!
//! 运行：`cargo run --release -p vxl-phys --example m6_float_sail -- [ticks]`
//! 门禁配方：`720`（见 `scripts/gate_gold.sh`）。
// 新文件：`glob-gate` 对新增文件零基线 ⇒ 显式导入（不用 `use vxl_phys::*`）。
use vxl_phys::{
    AeroExt, BodyBuildExt, FluidExt, Joint, JointKind, MeshExt, PhysConfig, Quat, Shape, Vec3,
    World,
};
use vxl_phys_aero::AeroConfig;

const GATE_TICKS: usize = 720;

/// 船体的初始高度（≈ 实测吃水线，见 `scene` 里的注释）。
const HULL_Y: f32 = 1.30;

/// 帆的几何：竖直矩形（`x = 0` 平面内、`y ∈ [0, h]`、`z ∈ [-w/2, w/2]`），两张三角面。
fn sail(w: f32, h: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let pts = vec![
        Vec3::new(0.0, 0.0, -0.5 * w),
        Vec3::new(0.0, 0.0, 0.5 * w),
        Vec3::new(0.0, h, 0.5 * w),
        Vec3::new(0.0, h, -0.5 * w),
    ];
    (pts, vec![[0, 1, 2], [0, 2, 3]])
}

/// 水槽 + 铸装水块 + 船体 + 帆 ⇒ 返回 `(世界, 船体 id)`。
fn scene(wind: [f32; 3]) -> (World, u32) {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    // 围堰：只留中心格 (2,2) 敞开（同 `m5_float_box` / `fluid_coupling.rs`）。
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    let voxel_id = w.add_voxel(vol);
    let sys = vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig::default(),
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    );
    w.add_fluid(sys, &[voxel_id]);
    // 船体：长条轻盒（长轴沿风向 +x），**从吃水线附近起步** —— 起步太深会靠浮力猛冲上来，
    // 那一下的水流扰动会把"顺风漂移"的判据污染掉（实测 `y = 1.06` 起步时零风末态
    // `x = −0.0963`，抬到 1.30 后变 `+0.1094`；见文件头"为什么判据是差分"）。
    let hull = w.add_dynamic(
        Shape::Box {
            half: Vec3::new(0.10, 0.04, 0.14),
        },
        Vec3::new(0.0, HULL_Y, 0.0),
        Quat::IDENTITY,
        300.0,
    );
    // 帆：竖直面板，法线沿 ±x（正对风），钉在船体上方。
    let (sp, st) = sail(0.24, 0.30);
    let mesh = w.add_trimesh(sp, st);
    let half = w.trimesh_half_extents(mesh);
    let mast = w.add_dynamic(
        Shape::TriMesh { mesh, half },
        Vec3::new(0.0, HULL_Y + 0.04, 0.0),
        Quat::IDENTITY,
        120.0,
    );
    // 桅杆：帆与船体**固定**（一个刚体组），气动力因此能传到船体。
    w.joints.add(Joint::new(
        JointKind::Fixed,
        hull,
        mast,
        Vec3::ZERO,
        Vec3::ZERO,
    ));
    w.set_aero(AeroConfig {
        wind,
        ..AeroConfig::default()
    });
    (w, hull)
}

/// 跑 `ticks`，返回 `(粒子数, 船心 x, 船心 y)`。
fn run(wind: [f32; 3], ticks: usize) -> (usize, f32, f32) {
    let (mut w, hull) = scene(wind);
    for _ in 0..ticks {
        w.step();
    }
    let n = w
        .fluids()
        .first()
        .map(|(sys, ..)| sys.positions().len())
        .unwrap_or(0);
    let p = w.bodies.position[hull as usize];
    (n, p.x, p.y)
}

/// 冻结的末态读数（`f32` **位模式**，与 `m4_cantilever` / `m5_float_box` 同口径）。
///
/// 真值来源：`GATE_TICKS = 720` 的实测，两次独立运行逐位相同。
struct Frozen {
    n: usize,
    x_wind: u32,
    y_wind: u32,
    x_calm: u32,
    y_calm: u32,
}

fn frozen() -> Frozen {
    Frozen {
        n: 512,
        x_wind: 0x3e5a_572f,
        y_wind: 0x3f9a_dac2,
        x_calm: 0x3de0_1a58,
        y_calm: 0x3f9b_5d76,
    }
}

fn main() {
    let ticks: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(GATE_TICKS);
    let (n_w, x_w, y_w) = run([8.0, 0.0, 0.0], ticks);
    let (n_c, x_c, y_c) = run([0.0, 0.0, 0.0], ticks);
    println!("  [顺风 8 m/s] 粒子 {n_w} | 船心 x={x_w:.9} y={y_w:.9}");
    println!("  [零风对照 ] 粒子 {n_c} | 船心 x={x_c:.9} y={y_c:.9}");
    println!(
        "  差分 x_wind − x_calm = {:.9}（须 > 0：风确实推了船）",
        x_w - x_c
    );
    let f = frozen();
    let ok = n_w == f.n
        && n_c == f.n
        && x_w.to_bits() == f.x_wind
        && y_w.to_bits() == f.y_wind
        && x_c.to_bits() == f.x_calm
        && y_c.to_bits() == f.y_calm
        // 「4」差分：顺风档必须比零风档漂得更远 —— 这条独立于上面的数值冻结，
        //    也是"风吹了船"这件事本身的可读判据。
        && x_w > x_c;
    if !ok {
        eprintln!(
            "❌ 金样基线 FAIL（m6_float_sail / {ticks} tick）\n   \
             期望 n={} x_wind=0x{:08x} y_wind=0x{:08x} x_calm=0x{:08x} y_calm=0x{:08x} 且 x_wind>x_calm\n   \
             实测 n_w={n_w} x_wind=0x{:08x} y_wind=0x{:08x} n_c={n_c} x_calm=0x{:08x} y_calm=0x{:08x}",
            f.n,
            f.x_wind,
            f.y_wind,
            f.x_calm,
            f.y_calm,
            x_w.to_bits(),
            y_w.to_bits(),
            x_c.to_bits(),
            y_c.to_bits(),
        );
        std::process::exit(1);
    }
    println!("✅ 金样基线 PASS（m6_float_sail / {ticks} tick）");
}
