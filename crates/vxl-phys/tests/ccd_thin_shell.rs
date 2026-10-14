//! **薄壳 TOI 判据**（2026-10-13）：薄壁/薄壳这一类目标在扫掠 CCD 下**不该穿**，而且**不该**
//! 靠"退到上一安全采样点"凑合——那条退法在薄壁上是**两处实测缺陷**（`SURVEY-RIGID-HARD-CASES §7.2③`）：
//!
//! 1. **宽相漏配**：扫掠 AABB 只覆盖**质心路径 ± 皮肤**（旧写法）⇒ 大体积体撞薄壁时，接触发生在
//!    质心还差半个体长的地方、**压根不在质心路径上** ⇒ 配对进不来、窄相一片空白、扫掠报"无命中"
//!    ⇒ 体**径直穿过**（实测：60 m/s 的 1 m 盒穿过 5 mm 立板后继续飞、`|v|` 不减，见
//!    `examples/ccd_thin_shell_probe.rs` 的 `单页三角网` 那格至今仍是这条洞）。
//! 2. **停位精度 = 一整段采样长**：命中后一律退到上一样本 ⇒ 1 m 盒停在墙前 **0.25 m** 且法向速度
//!    已清零 ⇒ **悬停**（间距远大于皮肤带 0.02 ⇒ 离散管线接不上手、不落位也不入睡）。
//!
//! 修法：① 扫掠 AABB 改成**体在两端各自 AABB 之并**（`shape_aabb`）；② 命中后**二分细化 TOI**：
//! 在"带内接触之前那个样本"与"**表面真的贴上**（`depth ≥ 0`）那个样本"之间二分 ⇒ 停在**贴合
//! 那一瞬**（穿透 ≤ 二分分辨率 `|v|·sub/2⁸`）、接触实打实成立 ⇒ 离散接触接手落位与入睡。
//!
//! | 用例 | 旧写法 | 现在 |
//! |---|---|---|
//! | ① 5 mm 立板 × 1 m 盒 @60 m/s | **穿过**（继续飞） | 停在**贴合面**（0.5 mm 穿透）、**入睡** |
//! | ② 同档 @3000 m/s（预算闸咬住） | 穿 | 同上：0.5 mm 穿透、入睡 |
//! | ③ 起始偏移扫 16 相位（覆盖一整段） | — | 间距 ≥ **−0.0005 m**、最大嵌入 **≤ 0.0005**（**无相位敏感**） |
//!
//! ⚠️ 停位精度**就是二分分辨率**（`|v|·sub/2⁸`；60 m/s × 1/120 s ÷ 256 ≈ 2 mm，实测 0.5 mm），
//! 所以判据把"零嵌入"放宽到**亚毫米**。⚠️ 另一条**实测教训**（写在 `scan`/`refine_touch`）：
//! 大体积体高速撞**薄**壁时采样可能**整段跳过皮肤带**、第一个接触样本就已深嵌 ⇒ 二分的下界
//! 必须取"带内接触**之前**那个样本"，否则两端谓词同真、收敛到深嵌侧（实测最坏相位嵌 **0.1964 m**）。
//!
//! ⚠️ **已知仍存的洞**（登记在 `OPEN-PROBLEMS.md` P19）：**小盒 × 大三角网薄壳**——网格侧只采样
//! **网格顶点/边 vs 体**，1 m 盒骑在 4×4 m 的三角形面上时盒的 8 个角离面 0.5 m、网格顶点全在盒外
//! ⇒ 零接触 ⇒ 直接穿过。**④ 是它的金丝雀**（本仓惯例：缺口也钉住，修好那天它会红 ⇒ 翻面）。

use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// 动态盒半长（1 m 立方；最小半长 0.5 ⇒ 段长上限 0.25 ⇒ 旧停位精度就是 0.25）。
const HALF: f32 = 0.5;
/// 立板半厚（总厚 5 mm，**比皮肤带 0.02 还薄**）。
const TH: f32 = 0.0025;
/// 跑多少 tick。
const TICKS: u32 = 400;

/// 薄板 + 侧向飞来的 1 m 盒（`v` = 初速、`d0` = 盒心起始 x）。
fn scene(v: f32, d0: f32) -> (World, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0, // 放开上限，让 CCD 预算闸自己咬
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(TH, 2.0, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(d0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(v, 0.0, 0.0);
    (w, b)
}

/// 跑一遍：`(末态前缘到壁近面的间距, 全程最深嵌入)`。间距为负 = 前缘越过壁近面。
fn run(v: f32, d0: f32) -> (f32, f32, f32, bool) {
    let (mut w, b) = scene(v, d0);
    let mut deepest = 0.0f32;
    for _ in 0..TICKS {
        w.step();
        let gap = -(w.bodies.position[b].x + HALF + TH);
        deepest = deepest.max(-gap);
    }
    let gap = -(w.bodies.position[b].x + HALF + TH);
    (gap, deepest, w.bodies.linvel[b].length(), w.bodies.awake[b])
}

/// **① 薄壁（5 mm < 皮肤带）**：60 m/s 的 1 m 盒不许穿，且要停在**皮肤带内**（旧写法悬停在 0.25 m 外）。
#[test]
fn thin_wall_stops_the_box_at_the_skin_band() {
    let (gap, deepest, speed, awake) = run(60.0, -5.0);
    assert!(
        deepest < 0.005,
        "薄壁不许被嵌入（旧写法直接穿过；实测全程最深嵌入 {deepest}，上限取二分分辨率 0.005）"
    );
    assert!(
        (-0.005..=0.03).contains(&gap),
        "停位应贴在表面（±二分分辨率）：实测间距 {gap}（旧写法 0.25 = 一整段采样长、还是悬停）"
    );
    assert!(
        speed < 1e-3 && !awake,
        "停在带内 ⇒ 离散接触接手 ⇒ 应当停住并入睡（实测 |v| = {speed}、清醒 = {awake}）"
    );
}

/// **② 预算闸 + 薄壁**：3000 m/s 由预算闸限速到可行域后，同样不许穿、要停在带内。
#[test]
fn thin_wall_holds_the_speed_limited_box() {
    let (gap, deepest, speed, awake) = run(3000.0, -5.0);
    assert!(
        deepest < 0.005,
        "限速后同样不许嵌入（实测 {deepest}，上限取二分分辨率）"
    );
    assert!(
        (-0.005..=0.03).contains(&gap),
        "限速后停位也应贴在表面：实测 {gap}"
    );
    assert!(
        speed < 1e-3 && !awake,
        "应停住并入睡（实测 {speed} / {awake}）"
    );
}

/// **③ 相位扫描**：起始偏移扫过**一整段采样长**（0.25 m，16 档）——薄壁的判据不许有相位敏感：
/// 每一档都不许嵌入、都要停在带内。
#[test]
fn thin_wall_is_phase_insensitive() {
    let (mut min_gap, mut max_deep) = (f32::INFINITY, f32::MIN);
    for k in 0..16 {
        let d0 = -5.0 + (k as f32) * 0.25 / 16.0;
        let (gap, deepest, _sp, _aw) = run(60.0, d0);
        min_gap = min_gap.min(gap);
        max_deep = max_deep.max(deepest);
    }
    assert!(
        max_deep < 0.005,
        "16 个相位里最坏那档也不许嵌入（实测最深 {max_deep}，上限取二分分辨率）"
    );
    assert!(
        min_gap > -0.005,
        "16 个相位里最坏那档也要贴在表面（实测最小 {min_gap}）"
    );
}

/// **④ 薄壳：面穿过体（P19，已修 2026-10-13）**：小盒 × **大三角网薄壳** —— 必须**停在壳面上**。
///
/// 场景：4×4 m 的单页三角网竖在 `x = 0`，1 m 盒 60 m/s 沿 +x。
///
/// **修前**：盒**直穿**（末态盒心 **x = 395.0 m**，金丝雀曾把它钉住）。
/// **根因**：`contacts_box` 的 14 点采样（8 角 + 6 面心）量的都是"**点到面**的距离" —— 体骑在面两侧时，
/// 面心到面的距离 = `|盒心 x|` ⇒ 只有 `|x| ≤ skin(0.02)` 才出接触 ⇒ **接触窗口 = 皮肤带宽**（0.04 m），
/// 而盒每 tick 走 1 m、CCD 采样步长 0.25 m ⇒ **离散管线与 CCD 都整段跳过**。
/// **修法**（`vxl-phys-terrain/src/mesh.rs` 的 `contacts_surface_crossing`）：补一遍"**盒的 12 条棱 ×
/// 网格三角形**"的相交测试（Möller–Trumbore，`seg_tri`），命中即产出一条接触 ⇒ 窗口回到
/// "**体沿法向尺寸**"（1 m ⇒ ≥4 个采样步）⇒ 离散与 CCD 都能接住。
/// 接触口径：法线 = 三角环绕法线 × `sign`（`sign` 由"体心在三角哪一侧"定 ⇒ 推回**来的一侧**，
/// 不引入内外判定）、`depth = skin`、`feature = (1<<20) + ti`（与点采样的 `ti+1` 错开）。
///
/// ⚠️ 本档只覆盖**盒**；球/胶囊/圆柱/锥的"面穿过体"仍缺（登记在 `SURVEY-SHAPE-SUPPORT-MATRIX.md`
/// 与 `tests/shape_support_matrix.rs` 的矩阵/金丝雀里，**不是静默**）。
#[test]
fn mesh_shell_stops_the_box() {
    let x = mesh_shell_run(-5.0);
    println!("④ 薄壳：盒心末态 x = {x:.4}");
    assert!(
        (-0.7..=-0.3).contains(&x),
        "必须停在壳面上（盒心 ≈ −0.5）：实测 x = {x}（修前是 +395 直穿）"
    );
    assert!(
        x + HALF <= 0.02,
        "盒的前缘不许越过壳面（前缘 {}）：实测 x = {x}",
        x + HALF
    );
}

/// **⑤ 薄壳 × 相位扫描**：起始 x 偏移扫过**一整段采样长**（0.25 m，16 档）—— 修好后每一档都要停住。
/// （修前这一档会给出 16 个"直穿到 300 m 开外"的读数 ⇒ 这是这条修复的**覆盖面判据**。）
#[test]
fn mesh_shell_stops_the_box_across_phases() {
    let (mut worst, mut worst_k) = (f32::NEG_INFINITY, 0usize);
    for k in 0..16 {
        let x = mesh_shell_run(-5.0 + k as f32 * 0.25 / 16.0);
        if x > worst {
            worst = x;
            worst_k = k;
        }
    }
    assert!(
        worst <= -0.3,
        "16 个相位里最坏那档也必须停住（相位 {worst_k} 实测盒心 x = {worst}）"
    );
}

/// 薄壳档的单次跑：`d0` = 盒心起始 x，返回末态盒心 x。
fn mesh_shell_run(d0: f32) -> f32 {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(
        pts,
        vec![[0u32, 1, 2], [0, 2, 3]],
    ));
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        Vec3::new(d0, 0.0, 0.0),
        Quat::IDENTITY,
        1000.0,
    ) as usize;
    w.bodies.linvel[b] = Vec3::new(60.0, 0.0, 0.0);
    for _ in 0..TICKS {
        w.step();
    }
    w.bodies.position[b].x
}

/// **⑥ 形状矩阵（P19 覆盖面）**：同一面 4×4 m 薄壳、同一速度 60 m/s，逐形状量"末态中心 x"。
///
/// **实测（2026-10-14 复测，60 m/s × 4×4 m 单页薄壳）**：
///
/// | 形状 | 末态中心 x | 判定 |
/// |---|---|---|
/// | `Box`（0.4 立方） | **−0.402** | ✅ 拦住（`mesh_cross.rs` 的 12 棱相交：法线按**体心在哪一侧**定） |
/// | `Sphere`（r=0.4） | 395.0 | ❌ 穿（**语义冲突**，见下） |
/// | `Capsule`（0.4+0.2） | 395.0 | ❌ 穿（同上） |
/// | `Cylinder` / `Cone` | **−0.400** | ✅ 拦住（本轮：环采样按**体心在哪一侧**折法线 + 间隙深度） |
///
/// ⚠️ **球那档的根因（2026-10-14 查清，`examples/p19_mesh_shell_shapes_probe.rs`）**：
/// 查询**本来就是**半径感知的（`depth = r − sd(球心)`），问题在 `sd` 用三角的**环绕法线**定符号
/// （固定朝向）⇒ 体**从背面来**时 `sd < 0` ⇒ 深度算成 `r + 距离`（实测 **0.825 m**）且法线指向
/// "继续穿过去"的一侧 ⇒ CCD 的闭合判据（`vn < 0`）**永不成立**（逐子步：**360 采样 0 命中**）。
///
/// ⚠️⚠️ **但"折法线"这条出口被既有契约挡下**（本轮实测）：折向球心会直接把
/// `rest_height_contract.rs::mesh_sphere_inside_is_ejected` 推翻 —— 那条要求"球心在地表下
/// 0.20 m、半径 0.45 的球必须被**顶出**到 y ≈ r"，而它靠的正是**环绕法线**给出的
/// `depth = r + |sd|`（= 实体里**真的**穿透量）。同一个局部几何（球心在面背后）在
/// **薄壳**（该推回来处）与**实体**（该推出去）下的正确方向**相反** ⇒ **不是漏配，是语义**：
/// 要么给网格加"薄壳/实体"标记，要么接受"环绕法线必须朝向会来撞的那一侧"的约定。
/// ⇒ 登记为 `OPEN-PROBLEMS.md` P19 的**待裁决**项（体侧不需要这条裁决：那边 `point_shape`
/// 本来就是侧无关的口径，已在 ⑦″ 里修好并钉住）。
///
/// 本用例把**实测**钉住：拦住的那档是回归护栏（带期望停位），仍穿的三档是**已知缺口**
/// （哪天被修好会红 ⇒ 翻面成 `Some(停位)`）。
#[test]
fn mesh_shell_shape_matrix() {
    // (名字, 形状, 期望停位 x；`None` = 已知缺口、仍会穿)
    let cases: [(&str, Shape, Option<f32>); 5] = [
        (
            "Box",
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Some(-0.40),
        ),
        ("Sphere", Shape::Sphere { radius: 0.4 }, None),
        (
            "Capsule",
            Shape::Capsule {
                half_height: 0.4,
                radius: 0.2,
            },
            None,
        ),
        (
            "Cylinder",
            Shape::Cylinder {
                half_height: 0.4,
                radius: 0.4,
            },
            Some(-0.40),
        ),
        (
            "Cone",
            Shape::Cone {
                half_height: 0.4,
                radius: 0.4,
            },
            Some(-0.40),
        ),
    ];
    let mut report = String::new();
    for (name, shape, want) in cases {
        let x = mesh_shell_shape_run(shape);
        report.push_str(&format!(
            "{name}: 末态 x = {x:9.3} {}\n",
            if x > 100.0 { "穿过" } else { "拦住" }
        ));
        match want {
            Some(e) => assert!(
                (x - e).abs() < 0.05,
                "形状矩阵变了：{name} 实测 x = {x}（期望停在 {e} ± 0.05）\n{report}"
            ),
            None => assert!(
                x > 100.0,
                "已知缺口被修好了（好事）：{name} 实测 x = {x} —— 把它翻面成 `Some(停位)`\n{report}"
            ),
        }
    }
    println!("⑥ 形状矩阵（60 m/s × 4×4 m 薄壳）：\n{report}");
}

/// 形状档的单次跑：同 `mesh_shell_run`，但体形状可换（球/胶囊/圆柱/锥的起始位置也跟着调）。
fn mesh_shell_shape_run(shape: Shape) -> f32 {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    w.add_mesh(vxl_phys_terrain::mesh::TriMesh::new(
        pts,
        vec![[0u32, 1, 2], [0, 2, 3]],
    ));
    let b = w.add_dynamic(shape, Vec3::new(-5.0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    w.bodies.linvel[b] = Vec3::new(60.0, 0.0, 0.0);
    for _ in 0..TICKS {
        w.step();
    }
    w.bodies.position[b].x
}

/// **⑦ 的共用场景**：4×4 m 单页薄板（`Shape::TriMesh` **体**，位于 x = 0 平面、环绕法线 +X），
/// 以及从 `x0` 以 `v0` 沿 +X 撞来的 `shape`。`fixed` = 板设成**撞不动**（`inv_mass = 0`）。
///
/// 返回 `(world, 板体号, 撞击体号)`。
fn mesh_body_scene(fixed: bool, shape: Shape, x0: f32, v0: f32) -> (World, usize, usize) {
    let cfg = PhysConfig {
        gravity: Vec3::ZERO,
        max_linear_velocity: 5000.0,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    let pts = vec![
        Vec3::new(0.0, -2.0, -2.0),
        Vec3::new(0.0, 2.0, -2.0),
        Vec3::new(0.0, 2.0, 2.0),
        Vec3::new(0.0, -2.0, 2.0),
    ];
    let mesh = w.add_trimesh(pts, vec![[0u32, 1, 2], [0, 2, 3]]);
    let plate = w.spawn_trimesh_body(mesh, Vec3::ZERO, Quat::IDENTITY, 1.0e5, 0.02) as usize;
    if fixed {
        w.bodies.inv_mass[plate] = 0.0; // 只当障碍：金丝雀原意（密度大 ⇒ 撞不动）
        w.bodies.local_inv_inertia[plate] = Vec3::ZERO;
    }
    let b = w.add_dynamic(shape, Vec3::new(x0, 0.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    w.bodies.linvel[b] = Vec3::new(v0, 0.0, 0.0);
    (w, plate, b)
}

/// **⑦ 体侧薄壳 · 撞不动的板（P19 出口①的体侧）**：`Shape::TriMesh` 体 × 1 m 盒 60 m/s。
///
/// **修前（曾是金丝雀）**：末态 x = **395 m**（直穿、`|v|` 一路 60）—— 体-体通道（`mesh_pair`）
/// 只采**网顶点**（`point_shape` 那一路）⇒ 4×4 m 面的顶点全在盒外、盒的角点离面 0.5 m
/// ⇒ 该对**整场零候选**（不是"接触没进解算集"，是**从来没生成过接触**）。
/// **修法**（`mesh_pair → mesh_faces::box_faces`）：补"盒 14 个表面点 × 网三角"（接近量，
/// 窗口 = 点到三角的**真空距** ≤ skin）与"盒 12 棱 × 网三角"（穿越量）两向，与提供者侧同口径。
/// 判据 = **不许穿**（全程盒心 x ≤ −HALF + 余量）+ 撞上后法向速度被吃掉（60 → <1 m/s）。
#[test]
fn box_stops_on_mesh_body_shell() {
    let (mut w, _plate, b) = mesh_body_scene(
        true,
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        -5.0,
        60.0,
    );
    let mut worst = f32::MIN;
    for _ in 0..TICKS {
        w.step();
        worst = worst.max(w.bodies.position[b].x);
    }
    let x = w.bodies.position[b].x;
    let vx = w.bodies.linvel[b].x;
    println!("⑦ 体侧薄壳（板撞不动）：盒心末态 x = {x:.4}（全程最大 {worst:.4}）vx = {vx:.4}");
    assert!(
        worst < -HALF + 0.01,
        "盒不许穿过壳面（壳面 x = 0、盒半长 {HALF}）：全程最大盒心 x = {worst}"
    );
    assert!(
        vx.abs() < 1.0,
        "撞上后法向速度必须被吃掉（末态 vx = {vx}，初速 60）"
    );
}

/// **⑦′ 体侧薄壳 · 自由薄板（动量账）**：同一格几何，板是**自由动态体**（32000 kg 对 1000 kg）。
///
/// 这一档钉的是"**穿过去了没有**"与"**动量守恒不守恒**"这两件事 —— 自由板上盒**本来就该**
/// 被板带走（完全非弹性：`v = m₁v₁/(m₁+m₂)`），所以判据**不能**是"盒停在原地"
/// （上一轮把"盒没停住"当成"流形没到求解器"，是这条口径没分清 ⇒ 见 `OPEN-PROBLEMS.md` P19）。
/// 实测：盒 60 → **1.6911**、板 0 → **1.8222**，`Σp` 60000 → **60001**（0.002%）。
#[test]
fn box_vs_free_mesh_body_conserves_momentum() {
    let (mut w, plate, b) = mesh_body_scene(
        false,
        Shape::Box {
            half: Vec3::splat(HALF),
        },
        -5.0,
        60.0,
    );
    let m_box = 1.0 / w.bodies.inv_mass[b];
    let m_plate = 1.0 / w.bodies.inv_mass[plate];
    let p0 = m_box * 60.0;
    // 不许穿：盒的**前缘**（+HALF）任何一 tick 都不许跑到板的前方（余量 4·skin）。
    let mut worst_gap = f32::MIN;
    for _ in 0..TICKS {
        w.step();
        let gap = w.bodies.position[b].x + HALF - w.bodies.position[plate].x;
        worst_gap = worst_gap.max(gap);
    }
    let p1 = m_box * w.bodies.linvel[b].x + m_plate * w.bodies.linvel[plate].x;
    println!(
        "⑦′ 自由薄板：盒 vx = {:.4}、板 vx = {:.4}；Σp = {p1:.1}（初 {p0:.1}）；最大前缘越界 = {worst_gap:.4}",
        w.bodies.linvel[b].x,
        w.bodies.linvel[plate].x
    );
    assert!(
        worst_gap < 4.0 * PhysConfig::default().contact_skin,
        "盒前缘不许越过板面（最大越界 {worst_gap}）"
    );
    assert!(
        (p1 - p0).abs() < 5.0e-3 * p0,
        "完全非弹性的动量账必须守住：Σp = {p1}（初 {p0}）"
    );
}

/// **⑦″ 体侧薄壳 · 自带半径/回转体的四族（球 / 胶囊 / 圆柱 / 圆锥）**：同一格几何、同一速度。
///
/// 修前与 ⑦ 同病：体-体通道只采**网顶点**，4×4 m 面的顶点全在这些体**外面** ⇒ **整场零候选**
/// ⇒ 395 m 直穿。修法分两族（`mesh_faces.rs`）：
/// - 球/胶囊 ⇒ [`round_faces`]（沿轴采样点 × 网三角，**半径感知**：`depth = radius − 真距离`）；
/// - 圆柱/圆锥 ⇒ [`surface_hits`]（环采样点 = **表面点** ⇒ 深度取**带符号真距离**，法线按**体心**折；
///   与 provider 侧 `ring_provider_contacts` 同集合）。
///
/// 实测（板撞不动）：球 **−0.5000**、胶囊 **−0.2500**、圆柱 **−0.5000**、圆锥 **−0.5000**
/// （`vx` 全归零；修前四族都 395 m）。
#[test]
fn round_shapes_stop_on_mesh_body_shell() {
    // (名字, 形状, **沿 +X 的尺寸**)——四族的轴都取 +Y ⇒ `radius` 就是 +X 上的伸距。
    let cases: [(&str, Shape, f32); 4] = [
        ("球", Shape::Sphere { radius: HALF }, HALF),
        (
            "胶囊",
            Shape::Capsule {
                half_height: HALF * 0.5,
                radius: HALF * 0.5,
            },
            HALF * 0.5,
        ),
        (
            "圆柱",
            Shape::Cylinder {
                half_height: HALF * 0.5,
                radius: HALF,
            },
            HALF,
        ),
        (
            "圆锥",
            Shape::Cone {
                half_height: HALF * 0.5,
                radius: HALF,
            },
            HALF,
        ),
    ];
    for (name, shape, extent) in cases {
        let (mut w, _plate, b) = mesh_body_scene(true, shape, -5.0, 60.0);
        let mut worst = f32::MIN;
        for _ in 0..TICKS {
            w.step();
            worst = worst.max(w.bodies.position[b].x);
        }
        let vx = w.bodies.linvel[b].x;
        println!(
            "⑦″ {name}：末态 x = {:.4}（全程最大 {worst:.4}）vx = {vx:.4}",
            w.bodies.position[b].x
        );
        assert!(
            worst + extent <= 0.02,
            "{name} 不许穿过壳面（沿 +X 的尺寸 {extent}）：全程最大体心 x = {worst}"
        );
        assert!(
            vx.abs() < 1.0,
            "{name} 撞上后法向速度必须被吃掉（末态 vx = {vx}，初速 60）"
        );
    }
}

/// **⑦‴ 体侧 · 回转体的"低速度 × 相位"覆盖面判据**：`Shape::TriMesh` 体 × 圆柱/圆锥，
/// 速度取 **CCD 双判据都不满足**的那一段（`6–9.9 m/s`：速度 < 门槛 10、且单帧位移/最小半长
/// = `0.1/0.5 = 0.2 < 0.5`）⇒ **扫掠整趟不跑**，此时唯一的通道是"采样点落在皮肤带内"
/// （体侧窗口恒 = `2·skin` = 0.04 m，**不是** provider 侧那条速度自适应的 `velocity_band`）。
///
/// 子步位移 `v/120` 在 `v > 4.8 m/s` 时就超过那个窗口 ⇒ **相位敏感**：实测相位扫描
/// （64 档扫过一个子步位移，`examples/p19_body_side_probe.rs` 的 `PROBE_SCAN`）——
///
/// | 形状 | 修前（穿过档数，v = 6 / 7.5 / 9 / 9.9） | 修后 |
/// |---|---|---|
/// | 盒 | **0 / 0 / 0 / 0**（有 12 棱穿越通道） | 0 / 0 / 0 / 0 |
/// | 圆柱 | **13 / 23 / 29 / 33**（最坏跑到 55–61 m） | **0 / 0 / 0 / 0** |
/// | 圆锥 | 0 / 23 / 0 / 0（相位相关） | **0 / 0 / 0 / 0** |
///
/// 修法 = `mesh_faces.rs::revolution_edges`（**端面圆折线 + 母线 × 网三角**，用 `seg_tri` 判交、
/// 法线按体心定侧、深度记满皮肤带）—— 与盒的 12 棱**同一条穿越通道**（实测修后圆柱的读数与盒
/// 逐字相同）。
#[test]
fn body_revolution_stops_across_speeds_and_phases() {
    const PHASES: usize = 64;
    let cases: [(&str, Shape); 2] = [
        (
            "圆柱",
            Shape::Cylinder {
                half_height: HALF * 0.5,
                radius: HALF,
            },
        ),
        (
            "圆锥",
            Shape::Cone {
                half_height: HALF * 0.5,
                radius: HALF,
            },
        ),
    ];
    for (name, shape) in cases {
        for v in [6.0f32, 9.9] {
            let sub = v / 120.0; // 子步位移 = 离散窄相的采样周期
            let mut worst = f32::MIN;
            for k in 0..PHASES {
                let x0 = -5.0 + sub * k as f32 / PHASES as f32;
                let (mut w, _plate, b) = mesh_body_scene(true, shape, x0, v);
                for _ in 0..TICKS {
                    w.step();
                }
                worst = worst.max(w.bodies.position[b].x);
            }
            println!(
                "⑦‴ {name} v={v}：{PHASES} 相位里最坏末态体心 x = {worst:.4}（前缘 {:.4}）",
                worst + HALF
            );
            assert!(
                worst + HALF <= 0.02,
                "{name} v={v}：有相位穿过壳面（最坏体心 x = {worst}、前缘 {:.4}）",
                worst + HALF
            );
        }
    }
}
