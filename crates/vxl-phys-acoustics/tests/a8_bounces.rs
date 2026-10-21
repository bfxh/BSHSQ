//! **A8 判据**：多次反射（二阶镜像展开）。
//!
//! 场景刻意做成"**两个水平面**"（地板 `y = 0`、天花板 `y = 3`）—— 这时展开有**闭式**：
//!
//! | 路径 | 听者镜像后的高度 | 展开长度 |
//! |---|---|---|
//! | 一阶·地板 | `-y_l` | `√(Δx² + (y_s + y_l)² + Δz²)` |
//! | 一阶·天花板 | `6 - y_l` | `√(Δx² + (y_s - 6 + y_l)² + Δz²)` |
//! | 二阶·F→C | `y_l - 6` | `√(Δx² + (y_s - y_l + 6)² + Δz²)` |
//! | 二阶·C→F | `6 + y_l` | `√(Δx² + (y_s - 6 - y_l)² + Δz²)` |
//!
//! 能量 = `Π(1 - α_面)(1 - s_面) / L²`（逐面相乘、球面扩散按总程长）。判据里**不重跑**内核的
//! 展开算法，只把这些闭式与逐面系数写出来 —— 声源位置从 `impacts()` **读**（落体砸台面的接触点）。
//!
//! 六段：① 一阶两条 + 二阶两条逐条对上（时延 ±1e-3 相对）；② `reflection_order = 1`（默认）时
//! **两条二阶都不在**；③ **有效性**：把地板换成一个三角形小到装不下反射点 ⇒ 所有"碰地板"的路径
//! （一阶 + 两条二阶）全消失、只剩天花板那条；④ 挡死二阶中段的挡板 ⇒ 二阶全没、一阶还在；
//! ⑤ 预算（每阶段 `max_reflections` 面；`= 0` 一条反射都没有，且 `dropped_budget` 如实上报）；
//! ⑥ 确定性（两跑逐位一致）+ `state_hash` 不变 + `order > 2` 按 2 处理。

use vxl_phys::{BodyBuildExt, DiagExt, ImpactExt, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_acoustics::{
    AcousticConfig, AcousticGeometry, AcousticsExt, Listener, PathKind, SoundPath, BANDS,
};

const SPEED: f32 = 343.0;
/// 逐面吸声（两块面都给 0.2；散射 0 ⇒ `(1-s) = 1`）。
const ALPHA: f32 = 0.2;
/// 天花板高度。
const CEIL: f32 = 3.0;
/// 听者位置。
const LISTENER: Vec3 = Vec3 {
    x: 3.0,
    y: 1.5,
    z: 2.0,
};

/// 几何：地板（大三角）与天花板（大三角）各一块 —— `floor_big = false` 时地板换成"装不下
/// 反射点"的小三角（判据③用）。`plate = true` 时再加一块挡板（判据④用）。
struct Geom {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    scatter: Vec<[f32; BANDS]>,
}

impl Geom {
    fn new(floor_big: bool, plate: bool) -> Self {
        let mut verts = vec![
            // 地板（大：`x,z ≥ 0`、`x + z ≤ 8`；小：`x ∈ [1.5,2.5]`、`z ∈ [1.5,2.5]` 那一片）
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(8.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 8.0),
            // 天花板（跟地板同footprint，`y = 3`）
            Vec3::new(0.0, CEIL, 0.0),
            Vec3::new(8.0, CEIL, 0.0),
            Vec3::new(0.0, CEIL, 8.0),
        ];
        let mut tris = vec![[0u32, 1, 2], [3, 4, 5]];
        let mut absorb = vec![[ALPHA; BANDS]; 2];
        let mut scatter = vec![[0.0f32; BANDS]; 2];
        if !floor_big {
            // 小地板：`(1.5,0,1.5) (2.5,0,1.5) (1.5,0,2.5)` —— 手算的反射点 `(≈2.4, 0, 2)`
            // 落在它外面（`x + z ≈ 4.4 > 2.5`）⇒ 这条链不该成立。
            verts[0] = Vec3::new(1.5, 0.0, 1.5);
            verts[1] = Vec3::new(2.5, 0.0, 1.5);
            verts[2] = Vec3::new(1.5, 0.0, 2.5);
        }
        if plate {
            // 挡板：`y = 1.5` 上、`x ∈ [1.9, 3.1]`、`z ∈ [1.6, 2.4]` 的一片 —— 手算的二阶中段
            // （`p₁ ≈ (2.18,0,2) → p₂ ≈ (2.73,3,2)`）从它中间穿过；一阶那条"面 → 听者"的腿在
            // `y = 1.5` 处落在 `x ≈ 3`（`z = 2`）—— 左右都留了余量，所以只挡二阶。
            let base = verts.len() as u32;
            verts.extend([
                Vec3::new(1.9, 1.5, 1.6),
                Vec3::new(3.1, 1.5, 1.6),
                Vec3::new(3.1, 1.5, 2.4),
                Vec3::new(1.9, 1.5, 2.4),
            ]);
            tris.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
            absorb.extend([[0.2f32; BANDS]; 2]);
            scatter.extend([[0.0f32; BANDS]; 2]);
        }
        Self {
            verts,
            tris,
            absorb,
            scatter,
        }
    }

    fn geom(&self) -> AcousticGeometry<'_> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &[],
            scattering: &self.scatter,
            portals: &[],
            edges: &[],
            medium: None,
        }
    }
}

/// 声源：落体（初速 −6 m/s）砸在一块**体素台面**上 ⇒ 接触点 ≈ `(2, 1.23, 2)`（判据里**读**它）。
///
/// ⚠️ 两个坑：① 台面必须是**体素**（`impacts()` 是地形驱动的，静态盒子砸上去不记冲击）；
/// ② 得有初速（软着陆过不了峰值阈值 ⇒ 场景"空洞"）。
fn world_with_source() -> World {
    let mut w = World::new(PhysConfig::default());
    // 体素台面：`x,z ∈ [1.4, 2.6]`、`y ∈ [1.0, 1.2]`（一格高）。
    let mut vol = vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(1.4, 1.0, 1.4), 0.2, 6, 1, 6);
    vol.fill_box(Vec3::new(1.4, 1.0, 1.4), Vec3::new(2.6, 1.2, 2.6));
    w.add_voxel(vol);
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(2.0, 1.9, 2.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.bodies.linvel[b as usize] = Vec3::new(0.0, -6.0, 0.0);
    for _ in 0..120 {
        w.step();
        if !w.impacts().is_empty() {
            break;
        }
    }
    w
}

fn cfg(order: u32, max_reflections: u32) -> AcousticConfig {
    AcousticConfig {
        reflection_order: order,
        max_reflections,
        ..AcousticConfig::default()
    }
}

/// 展开长度（闭式）：`image_y` = 听者沿这条链镜像后的高度。
fn unfolded(src: Vec3, image_y: f32) -> f32 {
    let dx = LISTENER.x - src.x;
    let dz = LISTENER.z - src.z;
    let dy = src.y - image_y;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 一条路径的**手算**期望（时延, 能量）：`faces` = 逐面吸声，`image_y` = 链的镜像高度。
fn expect(src: Vec3, image_y: f32, faces: &[f32]) -> (f32, f32) {
    let l = unfolded(src, image_y);
    let mut e = 1.0f32;
    for a in faces {
        e *= 1.0 - a;
    }
    (l / SPEED, e / (l * l))
}

/// 跑一次归约。
fn cues(
    geom: &Geom,
    order: u32,
    max_reflections: u32,
) -> (Vec<SoundPath>, vxl_phys_acoustics::AcousticStats, Vec3) {
    let w = world_with_source();
    assert!(!w.impacts().is_empty(), "场景没产生声源 ⇒ 判据会空洞");
    let src = w.impacts()[0].point;
    let geo = geom.geom();
    let listener = Listener {
        pos: LISTENER,
        radius: 0.4,
    };
    let mut out = Vec::new();
    let st = w.acoustic_cues(&geo, &listener, &cfg(order, max_reflections), &mut out);
    (out, st, src)
}

/// 找一条反射路径（按手算时延认，±0.3 ms）。
fn find(paths: &[SoundPath], delay: f32) -> Option<SoundPath> {
    paths
        .iter()
        .find(|p| p.kind == PathKind::Reflected && (p.delay - delay).abs() <= 3e-4)
        .copied()
}

fn close(got: f32, want: f32, tag: &str) {
    assert!(
        (got - want).abs() <= want.abs() * 1e-3,
        "{tag}：实测 {got} vs 手算 {want}"
    );
}

/// ① 一阶两条 + 二阶两条，时延与能量逐条对上。
#[test]
fn second_order_matches_closed_form() {
    let (paths, st, src) = cues(&Geom::new(true, false), 2, 24);
    let want = [
        expect(src, -LISTENER.y, &[ALPHA]),                    // 一阶·地板
        expect(src, 2.0 * CEIL - LISTENER.y, &[ALPHA]),        // 一阶·天花板
        expect(src, LISTENER.y - 2.0 * CEIL, &[ALPHA, ALPHA]), // 二阶·F→C
        expect(src, 2.0 * CEIL + LISTENER.y, &[ALPHA, ALPHA]), // 二阶·C→F
    ];
    assert_eq!(st.direct, 1, "直达该在（两片面都不挡声源—听者的直线）");
    assert!(
        st.reflected >= 4,
        "两条一阶 + 两条二阶，实际 {}",
        st.reflected
    );
    for (i, (delay, energy)) in want.iter().enumerate() {
        // ⚠️ 用 `assert!` 而不是 `panic!`（`todo-gate` 把 `panic!` 计入棘轮 —— 与
        // `crates/vxl-phys-gpu/tests/broad_narrow_2d.rs` 同款写法）。
        let found = find(&paths, *delay);
        assert!(
            found.is_some(),
            "第 {i} 条不在（期望时延 {delay}）：{paths:?}"
        );
        let Some(p) = found else { return };
        close(p.delay, *delay, &format!("第 {i} 条时延"));
        close(p.bands[1], *energy, &format!("第 {i} 条能量"));
    }
}

/// ② 默认（`reflection_order = 1`）：两条二阶都不在，一阶照旧。
#[test]
fn default_order_has_no_second_order() {
    let (paths, st, src) = cues(&Geom::new(true, false), 1, 24);
    assert_eq!(st.reflected, 2, "只有两条一阶，实际 {}", st.reflected);
    for (tag, image_y) in [
        ("F→C", LISTENER.y - 2.0 * CEIL),
        ("C→F", 2.0 * CEIL + LISTENER.y),
    ] {
        let (delay, _) = expect(src, image_y, &[ALPHA, ALPHA]);
        assert!(
            find(&paths, delay).is_none(),
            "order = 1 时不该有 {tag}（时延 {delay}）"
        );
    }
    let (d1, _) = expect(src, -LISTENER.y, &[ALPHA]);
    assert!(find(&paths, d1).is_some(), "一阶该照旧在");
}

/// ③ **有效性**：地板换成小三角后，所有"碰地板"的链都不成立 ⇒ 只剩天花板那条一阶。
#[test]
fn reflection_point_must_land_inside_the_face() {
    let (paths, st, src) = cues(&Geom::new(false, false), 2, 24);
    assert_eq!(
        st.reflected, 1,
        "只剩天花板那条一阶（实际 {}）",
        st.reflected
    );
    let (ceil_delay, _) = expect(src, 2.0 * CEIL - LISTENER.y, &[ALPHA]);
    assert!(find(&paths, ceil_delay).is_some(), "天花板那条该还在");
    for (tag, image_y) in [
        ("一阶·地板", -LISTENER.y),
        ("二阶·F→C", LISTENER.y - 2.0 * CEIL),
        ("二阶·C→F", 2.0 * CEIL + LISTENER.y),
    ] {
        let (delay, _) = expect(src, image_y, &[ALPHA]);
        assert!(
            find(&paths, delay).is_none(),
            "{tag} 的反射点落在地板外面 ⇒ 不该产出（时延 {delay}）"
        );
    }
}

/// ④ 挡死二阶**中段**的挡板：二阶两条全没、一阶两条还在（非空洞否证）。
#[test]
fn blocked_middle_leg_kills_only_the_second_order() {
    let (paths, st, src) = cues(&Geom::new(true, true), 2, 24);
    let (d1, _) = expect(src, -LISTENER.y, &[ALPHA]);
    let (d2, _) = expect(src, 2.0 * CEIL - LISTENER.y, &[ALPHA]);
    assert!(find(&paths, d1).is_some(), "一阶·地板该还在");
    assert!(find(&paths, d2).is_some(), "一阶·天花板该还在");
    for (tag, image_y) in [
        ("F→C", LISTENER.y - 2.0 * CEIL),
        ("C→F", 2.0 * CEIL + LISTENER.y),
    ] {
        let (delay, _) = expect(src, image_y, &[ALPHA, ALPHA]);
        assert!(
            find(&paths, delay).is_none(),
            "{tag} 的中段被挡 ⇒ 不该产出（实际 reflected = {}）",
            st.reflected
        );
    }
}

/// ⑤ 预算：每阶段 `max_reflections` 面；`= 0` ⇒ 一条反射都没有且如实上报。
#[test]
fn budget_is_per_stage_and_reported() {
    let (paths, st, src) = cues(&Geom::new(true, false), 2, 1);
    let second = paths
        .iter()
        .filter(|p| p.kind == PathKind::Reflected)
        .filter(|p| {
            let (d, _) = expect(src, LISTENER.y - 2.0 * CEIL, &[ALPHA, ALPHA]);
            (p.delay - d).abs() <= 3e-4
        })
        .count();
    assert_eq!(second, 1, "每阶段只试 1 面 ⇒ 只剩 F→C 那条，实际 {second}");
    assert!(st.dropped_budget > 0, "顶满预算要如实上报");
    let (paths, st, _) = cues(&Geom::new(true, false), 2, 0);
    assert_eq!(st.reflected, 0, "预算 0 ⇒ 一条反射都没有");
    assert!(paths.iter().all(|p| p.kind != PathKind::Reflected));
}

/// ⑥ 确定性 + `state_hash` 不变 + `order > 2` 按 2 处理。
#[test]
fn determinism_read_only_and_order_cap() {
    let geom = Geom::new(true, false);
    let (a, _, _) = cues(&geom, 2, 24);
    let (b, _, _) = cues(&geom, 2, 24);
    assert_eq!(a, b, "同输入两跑必须逐位一致");
    let (c, _, _) = cues(&geom, 5, 24);
    assert_eq!(a, c, "`reflection_order > 2` 按 2 处理（与 2 结果相同）");
    let mut w = world_with_source();
    let before = w.state_hash();
    let geo = geom.geom();
    let listener = Listener {
        pos: LISTENER,
        radius: 0.4,
    };
    let mut out = Vec::new();
    let _ = w.acoustic_cues(&geo, &listener, &cfg(2, 24), &mut out);
    assert_eq!(w.state_hash(), before, "声学层必须只读（state_hash 不变）");
}
