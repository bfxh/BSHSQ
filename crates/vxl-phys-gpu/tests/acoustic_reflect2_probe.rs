//! **二阶反射核的探针（GPU；需适配器）**：`声源 →(面 A) →(面 B) → 听者` —— 与**两平行平面的
//! 闭式展开**对拍。
//!
//! 场景刻意做成"**两个水平面**"（地板 `y = 0`、天花板 `y = 3`）—— 这时展开有**闭式**
//! （从 CPU 的 A8 判据 `crates/vxl-phys-acoustics/tests/a8_bounces.rs` 抄来的同一张表）：
//!
//! | 路径 | 听者镜像后的高度 | 展开长度 |
//! |---|---|---|
//! | 一阶·地板 | `-y_l` | `√(Δx² + (y_s + y_l)² + Δz²)` |
//! | 一阶·天花板 | `6 - y_l` | `√(Δx² + (y_s - 6 + y_l)² + Δz²)` |
//! | 二阶·F→C | `y_l - 6` | `√(Δx² + (y_s - y_l + 6)² + Δz²)` |
//! | 二阶·C→F | `6 + y_l` | `√(Δx² + (y_s - 6 - y_l)² + Δz²)` |
//!
//! 能量 = `Π(1 - α_面) / L²`。判据里**不重跑**卡上的展开算法（镜像听者 + 逐段求交 + 内点 + 遮挡），
//! 只把这些闭式写出来 ⇒ 与卡上不是同一份实现，非自证。
//!
//! ⚠️ **卡上不做预算截断**（与 M1–M5 同口径）⇒ "与 CPU `reflection_order = 2` 逐对一致"的**前提**
//! 是 CPU 侧 `max_reflections ≥ tris.len()`。本探针用**几何否定**（反射点出界 / 单面 / 中段被挡）来钉
//! 有效性、阶数上界与遮挡，而不是去调 CPU（`vxl-phys-gpu` 反过来依赖 `vxl-phys-acoustics` 会撞
//! 本仓 `dag-gate`，见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9 坑 #4）。
//!
//! ⚠️ 无适配器 ⇒ 跳过（`[SKIP]`）；本仓 CI 的口径 `--exclude vxl-phys-gpu` ⇒ 只在有卡的机器上跑。

use vxl_phys_core::Vec3;
use vxl_phys_gpu::acoustic::{
    reflected2_on_adapter, reflected_on_adapter, Reflect2Out, ReflectOut,
};

const SPEED: f32 = 343.0;
/// 逐面吸声（两平面都给 0.2；散射不参与 —— 卡上 `reflect2` 不吃散射系数）。
const ALPHA: f32 = 0.2;
/// 天花板高度。
const CEIL: f32 = 3.0;
/// 声源：与 CPU A8 判据同一个点（落体砸台面后的接触点 ≈ `(2, 1.23, 2)`）。
const SRC: Vec3 = Vec3 {
    x: 2.0,
    y: 1.23,
    z: 2.0,
};
/// 听者位置。
const DST: Vec3 = Vec3 {
    x: 3.0,
    y: 1.5,
    z: 2.0,
};
/// 一阶·地板：听者镜像到 `-y_l`。
const IMG_F1: f32 = -DST.y;
/// 一阶·天花板：镜像到 `2·CEIL - y_l`。
const IMG_F2: f32 = 2.0 * CEIL - DST.y;
/// 二阶·地板→天花板：镜像到 `y_l - 2·CEIL`。
const IMG_FC: f32 = DST.y - 2.0 * CEIL;
/// 二阶·天花板→地板：镜像到 `2·CEIL + y_l`。
const IMG_CF: f32 = 2.0 * CEIL + DST.y;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 地板（大三角；`y = 0`、`x,z ≥ 0`、`x + z ≤ 8`）。
fn floor_big_tri() -> [Vec3; 3] {
    [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(8.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 8.0),
    ]
}

/// 小地板（同一平面上的一小片）：手算的反射点 `(≈2.4, 0, 2)` 与二阶两个反射点都落在它**外面**
/// （`x + z ≈ 4.4 > 4`）⇒ 所有"碰地板"的链都不成立。
fn floor_small_tri() -> [Vec3; 3] {
    [
        Vec3::new(1.5, 0.0, 1.5),
        Vec3::new(2.5, 0.0, 1.5),
        Vec3::new(1.5, 0.0, 2.5),
    ]
}

/// 天花板（与地板同 footprint，`y = CEIL`）。
fn ceil_tri() -> [Vec3; 3] {
    [
        Vec3::new(0.0, CEIL, 0.0),
        Vec3::new(8.0, CEIL, 0.0),
        Vec3::new(0.0, CEIL, 8.0),
    ]
}

/// 挡板（`y = 1.5` 上一片，两块三角）：手算的二阶**中段**（`p₁ ≈ (2.18,0,2) → p₂ ≈ (2.73,3,2)`）
/// 从它中间穿过；一阶"面 → 听者"那条腿在 `y = 1.5` 处落在**端点**上（`hits` 端点共面不计）⇒ 只挡二阶。
fn plate_tris() -> [[Vec3; 3]; 2] {
    [
        [
            Vec3::new(1.9, 1.5, 1.6),
            Vec3::new(3.1, 1.5, 1.6),
            Vec3::new(3.1, 1.5, 2.4),
        ],
        [
            Vec3::new(1.9, 1.5, 1.6),
            Vec3::new(3.1, 1.5, 2.4),
            Vec3::new(1.9, 1.5, 2.4),
        ],
    ]
}

/// 展开长度（闭式）：`image_y` = 听者沿这条链镜像后的高度。
fn unfolded(image_y: f32) -> f32 {
    let dx = DST.x - SRC.x;
    let dy = SRC.y - image_y;
    let dz = DST.z - SRC.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 一条路径的**手算**期望 `(时延, 能量)`：`faces` = 链上碰了几张面（逐面吸声相乘）。
fn expect(image_y: f32, faces: usize) -> (f32, f32) {
    let l = unfolded(image_y);
    let mut e = 1.0f32;
    for _ in 0..faces {
        e *= 1.0 - ALPHA;
    }
    (l / SPEED, e / (l * l))
}

/// 场景 = 三角表 + 逐三角吸收表。
struct Scene {
    tris: Vec<[Vec3; 3]>,
    absorb: Vec<f32>,
}

/// 两平面场景（`small_floor = true` ⇒ 地板换成小三角）。
fn two_planes(small_floor: bool) -> Scene {
    let f = if small_floor {
        floor_small_tri()
    } else {
        floor_big_tri()
    };
    Scene {
        tris: vec![f, ceil_tri()],
        absorb: vec![ALPHA, ALPHA],
    }
}

/// 两平面 + 挡板：挡板给**全吸收** ⇒ 它自己抢不到"最强"那一位（只遮不反），与既有 M2 探针同款。
fn two_planes_plus_plate() -> Scene {
    let mut tris = vec![floor_big_tri(), ceil_tri()];
    tris.extend(plate_tris());
    Scene {
        tris,
        absorb: vec![ALPHA, ALPHA, 1.0, 1.0],
    }
}

/// `Err`（适配器/核起不来）⇒ 打 `[SKIP]` 并给 `None` —— 环境问题不该判失败。
fn soft<T>(r: Result<T, String>) -> Option<T> {
    match r {
        Ok(v) => Some(v),
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            None
        }
    }
}

/// 跑一趟 M6（二阶；单对）。
fn run2(sc: &Scene) -> Result<(String, Reflect2Out), String> {
    let (name, mut got) = reflected2_on_adapter(0, &[SRC], &[DST], &sc.tris, &sc.absorb, SPEED)?;
    assert_eq!(got.len(), 1, "单对 ⇒ 读数该是 1 条（实际 {}）", got.len());
    Ok((name, got.remove(0)))
}

/// 跑一趟 M2（一阶；单对）。
fn run1(sc: &Scene) -> Result<(String, ReflectOut), String> {
    let (name, mut got) = reflected_on_adapter(0, &[SRC], &[DST], &sc.tris, &sc.absorb, SPEED)?;
    assert_eq!(got.len(), 1, "单对 ⇒ 读数该是 1 条（实际 {}）", got.len());
    Ok((name, got.remove(0)))
}

fn close(got: f32, want: f32, tag: &str) {
    assert!(
        (got - want).abs() <= want.abs() * 1e-3,
        "{tag}：卡上 {got} vs 手算 {want}"
    );
}

/// 六段顺序跑（**并行跑会各建一个 wgpu 设备，实测会互相卡住** ⇒ 合成一条）。
#[test]
fn second_order_matches_closed_form_and_denials() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    // ① 两平面：二阶 = **最强那条**（F→C，比 C→F 短），时延/能量对上手算；第一跳报在地板（三角 0）。
    let sc = two_planes(false);
    let Some((name, r2)) = soft(run2(&sc)) else {
        return;
    };
    let (want_delay, want_gain) = expect(IMG_FC, 2);
    let (_, other_gain) = expect(IMG_CF, 2);
    println!(
        "[gpu] {name}：二阶 delay {:.6} gain {:.8} face {:.0}（手算 delay {want_delay:.6} gain {want_gain:.8}）",
        r2.delay, r2.gain, r2.face
    );
    assert!(r2.hit, "两平面之间该有二阶路径（否则判据空跑）");
    close(r2.delay, want_delay, "二阶·F→C 时延");
    close(r2.gain, want_gain, "二阶·F→C 能量");
    assert!(
        r2.gain > other_gain,
        "C→F 那条更弱（{other_gain}）⇒ 该被 F→C（{}）压过",
        r2.gain
    );
    assert_eq!(r2.face, 0.0, "两跳链的第一跳落在地板上（三角 0）");
    // 同一片上，一阶（M2）读数必须与手算的地板那条对上 —— `blocked` 收成 `blocked2` 的单面退化
    // 之后**逐位不变**（M2 既有探针是主护栏，这里顺手再钉一次）。
    let Some((_, r1)) = soft(run1(&sc)) else {
        return;
    };
    let (d1, g1) = expect(IMG_F1, 1);
    assert!(r1.hit, "一阶该在（两平面都不挡声源—听者那条腿）");
    close(r1.delay, d1, "一阶·地板 时延");
    close(r1.gain, g1, "一阶·地板 能量");
    assert!(
        r2.gain < r1.gain,
        "两跳该比一阶弱（{} vs {}）",
        r2.gain,
        r1.gain
    );

    // ② **阶数上界**：只有一张面 ⇒ 没有"第二张面"可碰（`A == B` 的退化链被跳过）⇒ 二阶一条都不产。
    let one = Scene {
        tris: vec![floor_big_tri()],
        absorb: vec![ALPHA],
    };
    let two = soft(run2(&one)).is_some_and(|(_, v)| !v.hit);
    let one1 = soft(run1(&one)).is_some_and(|(_, v)| v.hit);
    println!("[gpu] 单面场景：二阶不产 = {two}，一阶照旧 = {one1}");
    assert!(two, "单面场景不该有二阶路径");
    assert!(one1, "同一片上的一阶该照旧在（否则②是空跑）");

    // ③ **有效性**：地板换成小三角 ⇒ 所有"碰地板"的链（一阶地板 + 两条二阶）都不成立。
    let small = two_planes(true);
    let no2 = soft(run2(&small)).is_some_and(|(_, v)| !v.hit);
    let (d2, g2) = expect(IMG_F2, 1);
    let Some((_, c1)) = soft(run1(&small)) else {
        return;
    };
    println!(
        "[gpu] 小地板场景：二阶不产 = {no2}，一阶天花板 hit = {}",
        c1.hit
    );
    assert!(no2, "两条二阶链都要碰地板 ⇒ 该一条都不产");
    assert!(c1.hit, "天花板那条一阶该还在");
    close(c1.delay, d2, "小地板场景下的一阶·天花板 时延");
    close(c1.gain, g2, "小地板场景下的一阶·天花板 能量");

    // ④ **中段被挡**：全吸收挡板切在二阶中段上 ⇒ 二阶全没；一阶地板那条照旧（挡板增益 0 抢不走）。
    let plate = two_planes_plus_plate();
    let no2b = soft(run2(&plate)).is_some_and(|(_, v)| !v.hit);
    let Some((_, p1)) = soft(run1(&plate)) else {
        return;
    };
    println!(
        "[gpu] 带挡板场景：二阶不产 = {no2b}，一阶地板 hit = {}",
        p1.hit
    );
    assert!(no2b, "二阶中段被挡死 ⇒ 不该再有二阶路径");
    assert!(p1.hit, "一阶地板那条该还在（非空洞否证）");
    close(p1.delay, d1, "带挡板场景下的一阶·地板 时延");

    // ⑤ 守卫：逐三角表与三角表不一一对应 ⇒ 直接报错（核内是直接索引 ⇒ 给短了就是越界读）。
    let bad = reflected2_on_adapter(0, &[SRC], &[DST], &sc.tris, &[0.1], SPEED);
    assert!(bad.is_err(), "表长不符该报错（实际 {bad:?}）");

    // ⑥ 确定性：同输入两跑**逐位一致**（卡上全枚举 + `>` 严格比较，无随机、无原子归约）。
    let Some((_, a)) = soft(run2(&sc)) else {
        return;
    };
    let Some((_, b)) = soft(run2(&sc)) else {
        return;
    };
    assert_eq!(a, b, "同输入两跑必须逐位一致");
}
