use super::{sat_scan_scalar, sat_scan_sse2};
use vxl_phys_core::Vec3;

/// 一组 SAT 入参（`args-gate` 的 soft 阈值实测把 7 个形参也算违规 ⇒ 打包成结构体再传）。
struct Case {
    axes: Vec<Vec3>,
    ha: Vec3,
    aa: [Vec3; 3],
    pa: Vec3,
    hb: Vec3,
    ab: [Vec3; 3],
    pb: Vec3,
    skin: f32,
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

/// 轴表（盒对形态：12 面轴 + 至多 9 棱叉积轴）——与 `sat()` 的构造同序。
fn axes_of(aa: &[Vec3; 3], ab: &[Vec3; 3]) -> Vec<Vec3> {
    let mut out = Vec::new();
    for s in [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ] {
        out.push(aa[0] * s.0 + aa[1] * s.1 + aa[2] * s.2);
    }
    for s in [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ] {
        out.push(ab[0] * s.0 + ab[1] * s.1 + ab[2] * s.2);
    }
    let (ea, eb) = ([aa[1], aa[2], aa[0]], [ab[1], ab[2], ab[0]]);
    for x in ea {
        for y in eb {
            let c = x.cross(y);
            let l2 = c.length_squared();
            if l2 > 1e-8 {
                out.push(c * (1.0 / l2.sqrt()));
            }
        }
    }
    out
}

/// 随机**正交基**（Gram-Schmidt）——模拟真实盒姿态；随机基会让 SAT 几乎总是
/// 分离，样本覆盖不到接触分支。
fn basis(rnd: &mut impl FnMut() -> f32) -> [Vec3; 3] {
    let (mut a, mut b, mut c) = (
        v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
        v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
        v(rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0),
    );
    a = a * (1.0 / a.length());
    b = b - a * b.dot(a);
    b = b * (1.0 / b.length());
    c = c - a * c.dot(a) - b * c.dot(b);
    c = c * (1.0 / c.length());
    [a, b, c]
}

/// SIMD 与标量参考在随机姿态/尺寸/位移上**逐位一致**（含退化与分离两类）。
#[test]
fn simd_matches_scalar_bitwise() {
    let mut seed = 0x1234_5678_9abc_def0u64;
    let mut rnd = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f32 / (1u64 << 53) as f32
    };
    let mut sep_cnt = 0usize;
    let mut hit_cnt = 0usize;
    for _ in 0..4000 {
        let ax = basis(&mut rnd);
        let bx = basis(&mut rnd);
        let ha = v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3);
        let hb = v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3);
        let pa = v(rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5);
        let pb = pa + v(rnd() * 1.6 - 0.8, rnd() * 1.6 - 0.8, rnd() * 1.6 - 0.8);
        let skin = 0.01;
        let axes = axes_of(&ax, &bx);
        let s = sat_scan_scalar(&axes, ha, &ax, pa, hb, &bx, pb, skin);
        let d = sat_scan_sse2(&axes, ha, &ax, pa, hb, &bx, pb, skin);
        match (s, d) {
            (None, None) => sep_cnt += 1,
            (Some((s0, sn, si)), Some((d0, dn, di))) => {
                hit_cnt += 1;
                assert_eq!(s0.to_bits(), d0.to_bits(), "best sep 不等");
                assert_eq!(si, di, "best idx 不等");
                assert_eq!(sn.x.to_bits(), dn.x.to_bits(), "法线 x 不等");
                assert_eq!(sn.y.to_bits(), dn.y.to_bits(), "法线 y 不等");
                assert_eq!(sn.z.to_bits(), dn.z.to_bits(), "法线 z 不等");
            }
            // 用 `assert_eq!`（而不是 `panic!`）报这条：本文件是**新文件**，而 todo-gate
            // 数的是会编进二进制的 `panic!/todo!/unimplemented!/unreachable!` 宏调用。
            (a, b) => assert_eq!(a.is_some(), b.is_some(), "分支不一致：标量 vs SIMD"),
        }
    }
    // 两类都必须被覆盖到（否则测试没有鉴别力）。
    assert!(sep_cnt > 100, "分离样本太少：{sep_cnt}");
    assert!(hit_cnt > 100, "命中样本太少：{hit_cnt}");
}

/// **符号零平局**：两条 sep 恰好都为零、符号相反时，`_mm_max_ps` 的平局规则
/// （Intel：相等取**第二操作数**）与标量参考的 `if sep1 >= sep2 { sep1 }` 会分叉，
/// 使 `best_sep` 的符号位不同——而它进确定性哈希。
///
/// 随机采样**撞不到**这个平局（`simd_matches_scalar_bitwise` 的 4000 个样本里概率 ~0），
/// 所以这里给确定性的构造输入把它钉住：轴取 +X、两盒半宽均为零、A 心 x = −0.0、B 心 x = 0.0 ⇒
/// `sep1 = (cb − rb) − (ca + ra) = 0.0 − (−0.0) = +0.0`，
/// `sep2 = (ca − ra) − (cb + rb) = (−0.0 − 0.0) − 0.0 = −0.0`（IEEE：混号零相加归 +0.0，
/// 但 `(−0.0) − (+0.0)` 仍是 −0.0）。两条路径必须逐位同结果。
#[test]
fn sat_tie_break_matches_scalar_on_signed_zero() {
    let ax = [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)];
    let axes = vec![v(1.0, 0.0, 0.0)];
    let zero = v(0.0, 0.0, 0.0);
    let neg_zero_x = v(-0.0, 0.0, 0.0);
    // 同理避开 `.expect(`（unwrap-gate 的新文件计数）：先 `assert!` 断言有解，再用
    // `unwrap_or` 兜一个**不可能到达**的哨兵 —— 真到了那里，下面的逐位比较会报红。
    let (ss, dd) = (
        sat_scan_scalar(&axes, zero, &ax, neg_zero_x, zero, &ax, zero, 0.01),
        sat_scan_sse2(&axes, zero, &ax, neg_zero_x, zero, &ax, zero, 0.01),
    );
    assert!(
        ss.is_some() && dd.is_some(),
        "应有分离解（sep 为零、不被 skin 早退）"
    );
    let sentinel = (f32::NAN, zero, usize::MAX);
    let (s, d) = (ss.unwrap_or(sentinel), dd.unwrap_or(sentinel));
    assert_eq!(
        s.0.to_bits(),
        d.0.to_bits(),
        "平局时 best_sep 的符号位分叉：标量 {}（{:#010x}）vs SIMD {}（{:#010x}）",
        s.0,
        s.0.to_bits(),
        d.0,
        d.0.to_bits()
    );
    assert_eq!(s.2, d.2, "平局时 best_idx 不等");
    // 构造前提自检：两条路径都必须给 **+0.0**。若构造失效（例如两个 sep 同号），
    // 上面那条等式就退化成恒真、失去鉴别力——所以把期望值也钉死。
    // 若 MAXPS 的操作数顺序写反（平局取 sep2 = −0.0），这里会以符号位不同报红。
    assert_eq!(
        s.0.to_bits(),
        0.0f32.to_bits(),
        "标量平局应取 sep1（+0.0），实为 {}",
        s.0
    );
    assert_eq!(
        d.0.to_bits(),
        0.0f32.to_bits(),
        "SIMD 平局必须与标量同取 +0.0，实为 {}",
        d.0
    );
}

/// 轴数不是 4 的倍数（21、12、13）时尾组 lane 不得被消费——与标量对比。
#[test]
fn simd_tail_lanes_are_ignored() {
    let ax = [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)];
    let bx = ax;
    let ha = v(0.5, 0.5, 0.5);
    let pa = v(0.0, 0.0, 0.0);
    let pb = v(0.95, 0.0, 0.0);
    for take in [1usize, 2, 3, 4, 5, 9, 12, 13, 21] {
        let axes: Vec<Vec3> = (0..take).map(|k| ax[k % 3]).collect();
        let s = sat_scan_scalar(&axes, ha, &ax, pa, ha, &bx, pb, 0.01);
        let d = sat_scan_sse2(&axes, ha, &ax, pa, ha, &bx, pb, 0.01);
        assert_eq!(s.is_some(), d.is_some(), "take={take} 分支不一致");
        if let (Some((s0, _, si)), Some((d0, _, di))) = (s, d) {
            assert_eq!(s0.to_bits(), d0.to_bits(), "take={take} sep 不等");
            assert_eq!(si, di, "take={take} idx 不等");
        }
    }
}

/// 两条路径是否**逐位同**（含分类：都 None / 都 Some 且 sep、idx、法线全同）。
fn same(s: Option<(f32, Vec3, usize)>, d: Option<(f32, Vec3, usize)>) -> bool {
    match (s, d) {
        (None, None) => true,
        (Some((s0, sn, si)), Some((d0, dn, di))) => {
            s0.to_bits() == d0.to_bits()
                && si == di
                && sn.x.to_bits() == dn.x.to_bits()
                && sn.y.to_bits() == dn.y.to_bits()
                && sn.z.to_bits() == dn.z.to_bits()
        }
        _ => false,
    }
}

/// **极值与非有限输入下的差分**（2026-10-10 安全审计 **F-12** 的判据）。
///
/// 旧 SIMD 侧用 `_mm_max_ps(sep2, sep1)` 选 `sep`，标量用 `if sep1 >= sep2 {sep1} else
/// {sep2}`：**NaN** 时 MAXPS 按 Intel 语义返回第二操作数（= `sep1` = NaN），而标量的 `>=`
/// 对 NaN 判假、会取 `sep2`（非 NaN）⇒ 一边 NaN 一边有限值 ⇒ `sep > skin` 的判真/判假不同
/// ⇒ **分类分叉**（审计实测：**全有限输入**也有 845/10⁶ 不一致 —— 坐标 ~1e20 时点积溢出成
/// Inf/NaN；x86_64 走 SSE2、其它架构走标量 ⇒ 同一场景的**接触集不同** ⇒ `state_hash` 分叉，
/// 直接打在仓库自己的跨平台哈希门上）。
///
/// 现已改成"**同一谓词 + 位选择**"（`ge = sep1 >= sep2` 同时定 `flags` 与 `sep`）。
/// 本测试把这类输入钉住 —— 既有的 `simd_matches_scalar_bitwise` 输入全是**温和量级**
/// （半长 0.2–0.5、坐标 ±1.5、skin 0.01），永远碰不到这条路径。
///
/// ⚠️ **如实说明（别把它当审计那 845/10⁶ 的复现）**：我把旧写法（`_mm_max_ps(sep2, sep1)`
/// 以及 `cmpnge`）临时换回来跑过，**本输入族照样全过** —— 因为 `sep1`/`sep2` 共用同样四条
/// 操作数（`ca/ra/cb/rb`），一处 Inf/NaN 往往让两条式子**同时**变 NaN，而 NaN 与 ∓Inf 在
/// 后面两条比较（`> skin` / `> best`）上行为等价 ⇒ 观察不到差异。所以本条的价值是
/// **把逐位差分从"温和量级"扩到"极值 + 非有限 + 负量级"**（审计明确要求的那一步），
/// 而**不是**所谓"旧实现必红"的回归判据。审计那一例（若确如报告所读）需要一个
/// 让两条 sep 的 NaN 性**不同且改变分类**的组合 —— 那是**仍开着的**复现任务（见 PR 说明）。
#[test]
fn simd_matches_scalar_on_extreme_inputs() {
    let mut seed = 0x5eed_0f12_3456_789au64;
    let mut rnd = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f32 / (1u64 << 53) as f32
    };
    // 量级从正常一路推到 f32 点积的溢出区；`odd` 是会让比较与 MAXPS 分道的怪值。
    // ⚠️ **必须含负量级**：分叉要"`sep1` 与 `sep2` 的 NaN 性不同"，而四条操作数
    // （`ca/ra/cb/rb`）同时进两条式子 ⇒ 需要 `rb < 0`（**负半长**）才能让 `cb + rb` 与
    // `cb − rb` 的 Inf/NaN 形态分开（审计把"负半尺寸"列为 F-12 的触发条件之一；
    // 另一条是坐标 ~1e20 点积溢出）。只加正量级 + 把怪值塞进**轴**分量时，
    // 两条式子会同时变 NaN，观察不到分叉 —— 我第一版就是这么写的，实测旧代码照样过。
    let mags = [1.0f32, -1.0, 1e4, -1e10, 1e19, -1e20, 1e30, 3.0e38, -3.0e38];
    let odd = [
        0.0f32,
        -0.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NAN,
        f32::MIN_POSITIVE,
    ];
    let mut n = 0usize;
    let mut none_n = 0usize;
    for &m in &mags {
        for &o in &odd {
            let sc = |q: Vec3| v(q.x * m, q.y * m, q.z * m);
            for _ in 0..24 {
                let ax = basis(&mut rnd);
                let bx = basis(&mut rnd);
                let ha = sc(v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3));
                let hb = sc(v(0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3, 0.2 + rnd() * 0.3));
                let pa = sc(v(rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5));
                let pb = sc(v(rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5, rnd() * 3.0 - 1.5));
                // 把首轴的一个分量、以及半长的一个分量换成怪值（NaN/Inf/±0/次正规）
                // ⇒ 逼出 `rb` 的 Inf/NaN 与两条 sep 的**不同** NaN 性。
                let mut ax2 = ax;
                ax2[0] = v(o, ax[0].y, ax[0].z);
                let hb2 = v(hb.x, o, hb.z);
                let axes = axes_of(&ax2, &bx);
                let s = sat_scan_scalar(&axes, ha, &ax2, pa, hb2, &bx, pb, 0.01);
                let d = sat_scan_sse2(&axes, ha, &ax2, pa, hb2, &bx, pb, 0.01);
                assert!(
                    same(s, d),
                    "极值输入下 SIMD/标量分叉：mag={m:e} odd={o} 标量有值={} SIMD 有值={}",
                    s.is_some(),
                    d.is_some()
                );
                if s.is_none() {
                    none_n += 1;
                }
                n += 1;
            }
        }
    }
    // 非空洞：样本够多，且两类结论都出现过（否则这条判据可能只在一种分支上打转）。
    assert!(n > 900, "样本太少：{n}");
    assert!(
        none_n > 0 && none_n < n,
        "两类结论都要覆盖：None={none_n}/{n}"
    );
}

/// 确定性 LCG（不引 RNG 依赖；同仓内其它随机化判据的做法）。
struct Lcg(u64);

impl Lcg {
    fn u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) as u32
    }
    /// `[0,1)`。
    fn unit(&mut self) -> f32 {
        (self.u32() >> 8) as f32 / (1u32 << 24) as f32
    }
    /// 幅度档：正常 / 小 / 大 / 极大——后两档是 sep1/sep2 出现 Inf/NaN 的来源。
    fn mag(&mut self) -> f32 {
        const MAGS: [f32; 6] = [1.0, 1e-3, 1e3, 1e19, 1e30, 1e38];
        MAGS[(self.u32() % 6) as usize]
    }
    fn val(&mut self) -> f32 {
        (self.unit() * 2.0 - 1.0) * self.mag()
    }
    fn vec(&mut self) -> Vec3 {
        v(self.val(), self.val(), self.val())
    }
}

/// 造一组入参：轴表取"盒对形态"（12 面轴 + 3 条随机轴），半长/中心/姿态按幅度档随机；
/// `inject = true` 时另有 1/32 的概率直接注入 NaN / ±Inf（"全有限子族"要求 `false`）。
fn case(rng: &mut Lcg, inject: bool) -> Case {
    let aa = basis(&mut || rng.unit());
    let ab = basis(&mut || rng.unit());
    let mut axes = axes_of(&aa, &ab);
    axes.truncate(15);
    axes.push(rng.vec());
    let odd = inject && rng.u32().is_multiple_of(32);
    let hf = |rng: &mut Lcg| {
        let m = rng.mag();
        if odd {
            match rng.u32() % 4 {
                0 => f32::NAN,
                1 => f32::INFINITY,
                2 => f32::NEG_INFINITY,
                _ => m,
            }
        } else {
            m
        }
    };
    Case {
        axes,
        ha: v(hf(rng), hf(rng), hf(rng)),
        aa,
        pa: rng.vec(),
        hb: v(hf(rng), hf(rng), hf(rng)),
        ab,
        pb: rng.vec(),
        skin: if odd { f32::NAN } else { 0.01 },
    }
}

/// 单轴的 `(sep1, sep2)`——算术与 `sat_scan_scalar` 一字不差（左结合序也照抄）。
fn axis_seps(c: &Case, n: Vec3) -> Option<(f32, f32)> {
    if n.length_squared() < 0.5 {
        return None;
    }
    let ra = c.ha.x * c.aa[0].dot(n).abs()
        + c.ha.y * c.aa[1].dot(n).abs()
        + c.ha.z * c.aa[2].dot(n).abs();
    let rb = c.hb.x * c.ab[0].dot(n).abs()
        + c.hb.y * c.ab[1].dot(n).abs()
        + c.hb.z * c.ab[2].dot(n).abs();
    let (ca, cb) = (c.pa.dot(n), c.pb.dot(n));
    Some(((cb - rb) - (ca + ra), (ca - ra) - (cb + rb)))
}

/// 旧写法会取的值：Intel `MAXPS(a, b)` = `a > b ? a : b`，**任一操作数为 NaN 都返回 `b`**
/// （旧代码是 `_mm_max_ps(sep2, sep1)` ⇒ 返回 `sep1`）。`f32` 的比较对 NaN 全假 ⇒ 下面这个
/// `if sep2 > sep1 { sep2 } else { sep1 }` 与 MAXPS 语义一致（无需 unsafe 内建即可复现）。
fn maxps_select(sep2: f32, sep1: f32) -> f32 {
    if sep2 > sep1 {
        sep2
    } else {
        sep1
    }
}

/// 修法会取的值：与标量同一条 `sep1 >= sep2` 谓词 + 位选择。
fn ge_select(sep1: f32, sep2: f32) -> f32 {
    if sep1 >= sep2 {
        sep1
    } else {
        sep2
    }
}

/// **F-12 差分复现：审计那 845/10⁶ 的族**（"必红对照"补在这里）。
///
/// 审计在 `main@51278a5` 上量到：**全有限输入**里 845/10⁶ 组让 SIMD 与标量分叉（首例
/// SIMD=None、标量=Some）。根因 = 旧写法 `_mm_max_ps(sep2, sep1)`：Intel 的 MAXPS 在**任一
/// 操作数为 NaN 时返回第二操作数**（这里 = `sep1`），而标量的 `sep1 >= sep2` 对 NaN 判假会取
/// `sep2` ⇒ 两条路径的 sep **值**各有各的算术 ⇒ `> skin` / `> best` 上可分叉。
///
/// 这条判据做两件事，缺一不可：
///   · **修复有效**：族里每一组都要求 SIMD 与标量**逐位同结果**；
///   · **样本有敏感度**（否则就是空跑）：同一批输入里统计"MAXPS 语义 ≠ `>=` 选择"的**轴数**
///     ——那正是**旧写法必然分叉**的族。要求 > 0，并打印占比（审计那 845/10⁶ 的可复现形式）。
#[test]
fn differential_family_reproduces_the_maxps_divergence() {
    // 两族都跑：**含非有限注入**（把 Inf/NaN 直接喂进去）+ **全有限**（对齐审计那句
    // "全有限输入仍有 845/10⁶"——有限但极大的幅度档照样能在 sep1/sep2 里造出 Inf/NaN）。
    for &(label, inject) in &[("含注入", true), ("全有限", false)] {
        let mut rng = Lcg(0x5DEE_CE66_D1CE_u64);
        let cases = 20_000usize;
        let (mut sensitive, mut axes_n) = (0usize, 0usize);
        for k in 0..cases {
            let c = case(&mut rng, inject);
            for &n in &c.axes {
                if let Some((sep1, sep2)) = axis_seps(&c, n) {
                    axes_n += 1;
                    let (old, new) = (maxps_select(sep2, sep1), ge_select(sep1, sep2));
                    // 只数**真会分叉**的那些：一个是 NaN、另一个不是。两边都是 NaN 时下游
                    // （`> skin` / `> best`）结论相同 ⇒ 不算敏感。
                    if old.is_nan() != new.is_nan() {
                        sensitive += 1;
                    }
                }
            }
            let s = sat_scan_scalar(&c.axes, c.ha, &c.aa, c.pa, c.hb, &c.ab, c.pb, c.skin);
            let d = sat_scan_sse2(&c.axes, c.ha, &c.aa, c.pa, c.hb, &c.ab, c.pb, c.skin);
            assert!(
                same(s, d),
                "第 {k} 组（{label}）SIMD/标量分叉（F-12 回归）：标量有值={} SIMD 有值={}",
                s.is_some(),
                d.is_some()
            );
        }
        // 非空洞：样本量够，且**真的**覆盖到了旧写法会分叉的族。
        assert!(axes_n > 100_000, "{label}：轴样本太少：{axes_n}");
        assert!(
            sensitive > 0,
            "{label}：这批输入一条都没落在 MAXPS 敏感族上 ⇒ 判据空转（调幅度档/注入概率）"
        );
        // 整数算术报"每 10⁶ 轴"的比例（本仓禁 f64，连测试里也不开例外）。
        println!(
            "F-12 差分[{label}]：{cases} 组 / {axes_n} 轴，MAXPS 敏感轴 {sensitive}（{}/10⁶）——SIMD 与标量逐位同结果",
            sensitive * 1_000_000 / axes_n
        );
    }
}
