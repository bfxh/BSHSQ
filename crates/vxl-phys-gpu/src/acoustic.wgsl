// **声学核（GPU 档 M1/M2/M3/M4/M5）**：每个 `(声源, 听者)` 对是一个独立工作项。
//
// - `direct`（M1）：直达 —— 遮挡 + `dist/speed` 与 `1/dist²`；
// - `reflect`（M2）：**一次镜面反射**（镜像源法）—— 逐三角找镜像路径，取**最强那条**
//   （平局按三角序号取小 ⇒ 确定）；增益 `(1-a)/(d₁+d₂)²`、时延 `(d₁+d₂)/speed`；
// - `diffract`（M3）：**绕射**（工程近似）—— 逐棱取 **Fermat 点**（定次数 48 的二分，与 CPU
//   的 `FERMAT_ITERS` 同值 ⇒ 同样的定次数、无早退），**只在阴影里**产出（直达通视就整对跳过），
//   取最强那条棱（平局按棱序号取小）；增益 `coeff·(1+cosθ)/2/(d₁·d₂)`、时延 `(d₁+d₂)/speed`。
// - `scatter`（M4）：**漫反射散射**（面当二级声源，Lambertian 工程口径）—— 逐三角取**面中心**，
//   要求声源/听者在**同侧**（法向朝声源那侧、两个余弦 > 0）且两条腿都不被挡，取**最强那个面**
//   （平局按三角序号取小）；增益 `s·cos_in·cos_out·A/(d₁²·d₂²)`（`A = |cross|/2`）、
//   时延 `(d₁+d₂)/speed`。
// - `medium`（M5）：**介质积分**（水平层表 + **定步数中点采样**，无早退 ⇒ 逐位可复现）——
//   逐子段取中点、查层表拿 `(c, α_低, α_中, α_高)`，累加 `len/c` 与 `Π exp(-α_b·len)`；
//   输出行是 `(用时, 衰减低, 衰减中, 衰减高)`（**这一条没有 hit 位**，行首就是用时）。
//   ⚠️ **折线（折射）由宿主逐段调用后组合**：时延**可加**、衰减**可乘** ⇒ 与"一次算完折线"
//   等价（判据里钉了这条组合性）。层表按 y 升序、不重叠；取**第一命中**，没命中就是空气。
// - `reflect2`（M6）：**二阶镜面反射**（两跳镜像展开）—— 逐对面 `(A,B)` 展开，两个反射点都要落在
//   各自三角内、三段腿都不许被**链外的面**挡（链上两张面跳过），取**最强那条**（平局按 `(A,B)`
//   序号取小）；增益 `(1-a_A)(1-a_B)/(总程长)²`、时延 `总程长/speed`。⚠️ 卡上**不做预算截断**
//   ⇒ 与 CPU `reflection_order = 2`（A8）同解的**前提**是 CPU 侧 `max_reflections ≥ n_tris`。
//
// 与 CPU 端 `vxl-phys-acoustics` 的对应那几条**同口径**：
//   ① 遮挡：Möller–Trumbore，端点共面不计（±1e-6 的边界容差，与 CPU 同款）；
//   ② 时延 = `dist / speed`、增益 = `1 / dist²`（球面扩散）；
//   ③ 逐轴展开、不用内建 `dot`/`cross`（归约序未规定 —— 与 `body_integrate.wgsl` 同一纪律）。
//
// **量纲/边界**：卡上到 M1–M6。仍留在 CPU 侧的：统计混响尾巴（A7）与穿孔传声（portal）；
// 这里要钉的是"同一批输入 ⇒ 与 CPU 参考**逐对一致**（浮点容差、拓扑逐位）"。
//
// ⚠️ **两条刻意的独立口径**（别当成 bug）：① `reflect` 只按 `(1-a)` 打折，**不做** CPU 侧
// "开散射时镜面那条再乘 `(1-s)`"的分工（能量守恒式分工要等两条腿在卡上合流的那一片）；
// ② 各入口的系数都是**单频带标量**（频带表还没上卡）。

struct Params {
    n_src: u32,
    n_lst: u32,
    n_tris: u32,
    speed: f32,
    /// 衍射棱条数（0 ⇒ `diffract` 不产出 —— 无棱可绕）。
    n_edges: u32,
    /// 介质层表条数（0 ⇒ 纯空气：`time = len/speed`、衰减退化 1）。
    n_slabs: u32,
    /// 每段的**定步数**（`0` 按 1 算）—— 定步数、无早退 ⇒ 逐位可复现。
    medium_steps: u32,
    /// **A9a 波长哨兵**（m）：`≤ 0` ⇒ 绕射退回旧口径 `(1+cosθ)/2`（**逐位不变**）；
    /// `> 0` ⇒ 用真 Fresnel 刀刃系数 + A9b 楔角修正（宿主按 `c / band_hz` 算好）。
    /// （这一格原为 pad —— 8 个字段仍是 32 B，uniform 对齐不变。）
    lambda: f32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> srcs: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> dsts: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> tris: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read_write> out: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read> absorb: array<f32>;
/// 衍射棱：每条 **2 个 `vec4`**（`a.xyz|pad`、`b.xyz|pad`）。
@group(0) @binding(6) var<storage, read> edges: array<vec4<f32>>;
/// 逐棱绕射系数（**单频带口径**：频带表还没上卡，见 `docs/SURVEY-ACOUSTIC-RAYTRACING.md` §9）。
@group(0) @binding(7) var<storage, read> edge_coeff: array<f32>;
/// 逐三角**散射系数**（**单频带口径**；`0` = 纯镜面 ⇒ 不参与散射）。
@group(0) @binding(8) var<storage, read> scatter_coef: array<f32>;
/// 介质层表：每条 **2 个 `vec4`**（`y_lo|y_hi|c|α_低`、`α_中|α_高|0|0`）。按 `y` 升序、不重叠。
@group(0) @binding(9) var<storage, read> slabs: array<vec4<f32>>;
/// **A9b 逐棱楔角**：每条 **1 个 `vec4`**（`Ω|face0.x|face0.y|face0.z`；`Ω` = 楔体内角，弧度）。
/// 与 `edges` **一一对应**；`Ω ≤ 0`（默认 / `None`）⇒ 该棱按**刀口**算（`fac = 1`）。
@group(0) @binding(10) var<storage, read> edge_wedge: array<vec4<f32>>;

/// Fermat 二分次数：定次数、无早退 ⇒ 逐位可复现（与 CPU 的 `FERMAT_ITERS` 同值）。
const FERMAT_ITERS: u32 = 48u;

fn dot3(a: vec3<f32>, b: vec3<f32>) -> f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn cross3(a: vec3<f32>, b: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    );
}

fn len3(a: vec3<f32>) -> f32 {
    return sqrt(dot3(a, a));
}

/// 点是否在三角内（重心坐标，1e-5 容差 —— 与 CPU 的 `inside_tri` 同口径）。
fn inside(a: vec3<f32>, b: vec3<f32>, c: vec3<f32>, p: vec3<f32>) -> bool {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;
    let d00 = dot3(v0, v0);
    let d01 = dot3(v0, v1);
    let d11 = dot3(v1, v1);
    let d20 = dot3(v2, v0);
    let d21 = dot3(v2, v1);
    let den = d00 * d11 - d01 * d01;
    if (abs(den) <= 1e-12) {
        return false;
    }
    let u = (d11 * d20 - d01 * d21) / den;
    let v = (d00 * d21 - d01 * d20) / den;
    return u >= -1e-5 && v >= -1e-5 && u + v <= 1.0 + 1e-5;
}

/// 顺序扫全表：线段 `p→q` 是否被**除 `skip_a` / `skip_b` 这两张面之外**的任何三角挡住。
///
/// `skip_b = 0xffffffffu` 表示"没有第二张要跳过的面"（它越出 `n_tris` ⇒ 永不命中）——
/// 于是 [`blocked`] 就是它的单面退化。二阶（M6）的两跳链要求**两张链上面的子段**都不算挡。
fn blocked2(p: vec3<f32>, q: vec3<f32>, skip_a: u32, skip_b: u32) -> bool {
    for (var t = 0u; t < P.n_tris; t = t + 1u) {
        if (t == skip_a || t == skip_b) {
            continue;
        }
        let a = tris[t * 3u].xyz;
        let b = tris[t * 3u + 1u].xyz;
        let c = tris[t * 3u + 2u].xyz;
        if (hits(a, b, c, p, q)) {
            return true;
        }
    }
    return false;
}

/// 单面退化（M1–M4 共用）。**逐位与重构前等价**：多出来的那次 `t == skip_b` 恒为假。
fn blocked(p: vec3<f32>, q: vec3<f32>, skip: u32) -> bool {
    return blocked2(p, q, skip, 0xffffffffu);
}

/// 线段 `p→q` 是否被三角 `(a,b,c)` 挡住（Möller–Trumbore；端点共面不计）。
fn hits(a: vec3<f32>, b: vec3<f32>, c: vec3<f32>, p: vec3<f32>, q: vec3<f32>) -> bool {
    let d = q - p;
    let len = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (len <= 1e-6) {
        return false;
    }
    let dir = d * (1.0 / len);
    let e1 = b - a;
    let e2 = c - a;
    // pv = dir × e2（逐轴展开）
    let pv = vec3<f32>(
        dir.y * e2.z - dir.z * e2.y,
        dir.z * e2.x - dir.x * e2.z,
        dir.x * e2.y - dir.y * e2.x,
    );
    let det = e1.x * pv.x + e1.y * pv.y + e1.z * pv.z;
    if (abs(det) <= 1e-9) {
        return false;
    }
    let inv = 1.0 / det;
    let tv = p - a;
    let u = (tv.x * pv.x + tv.y * pv.y + tv.z * pv.z) * inv;
    if (u < -1e-6 || u > 1.0 + 1e-6) {
        return false;
    }
    // qv = tv × e1（逐轴展开）
    let qv = vec3<f32>(
        tv.y * e1.z - tv.z * e1.y,
        tv.z * e1.x - tv.x * e1.z,
        tv.x * e1.y - tv.y * e1.x,
    );
    let v = (dir.x * qv.x + dir.y * qv.y + dir.z * qv.z) * inv;
    if (v < -1e-6 || u + v > 1.0 + 1e-6) {
        return false;
    }
    let t = (e2.x * qv.x + e2.y * qv.y + e2.z * qv.z) * inv;
    return t > 1e-4 && t < len - 1e-4;
}

@compute @workgroup_size(64)
fn direct(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let s = srcs[i / P.n_lst].xyz;
    let d = dsts[i % P.n_lst].xyz;
    var blocked = 0.0;
    for (var t = 0u; t < P.n_tris; t = t + 1u) {
        let a = tris[t * 3u].xyz;
        let b = tris[t * 3u + 1u].xyz;
        let c = tris[t * 3u + 2u].xyz;
        if (hits(a, b, c, s, d)) {
            blocked = 1.0;
        }
    }
    let v = d - s;
    let dist = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    let kind = select(1.0, 0.0, blocked > 0.5);
    out[i] = vec4<f32>(kind, dist / P.speed, 1.0 / max(dist * dist, 1e-6), blocked);
}

@compute @workgroup_size(64)
fn reflect(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let s = srcs[i / P.n_lst].xyz;
    let l = dsts[i % P.n_lst].xyz;
    var best_gain = 0.0;
    var best_delay = 0.0;
    var best_tri = -1.0;
    for (var t = 0u; t < P.n_tris; t = t + 1u) {
        let a = tris[t * 3u].xyz;
        let b = tris[t * 3u + 1u].xyz;
        let c = tris[t * 3u + 2u].xyz;
        let cr = cross3(b - a, c - a);
        let ln = len3(cr);
        if (ln <= 1e-9) {
            continue; // 退化面：不参与反射
        }
        let n = cr * (1.0 / ln);
        // 镜像听者：l' = l - 2·((l - a)·n)·n
        let lp = l - n * (2.0 * dot3(l - a, n));
        let seg = lp - s;
        let den = dot3(seg, n);
        if (abs(den) <= 1e-9) {
            continue; // 与面平行：无镜面反射点
        }
        let tt = dot3(a - s, n) / den;
        if (tt < 0.0 || tt > 1.0) {
            continue;
        }
        let hit = s + seg * tt;
        if (!inside(a, b, c, hit)) {
            continue;
        }
        if (blocked(s, hit, t) || blocked(hit, l, t)) {
            continue;
        }
        let total = len3(hit - s) + len3(l - hit);
        let g = (1.0 - absorb[t]) / max(total * total, 1e-6);
        if (g > best_gain) {
            best_gain = g;
            best_delay = total / P.speed;
            best_tri = f32(t);
        }
    }
    out[i] = vec4<f32>(select(0.0, 1.0, best_gain > 0.0), best_delay, best_gain, best_tri);
}

/// **M6：二阶镜面反射**（两跳镜像展开）—— 逐对取**最强那条**两跳路径。
///
/// 链 = `(面 A, 面 B)`、`A ≠ B`（同一面连打两次 = 原路返回，退化 ⇒ 跳过）；听者先按 `B`、再按 `A`
/// 逐面镜像 ⇒ 第一跳点 `pA` 落在 `A` 内、第二跳点 `pB` 落在 `B` 内（两个点都用重心内点判）；
/// 三段腿（`s→pA`、`pA→pB`、`pB→l`）都不得被**除 A、B 之外**的面挡（[`blocked2`]）；
/// 增益 = `(1-a_A)(1-a_B)/(总程长²)`、时延 = `总程长/speed`。平局按 `(A,B)` 序号取小 ⇒ 确定。
///
/// ⚠️ **卡上不做预算截断**（与 M1–M5 同口径：卡只负责"给定场景逐对取最强"，预算/阶数归宿主）
/// ⇒ 与 CPU `reflection_order = 2`（A8）逐对一致的**前提**是 CPU 侧 `max_reflections ≥ n_tris`
/// （不截断才同解）；CPU 的 `dropped_budget` 在卡上没有对应信号。代价是每对 `O(n_tris³)`
/// （两跳枚举 × 每段遮挡扫全表）⇒ 只对"离线、小场景"有意义。
///
/// `out.w` 报的是**第一跳**的面序号（诊断用；CPU 侧路径本来不带面序号）。
@compute @workgroup_size(64)
fn reflect2(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let s = srcs[i / P.n_lst].xyz;
    let l = dsts[i % P.n_lst].xyz;
    var best_gain = 0.0;
    var best_delay = 0.0;
    var best_face = -1.0;
    for (var ta = 0u; ta < P.n_tris; ta = ta + 1u) {
        let a0 = tris[ta * 3u].xyz;
        let a1 = tris[ta * 3u + 1u].xyz;
        let a2 = tris[ta * 3u + 2u].xyz;
        let na = cross3(a1 - a0, a2 - a0);
        let lna = len3(na);
        if (lna <= 1e-9) {
            continue; // 退化面：不当反射面
        }
        let nA = na * (1.0 / lna);
        for (var tb = 0u; tb < P.n_tris; tb = tb + 1u) {
            if (tb == ta) {
                continue; // 同一面连着两次：退化
            }
            let b0 = tris[tb * 3u].xyz;
            let b1 = tris[tb * 3u + 1u].xyz;
            let b2 = tris[tb * 3u + 2u].xyz;
            let nb = cross3(b1 - b0, b2 - b0);
            let lnb = len3(nb);
            if (lnb <= 1e-9) {
                continue;
            }
            let nB = nb * (1.0 / lnb);
            // 听者镜像两跳：`mB` = 听者按 B 镜像、`mA` = 再按 A 镜像。
            let mB = l - nB * (2.0 * dot3(l - b0, nB));
            let mA = mB - nA * (2.0 * dot3(mB - a0, nA));
            // 第一跳：`s→mA` 与面 A 求交（越界 / 平行 / 出三角都作废）。
            let segA = mA - s;
            let denA = dot3(segA, nA);
            if (abs(denA) <= 1e-9) {
                continue;
            }
            let tA = dot3(a0 - s, nA) / denA;
            if (tA < 0.0 || tA > 1.0) {
                continue;
            }
            let pA = s + segA * tA;
            if (!inside(a0, a1, a2, pA)) {
                continue;
            }
            // 第二跳：`pA→mB` 与面 B 求交。
            let segB = mB - pA;
            let denB = dot3(segB, nB);
            if (abs(denB) <= 1e-9) {
                continue;
            }
            let tB = dot3(b0 - pA, nB) / denB;
            if (tB < 0.0 || tB > 1.0) {
                continue;
            }
            let pB = pA + segB * tB;
            if (!inside(b0, b1, b2, pB)) {
                continue;
            }
            // 三段腿都不许被"链外的面"挡（链上的两张面本来就在腿上起落 ⇒ 跳过）。
            if (blocked2(s, pA, ta, tb) || blocked2(pA, pB, ta, tb) || blocked2(pB, l, ta, tb)) {
                continue;
            }
            let total = len3(pA - s) + len3(pB - pA) + len3(l - pB);
            let g = (1.0 - absorb[ta]) * (1.0 - absorb[tb]) / max(total * total, 1e-6);
            if (g > best_gain) {
                best_gain = g;
                best_delay = total / P.speed;
                best_face = f32(ta);
            }
        }
    }
    out[i] = vec4<f32>(select(0.0, 1.0, best_gain > 0.0), best_delay, best_gain, best_face);
}

/// 查层表：`y` 落在**第一命中**的层里 ⇒ 返回 `(c, α_低, α_中, α_高)`；没命中 ⇒ 空气（α 全 0）。
fn slab_at(y: f32) -> vec4<f32> {
    for (var k = 0u; k < P.n_slabs; k = k + 1u) {
        let lo = slabs[k * 2u].x;
        let hi = slabs[k * 2u].y;
        if (y >= lo && y <= hi) {
            return vec4<f32>(
                slabs[k * 2u].z,
                slabs[k * 2u].w,
                slabs[k * 2u + 1u].x,
                slabs[k * 2u + 1u].y,
            );
        }
    }
    return vec4<f32>(P.speed, 0.0, 0.0, 0.0);
}

/// **介质积分**（M5）：直线段 `srcs[i] → dsts[i]` 上按**定步数**取中点采样，累加用时与逐带衰减。
///
/// 输出行是 `(用时, 衰减低, 衰减中, 衰减高)` —— **这一条没有 hit 位**（直线段永远算得出来）。
/// 折线（折射）不在这里：宿主逐段调用后**时延相加、衰减相乘**（两者都可分量 ⇒ 等价）。
@compute @workgroup_size(64)
fn medium(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let a = srcs[i / P.n_lst].xyz;
    let b = dsts[i % P.n_lst].xyz;
    let seg = b - a;
    let len = len3(seg);
    var time = 0.0;
    var att = vec3<f32>(1.0, 1.0, 1.0);
    if (len > 1e-6) {
        let steps = max(P.medium_steps, 1u);
        let inv = 1.0 / f32(steps);
        for (var k = 0u; k < steps; k = k + 1u) {
            // 中点采样（与 CPU 的固定步长回退同法）：定步数、无早退 ⇒ 逐位可复现。
            let p = a + seg * ((f32(k) + 0.5) * inv);
            let m = slab_at(p.y);
            let dl = len * inv;
            time += dl / max(m.x, 1e-3);
            att = att * vec3<f32>(exp(-m.y * dl), exp(-m.z * dl), exp(-m.w * dl));
        }
    }
    out[i] = vec4<f32>(time, att.x, att.y, att.z);
}

/// 线段 `p→q` 是否被**任何**三角挡住（不带 `skip`：直达通视检查用）。
fn blocked_any(p: vec3<f32>, q: vec3<f32>) -> bool {
    // `n_tris` 远小于 u32 上限 ⇒ 这个哨兵值恒不等于任何三角序号。
    return blocked(p, q, 0xffffffffu);
}

/// 棱上让 `|P−src| + |P−dst|` 最小的点（`P` 限定在 `a→b` 上）—— 与 CPU 的 `fermat_point` 同法：
/// `f'(u) = cos∠(dir, P−src) + cos∠(dir, P−dst)` 单调不减 ⇒ 在两个投影夹出的区间上**定次数二分**，
/// 再把参数夹进 `[0, len]`（夹到端点就是"从棱的端点绕"，凸函数在区间上的最小就在端点）。
fn fermat(a: vec3<f32>, b: vec3<f32>, src: vec3<f32>, dst: vec3<f32>) -> vec3<f32> {
    let seg = b - a;
    let len = len3(seg);
    if (len <= 1e-6) {
        return a;
    }
    let dir = seg * (1.0 / len);
    let ps = clamp(dot3(src - a, dir), 0.0, len);
    let pd = clamp(dot3(dst - a, dir), 0.0, len);
    var lo = min(ps, pd);
    var hi = max(ps, pd);
    if (hi - lo <= 1e-7) {
        return a + dir * (0.5 * (lo + hi));
    }
    for (var k = 0u; k < FERMAT_ITERS; k = k + 1u) {
        let mid = 0.5 * (lo + hi);
        let p = a + dir * mid;
        let va = p - src;
        let vb = p - dst;
        let slope = dot3(va, dir) / max(len3(va), 1e-6) + dot3(vb, dir) / max(len3(vb), 1e-6);
        if (slope < 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return a + dir * (0.5 * (lo + hi));
}

// ===========================================================================
// A9a / A9b：真 Fresnel 刀刃系数 + 一般楔角修正（与 CPU `vxl-phys-acoustics::utd` **同口径**）。
// 全 `f32`；只在 `P.lambda > 0` 时参与（否则退回旧口径 `(1+cosθ)/2`，**逐位不变**）。
// ===========================================================================

/// 幂级数项数上限（`x < CROSSOVER` 时用不到这么多；防病态入参把循环拉长）。
const MAX_TERMS: u32 = 4096u;
/// 渐近级数项数上限（取到最小项为止）。
const MAX_ASYM: u32 = 64u;
/// 幂级数 / 渐近的交接点（与 CPU `utd::CROSSOVER` 同值）。
const CROSSOVER: f32 = 2.5;
/// A9b 修正因子钳位上界（与 CPU `utd::WEDGE_FACTOR_MAX` 同值）。
const WEDGE_FACTOR_MAX: f32 = 4.0;
const PI: f32 = 3.141592653589793;
const FRAC_PI_4: f32 = 0.7853981633974483;

/// `C/S` 幂级数（`x ≥ 0`；与 CPU `utd::series` 逐项一致）。
fn fresnel_series(x: f32) -> vec2<f32> {
    let p = 0.5 * PI;
    let x2 = x * x;
    let fac = p * p * x2 * x2;
    let terms = min(u32(p * x2) + 64u, MAX_TERMS);
    var base_c = x;
    var base_s = p * x * x2;
    var c = 0.0;
    var s = 0.0;
    for (var k = 0u; k < terms; k = k + 1u) {
        let kf = f32(k);
        c = c + base_c / (4.0 * kf + 1.0);
        s = s + base_s / (4.0 * kf + 3.0);
        base_c = base_c * (-fac / ((2.0 * kf + 1.0) * (2.0 * kf + 2.0)));
        base_s = base_s * (-fac / ((2.0 * kf + 2.0) * (2.0 * kf + 3.0)));
    }
    return vec2<f32>(c, s);
}

/// `C/S` 渐近展开（`x ≥ CROSSOVER`；取到最小项即止）。
fn fresnel_asym(x: f32) -> vec2<f32> {
    let u = 1.0 / (PI * x);
    let t = 1.0 / (PI * x * x);
    let t2 = t * t;
    let a2 = 0.5 * PI * x * x;
    let s2 = sin(a2);
    let c2 = cos(a2);
    var a_sum = 0.0;
    var b_sum = 0.0;
    var ta = 1.0;
    var tb = t;
    var prev = 1e38;
    var m = 0u;
    loop {
        let mag = max(abs(ta), abs(tb));
        if (mag > prev || m >= MAX_ASYM) {
            break;
        }
        prev = mag;
        a_sum = a_sum + ta;
        b_sum = b_sum + tb;
        let mm = f32(m + 1u);
        ta = ta * (-(4.0 * mm - 1.0) * (4.0 * mm - 3.0) * t2);
        tb = tb * (-(4.0 * mm - 1.0) * (4.0 * mm + 1.0) * t2);
        m = m + 1u;
    }
    let a = u * a_sum;
    let b = u * b_sum;
    return vec2<f32>(0.5 - (-s2 * a + c2 * b), 0.5 - (c2 * a + s2 * b));
}

/// Fresnel 积分 `C(x)/S(x)`（奇对称；无递归：取 `|x|` 再按符号翻）。
fn fresnel_cs(x: f32) -> vec2<f32> {
    let ax = abs(x);
    var r = vec2<f32>(0.0, 0.0);
    if (ax >= CROSSOVER) {
        r = fresnel_asym(ax);
    } else if (ax > 0.0) {
        r = fresnel_series(ax);
    }
    return select(-r, r, x >= 0.0);
}

/// `ν = 2·√(Δ/λ)`（`Δ` = 多余路程）；`λ ≤ 0` 或 `Δ ≤ 0` ⇒ `0`。
fn fresnel_nu(excess: f32, lambda: f32) -> f32 {
    if (lambda <= 0.0) {
        return 0.0;
    }
    return 2.0 * sqrt(max(excess, 0.0) / lambda);
}

/// 刀刃（半平面）绕射的**能量比** `|E_d/E₀|²`（`ν = 0` ⇒ `1/4`，恰好 −6.02 dB）。
fn knife_gain(nu: f32) -> f32 {
    let cs = fresnel_cs(nu);
    let a = 0.5 - cs.x;
    let b = 0.5 - cs.y;
    return 0.5 * (a * a + b * b);
}

/// `cot`；`sin → 0` 时给**有限大数**（`±1e12`）而非 `±∞`（与 CPU 同款）。
fn wedge_cot(x: f32) -> f32 {
    let s = sin(x);
    if (abs(s) < 1e-12) {
        if (cos(x) >= 0.0) {
            return 1e12;
        }
        return -1e12;
    }
    return cos(x) / s;
}

/// 平面内极角（`v` 相对基准 `ax`、正向 `bx`），落在 `[0, 2π)`。
fn angle_in_plane(v: vec3<f32>, ax: vec3<f32>, bx: vec3<f32>) -> f32 {
    let t = atan2(dot3(v, bx), dot3(v, ax));
    if (t < 0.0) {
        return t + 2.0 * PI;
    }
    return t;
}

/// `(m, q)`：`m = max(|re|,|im|)`、`|D| = m·q`（缩放幅值，乘/除不溢出）。
fn mag_scaled(re: f32, im: f32) -> vec2<f32> {
    let m = max(abs(re), abs(im));
    if (m <= 0.0) {
        return vec2<f32>(0.0, 0.0);
    }
    let nr = re / m;
    let ni = im / m;
    return vec2<f32>(m, sqrt(nr * nr + ni * ni));
}

/// 过渡函数 `F(χ)`（`χ ≤ 0 ⇒ 0`），用 `C/S` 闭环求值。
fn transition(chi: f32) -> vec2<f32> {
    if (chi <= 0.0) {
        return vec2<f32>(0.0, 0.0);
    }
    let u = sqrt(2.0 * chi / PI);
    let cs = fresnel_cs(u);
    let q = sqrt(0.5 * PI);
    let ire = q * (0.5 - cs.x);
    let iim = -q * (0.5 - cs.y);
    let ex = cos(chi);
    let ey = sin(chi);
    let r = sqrt(chi);
    let re = 2.0 * r * (ex * ire - ey * iim);
    let im = 2.0 * r * (ex * iim + ey * ire);
    return vec2<f32>(-im, re);
}

/// `a±(β) = 2·cos²((2πn·N∓ − β)/2)`，`N∓ = round((β ∓ π)/(2πn))`（`plus` ⇒ `(β+π)` 那支）。
fn apm(beta: f32, n: f32, plus: bool) -> f32 {
    var num = beta - PI;
    if (plus) {
        num = beta + PI;
    }
    let nn = round(num / (2.0 * n * PI));
    let t = (2.0 * n * PI * nn - beta) / 2.0;
    return 2.0 * t * t;
}

/// `T(β) = cot((π+β)/(2n))·F(kL·a⁺) + cot((π−β)/(2n))·F(kL·a⁻)`（复式两项和）。
fn utd_term(beta: f32, n: f32, k: f32, l: f32) -> vec2<f32> {
    let a = transition(k * l * apm(beta, n, true));
    let b = transition(k * l * apm(beta, n, false));
    let c1 = wedge_cot((PI + beta) / (2.0 * n));
    let c2 = wedge_cot((PI - beta) / (2.0 * n));
    return vec2<f32>(c1 * a.x + c2 * b.x, c1 * a.y + c2 * b.y);
}

/// KP **软边界**绕射系数 `D`（实部/虚部）。
fn utd_soft(n: f32, phi: f32, phip: f32, k: f32, l: f32) -> vec2<f32> {
    let pre = 1.0 / (2.0 * n * sqrt(max(2.0 * PI * k, 1e-12)));
    let pr = -cos(FRAC_PI_4) * pre;
    let pim = sin(FRAC_PI_4) * pre;
    let a = utd_term(phi - phip, n, k, l);
    let b = utd_term(phi + phip, n, k, l);
    let tr = a.x - b.x;
    let ti = a.y - b.y;
    return vec2<f32>(pr * tr - pim * ti, pr * ti + pim * tr);
}

/// A9b 楔角修正因子 `fac ∈ [0, WEDGE_FACTOR_MAX]`（`n ≥ 2` ⇒ 恒 `1`；`n ≤ 1` ⇒ `0`）。
fn wedge_factor(n: f32, phi: f32, phip: f32, k: f32, l: f32) -> f32 {
    if (n >= 2.0) {
        return 1.0;
    }
    if (n <= 1.0) {
        return 0.0;
    }
    let sn = utd_soft(n, phi, phip, k, l);
    let s2 = utd_soft(2.0, phi, phip, k, l);
    let dn = mag_scaled(sn.x, sn.y);
    let d2 = mag_scaled(s2.x, s2.y);
    if (d2.x <= 0.0) {
        if (dn.x > 0.0) {
            return WEDGE_FACTOR_MAX;
        }
        return 0.0;
    }
    let tt = (dn.x / d2.x) * (dn.y / d2.y);
    let fac = min(tt * tt, WEDGE_FACTOR_MAX);
    if (fac != fac) {
        return WEDGE_FACTOR_MAX;
    }
    return fac;
}

/// 楔角几何框架（`φ`、`φ'`、`n`、`l`）；退化时 `ok = false`（调用侧退回刀口）。
struct WedgeFrame {
    phi: f32,
    phip: f32,
    n: f32,
    l: f32,
    ok: bool,
};

/// 从棱点 `apex`、棱方向 `edge_dir`（单位）、声源/听者构造（退化 ⇒ `ok = false`）。
fn build_frame(omega_in: f32, face0: vec3<f32>, apex: vec3<f32>, edge_dir: vec3<f32>, src: vec3<f32>, dst: vec3<f32>) -> WedgeFrame {
    var f = WedgeFrame(0.0, 0.0, 0.0, 0.0, false);
    let proj = face0 - edge_dir * dot3(face0, edge_dir);
    let ax = proj * (1.0 / len3(proj));
    if (dot3(ax, ax) < 0.5) {
        return f;
    }
    let bx = cross3(edge_dir, ax);
    let vi = src - apex;
    let vr = dst - apex;
    if (dot3(vi, vi) < 1e-6 || dot3(vr, vr) < 1e-6) {
        return f;
    }
    let ui = vi - edge_dir * dot3(vi, edge_dir);
    let ur = vr - edge_dir * dot3(vr, edge_dir);
    if (dot3(ui, ui) < 1e-6 || dot3(ur, ur) < 1e-6) {
        return f;
    }
    let d1 = len3(vi);
    let d2 = len3(vr);
    let omega = clamp(omega_in, 0.0, PI);
    f.phi = angle_in_plane(ur, ax, bx);
    f.phip = angle_in_plane(ui, ax, bx);
    f.n = 2.0 - omega / PI;
    f.l = d1 * d2 / (d1 + d2);
    f.ok = true;
    return f;
}

@compute @workgroup_size(64)
fn diffract(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let s = srcs[i / P.n_lst].xyz;
    let l = dsts[i % P.n_lst].xyz;
    var best_gain = 0.0;
    var best_delay = 0.0;
    var best_edge = -1.0;
    // **只在阴影里**产出：直达通视就整对跳过（免得与直达重复计能）。
    if (!blocked_any(s, l)) {
        out[i] = vec4<f32>(0.0, 0.0, 0.0, -1.0);
        return;
    }
    for (var e = 0u; e < P.n_edges; e = e + 1u) {
        let a = edges[e * 2u].xyz;
        let b = edges[e * 2u + 1u].xyz;
        let p = fermat(a, b, s, l);
        let v1 = p - s;
        let v2 = l - p;
        let d1 = len3(v1);
        let d2 = len3(v2);
        if (d1 <= 1e-3 || d2 <= 1e-3) {
            continue;
        }
        if (blocked_any(s, p) || blocked_any(p, l)) {
            continue;
        }
        let cos_t = clamp(dot3(v1, v2) / (d1 * d2), -1.0, 1.0);
        var dir_factor = 0.5 * (1.0 + cos_t);
        if (P.lambda > 0.0) {
            // 旧口径是**工程近似** `(1+cosθ)/2`；给了波长就换真刀刃 Fresnel 系数，
            // 再乘逐棱楔角修正（`Ω ≤ 0` / 退化 ⇒ `fac = 1` ⇒ 退回纯刀口）。
            let excess = (d1 + d2) - len3(l - s);
            var fac = 1.0;
            let om = edge_wedge[e].x;
            let seg = b - a;
            let elen = len3(seg);
            if (om > 0.0 && elen > 1e-6) {
                let k = 2.0 * PI / P.lambda;
                let fr = build_frame(om, edge_wedge[e].yzw, p, seg * (1.0 / elen), s, l);
                if (fr.ok) {
                    fac = wedge_factor(fr.n, fr.phi, fr.phip, k, fr.l);
                }
            }
            dir_factor = knife_gain(fresnel_nu(excess, P.lambda)) * fac;
        }
        let g = edge_coeff[e] * dir_factor / (d1 * d2);
        if (g > best_gain) {
            best_gain = g;
            best_delay = (d1 + d2) / P.speed;
            best_edge = f32(e);
        }
    }
    out[i] = vec4<f32>(select(0.0, 1.0, best_gain > 0.0), best_delay, best_gain, best_edge);
}

@compute @workgroup_size(64)
fn scatter(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= P.n_src * P.n_lst) {
        return;
    }
    let s = srcs[i / P.n_lst].xyz;
    let l = dsts[i % P.n_lst].xyz;
    var best_gain = 0.0;
    var best_delay = 0.0;
    var best_tri = -1.0;
    for (var t = 0u; t < P.n_tris; t = t + 1u) {
        let k = scatter_coef[t];
        if (k <= 0.0) {
            continue;
        }
        let a = tris[t * 3u].xyz;
        let b = tris[t * 3u + 1u].xyz;
        let c = tris[t * 3u + 2u].xyz;
        let cr = cross3(b - a, c - a);
        let area2 = len3(cr);
        if (area2 <= 1e-9) {
            continue; // 退化面：没有面积，不参与散射
        }
        let p = (a + b + c) * (1.0 / 3.0);
        let v1 = s - p;
        let v2 = l - p;
        let d1 = len3(v1);
        let d2 = len3(v2);
        if (d1 <= 1e-3 || d2 <= 1e-3) {
            continue;
        }
        // 法向朝"声源那一侧"：声与听者必须**同侧**，才谈得上反射式散射。
        var n = cr * (1.0 / area2);
        if (dot3(n, v1) < 0.0) {
            n = -n;
        }
        let cos_in = max(dot3(n, v1 * (1.0 / d1)), 0.0);
        let cos_out = max(dot3(n, v2 * (1.0 / d2)), 0.0);
        if (cos_in <= 0.0 || cos_out <= 0.0) {
            continue;
        }
        if (blocked_any(s, p) || blocked_any(p, l)) {
            continue;
        }
        let geom = 0.5 * area2 * cos_in * cos_out / (d1 * d1 * d2 * d2);
        let g = clamp(k, 0.0, 1.0) * geom;
        if (g > best_gain) {
            best_gain = g;
            best_delay = (d1 + d2) / P.speed;
            best_tri = f32(t);
        }
    }
    out[i] = vec4<f32>(select(0.0, 1.0, best_gain > 0.0), best_delay, best_gain, best_tri);
}
