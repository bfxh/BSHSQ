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
//
// 与 CPU 端 `vxl-phys-acoustics` 的对应那几条**同口径**：
//   ① 遮挡：Möller–Trumbore，端点共面不计（±1e-6 的边界容差，与 CPU 同款）；
//   ② 时延 = `dist / speed`、增益 = `1 / dist²`（球面扩散）；
//   ③ 逐轴展开、不用内建 `dot`/`cross`（归约序未规定 —— 与 `body_integrate.wgsl` 同一纪律）。
//
// **量纲/边界**：卡上就到这五条。绕射里的 Fresnel 积分、以及多次反射仍在 CPU 侧；这里要钉的是
// "同一批输入 ⇒ 与 CPU 参考**逐对一致**（浮点容差、拓扑逐位）"。
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
    /// uniform 结构体必须是 16 字节的倍数 ⇒ 补到 32 B（少一个 pad 就是整块错位）。
    _pad: u32,
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

/// 顺序扫全表：线段 `p→q` 是否被**除 `skip` 之外**的任何三角挡住。
fn blocked(p: vec3<f32>, q: vec3<f32>, skip: u32) -> bool {
    for (var t = 0u; t < P.n_tris; t = t + 1u) {
        if (t == skip) {
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
        let dir_factor = 0.5 * (1.0 + cos_t);
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
