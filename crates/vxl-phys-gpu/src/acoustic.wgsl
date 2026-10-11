// **声学核（GPU 档 M1/M2）**：每个 `(声源, 听者)` 对是一个独立工作项。
//
// - `direct`（M1）：直达 —— 遮挡 + `dist/speed` 与 `1/dist²`；
// - `reflect`（M2）：**一次镜面反射**（镜像源法）—— 逐三角找镜像路径，取**最强那条**
//   （平局按三角序号取小 ⇒ 确定）；增益 `(1-a)/(d₁+d₂)²`、时延 `(d₁+d₂)/speed`。
//
// 与 CPU 端 `vxl-phys-acoustics` 的直达那条**同口径**：
//   ① 遮挡：Möller–Trumbore，端点共面不计（±1e-6 的边界容差，与 CPU 同款）；
//   ② 时延 = `dist / speed`、增益 = `1 / dist²`（球面扩散）；
//   ③ 逐轴展开、不用内建 `dot`/`cross`（归约序未规定 —— 与 `body_integrate.wgsl` 同一纪律）。
//
// **量纲/边界**：只做直达。反射/绕射/散射/介质积分仍在 CPU 侧；这里要钉的是
// "同一批输入 ⇒ 与 CPU 参考**逐对一致**（浮点容差、拓扑逐位）"。

struct Params {
    n_src: u32,
    n_lst: u32,
    n_tris: u32,
    speed: f32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> srcs: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> dsts: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> tris: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read_write> out: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read> absorb: array<f32>;

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
