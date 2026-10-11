// **声学「直达路径」核**（A5 GPU 档 M1）：每个 `(声源, 听者)` 对是一个独立工作项。
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
