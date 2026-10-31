//! 接触求解扫描（Jacobi，**两趟**：每接触算增量 → 每体按 CSR 升序累加）。
//!
//! 布局（与 `gpu_solve_probe.rs` 的打包逐字对应；全 flat `array<f32>`，手写偏移避开 struct 对齐坑）：
//! - `bv`   : 10 f32/体 = lin3 · ang3 · inv_mass · inv_i_diag3 · pad1（世界逆惯量**用对角近似**，
//!            本探针只量吞吐，不主张与主机档同精度）
//! - `ct`   : 15 f32/接触 = a,b（`f32::to_bits` 后当 f32 存）· n3 · ra3 · rb3 · depth · mu · kn · kt
//! - `imp`  : 3 f32/接触 = 累计 (pn, pt1, pt2)
//! - `dimp` : 3 f32/接触 = 本趟增量（体累加读它）
//! - `off`  : (体数+1) u32 位模式 = CSR 偏移；`items`: 2×接触数 = CSR 项（接触序号，升序）
struct Params {
    n_contacts: u32,
    n_bodies: u32,
    beta: f32,
    slop: f32,
    dt: f32,
    _p0: f32,
    _p1: f32,
    _p2: f32,
};

@group(0) @binding(0) var<storage, read_write> bv: array<f32>;
@group(0) @binding(1) var<storage, read> ct: array<f32>;
@group(0) @binding(2) var<storage, read_write> imp: array<f32>;
@group(0) @binding(3) var<storage, read_write> dimp: array<f32>;
@group(0) @binding(4) var<storage, read> off: array<f32>;
@group(0) @binding(5) var<storage, read> items: array<f32>;
@group(0) @binding(6) var<uniform> P: Params;

fn u(v: f32) -> u32 {
    return bitcast<u32>(v);
}

struct Tangents {
    t1: vec3<f32>,
    t2: vec3<f32>,
};

/// 切向基（与 CPU 参考同一算法）。
fn tangents(n: vec3<f32>) -> Tangents {
    var r = vec3<f32>(1.0, 0.0, 0.0);
    if (abs(n.x) > 0.9) {
        r = vec3<f32>(0.0, 1.0, 0.0);
    }
    let t1 = normalize(cross(n, r));
    return Tangents(t1, cross(n, t1));
}

/// 第 1 趟：每接触一线程。**Jacobi**：只读体速度，写自己的累计冲量与本趟增量。
@compute @workgroup_size(64)
fn contact_sweep(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * (65535u * 64u);
    if (i >= P.n_contacts) {
        return;
    }
    let o = i * 15u;
    let ia = u32(ct[o]);
    let ib = u32(ct[o + 1u]);
    let n = vec3<f32>(ct[o + 2u], ct[o + 3u], ct[o + 4u]);
    let ra = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
    let rb = vec3<f32>(ct[o + 8u], ct[o + 9u], ct[o + 10u]);
    let depth = ct[o + 11u];
    let mu = ct[o + 12u];
    let kn = ct[o + 13u];
    let kt = ct[o + 14u];

    let oa = ia * 10u;
    let ob = ib * 10u;
    let va = vec3<f32>(bv[oa], bv[oa + 1u], bv[oa + 2u])
        + cross(vec3<f32>(bv[oa + 3u], bv[oa + 4u], bv[oa + 5u]), ra);
    let vb = vec3<f32>(bv[ob], bv[ob + 1u], bv[ob + 2u])
        + cross(vec3<f32>(bv[ob + 3u], bv[ob + 4u], bv[ob + 5u]), rb);
    let vrel = vb - va;

    let io = i * 3u;
    let pn0 = imp[io];
    let bias = max(0.0, -P.beta / P.dt * (depth - P.slop));
    let pn = clamp(pn0 + (-dot(vrel, n) - bias) * kn, 0.0, 1.0e9);
    imp[io] = pn;
    dimp[io] = pn - pn0;

    let ts = tangents(n);
    let lim = mu * pn;

    let pt1_0 = imp[io + 1u];
    let pt1 = clamp(pt1_0 - dot(vrel, ts.t1) * kt, -lim, lim);
    imp[io + 1u] = pt1;
    dimp[io + 1u] = pt1 - pt1_0;

    let pt2_0 = imp[io + 2u];
    let pt2 = clamp(pt2_0 - dot(vrel, ts.t2) * kt, -lim, lim);
    imp[io + 2u] = pt2;
    dimp[io + 2u] = pt2 - pt2_0;
}

/// 第 2 趟：每体一线程，按 **CSR 升序**累加（确定性；CPU 参考同序）。
@compute @workgroup_size(64)
fn body_apply(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x + gid.y * (65535u * 64u);
    if (b >= P.n_bodies) {
        return;
    }
    let o = b * 10u;
    let im = bv[o + 6u];
    if (im <= 0.0) {
        return;
    }
    let s = u(off[b]);
    let e = u(off[b + 1u]);
    var dp = vec3<f32>(0.0, 0.0, 0.0);
    var dtq = vec3<f32>(0.0, 0.0, 0.0);
    for (var k = s; k < e; k = k + 1u) {
        let ci = u(items[k]);
        let co = ci * 15u;
        let ia = u32(ct[co]);
        let ib = u32(ct[co + 1u]);
        let n = vec3<f32>(ct[co + 2u], ct[co + 3u], ct[co + 4u]);
        let ra = vec3<f32>(ct[co + 5u], ct[co + 6u], ct[co + 7u]);
        let rb = vec3<f32>(ct[co + 8u], ct[co + 9u], ct[co + 10u]);
        let ts = tangents(n);
        let io = ci * 3u;
        let j = n * dimp[io] + ts.t1 * dimp[io + 1u] + ts.t2 * dimp[io + 2u];
        if (b == ia) {
            dp = dp - j;
            dtq = dtq - cross(ra, j);
        } else if (b == ib) {
            dp = dp + j;
            dtq = dtq + cross(rb, j);
        }
    }
    let lv = vec3<f32>(bv[o], bv[o + 1u], bv[o + 2u]) + dp * im;
    let ii = vec3<f32>(bv[o + 7u], bv[o + 8u], bv[o + 9u]);
    let av = vec3<f32>(bv[o + 3u], bv[o + 4u], bv[o + 5u]) + dtq * ii;
    bv[o] = lv.x;
    bv[o + 1u] = lv.y;
    bv[o + 2u] = lv.z;
    bv[o + 3u] = av.x;
    bv[o + 4u] = av.y;
    bv[o + 5u] = av.z;
}
