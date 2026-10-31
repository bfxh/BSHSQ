//! 端到端（B1+B3+B4）单核集：世界逆惯量 → 构建 → Jacobi 扫描 → 体累加。
//!
//! 布局（flat f32；与 `gpu_resident_probe.rs` 逐字对应）：
//! - `bd`  : 11/体 = pos3 · quat4 · inv_mass · local_inv_i3
//! - `iw`  : 9/体 = 世界逆惯量（列主序）
//! - `vel` : 6/体 = lin3 · ang3（常驻，读改写）
//! - `ct`  : 9/接触 = a,b(体号) · point3 · normal3 · mu
//! - `con` : 10/接触 = ra3 · rb3 · kn · kt1 · kt2 · kcross（构建产物）
//! - `imp` : 3/接触 = 累计 (pn, pt1, pt2)
//! - `dimp`: 3/接触 = 本趟增量
//! - `off`/`items`: CSR（体 → 接触序号，升序）
struct Params {
    n_c: u32,
    n_b: u32,
    beta: f32,
    slop: f32,
    dt: f32,
    _p0: f32,
    _p1: f32,
    _p2: f32,
};

@group(0) @binding(0) var<storage, read_write> bd: array<f32>;
@group(0) @binding(1) var<storage, read_write> iw: array<f32>;
@group(0) @binding(2) var<storage, read_write> vel: array<f32>;
@group(0) @binding(3) var<storage, read> ct: array<f32>;
@group(0) @binding(4) var<storage, read_write> con: array<f32>;
@group(0) @binding(5) var<storage, read_write> imp: array<f32>;
@group(0) @binding(6) var<storage, read_write> dimp: array<f32>;
@group(0) @binding(7) var<storage, read> off: array<f32>;
@group(0) @binding(8) var<storage, read> items: array<f32>;
@group(0) @binding(9) var<uniform> P: Params;

fn ub(v: f32) -> u32 {
    return bitcast<u32>(v);
}

fn rmat(q: vec4<f32>) -> mat3x3<f32> {
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let w = q.w;
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let xw = x * w;
    let yw = y * w;
    let zw = z * w;
    return mat3x3<f32>(
        vec3<f32>(1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw), 2.0 * (xz - yw)),
        vec3<f32>(2.0 * (xy - zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw)),
        vec3<f32>(2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - 2.0 * (xx + yy)),
    );
}

fn load_iw(b: u32) -> mat3x3<f32> {
    let o = b * 9u;
    return mat3x3<f32>(
        vec3<f32>(iw[o], iw[o + 1u], iw[o + 2u]),
        vec3<f32>(iw[o + 3u], iw[o + 4u], iw[o + 5u]),
        vec3<f32>(iw[o + 6u], iw[o + 7u], iw[o + 8u]),
    );
}

struct Tangents {
    t1: vec3<f32>,
    t2: vec3<f32>,
};

fn tangents(n: vec3<f32>) -> Tangents {
    var r = vec3<f32>(1.0, 0.0, 0.0);
    if (abs(n.x) > 0.9) {
        r = vec3<f32>(0.0, 1.0, 0.0);
    }
    let t1 = normalize(cross(n, r));
    return Tangents(t1, cross(n, t1));
}

/// 每体：`M = R·diag·Rᵀ`（B1 的常驻量；位姿变化时重算）。
@compute @workgroup_size(64)
fn make_iw(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x + gid.y * (65535u * 64u);
    if (b >= P.n_b) {
        return;
    }
    let q = b * 11u;
    let r = rmat(vec4<f32>(bd[q + 3u], bd[q + 4u], bd[q + 5u], bd[q + 6u]));
    let d = vec3<f32>(bd[q + 8u], bd[q + 9u], bd[q + 10u]);
    let rd = mat3x3<f32>(r[0] * d.x, r[1] * d.y, r[2] * d.z);
    let m = rd * transpose(r);
    let o = b * 9u;
    for (var k = 0u; k < 3u; k = k + 1u) {
        iw[o + k * 3u] = m[k].x;
        iw[o + k * 3u + 1u] = m[k].y;
        iw[o + k * 3u + 2u] = m[k].z;
    }
}

fn mass(im_a: f32, im_b: f32, ma: mat3x3<f32>, mb: mat3x3<f32>, ra: vec3<f32>, rb: vec3<f32>, d: vec3<f32>) -> f32 {
    var k = im_a + im_b;
    if (im_a > 0.0) {
        let w = ma * cross(ra, d);
        k = k + dot(cross(w, ra), d);
    }
    if (im_b > 0.0) {
        let w = mb * cross(rb, d);
        k = k + dot(cross(w, rb), d);
    }
    if (k > 1e-12) {
        return 1.0 / k;
    }
    return 0.0;
}

/// B3：每接触 `ra/rb` + 四个有效质量项。
@compute @workgroup_size(64)
fn build(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * (65535u * 64u);
    if (i >= P.n_c) {
        return;
    }
    let o = i * 9u;
    let ia = u32(ct[o]);
    let ib = u32(ct[o + 1u]);
    let pt = vec3<f32>(ct[o + 2u], ct[o + 3u], ct[o + 4u]);
    let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
    let qa = ia * 11u;
    let qb = ib * 11u;
    let ra = pt - vec3<f32>(bd[qa], bd[qa + 1u], bd[qa + 2u]);
    let rb = pt - vec3<f32>(bd[qb], bd[qb + 1u], bd[qb + 2u]);
    let im_a = bd[qa + 7u];
    let im_b = bd[qb + 7u];
    let ma = load_iw(ia);
    let mb = load_iw(ib);
    let ts = tangents(n);
    let kn = mass(im_a, im_b, ma, mb, ra, rb, n);
    let kt1 = mass(im_a, im_b, ma, mb, ra, rb, ts.t1);
    let kt2 = mass(im_a, im_b, ma, mb, ra, rb, ts.t2);
    var kc = (im_a + im_b) * dot(ts.t1, ts.t2);
    if (im_a > 0.0) {
        let w = ma * cross(ra, ts.t2);
        kc = kc + dot(cross(ra, ts.t1), w);
    }
    if (im_b > 0.0) {
        let w = mb * cross(rb, ts.t2);
        kc = kc + dot(cross(rb, ts.t1), w);
    }
    let w0 = i * 10u;
    con[w0] = ra.x;
    con[w0 + 1u] = ra.y;
    con[w0 + 2u] = ra.z;
    con[w0 + 3u] = rb.x;
    con[w0 + 4u] = rb.y;
    con[w0 + 5u] = rb.z;
    con[w0 + 6u] = kn;
    con[w0 + 7u] = kt1;
    con[w0 + 8u] = kt2;
    con[w0 + 9u] = kc;
}

/// B4 第一趟：每接触一线程（Jacobi；只读速度表）。
@compute @workgroup_size(64)
fn contact_sweep(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * (65535u * 64u);
    if (i >= P.n_c) {
        return;
    }
    let o = i * 9u;
    let ia = u32(ct[o]);
    let ib = u32(ct[o + 1u]);
    let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
    let mu = ct[o + 8u];
    let w = i * 10u;
    let ra = vec3<f32>(con[w], con[w + 1u], con[w + 2u]);
    let rb = vec3<f32>(con[w + 3u], con[w + 4u], con[w + 5u]);
    let kn = con[w + 6u];
    let kt = 0.5 * (con[w + 7u] + con[w + 8u]);
    let qa = ia * 6u;
    let qb = ib * 6u;
    let va = vec3<f32>(vel[qa], vel[qa + 1u], vel[qa + 2u])
        + cross(vec3<f32>(vel[qa + 3u], vel[qa + 4u], vel[qa + 5u]), ra);
    let vb = vec3<f32>(vel[qb], vel[qb + 1u], vel[qb + 2u])
        + cross(vec3<f32>(vel[qb + 3u], vel[qb + 4u], vel[qb + 5u]), rb);
    let vr = vb - va;
    let io = i * 3u;
    let pn0 = imp[io];
    let pn = clamp(pn0 + (-dot(vr, n)) * kn, 0.0, 1.0e9);
    imp[io] = pn;
    dimp[io] = pn - pn0;
    let ts = tangents(n);
    let lim = mu * pn;
    let p1_0 = imp[io + 1u];
    let p1 = clamp(p1_0 - dot(vr, ts.t1) * kt, -lim, lim);
    imp[io + 1u] = p1;
    dimp[io + 1u] = p1 - p1_0;
    let p2_0 = imp[io + 2u];
    let p2 = clamp(p2_0 - dot(vr, ts.t2) * kt, -lim, lim);
    imp[io + 2u] = p2;
    dimp[io + 2u] = p2 - p2_0;
}

/// B4 第二趟：每体按 CSR 升序累加并写回速度。
@compute @workgroup_size(64)
fn body_apply(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x + gid.y * (65535u * 64u);
    if (b >= P.n_b) {
        return;
    }
    let q = b * 11u;
    let im = bd[q + 7u];
    if (im <= 0.0) {
        return;
    }
    let s = ub(off[b]);
    let e = ub(off[b + 1u]);
    var dp = vec3<f32>(0.0, 0.0, 0.0);
    var dq = vec3<f32>(0.0, 0.0, 0.0);
    for (var k = s; k < e; k = k + 1u) {
        let ci = ub(items[k]);
        let o = ci * 9u;
        let ia = u32(ct[o]);
        let ib = u32(ct[o + 1u]);
        let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
        let w = ci * 10u;
        let ra = vec3<f32>(con[w], con[w + 1u], con[w + 2u]);
        let rb = vec3<f32>(con[w + 3u], con[w + 4u], con[w + 5u]);
        let ts = tangents(n);
        let io = ci * 3u;
        let j = n * dimp[io] + ts.t1 * dimp[io + 1u] + ts.t2 * dimp[io + 2u];
        if (b == ia) {
            dp = dp - j;
            dq = dq - cross(ra, j);
        } else if (b == ib) {
            dp = dp + j;
            dq = dq + cross(rb, j);
        }
    }
    let v = b * 6u;
    let lv = vec3<f32>(vel[v], vel[v + 1u], vel[v + 2u]) + dp * im;
    let o2 = b * 9u;
    let ii = vec3<f32>(iw[o2], iw[o2 + 4u], iw[o2 + 8u]);
    let av = vec3<f32>(vel[v + 3u], vel[v + 4u], vel[v + 5u]) + dq * ii;
    vel[v] = lv.x;
    vel[v + 1u] = lv.y;
    vel[v + 2u] = lv.z;
    vel[v + 3u] = av.x;
    vel[v + 4u] = av.y;
    vel[v + 5u] = av.z;
}
