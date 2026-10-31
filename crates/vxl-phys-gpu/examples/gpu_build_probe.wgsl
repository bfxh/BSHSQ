//! 约束构建内核（B3 探针）：`make_iw` 每体算世界逆惯量，`build` 每接触算 `ra/rb` 与四个有效质量项。
//!
//! 与主机 `helpers::contact_mass` / `contact_mass_cross` **同式**（含 `im_x > 0` 的静态体分支），
//! 用于口径 B 对拍。布局（flat f32，手写偏移）：
//! - `bd`: 11/体 = pos3 · quat4 · inv_mass · local_inv_i3
//! - `iw`: 9/体 = 世界逆惯量（列主序：m[0].xyz, m[1].xyz, m[2].xyz）
//! - `ct`: 9/接触 = a,b(体号当 f32) · point3 · normal3
//! - `out`: 10/接触 = ra3 · rb3 · kn · kt1 · kt2 · kcross
struct Params {
    n_c: u32,
    n_b: u32,
    _p0: f32,
    _p1: f32,
};

@group(0) @binding(0) var<storage, read> bd: array<f32>;
@group(0) @binding(1) var<storage, read_write> iw: array<f32>;
@group(0) @binding(2) var<storage, read> ct: array<f32>;
@group(0) @binding(3) var<storage, read_write> out: array<f32>;
@group(0) @binding(4) var<uniform> P: Params;

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

/// 每体：`M = R·diag(local_inv_i)·Rᵀ`（世界逆惯量）。
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

/// 与 `contact_mass` 同式。
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

/// 每接触：`ra/rb` + `kn/kt1/kt2/kcross`。
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
    // 交叉项（`contact_mass_cross` 同式）。
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
    out[w0] = ra.x;
    out[w0 + 1u] = ra.y;
    out[w0 + 2u] = ra.z;
    out[w0 + 3u] = rb.x;
    out[w0 + 4u] = rb.y;
    out[w0 + 5u] = rb.z;
    out[w0 + 6u] = kn;
    out[w0 + 7u] = kt1;
    out[w0 + 8u] = kt2;
    out[w0 + 9u] = kc;
}
