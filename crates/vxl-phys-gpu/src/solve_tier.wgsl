//! 求解**卡上档**的核（`PLAN-gpu.md` §30 B 片：B1 差量上传 + B3 构建 + B4 Jacobi）。
//!
//! 布局与 `vxl_phys_core::solve_tier` 的常量**逐字对应**（改一处要同步另两处）：
//! - `bd`  : 11/体 = pos3 · quat4 · inv_mass · local_inv_i3（**睡眠体按质量无限打包**）
//! - `iw`  : 9/体 = 世界逆惯量（列主序，`make_iw` 的产物）
//! - `vel` : 6/体 = lin3 · ang3（每趟整表上传，解算后整表回读）
//! - `ct`  : 12/接触 = a,b(体号) · point3 · normal3 · depth · mu · e · pad
//! - `con` : 12/接触 = ra3 · rb3 · kn · kt1 · kt2 · kc · rhs · cfm · mu · pad2
//! - `imp` / `dimp` : 3/接触 = (法向, 切1, 切2) 的累计与本趟增量
//! - `off` / `items`: CSR（体 → 接触序号，**升序** ⇒ 体累加的求和序确定）
//!
//! 式子逐条对齐主机（`vxl-phys-solver`）：`helpers::tangents`、`contact_mass`、
//! `contact_mass_cross`、`constraint::soft_contact`、`island::iterate` 的法向/摩擦两通道；
//! ⚠️ **扫描图是 Jacobi**（同一趟内所有接触读同一份 `vel`，`body_apply` 才写回），
//! 主机是顺序 GS；且本核**无暖启动、无锚点回收、无切向漂移回拉**（`trhs` 恒 0）。

struct Params {
    n_c: u32,
    n_b: u32,
    n_upd: u32,
    sweeps: u32,
    dt: f32,
    inv_dt: f32,
    erp_dyn: f32,
    erp_static: f32,
    slop: f32,
    max_corr: f32,
    cfm_dyn: f32,
    cfm_static: f32,
    e_threshold: f32,
    bias: f32,
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
@group(0) @binding(7) var<storage, read> off: array<u32>;
@group(0) @binding(8) var<storage, read> items: array<u32>;
@group(0) @binding(9) var<uniform> P: Params;
/// B1 增量：`upd_idx[i]` = 体号 → `upd_rec[i*11 ..]` = 新记录。
@group(0) @binding(10) var<storage, read> upd_idx: array<u32>;
@group(0) @binding(11) var<storage, read> upd_rec: array<f32>;
/// 诊断字（每趟清零后累加）：`[0]` = 体号越界次数，`[1]` = 有效质量退化次数。
@group(0) @binding(12) var<storage, read_write> diag: array<atomic<u32>>;

fn gsz() -> u32 {
    return 65535u * 64u;
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

fn pos(b: u32) -> vec3<f32> {
    let q = b * 11u;
    return vec3<f32>(bd[q], bd[q + 1u], bd[q + 2u]);
}

fn im_of(b: u32) -> f32 {
    return bd[b * 11u + 7u];
}

fn vel_at(b: u32, r: vec3<f32>) -> vec3<f32> {
    let q = b * 6u;
    let v = vec3<f32>(vel[q], vel[q + 1u], vel[q + 2u]);
    let w = vec3<f32>(vel[q + 3u], vel[q + 4u], vel[q + 5u]);
    return v + cross(w, r);
}

/// 切向基（= 主机 `helpers::tangents`，含 `refr` 的选择与手性）。
fn tangents(n: vec3<f32>) -> mat3x3<f32> {
    var refr = vec3<f32>(0.0, 1.0, 0.0);
    if (abs(n.y) >= 0.9) {
        refr = vec3<f32>(1.0, 0.0, 0.0);
    }
    let t1 = normalize(cross(n, refr));
    return mat3x3<f32>(t1, cross(n, t1), n);
}

/// 有效质量 1/k（= 主机 `contact_mass` 的收尾：k 太小时给 0，不除爆）。
fn eff_mass(im_a: f32, im_b: f32, ma: mat3x3<f32>, mb: mat3x3<f32>, ra: vec3<f32>, rb: vec3<f32>, d: vec3<f32>) -> f32 {
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
    // 退化（双静态对混进来了 / 质量表被写坏）⇒ 记一笔，别静默。
    atomicAdd(&diag[1u], 1u);
    return 0.0;
}

/// 切向 2×2 的交叉项 k12（= 主机 `contact_mass_cross`，K 各向异性时必须联立）。
fn mass_cross(im_a: f32, im_b: f32, ma: mat3x3<f32>, mb: mat3x3<f32>, ra: vec3<f32>, rb: vec3<f32>, d1: vec3<f32>, d2: vec3<f32>) -> f32 {
    var k = (im_a + im_b) * dot(d1, d2);
    if (im_a > 0.0) {
        let w = ma * cross(ra, d2);
        k = k + dot(cross(ra, d1), w);
    }
    if (im_b > 0.0) {
        let w = mb * cross(rb, d2);
        k = k + dot(cross(rb, d1), w);
    }
    return k;
}

/// B1：变动记录散写进常驻体表（与主核同一次提交 ⇒ 不额外加同步）。
@compute @workgroup_size(64)
fn scatter_records(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * gsz();
    if (i >= P.n_upd) {
        return;
    }
    let b = upd_idx[i];
    if (b >= P.n_b) {
        atomicAdd(&diag[0u], 1u);
        return;
    }
    let dst = b * 11u;
    let src = i * 11u;
    for (var k = 0u; k < 11u; k = k + 1u) {
        bd[dst + k] = upd_rec[src + k];
    }
}

/// 每体世界逆惯量 `M = R·diag(local)·Rᵀ`（B1 的常驻量；位姿变了才重算，本档每趟全算）。
@compute @workgroup_size(64)
fn make_iw(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x + gid.y * gsz();
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

/// B3：每接触算力臂 + 四个质量项 + 软接触目标（`rhs`/`cfm`），写进 `con`。
@compute @workgroup_size(64)
fn build(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * gsz();
    if (i >= P.n_c) {
        return;
    }
    let o = i * 12u;
    let ia = u32(ct[o]);
    let ib = u32(ct[o + 1u]);
    if (ia >= P.n_b || ib >= P.n_b) {
        atomicAdd(&diag[0u], 1u);
        return;
    }
    let pt = vec3<f32>(ct[o + 2u], ct[o + 3u], ct[o + 4u]);
    let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
    let depth = ct[o + 8u];
    let mu = ct[o + 9u];
    let e = ct[o + 10u];
    let ra = pt - pos(ia);
    let rb = pt - pos(ib);
    let im_a = im_of(ia);
    let im_b = im_of(ib);
    let ma = load_iw(ia);
    let mb = load_iw(ib);
    let tf = tangents(n);
    let t1 = tf[0];
    let t2 = tf[1];
    let kn = eff_mass(im_a, im_b, ma, mb, ra, rb, n);
    let kt1 = eff_mass(im_a, im_b, ma, mb, ra, rb, t1);
    let kt2 = eff_mass(im_a, im_b, ma, mb, ra, rb, t2);
    let kc = mass_cross(im_a, im_b, ma, mb, ra, rb, t1, t2);
    // —— 软接触目标（= 主机 `constraint::soft_contact`；深度取窄相裁剪点原值，无回收推进）——
    let vr = vel_at(ib, rb) - vel_at(ia, ra);
    let vn = dot(vr, n);
    var bounce = 0.0;
    if (vn < -P.e_threshold) {
        bounce = -e * vn;
    }
    let spec = max(-depth, 0.0) * P.inv_dt;
    let stat_pair = (im_a <= 0.0) || (im_b <= 0.0);
    let erp = select(P.erp_dyn, P.erp_static, stat_pair);
    let pen = max(depth - P.slop, 0.0);
    let bias = min(erp * pen, P.max_corr) * P.bias;
    var rhs = bias;
    if (bounce > 0.0) {
        rhs = rhs + bounce;
    } else {
        rhs = rhs + bounce - spec;
    }
    var cfm = 1.0;
    if (depth < 0.0) {
        cfm = select(P.cfm_dyn, P.cfm_static, stat_pair);
    }
    let w = i * 12u;
    con[w] = ra.x;
    con[w + 1u] = ra.y;
    con[w + 2u] = ra.z;
    con[w + 3u] = rb.x;
    con[w + 4u] = rb.y;
    con[w + 5u] = rb.z;
    con[w + 6u] = kn;
    con[w + 7u] = kt1;
    con[w + 8u] = kt2;
    con[w + 9u] = kc;
    con[w + 10u] = rhs;
    con[w + 11u] = cfm;
    // 冲量清零 = **无暖启动**（默认档在这里读上一帧的暖冲量）。
    let io = i * 3u;
    imp[io] = 0.0;
    imp[io + 1u] = 0.0;
    imp[io + 2u] = 0.0;
    dimp[io] = 0.0;
    dimp[io + 1u] = 0.0;
    dimp[io + 2u] = 0.0;
}

/// B4 第一趟：每接触一线程（Jacobi——本趟只**读** `vel`，写 `imp`/`dimp`）。
@compute @workgroup_size(64)
fn contact_sweep(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x + gid.y * gsz();
    if (i >= P.n_c) {
        return;
    }
    let o = i * 12u;
    let ia = u32(ct[o]);
    let ib = u32(ct[o + 1u]);
    let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
    let mu = ct[o + 9u];
    let w = i * 12u;
    let ra = vec3<f32>(con[w], con[w + 1u], con[w + 2u]);
    let rb = vec3<f32>(con[w + 3u], con[w + 4u], con[w + 5u]);
    let kn = con[w + 6u];
    let kt1 = con[w + 7u];
    let kt2 = con[w + 8u];
    let kc = con[w + 9u];
    let rhs = con[w + 10u];
    let cfm = con[w + 11u];
    let vr = vel_at(ib, rb) - vel_at(ia, ra);
    let io = i * 3u;
    // —— 法向：λ ← cfm·(λ + m·(rhs − vn))，钳 ≥ 0（= 主机 `island.rs` 同式）——
    let pn0 = imp[io];
    let pn = max(cfm * (pn0 + kn * (rhs - dot(vr, n))), 0.0);
    let dn = pn - pn0;
    imp[io] = pn;
    dimp[io] = dn;
    // ⚠️ 每趟先清切向增量：下面任一提前 `return`（退化 K）若不清，`body_apply` 会把**上一趟**
    // 的切向增量重复累加一次（`dimp` 是跨趟复用的缓冲）。
    dimp[io + 1u] = 0.0;
    dimp[io + 2u] = 0.0;
    // —— 摩擦：**精确 2×2 联立** + 圆锥（各向同性 ⇒ 与主机同式；K 各向异性基本档不支持）——
    let tf = tangents(n);
    let t1 = tf[0];
    let t2 = tf[1];
    // 主机在法向冲量**之后**才重算相对速度做摩擦（点内顺序）⇒ 这里把同一笔修正解析地加进 vr。
    var vrf = vr;
    if (dn != 0.0) {
        let jj = n * dn;
        let im_a = im_of(ia);
        let im_b = im_of(ib);
        let dvr = jj * (im_a + im_b)
            + cross(load_iw(ib) * cross(rb, jj), rb)
            + cross(load_iw(ia) * cross(ra, jj), ra);
        vrf = vrf + dvr;
    }
    let k11 = select(0.0, 1.0 / kt1, kt1 > 0.0);
    let k22 = select(0.0, 1.0 / kt2, kt2 > 0.0);
    let det = k11 * k22 - kc * kc;
    if (det <= 1e-12) {
        return;
    }
    let inv = 1.0 / det;
    let vt1 = dot(vrf, t1);
    let vt2 = dot(vrf, t2);
    let dv1 = (-vt1 * k22 + vt2 * kc) * inv;
    let dv2 = (-vt2 * k11 + vt1 * kc) * inv;
    let old1 = imp[io + 1u];
    let old2 = imp[io + 2u];
    var a1 = old1 + dv1;
    var a2 = old2 + dv2;
    let max_f = mu * pn;
    let f2 = a1 * a1 + a2 * a2;
    if (f2 > max_f * max_f && f2 > 1e-20) {
        let s = max_f / sqrt(f2);
        a1 = a1 * s;
        a2 = a2 * s;
    }
    imp[io + 1u] = a1;
    imp[io + 2u] = a2;
    dimp[io + 1u] = a1 - old1;
    dimp[io + 2u] = a2 - old2;
}

/// B4 第二趟：每体按 CSR **升序**累加本趟增量并写回速度（角向用完整世界逆惯量矩阵）。
@compute @workgroup_size(64)
fn body_apply(@builtin(global_invocation_id) gid: vec3<u32>) {
    let b = gid.x + gid.y * gsz();
    if (b >= P.n_b) {
        return;
    }
    let im = im_of(b);
    if (im <= 0.0) {
        return;
    }
    let s = off[b];
    let e = off[b + 1u];
    var dp = vec3<f32>(0.0, 0.0, 0.0);
    var dq = vec3<f32>(0.0, 0.0, 0.0);
    for (var k = s; k < e; k = k + 1u) {
        let ci = items[k];
        let o = ci * 12u;
        let ia = u32(ct[o]);
        let ib = u32(ct[o + 1u]);
        let n = vec3<f32>(ct[o + 5u], ct[o + 6u], ct[o + 7u]);
        let w = ci * 12u;
        let ra = vec3<f32>(con[w], con[w + 1u], con[w + 2u]);
        let rb = vec3<f32>(con[w + 3u], con[w + 4u], con[w + 5u]);
        let tf = tangents(n);
        let io = ci * 3u;
        let j = n * dimp[io] + tf[0] * dimp[io + 1u] + tf[1] * dimp[io + 2u];
        if (b == ia) {
            dp = dp - j;
            dq = dq - cross(ra, j);
        } else if (b == ib) {
            dp = dp + j;
            dq = dq + cross(rb, j);
        }
    }
    let v = b * 6u;
    let m = load_iw(b);
    vel[v] = vel[v] + dp.x * im;
    vel[v + 1u] = vel[v + 1u] + dp.y * im;
    vel[v + 2u] = vel[v + 2u] + dp.z * im;
    let av = vec3<f32>(vel[v + 3u], vel[v + 4u], vel[v + 5u]);
    let nv = av + m * dq;
    vel[v + 3u] = nv.x;
    vel[v + 4u] = nv.y;
    vel[v + 5u] = nv.z;
}

/// 诊断清零（每趟在链首跑一次；64 线程足够覆盖诊断字数）。
@compute @workgroup_size(64)
fn clear_diag(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < arrayLength(&diag)) {
        atomicStore(&diag[gid.x], 0u);
    }
}
