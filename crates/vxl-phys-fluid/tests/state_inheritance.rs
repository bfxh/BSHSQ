//! **流体状态继承判据**（`StateBridge` 的②档，2026-10-08）：位置 + 速度 + 逐粒质量。
//!
//! 判据链（逐条钉）：
//! ① `export_state` 与 `export_positions` **同序同值**，逐粒质量 = `particle_mass()`（标定值）；
//! ② 刚体速度场 `v = u + ω×(p−c)` 导入后**逐位**取回 ⇒ 总动量 = `M·u`、绕质心角动量 = `I·ω`
//!    （两条都用**独立解析式**算，不复用桥的实现）；
//! ③ `handoff(src→dst)` 之后两系统状态**逐位相同**；
//! ④ **金丝雀**：位置桥（`import_positions`）清速度 ⇒ 两条路的动量必须可分辨（判据非恒真）；
//! ⑤ 长度不符 ⇒ 整体拒绝且**一字不动**（先校验、再写入，不许半写）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `panic!` / `.clone()`（新文件零基线）。
use vxl_phys_core::interop::{handoff, BridgeKind, BridgeState, StateBridge};
use vxl_phys_core::Vec3;
use vxl_phys_fluid::{FluidConfig, FluidSystem};

/// `dims` 个三轴晶格（间距 0.05、默认核半径）。
fn sys(dims: [usize; 3]) -> FluidSystem {
    FluidSystem::new(FluidConfig::default(), Vec3::splat(-0.2), dims, 0.05)
}

/// 刚体速度场 `v = u + ω×(p−c)`：状态继承的规范输入（不是逐点乱写）。
fn rigid_field(pos: &[Vec3], u: Vec3, w: Vec3, c: Vec3, out: &mut Vec<Vec3>) {
    out.clear();
    for p in pos {
        out.push(u + w.cross(*p - c));
    }
}

#[test]
fn state_bridge_carries_velocity_and_mass() {
    let mut f = sys([2, 2, 2]);
    let n = f.len();
    assert_eq!(n, 8);
    assert_eq!(f.kind(), BridgeKind::Particle);

    let mut pos = Vec::new();
    f.export_positions(&mut pos);
    let mut c = Vec3::ZERO;
    for p in &pos {
        c += *p;
    }
    c *= 1.0 / n as f32;

    let mut st = BridgeState::default();
    f.export_state(&mut st);
    assert_eq!(st.pos, pos, "状态桥与位置桥必须同序同值");
    assert!(st.vel.iter().all(|v| *v == Vec3::ZERO), "静止初态速度为零");
    let m = f.particle_mass();
    assert_eq!(st.mass.len(), n);
    assert!(st.mass.iter().all(|x| *x == m), "逐粒质量 = 晶格标定质量");

    // ② 刚体速度场导入 ⇒ 逐位取回
    let u = Vec3::new(0.3, -0.1, 0.2);
    let w = Vec3::new(0.0, 1.0, 0.0);
    let mut vel = Vec::new();
    rigid_field(&pos, u, w, c, &mut vel);
    st.vel = vel;
    assert!(f.import_state(&st), "长度相符必须接受");
    let mut got = BridgeState::default();
    f.export_state(&mut got);
    assert_eq!(got.pos, st.pos);
    assert_eq!(got.vel, st.vel, "速度必须逐位继承");
    assert_eq!(got.mass, st.mass);

    // 总动量 = M·u（M = n·m）
    assert!(got.is_full(), "速度与质量必须都已登记");
    let p = got.momentum().unwrap_or(Vec3::ZERO);
    let m_total = got.total_mass().unwrap_or(0.0);
    // 总质量按索引序逐个累加 ⇒ 与 n·m 允许 1 ulp 级差异（别拿浮点当精确算术）
    let want_total = m * n as f32;
    let rel_m = ((m_total - want_total) / want_total).abs();
    assert!(rel_m < 1e-6, "总质量应 = n·m：rel={rel_m:e}");
    let want_p = u * m_total;
    let rel_p = (p - want_p).length() / want_p.length();
    assert!(rel_p < 1e-6, "总动量应 = M·u：rel={rel_p:e}（p={p:?}）");

    // 绕质心的角动量 = I_c·ω（独立用惯量张量 Σm[|r|²ω − (r·ω)r] 算）
    let mut iw = Vec3::ZERO;
    for i in 0..n {
        let r = got.pos[i] - c;
        iw += (w * r.length_squared() - r * r.dot(w)) * got.mass[i];
    }
    let l = got.angular_momentum_about(c).unwrap_or(Vec3::ZERO);
    // 逐分量口径：ω 方向（y）给相对误差；横向分量是 f32 抵消残差（±0.025 量级位置相减）
    // ⇒ 只能给绝对容差 —— 拿合成向量做相对判据会被残差主导（本仓已知坑：小量相减抵消）。
    let rel_l = ((l.y - iw.y) / iw.y).abs();
    assert!(
        rel_l < 1e-5,
        "绕质心角动量（ω 方向）应 = I·ω：rel={rel_l:e}（L={l:?}）"
    );
    let lateral = (l.x - iw.x).abs().max((l.z - iw.z).abs());
    assert!(
        lateral < 1e-6,
        "横向分量应压在抵消残差内：{lateral:e}（L={l:?}）"
    );
}

#[test]
fn handoff_is_bitwise_and_positions_only_bridge_is_a_canary() {
    let mut src = sys([2, 2, 2]);
    let mut st = BridgeState::default();
    src.export_state(&mut st);
    for (i, v) in st.vel.iter_mut().enumerate() {
        *v = Vec3::new(0.1 * i as f32, -0.2, 0.05);
    }
    assert!(src.import_state(&st));

    // ③ 域间交接：同形两系统 ⇒ 逐位相同
    let mut dst = sys([2, 2, 2]);
    assert!(handoff(&src, &mut dst), "同形两系统必须交接成功");
    let mut a = BridgeState::default();
    let mut b = BridgeState::default();
    src.export_state(&mut a);
    dst.export_state(&mut b);
    assert_eq!(a, b, "交接后两系统状态逐位相同");
    assert!(a.momentum() != Some(Vec3::ZERO), "用例非平凡：源确实带速度");

    // ④ 金丝雀：位置桥清速度 ⇒ 两条路可分辨
    let mut flat = sys([2, 2, 2]);
    assert!(flat.import_positions(&a.pos));
    let mut fb = BridgeState::default();
    flat.export_state(&mut fb);
    assert_eq!(fb.momentum(), Some(Vec3::ZERO), "位置桥导入必须清速度");
    assert_ne!(
        fb.momentum(),
        a.momentum(),
        "位置桥与状态桥的动量必须可分辨（否则判据恒真）"
    );

    // ⑤ 点数不符 ⇒ 整体拒绝且一字不动
    let mut small = sys([1, 2, 2]);
    assert!(!handoff(&src, &mut small), "点数不符必须拒绝");
    let mut small_before = BridgeState::default();
    small.export_state(&mut small_before);
    let mut bad = BridgeState::default();
    src.export_state(&mut bad);
    bad.pos.pop();
    assert!(!small.import_state(&bad), "长度不符必须拒绝");
    let mut small_after = BridgeState::default();
    small.export_state(&mut small_after);
    assert_eq!(small_before, small_after, "拒绝的导入不许改动任何一位");
}
