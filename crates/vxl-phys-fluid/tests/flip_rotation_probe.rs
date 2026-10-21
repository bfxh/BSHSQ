//! **FLIP 传递耗散判据**（旋转团 + APIC A/B）：立方水团绕竖直轴刚体旋转，零重力/零外力/
//! 无边界 ⇒ 末态绕 y 角动量 `L_y` 的损失**只可能来自 P2G/G2P 那一趟三线性传输**。
//!
//! 主判据 = `flip_apic_derived_c_buys_rotation`（**正结果**：`C` 每子步从**粒子场**现算
//! ⇒ APIC 把旋转保住；`flip_apic` 仍是默认 `false` 的 A/B 实验档，理由是近静场代价）。
//!
//! 完整推导与读数表**不在此复制**，见 `src/config.rs` 的 `flip_apic` doc 与 `src/flip.rs`
//! 模块头 `## APIC` 节。
//!
//! 跑法：`cargo test -p vxl-phys-fluid --test flip_rotation_probe`
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Vec3};
use vxl_phys_fluid::{FluidConfig, FluidFamily, FluidSystem};

/// 水槽（半空间族）：地板 y=0 + 四壁内面 x/z = ±`cav`，堰高 `wall`（堰上不设墙）。
/// 与 `pbf_incompressibility_probe.rs::Basin` / `boundary_accuracy_probe.rs::Basin` 同口径。
struct Basin {
    cav: f32,
    wall: f32,
}

impl Basin {
    fn faces(&self) -> [(usize, f32, f32); 5] {
        [
            (1, 0.0, 1.0),
            (0, -self.cav, 1.0),
            (0, self.cav, -1.0),
            (2, -self.cav, 1.0),
            (2, self.cav, -1.0),
        ]
    }
}

impl ProviderColliders for Basin {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(Aabb {
            min: Vec3::new(-50.0, -50.0, -50.0),
            max: Vec3::new(50.0, 50.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, probe: f32, out: &mut Vec<InteropContact>) -> bool {
        for &(axis, plane, sign) in &self.faces() {
            let comp = [p.x, p.y, p.z][axis];
            if axis != 1 && comp > self.wall {
                continue;
            }
            let sdf = (comp - plane) * sign;
            if sdf > 2.0 * probe {
                continue;
            }
            let point = match axis {
                0 => Vec3::new(plane, p.y, p.z),
                1 => Vec3::new(p.x, plane, p.z),
                _ => Vec3::new(p.x, p.y, plane),
            };
            let normal = match axis {
                0 => Vec3::new(sign, 0.0, 0.0),
                1 => Vec3::new(0.0, sign, 0.0),
                _ => Vec3::new(0.0, 0.0, sign),
            };
            out.push(InteropContact {
                point,
                normal,
                depth: probe - sdf,
                feature: 0,
            });
        }
        true
    }
}

/// 位模式指纹（FNV-1a，与 `pbf_incompressibility_probe.rs` / `src/tests.rs` 同口径）。
fn digest(f: &FluidSystem) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut eat = |b: u32| {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    for p in f.positions().iter().chain(f.velocities()) {
        eat(p.x.to_bits());
        eat(p.y.to_bits());
        eat(p.z.to_bits());
    }
    h
}

/// 旋转团场景的读数：`l_keep` = 末态/起步的**绕竖直轴角动量**（绕团心口径）、
/// `rot_err` = 去平移后速度场偏离刚体旋转的相对 RMS、`ke`/`vmax` 同主表、`digest` 逐位复现用。
struct Rot {
    l_keep: f32,
    rot_err: f32,
    ke: f32,
    vmax: f32,
    digest: u64,
}

/// 团心（三轴均值；`Σ(p − 团心) = 0` 是角动量口径里消掉平移项的依据）。
fn centroid(f: &FluidSystem) -> Vec3 {
    let n = f.positions().len().max(1);
    let s = f.positions().iter().fold(Vec3::ZERO, |a, p| a + *p);
    s * (1.0 / n as f32)
}

/// 刚体旋转速度场 `ω·ŷ × d`（`d = p − 团心`）⇒ `(ω·d.z, 0, −ω·d.x)`。
fn swirl(p: Vec3, c: Vec3, omega: f32) -> Vec3 {
    let d = p - c;
    Vec3::new(omega * d.z, 0.0, -omega * d.x)
}

/// 绕竖直轴的角动量 `Σ (d.z·v.x − d.x·v.z)`（`d = p − 团心`；平移项被 `Σd = 0` 消掉，
/// 重力沿 y 对本分量零力矩 ⇒ 本量只被**传输/投影**耗散）。
fn ang_mom_y(f: &FluidSystem, c: Vec3) -> f32 {
    f.positions()
        .iter()
        .zip(f.velocities())
        .map(|(p, v)| {
            let d = *p - c;
            d.z * v.x - d.x * v.z
        })
        .sum()
}

/// 去平移后速度场偏离刚体旋转 `ω·ŷ×(p−c)` 的相对 RMS（**噪声**读数：FLIP 的网格噪声、
/// APIC 的仿射失真都在这里现形）。
fn rot_err(f: &FluidSystem, c: Vec3, omega: f32) -> f32 {
    let n = f.positions().len().max(1);
    let vmean = f.velocities().iter().fold(Vec3::ZERO, |a, v| a + *v) * (1.0 / n as f32);
    let (mut num, mut den) = (0.0f32, 0.0f32);
    for (p, v) in f.positions().iter().zip(f.velocities()) {
        let e = (*v - vmean) - swirl(*p, c, omega);
        num += e.length_squared();
        den += swirl(*p, c, omega).length_squared();
    }
    (num / den.max(f32::MIN_POSITIVE)).sqrt()
}

/// **旋转团场景**（APIC 判据的专用台架）：立方水团**绕竖直轴刚体旋转**，`v₀ = ω·ŷ × (x − x_c)`。
///
/// 场景设成这样的理由（判据的解释力全在这）：刚体旋转是**无散**场 ⇒ 压力投影无事可做
/// （`p ≈ 0`）；`gravity = ZERO` 且默认 `dynamic_viscosity = 0`（`laminar::pass` 自短路）
/// ⇒ **没有任何体外力**；再 `set_boundaries(&[])` 让壁面（含地板的无滑移）退出 ⇒ 末态 `L_y`
/// 的损失**只可能来自 P2G/G2P 那一趟三线性传输**——正是 APIC 那一刀要治的病，且四档
/// （PIC / APIC / FLIP / FLIP+APIC）之间**只差传输算子**。
fn rot_case(ratio: f32, apic: bool, ticks: usize) -> Rot {
    let h = 0.1f32;
    let s = h / 2.0;
    let cfg = FluidConfig {
        family: FluidFamily::CpuFlip,
        smoothing_radius: h,
        gravity: Vec3::ZERO,
        flip_ratio: ratio,
        flip_apic: apic,
        ..FluidConfig::default()
    };
    let mut f = FluidSystem::new(cfg, Vec3::new(-0.25, 0.55, -0.25), [10, 10, 10], s);
    f.set_boundaries(&[]);
    let c = centroid(&f);
    let omega = 2.0f32;
    let v0: Vec<Vec3> = f.positions().iter().map(|p| swirl(*p, c, omega)).collect();
    f.set_velocities(&v0);
    // 远壁（`cav`/`wall` 极大 ⇒ 即使被查到也在 1000 m 外；无边界时根本不会被查）。
    let far = Basin {
        cav: 1.0e3,
        wall: 1.0e6,
    };
    let l0 = ang_mom_y(&f, c);
    for _ in 0..ticks {
        f.step(1.0 / 60.0, &far);
    }
    let (mut ke, mut vmax) = (0.0f32, 0.0f32);
    for v in f.velocities() {
        ke += v.length_squared();
        vmax = vmax.max(v.length());
    }
    Rot {
        l_keep: ang_mom_y(&f, c) / l0.max(f32::MIN_POSITIVE),
        rot_err: rot_err(&f, c, omega),
        ke,
        vmax,
        digest: digest(&f),
    }
}

/// **APIC A/B 判据（实测的"买到红利"结果，逐条钉住）**：**每子步现算**的 `C`
/// （`flip/affine.rs::estimate_from_field`：粒子场加权最小二乘 + 岭 + 拟合质量收缩）
/// **真的买到了 APIC 的旋转红利**。
///
/// 台架见 `rot_case`（零重力/无外力/无边界 ⇒ 末态 `L_y` 只被 P2G/G2P 那一趟三线性传输耗散，
/// 与压力投影、体外力都无关）。判据把结论钉成**可回归的断言**：
/// ① **台架有判别力**：`θ = 0.95`（FLIP）保住旋转，比 PIC 高一个量级 ⇒ 这个台架真的看得见
///    传输耗散，②③ 才不是空断言；
/// ② **正结果**：同 `θ = 0`、只换 `flip_apic`，APIC 把 `l_keep` 从 PIC 的 0.145 提到 0.981
///    （≈ 无耗散），且**不得过冲**：`< 1.05`。这条是钉住"岭 `λ·tr D/3` + 收缩
///    `C ← (⟨C,M⟩/T)·C`"那一刀的——驯服前是 1.033（伪增益）、把 `C` 写死成解析 ω× 是 1.081，
///    而没有收缩时近静场会炸（见 ⑤ 的 P23 读数）；
/// ③ **场更接近刚体**：APIC 的 `rot_err` 必须明显低于 PIC（实测 0.429 vs 0.864）；
/// ④ **不搅乱**：`θ = 0.95` 开 APIC 不得把角动量搞坏（**门控落盘后该档 ≡ FLIP**，见 G1；
///    门控前实测反升 0.866 → 1.073 ⇒ 门控把这笔红利连同代价一起收掉）；
/// ⑤ **默认关**：`flip_apic` 默认 `false`（off 路径逐字节不动；开档是 A/B 实验档——
///    静水槽代价 `ke` +18%、`sink` +15%、`over_in` −61%，见 `docs/OPEN-PROBLEMS.md` P23）；
/// ⑥ **A/B 档也逐位复现**：同档跑两遍读数逐位相同（含 `digest`）。
///
/// 读数由本测试自己打印（`--nocapture`），同见 `diag_apic_rotation`。
/// 〔历史〕上一版这条判据断言的是**负结果**（tick 级 `C`：APIC 与 PIC 无差别，0.145 → 0.155）；
/// `C` 改成每子步从粒子场现算后事实翻转，故按本文件自己的约定（"先回头核文档与实现，再改这条
/// 断言"）重写成正结果。
#[test]
fn flip_apic_derived_c_buys_rotation() {
    let ticks = 30;
    let pic = rot_case(0.0, false, ticks);
    let apic = rot_case(0.0, true, ticks);
    let flip = rot_case(0.95, false, ticks);
    let flip_apic = rot_case(0.95, true, ticks);
    let again = rot_case(0.95, true, ticks);
    eprintln!("旋转团（30 tick，ω = 2 rad/s，零重力/无边界；`l_keep` = 末态/起步 L_y）");
    eprintln!(
        "{:12} {:9} {:9} {:11} {:8}",
        "档位", "l_keep", "rot_err", "ke", "vmax"
    );
    for (n, r) in [
        ("PIC", &pic),
        ("APIC", &apic),
        ("FLIP", &flip),
        ("FLIP+APIC", &flip_apic),
    ] {
        eprintln!(
            "{n:12} {:9.4} {:9.4} {:11.3e} {:8.4}",
            r.l_keep, r.rot_err, r.ke, r.vmax
        );
    }
    // ① 台架判别力：FLIP 必须显著优于 PIC（实测 0.866 vs 0.145，≈ 6×）。
    assert!(
        flip.l_keep > 4.0 * pic.l_keep,
        "旋转团台架看不见传输耗散：flip {} vs pic {} ⇒ 下面几条无从谈起",
        flip.l_keep,
        pic.l_keep
    );
    // ② 正结果：派生 C 在 θ = 0 下必须买到红利（实测 0.981 vs 0.145，≈ 6.8×；放进 4× 留余量）。
    assert!(
        apic.l_keep > 4.0 * pic.l_keep,
        "派生 C 没买到旋转红利（apic {} vs pic {}）⇒ 先查 `estimate_from_field` 是否真的接了\
         p2g、以及岭/收缩是否把 `C` 压死",
        apic.l_keep,
        pic.l_keep
    );
    // ②b 不过冲：伪增益必须被压住（驯服后实测 0.981；驯服前 1.033、解析真值 1.081）。
    assert!(
        apic.l_keep < 1.05,
        "派生 C 过冲到 {}（> 1.05）⇒ 岭/拟合质量收缩这两刀失效了（伪增益回来了）",
        apic.l_keep
    );
    // ③ 场更刚体：APIC 的偏离刚体的 RMS 必须明显低于 PIC（实测 0.429 vs 0.864，≈ 0.50×）。
    assert!(
        apic.rot_err < 0.6 * pic.rot_err,
        "APIC 没让场更接近刚体：rot_err {} vs pic {}",
        apic.rot_err,
        pic.rot_err
    );
    // ④ 不搅乱：FLIP 档开 APIC 不得把角动量搞坏 > 3%（门控落盘后该档 ≡ FLIP ⇒ 恒等通过；
    //    门控前实测反升到 1.073）。
    assert!(
        flip_apic.l_keep >= 0.97 * flip.l_keep,
        "APIC 把 FLIP 档的角动量搞坏了：{} vs {}",
        flip_apic.l_keep,
        flip.l_keep
    );
    // ⑤ 默认关：off 路径逐字节不动是本仓的硬不变量（A/B 档要开得显式设 true）。
    assert!(
        !FluidConfig::default().flip_apic,
        "`flip_apic` 默认值被翻成 true 了 ⇒ 所有近静场会付 P23 里记的那笔代价"
    );
    // ⑥ 表面完整性（阈值同主表）+ A/B 档逐位复现。
    for (n, r) in [
        ("PIC", &pic),
        ("APIC", &apic),
        ("FLIP", &flip),
        ("FLIP+APIC", &flip_apic),
    ] {
        assert!(r.vmax < 4.0, "{n} 表面炸开：vmax {}", r.vmax);
    }
    assert_eq!(
        again.digest, flip_apic.digest,
        "APIC 档不逐位复现（digest）"
    );
    assert_eq!(
        again.l_keep.to_bits(),
        flip_apic.l_keep.to_bits(),
        "APIC 档不逐位复现（l_keep 位不同）"
    );
}

/// **θ 门控 · 判据 G1**：`θ = 0.95`（**超** `θ_max = 0.5`）时，即使 `flip_apic = true`
/// 也必须与 `flip_apic = false` **逐位相同** ⇒ 门控把「红利/代价两头挨打」那一档彻底关掉，
/// 且关得**不留痕**（不是"近似退回"，是同一位模式）。
///
/// 门控实现见 `src/flip/affine.rs::apic_allowed`（`θ_max = APIC_MAX_THETA = 0.5`）。
#[test]
fn apic_theta_gate_shuts_high_theta_bit_exactly() {
    let ticks = 30;
    let off = rot_case(0.95, false, ticks);
    let on = rot_case(0.95, true, ticks);
    assert!(
        on.digest == off.digest,
        "θ = 0.95 超门控（θ_max = 0.5）⇒ APIC 档必须与关档逐位相同：digest {} vs {}",
        on.digest,
        off.digest
    );
    assert!(
        on.l_keep.to_bits() == off.l_keep.to_bits(),
        "θ = 0.95 超门控 ⇒ l_keep 位不同 ⇒ 门控没生效"
    );
    assert!(
        on.rot_err.to_bits() == off.rot_err.to_bits(),
        "θ = 0.95 超门控 ⇒ rot_err 位不同 ⇒ 门控没生效"
    );
}

/// **θ 门控 · 判据 G2**：`θ = 0.5`（**恰在**门控边界 `θ_max = 0.5`）时 APIC **必须真的生效**
/// 且红利仍在：`l_keep ≥ 0.95`（实测 1.0071）、`rot_err ≤ 0.50`（实测 0.4385）。
///
/// 这条钉的是门控的**另一头**：门控不能把红利那一档也一起关掉（否则门控只是"把 APIC 焊死"
/// 而不是"把代价那一头收掉"）。读数取自 `diag_apic_theta_scan`（30 tick，ω = 2 rad/s）：
/// **θ = 0.5 关档** `l_keep` 0.3760 / `rot_err` 0.6728 ⇒ 开档 `rot_err` 0.4385 明显优于关档 0.6728。
#[test]
fn apic_theta_gate_keeps_benefit_at_theta_half() {
    let ticks = 30;
    let on = rot_case(0.5, true, ticks);
    let off = rot_case(0.5, false, ticks);
    assert!(
        on.l_keep >= 0.95,
        "θ = 0.5 恰在门控内 ⇒ APIC 必须生效且保住旋转：l_keep {}（< 0.95）",
        on.l_keep
    );
    assert!(
        on.rot_err <= 0.50,
        "θ = 0.5 恰在门控内 ⇒ APIC 必须生效且场更接近刚体：rot_err {}（> 0.50）",
        on.rot_err
    );
    assert!(
        on.rot_err < off.rot_err,
        "θ = 0.5 门控内开 APIC 的 rot_err {} 未优于关档 {} ⇒ APIC 档没真生效",
        on.rot_err,
        off.rot_err
    );
}

/// `#[ignore]` 诊断（不进 CI）：旋转团台的四档原始读数；结论见 `src/config.rs` 的
/// `flip_apic` doc 与 `src/flip.rs` 模块头 `## APIC` 节。
#[test]
#[ignore = "旋转团台四档原始读数（仪表，只打印）；结论见 src/flip.rs 的 APIC 节"]
fn diag_apic_rotation() {
    eprintln!("== 旋转团（30 tick，ω = 2 rad/s，零重力/无边界）==");
    eprintln!(
        "{:12} {:9} {:9} {:11} {:8}",
        "档位", "l_keep", "rot_err", "ke", "vmax"
    );
    for (n, ratio, apic) in [
        ("PIC", 0.0f32, false),
        ("APIC", 0.0, true),
        ("FLIP", 0.95, false),
        ("FLIP+APIC", 0.95, true),
    ] {
        let r = rot_case(ratio, apic, 30);
        eprintln!(
            "{n:12} {:9.4} {:9.4} {:11.3e} {:8.4}",
            r.l_keep, r.rot_err, r.ke, r.vmax
        );
    }
}

/// `#[ignore]` 诊断（不进 CI）：**APIC 的红利随 θ 怎么变**（P23 的前置表·红利侧）。
///
/// 代价侧那张表在 `flip_incompressibility_probe.rs::diag_apic_theta_scan`（静水槽 × θ）。
/// 本表出红利侧（旋转团 × θ）⇒ 两表合起来才能判「**θ 门控**」（APIC 只在 θ ≤ θ_max 生效）：
/// 若红利在 θ → 0 时单调变大、而代价要到 θ → 0.95 才出现 ⇒ 两段分得开，门控可用。
/// **只打印、不断言**（仪表）；`flip_apic` 默认关 ⇒ 本诊断**不改**默认档任何读数。
///
/// 跑法：`cargo test -p vxl-phys-fluid --test flip_rotation_probe -- --ignored --nocapture`
#[test]
#[ignore = "旋转团 × θ 扫描（仪表，只打印）；跑法见该测试的 doc"]
fn diag_apic_theta_scan() {
    eprintln!("== 旋转团（30 tick，ω = 2 rad/s，零重力/无边界）：APIC 开/关 × θ 扫描 ==");
    eprintln!(
        "{:8} {:6} {:9} {:9} {:11} {:8}",
        "θ", "apic", "l_keep", "rot_err", "ke", "vmax"
    );
    for ratio in [0.0f32, 0.25, 0.5, 0.8, 0.95] {
        for apic in [false, true] {
            let r = rot_case(ratio, apic, 30);
            eprintln!(
                "{ratio:8} {apic:6} {:9.4} {:9.4} {:11.3e} {:8.4}",
                r.l_keep, r.rot_err, r.ke, r.vmax
            );
        }
    }
}
