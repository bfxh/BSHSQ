//! **FLIP 传递耗散判据**（旋转团 + APIC A/B）：立方水团绕竖直轴刚体旋转，零重力/零外力/
//! 无边界 ⇒ 末态绕 y 角动量 `L_y` 的损失**只可能来自 P2G/G2P 那一趟三线性传输**。
//!
//! 主判据 = `flip_apic_transfer_does_not_buy_rotation`（**负结果**：tick 级 `C` 拿不到 APIC
//! 红利 ⇒ `flip_apic` 默认 `false`）。
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

/// **APIC A/B 判据（实测的负结果，逐条钉住）**：**tick 级**的 `C` **拿不到 APIC 的红利**。
///
/// 台架见 `rot_case`（零重力/无外力/无边界 ⇒ 末态 `L_y` 只被 P2G/G2P 那一趟三线性传输耗散，
/// 与压力投影、体外力都无关）。四条判据把"负结果"钉成**可回归的断言**，防止以后有人凭印象
/// 把 `flip_apic` 当成旋转保真的解：
/// ① **台架有判别力**：`θ = 0.95`（FLIP）保住旋转，比 PIC 高一个量级 ⇒ 这个台架真的看得见
///    传输耗散，②③ 才不是空断言；
/// ② **负结果**：同 `θ = 0`、只换 `flip_apic`，APIC **没有**显著改善（实测 0.145 → 0.155）。
///    这正是"`C` 每 tick 清零 ⇒ 首子步就是纯 PIC；而 PIC 的 `v ← v_网格` 重置把粒子场在
///    同格 8 节点上**拉平**（等权时一阶矩 `Σ w·v⊗d` 抵消）⇒ 之后从网格估出的 `C` 是**磨过
///    一次**的梯度"这条推导的读数。**真 APIC 要求 `C` 活过子步（进粒子状态）** ⇒ 下一刀；
///    若哪天 ② 真的红了（APIC 买到了红利），先回来改 `flip_apic` 的文档与这条结论，再谈别的。
/// ③ **不搅乱**：`θ = 0.95` 开 APIC 不得把角动量搞坏（实测 0.866 → 0.871）；
/// ④ **A/B 档也逐位复现**：同档跑两遍读数逐位相同（含 `digest`）。
/// 读数由本测试自己打印（`--nocapture`），同见 `diag_apic_rotation`。
#[test]
fn flip_apic_transfer_does_not_buy_rotation() {
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
        "旋转团台架看不见传输耗散：flip {} vs pic {} ⇒ 下面两条无从谈起",
        flip.l_keep,
        pic.l_keep
    );
    // ② 负结果：tick 级 C 在 θ = 0 下与 PIC ≈ 无差别（实测比值 1.06；放到 1.35 只为容忍
    //    浮点/编译器差异——真红利是 ① 里那个量级差）。
    assert!(
        apic.l_keep < 1.35 * pic.l_keep,
        "tick 级 C 竟然买到了旋转红利（apic {} vs pic {}）⇒ 与 `flip_apic` 字段 doc 的结论矛盾，\
         先回头核文档与实现，再改这条断言",
        apic.l_keep,
        pic.l_keep
    );
    // ③ 不搅乱：FLIP 档开 APIC 不得把角动量搞坏 > 3%（实测反升 0.6%）。
    assert!(
        flip_apic.l_keep >= 0.97 * flip.l_keep,
        "APIC 把 FLIP 档的角动量搞坏了：{} vs {}",
        flip_apic.l_keep,
        flip.l_keep
    );
    // ④ 表面完整性（阈值同主表）+ A/B 档逐位复现。
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

/// `#[ignore]` 诊断（不进 CI）：旋转团台的四档原始读数；结论见 `src/config.rs` 的
/// `flip_apic` doc 与 `src/flip.rs` 模块头 `## APIC` 节。
#[test]
#[ignore]
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
