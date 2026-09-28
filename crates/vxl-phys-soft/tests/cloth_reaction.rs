//! **布片 × 动态刚体**（软体切片 2b-ii）：**反作用两腿** —— `body.dv`（速度口径，门面
//! `linvel += dv`）与 `body.dx`（位置口径补足，§8.4.9，门面 `position += dx`）。
//!
//! **口径与 `rope::apply_body_hit` 同款**（计划原文："动量守恒口径与 §8.4.20 同款，
//! **不再另发明一套**"），§8.4.20 的**三条使能条件**本片一条不缺：
//! ① **位置口径回填**（`BodyCoupling::dx`）；② **角反作用停用**（本片根本不引入该开关）；
//! ③ **摩擦的相对速度与法向同口径**（两支共用 `v_point = v_b + ω × r`）。
//!
//! **⭐ 本片对 rope 口径的唯一偏离（实测驱动，见 `body_contacts_dynamic` 的注）**：位置腿
//! **也进虚拟位姿 `disp`**。理由是"虚拟位姿要忠实模拟体在 tick 内的真实轨迹"，而位置腿是这条
//! 轨迹的一部分；不进去 ⇒ **同一份穿透被"接触数 × 子步数"重复计账**。
//! 实测（同一场景、只差这一行）：**无这一行 ⇒ 1.8 cm 穿透被放大成 12.2 cm 单 tick 上抛
//! ⇒ 盒在布上弹跳（稳态 y 波动 0.1486 m、接触集反复丢失、末速 −0.21）；有这一行 ⇒
//! 稳态 y 波动 0.0000、每 tick `dv.y` 恒为 `g·dt`、净漂移 ≈ 0（真正的静平衡）**。
//!
//! **判据（全部机器无关：无计时、无随机、无平台分支）**：
//! ① **接住**：动态盒落到（四周钉住的）布片上 ⇒ 长窗**停在布片上**、横向留在自身足迹内；
//!    对照：自由落体 1800 tick 该掉 **4415 m**。
//! ② **不打摆**（稳态 y 波动 < 1 cm）：这一条就是上面那一行修正的**金丝雀**——去掉它必红。
//! ③ **速度腿承重**：稳态窗 `Σ dv.y ≈ g·dt·N`（顶住的是**整份重量**，不是一部分）。
//! ④ **位置腿补漏**：稳态窗 `Σ dx.y > 0` 且净垂直漂移 ≈ 0（"只回速度不回位置"的漏被补上）。
//! ⑤ **金丝雀（关反作用）**：自扮引擎**不消费**反作用 ⇒ 判据 ① 必红。
//! ⑥ **泛化表**：质量 × 落差 × 子步 × 时长全部接住（防"只在一组参数上成立"的假修，§8.4.21/§8.4.26）。

use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_soft::{ClothSheet, RigidProxy, Stiffness};

const DT: f32 = 1.0 / 60.0;
const G: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// 布片：8×8 格、跨度 1.0 m ⇒ **网格间距 0.125 m**（盒半宽 0.2 ⇒ 底面上有 **3×3 = 9 个**粒子）。
const N: usize = 8;
const W: usize = N + 1;
/// **不打摆阈值**（稳态窗内 y 的极差）：静态平衡实测 **0.0000**；弹跳态实测 **0.1486**。
const STEADY_TOL: f32 = 0.01;

/// 平铺网格（`n×n` 格、跨度 `±size`、落在 **y = 0 平面**；行主序 = `z` 外层、`x` 内层）。
fn plate(n: usize, size: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut pts = Vec::new();
    let mut tris = Vec::new();
    for iz in 0..=n {
        for ix in 0..=n {
            let s = 2.0 * size / n as f32;
            pts.push(Vec3::new(-size + s * ix as f32, 0.0, -size + s * iz as f32));
        }
    }
    for iz in 0..n as u32 {
        for ix in 0..n as u32 {
            let a = iz * (n as u32 + 1) + ix;
            let (c, d) = (a + 1, a + n as u32 + 1);
            tris.push([a, d, c]);
            tris.push([c, d, d + 1]);
        }
    }
    (pts, tris)
}

/// **悬挂布片**：1.0 m 见方、**四周一圈钉住**（中间自由）⇒ 会自己垂下来（与 rope 的"两端钉住"同族）。
fn hung_sheet(substeps: u32) -> ClothSheet {
    let (pts, tris) = plate(N, 0.5);
    let mut s = ClothSheet::new(pts, tris, 1000.0, 0.01, Stiffness::Hard);
    s.substeps = substeps;
    s.damping = 0.999; // 与 `rigid_coupling_gap` 同口径：把"停住"做成稳态读数
    for iz in 0..W {
        for ix in 0..W {
            if iz == 0 || ix == 0 || iz == W - 1 || ix == W - 1 {
                s.set_pinned(iz * W + ix, true);
            }
        }
    }
    s
}

/// 一次跑完的读数（稳态窗 = **后一半** tick）。
struct Read {
    /// 60 tick（短窗）与末 tick 的盒 y。
    y_60: f32,
    y_end: f32,
    /// 横向位移的绝对值上界。
    x_max: f32,
    /// 稳态窗内 `Σ dv.y` / `Σ dx.y`（正 = 向上）。
    dv_y: f32,
    dx_y: f32,
    /// 稳态窗内的**盒 y 波动**（判"静平衡"还是"蹦床"）与窗首 y（判净漂移）。
    y_win_min: f32,
    y_win_max: f32,
    y_win_start: f32,
    /// 稳态窗内单 tick 的 `dv.y` / `dx.y` 极值（判"每 tick 都在传"还是"偶发"）。
    dv_lo: f32,
    dv_hi: f32,
    dx_lo: f32,
    dx_hi: f32,
    /// 窗口 tick 数（判据 ③④ 的 N）。
    win: usize,
    /// 布片垂稳后的中心高度（判据 ① 的参照）。
    y_rest: f32,
    finite: bool,
}

/// 场景参数（泛化表逐项换的就是这几个）。
#[derive(Clone, Copy)]
struct Cfg {
    mass: f32,
    /// 初始落差（相对**垂稳后**的布片中心）。
    drop: f32,
    substeps: u32,
    ticks: usize,
}

/// 自扮引擎跑一遍：`apply_reaction = false` ⇒ **金丝雀**（不消费两腿反作用）。
fn run(cfg: Cfg, apply_reaction: bool) -> Read {
    let mut s = hung_sheet(cfg.substeps);
    // 先让布片自己垂稳（600 tick）—— 读数取"垂稳后"的形态，别把初态的瞬时下坠混进来。
    for _ in 0..600 {
        s.step(DT, G, &NoProviders, 0, &[]);
    }
    let y_rest = s.pos[(N / 2) * W + N / 2].y;
    // 动态盒：底面 0.4×0.4（⇒ 布片底面上 9 个粒子在足迹内）。
    let shape = Shape::Box {
        half: Vec3::new(0.2, 0.05, 0.2),
    };
    let mut pos = Vec3::new(0.0, y_rest + cfg.drop, 0.0);
    let mut vel = Vec3::ZERO;
    let win = cfg.ticks / 2;
    let mut r = Read {
        y_60: 0.0,
        y_end: 0.0,
        x_max: 0.0,
        dv_y: 0.0,
        dx_y: 0.0,
        y_win_min: f32::MAX,
        y_win_max: f32::MIN,
        y_win_start: 0.0,
        dv_lo: f32::MAX,
        dv_hi: f32::MIN,
        dx_lo: f32::MAX,
        dx_hi: f32::MIN,
        win,
        y_rest,
        finite: true,
    };
    for t in 0..cfg.ticks {
        let proxy = RigidProxy {
            body: 0,
            shape,
            pos,
            rot: Quat::IDENTITY,
            linvel: vel,
            angvel: Vec3::ZERO, // 本判据场景体不转（ω = 0 ⇒ 转动项恒等）
            local_inv_inertia: Vec3::ZERO, // 角反作用停用（§8.4.20 条件②）
            inv_mass: 1.0 / cfg.mass,
        };
        s.step(DT, G, &NoProviders, 0, std::slice::from_ref(&proxy));
        vel += G * DT;
        pos += vel * DT;
        let (mut dvy, mut dxy) = (0.0f32, 0.0f32);
        if apply_reaction {
            // **两条腿**（与门面 `cloth_pass` 同口径、同段位：都在体解算之后施加）。
            if let Some(dv) = s.body.dv.first() {
                vel += *dv;
                dvy = dv.y;
            }
            if let Some(dx) = s.body.dx.first() {
                pos += *dx;
                dxy = dx.y;
            }
        }
        if t == 59 {
            r.y_60 = pos.y;
        }
        r.x_max = r.x_max.max(pos.x.abs());
        if t >= cfg.ticks - win {
            if t == cfg.ticks - win {
                r.y_win_start = pos.y;
            }
            r.dv_y += dvy;
            r.dx_y += dxy;
            r.y_win_min = r.y_win_min.min(pos.y);
            r.y_win_max = r.y_win_max.max(pos.y);
            r.dv_lo = r.dv_lo.min(dvy);
            r.dv_hi = r.dv_hi.max(dvy);
            r.dx_lo = r.dx_lo.min(dxy);
            r.dx_hi = r.dx_hi.max(dxy);
        }
    }
    r.y_end = pos.y;
    r.finite = s.pos.iter().all(|p| p.is_finite()) && pos.is_finite() && vel.is_finite();
    r
}

/// **接住** = 末 y 仍在布片附近 + 横向留在足迹内 + **稳态不打摆**（判据 ①②③）。
fn held(r: &Read) -> bool {
    r.finite
        && r.y_end > r.y_rest - 0.3
        && r.x_max < 0.3
        && (r.y_win_max - r.y_win_min) < STEADY_TOL
}

/// 打印一格的读数（探针格式保持稳定，便于跨提交对读）。
fn report(tag: &str, r: &Read) {
    println!(
        "  {tag:<34} 末 y={:+.4}（布片垂稳 {:+.4}）| 短窗 y={:+.4} | |x|max={:.4}\n\
         {:<37}稳态窗：y 波动 {:.5} | 净漂移 {:+.5} | Σdv.y={:+.2}（解析 {:+.2}）\
         | Σdx.y={:+.3} | 单 tick dv.y∈[{:+.5},{:+.5}] dx.y∈[{:+.5},{:+.5}] ⇒ {}",
        r.y_end,
        r.y_rest,
        r.y_60,
        r.x_max,
        "",
        r.y_win_max - r.y_win_min,
        r.y_end - r.y_win_start,
        r.dv_y,
        9.81 * DT * r.win as f32,
        r.dx_y,
        r.dv_lo,
        r.dv_hi,
        r.dx_lo,
        r.dx_hi,
        if held(r) {
            "托住 ✅"
        } else {
            "**未托住 ❌**"
        }
    );
}

#[test]
fn dynamic_box_is_held_by_pinned_cloth() {
    let cfg = Cfg {
        mass: 1.0,
        drop: 0.3,
        substeps: 8,
        ticks: 1800,
    };
    let r = run(cfg, true);
    let free_fall = 0.5 * 9.81 * (cfg.ticks as f32 * DT).powi(2);
    println!(
        "[判据①接住] 自由落体对照：同样 {} tick 该掉 {:.0} m（实测掉 {:.4} m）",
        cfg.ticks,
        free_fall,
        r.y_rest - r.y_end
    );
    report("基准", &r);
    assert!(r.finite, "出现非有限值（NaN/inf）");
    assert!(
        r.y_end > r.y_rest - 0.3,
        "长窗该**停在布片上**（实测 y={:+.4} vs 布片垂稳 {:+.4}）——红了说明两腿有腿被改坏\
         （位置口径回填 §8.4.9 / 摩擦相对速度含 dv §8.4.19）",
        r.y_end,
        r.y_rest
    );
    assert!(
        r.x_max < 0.3,
        "横向该**留在自身足迹内**（半宽 0.2；实测 |x|max={:.4}）——红了说明有横向棘轮（§8.4.25）",
        r.x_max
    );
    // **判据②：不打摆**（= 位置腿进虚拟位姿那一行的金丝雀；去掉那一行实测波动 0.1486 m）。
    assert!(
        r.y_win_max - r.y_win_min < STEADY_TOL,
        "稳态窗内 y 极差 {:.5} ≥ {STEADY_TOL} ⇒ 盒在**布上弹跳**而不是停住\
         （回看 `body_contacts_dynamic`：位置腿必须在 `disp` 里，否则同一份穿透被\
         接触数 × 子步数重复计账、单 tick 上抛十几厘米）",
        r.y_win_max - r.y_win_min
    );
    // **判据③：速度腿承重**（静平衡下每 tick 的 `dv.y` 恰为 `g·dt` ⇒ 顶住的是**整份重量**）。
    let dv_want = 9.81 * DT * r.win as f32;
    assert!(
        (r.dv_y / dv_want - 1.0).abs() < 0.10,
        "稳态窗 Σ dv.y = {:+.2} 该 ≈ 解析 {dv_want:.2}（±10%）——红了说明布片只顶住了部分重量\
         （或弹跳态下「平均恰好」掩盖了偶发冲击）",
        r.dv_y
    );
    // **判据④：位置腿补漏**（`dx` 正向上、每 tick 都在传、且净漂移被压平）。
    assert!(
        r.dx_lo > 0.0,
        "稳态窗内每一 tick 的 dx.y 都该 > 0（位置腿在补「只回速度不回位置」的漏；实测最小 {:.6}）",
        r.dx_lo
    );
    assert!(
        (r.y_end - r.y_win_start).abs() < STEADY_TOL,
        "稳态窗内净垂直漂移 {:+.5} 该 ≈ 0 —— 漂移不为零就是「位置漏」又回来了（§8.4.9）",
        r.y_end - r.y_win_start
    );
}

/// **判据⑤ 金丝雀**：同一场景、**不消费两腿反作用** ⇒ 盒必须掉穿（判据 ① 必红）。
/// 没有这条，"接住"完全可能是别的路径凑出来的。
#[test]
fn canary_without_reaction_the_box_falls_through() {
    let cfg = Cfg {
        mass: 1.0,
        drop: 0.3,
        substeps: 8,
        ticks: 1800,
    };
    let r = run(cfg, false);
    println!("[金丝雀⑤] 不消费反作用：");
    report("金丝雀（无反作用）", &r);
    assert!(
        !held(&r),
        "金丝雀本该掉穿（实测末 y={:+.4}）——仍然被接住 ⇒ 说明「接住」不是两腿反作用带来的，\
         判据没有分辨力",
        r.y_end
    );
}

/// **判据⑥ 泛化表**：质量 × 落差 × 子步 × 时长 —— 每一格都必须接住。
/// （§8.4.21 的教训：只看一组参数会漏掉"换盆"；§8.4.26 把那张表升成了在树判据。）
#[test]
fn dynamic_box_is_held_across_parameters() {
    let base = Cfg {
        mass: 1.0,
        drop: 0.3,
        substeps: 8,
        ticks: 1800,
    };
    let cases: Vec<(&str, Cfg)> = vec![
        ("基准 1kg / 落差0.3 / 8子步 / 1800", base),
        ("质量 0.5 kg", Cfg { mass: 0.5, ..base }),
        ("质量 2 kg", Cfg { mass: 2.0, ..base }),
        ("落差 0.6 m", Cfg { drop: 0.6, ..base }),
        (
            "子步 16",
            Cfg {
                substeps: 16,
                ..base
            },
        ),
        (
            "时长 3600 tick",
            Cfg {
                ticks: 3600,
                ..base
            },
        ),
    ];
    let mut bad = Vec::new();
    println!("[判据⑥泛化表]");
    for (name, cfg) in cases {
        let r = run(cfg, true);
        report(name, &r);
        if !held(&r) {
            bad.push(format!(
                "{name}（末 y={:+.4} vs 布片垂稳 {:+.4}，波动 {:.5}，|x|max={:.4}）",
                r.y_end,
                r.y_rest,
                r.y_win_max - r.y_win_min,
                r.x_max
            ));
        }
    }
    assert!(bad.is_empty(), "泛化表有格未托住：{}", bad.join("；"));
}
