//! misc：R8/R9/R10（睡眠位置代价 / 破坏触发步精度 / 堆叠层间载荷）——从 `main.rs` 按域拆出
//! （**纯搬移，输出逐字不变**）。拆的理由：`probe.rs` 已顶到 god 门 file_lines 阈值 800。
use super::{pct_sorted, G};
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};

/// R10 —— **多体堆叠的层级载荷方差**（评审 §1.5 标 MISSING 的那条）。
///
/// 仓库有 R3（**单**接触的静置力连续性），但没有**多层堆叠**的。逐点冲量不暴露，但可以
/// 绕开：**第 `j` 层边界传递的载荷 = 其上方所有盒子的净接触力之和**（逐层望远镜求和），
/// 而逐盒净接触力能用速度差分测出来（R3 同招）：`F_i = m·Δv_i/dt + m·g`。
/// ⇒ 每层边界的 `mean/期望` 应 ≈1、`std/期望` 应很小；两者都是堆叠质量的可回归判据。
pub(super) fn stack_load_steadiness(layers: usize) {
    let early = stack_load_window(layers, 120, 480);
    let settled = stack_load_window(layers, 600, 480);
    println!(
        "{{\"probe\":\"R10_stack_load_steadiness\",\"layers\":{layers},\
         \"std_over_load_pct_early_2s\":{:.4},\"std_over_load_pct_settled_10s\":{:.4},\
         \"mean_err_pct_settled\":{:.4}}}",
        early.0, settled.0, settled.1,
    );
}

/// 一个测量窗口：返回 `(最差层间载荷 std / 该层载荷, 最差层间载荷均值误差)`，单位 %。
fn stack_load_window(layers: usize, warm: usize, meas: usize) -> (f64, f64) {
    let cfg = PhysConfig {
        sleep_linear: 0.0,
        sleep_angular: 0.0,
        ..PhysConfig::default()
    };
    let dt = cfg.dt as f64;
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(3.0, 0.5, 3.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    let mut boxes = Vec::with_capacity(layers);
    for k in 0..layers {
        boxes.push(w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.0, 0.5 + k as f32 * 1.001, 0.0),
            Quat::IDENTITY,
            1000.0,
        ) as usize);
    }
    let m = 1.0 / w.bodies.inv_mass[boxes[0]] as f64;
    for _ in 0..warm {
        w.step();
    }
    let mut prev: Vec<f64> = boxes.iter().map(|&i| w.bodies.linvel[i].y as f64).collect();
    // 每层边界一个样本序列（边界 j = 盒子 j 与 j+1 之间；j = layers-1 是最上面那层之上的"空载"）。
    let mut series: Vec<Vec<f64>> = vec![Vec::with_capacity(meas); layers];
    for _ in 0..meas {
        w.step();
        // 逐盒净接触力（向上为正），再按"上方所有盒子求和"得到各层边界的载荷。
        let mut acc = 0.0f64;
        let mut per_layer = vec![0.0f64; layers];
        for (k, &i) in boxes.iter().enumerate().rev() {
            let vy = w.bodies.linvel[i].y as f64;
            let f = m * (vy - prev[k]) / dt + m * G as f64;
            prev[k] = vy;
            acc += f; // 上方盒子求和 ⇒ 边界 k 的载荷
            per_layer[k] = acc;
        }
        for k in 0..layers {
            series[k].push(per_layer[k]);
        }
    }
    // 每层边界的统计：边界 `k` = 盒 `k` **下方**那个接触，承载 `k..top` 共 `layers-k` 个盒
    // 的重力 ⇒ 期望 = `(layers - k)·m·g`（`per_layer[k]` 正是那个望远镜求和）。
    let mut worst = 0.0f64;
    let mut worst_mean = 0.0f64;
    for k in 0..layers {
        let expect = m * G as f64 * (layers - k) as f64;
        let v = &series[k];
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        let sd = (v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / v.len() as f64).sqrt();
        worst = worst.max(sd / expect);
        worst_mean = worst_mean.max((mean / expect - 1.0).abs());
    }
    (100.0 * worst, 100.0 * worst_mean)
}

/// R9 —— **破坏触发的步精度**：解析预测的撞击 tick 必须与实际触发 tick 一致。
///
/// 仓里已有"碎块数 / 末态哈希可复现"的判据（`tests/destruction_tiered.rs`），但没有
/// **触发时刻**的判据——而触发时刻正是历史 bug 出没过的地方：`record_impacts` 曾经读
/// "解算后速度"，子步解算已经把冲击吃掉了 ⇒ 炮弹打上去不挖洞。本探针把它钉成数字。
///
/// 口径：关重力 ⇒ 弹道是直线；弹体从 `GAP` 处以 `V` 沿 +x 飞向体素墙。窄相的**投机
/// 皮肤带**（`contact_skin`）先于几何接触生成流形 ⇒ 解析触发时刻 = `(GAP − skin)/V`，
/// 换算成 tick 再上取整。**判据 = 实际触发 tick 与解析值相差 ≤1**（时间离散 + 记录
/// 相位允许一 tick）。另一条是反向的精度判据：**亚阈冲击一次都不许触发**。
pub(super) fn destruction_trigger_tick(ticks: usize) {
    const V: f32 = 12.0;
    const GAP: f32 = 0.8;
    /// 亚阈档的阈值：远高于 `V` ⇒ 不该有任何触发。
    const SUB_THRESHOLD: f32 = 5.0 * V;
    /// 触发一记毁伤所需的接近速度阈值（现实值；`12 m/s` 撞击显然该过）。
    const THRESHOLD: f32 = 5.0;
    let build = |substeps: u32| -> (World, u32, u32) {
        let cfg = PhysConfig {
            gravity: Vec3::ZERO,
            substeps,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        // 一块厚 2.4 m 的体素块（x ∈ [0, 2.4)）——**必须够厚**：弹坑球半径最大 0.9 m、
        // 球心落在接触点内侧 1.05r，薄墙（0.3 m）会让整颗球落在材料外、挖不出碎块。
        let mut vol =
            vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(0.0, 0.0, -1.0), 0.1, 24, 20, 20);
        vol.fill_box(Vec3::new(0.0, 0.0, -1.0), Vec3::new(2.4, 2.0, 1.0));
        let vid = w.add_voxel(vol);
        let bullet = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            Vec3::new(-0.2 - GAP, 1.0, 0.0),
            Quat::IDENTITY,
            2000.0,
        );
        w.bodies.linvel[bullet as usize] = Vec3::new(V, 0.0, 0.0);
        (w, vid, bullet)
    };
    let cfg0 = PhysConfig::default();
    let dt_sub = cfg0.dt as f64 / cfg0.substeps.max(1) as f64;
    let skin = cfg0.contact_skin as f64;
    // **顺序感知**的解析预测：窄相用的是**上一子步末**的位置 ⇒ 首个"看见接触"的子步是
    // `floor((GAP−skin)/(V·dt_sub)) + 2`（+1 是那一拍的位移、+1 是索引），再换成 tick。
    let substep_hit = ((GAP as f64 - skin) / (V as f64 * dt_sub)).ceil() + 1.0;
    let analytic = (substep_hit / cfg0.substeps.max(1) as f64).ceil() as usize;
    let run = |substeps: u32, thresh: f32| -> (usize, usize) {
        let (mut w, vid, _) = build(substeps);
        let mut first = 0usize;
        let mut total = 0usize;
        for t in 1..=ticks {
            w.step();
            let n = w.apply_impact_destruction(vid, thresh, 1000.0);
            if n > 0 && first == 0 {
                first = t;
            }
            total += n;
        }
        (first, total)
    };
    let (trig, pieces) = run(cfg0.substeps, THRESHOLD);
    let (trig_s1, pieces_s1) = run(1, THRESHOLD);
    let (_, sub_pieces) = run(cfg0.substeps, SUB_THRESHOLD);
    println!(
        "{{\"probe\":\"R9_destruction_trigger_tick\",\"ticks\":{ticks},\"analytic_tick\":{analytic},\
         \"trigger_tick\":{trig},\"trigger_tick_substeps1\":{trig_s1},\
         \"debris_pieces\":{pieces},\"debris_pieces_substeps1\":{pieces_s1},\
         \"sub_threshold_pieces\":{sub_pieces}}}",
    );
}

/// R8 —— **睡眠的位置代价**：睡着不能把体冻在错位（外部评审「方向 14」的真实性半边）。
///
/// 仓库此前只测**入睡率/唤醒率**，不测"睡眠把体冻在哪儿"。本探针两档**同场景同输入**：
/// A = 默认（睡眠开）、B = 关睡眠（`sleep_time = +∞`），比末态**逐体位置**（L2）与**姿态**
/// （相对转角）。A 档必须**真的睡着**（否则本探针无意义 ⇒ 判据红，不假绿）。
/// 机器无关（位置/姿态是确定函数）。
pub(super) fn sleep_position_error(ticks: usize) {
    const LAYERS: u32 = 2;
    const SIDE: u32 = 4;
    let build = |sleep: bool| -> (World, Vec<u32>) {
        let cfg = PhysConfig {
            sleep_time: if sleep { 0.5 } else { f32::INFINITY },
            threads: 1,
            ..PhysConfig::default()
        };
        let mut w = World::new(cfg);
        w.add_static(
            Shape::Box {
                half: Vec3::new(6.0, 0.5, 6.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            Quat::IDENTITY,
        );
        let mut dyn_ids = Vec::new();
        // SIDE×SIDE 基础 × LAYERS 层；层高 1.001 起堆（微隙 ⇒ 初始不嵌合）。
        for layer in 0..LAYERS {
            for gy in 0..SIDE {
                for gx in 0..SIDE {
                    let x = gx as f32 - (SIDE as f32 - 1.0) * 0.5;
                    let z = gy as f32 - (SIDE as f32 - 1.0) * 0.5;
                    let y = 0.5 + layer as f32 * 1.001;
                    dyn_ids.push(w.add_dynamic(
                        Shape::Box {
                            half: Vec3::splat(0.5),
                        },
                        Vec3::new(x, y, z),
                        Quat::IDENTITY,
                        1000.0,
                    ));
                }
            }
        }
        (w, dyn_ids)
    };
    let (mut a, ids) = build(true);
    let (mut b, _) = build(false);
    for _ in 0..ticks {
        a.step();
        b.step();
    }
    let n_dyn = ids.len();
    let mut devs: Vec<f32> = Vec::with_capacity(n_dyn);
    let mut angs: Vec<f32> = Vec::with_capacity(n_dyn);
    for i in 0..n_dyn {
        let (ai, bi) = (ids[i] as usize, ids[i] as usize);
        devs.push((a.bodies.position[ai] - b.bodies.position[bi]).length());
        let mut q = a.bodies.rot(ai) * b.bodies.rot(bi).conjugate();
        if q.w < 0.0 {
            q = Quat::new(-q.x, -q.y, -q.z, -q.w);
        }
        let v = Vec3::new(q.x, q.y, q.z);
        angs.push(2.0 * v.length().atan2(q.w).to_degrees());
    }
    devs.sort_by(f32::total_cmp);
    angs.sort_by(f32::total_cmp);
    let slept = ids
        .iter()
        .filter(|&&i| !a.bodies.awake[i as usize])
        .count();
    println!(
        "{{\"probe\":\"R8_sleep_position_error\",\"ticks\":{ticks},\"dynamic\":{n_dyn},\
         \"slept_a\":{slept},\"pos_dev_max_m\":{:.5},\"pos_dev_p99_m\":{:.5},\
         \"ang_dev_max_deg\":{:.5}}}",
        devs[n_dyn - 1],
        pct_sorted(&devs, 99.0),
        angs[n_dyn - 1],
    );
}
