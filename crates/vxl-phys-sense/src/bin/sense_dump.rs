//! `sense_dump`：把示例场景跑出**感官 cue 流**（JSONL 到 stdout，统计到 stderr）。
//!
//! 工具链的第一件（`docs/ROUTE.md` §3.2）：音效师拿它对着场景调表，CI 拿它取基线
//! （`scripts/gate_sense.sh` 跑两遍逐字对拍 + 预算上界核对）。
//!
//! 用法：`sense_dump [ticks=300] [bodies=64] [budget=16] [mode=impact|slide|bank|haptic] [table=<路径>]`
//!
//! - `impact`（默认，**S1**）：体素地板 + 一网格盒雨，落点/初高错开 ⇒ 冲击成串、能同时触发
//!   预算截断与"轻碰不算响"两条规则。**地板必须是 provider**：冲击记录只对 `Shape::Provider`
//!   接触产生（见 `World::record_impacts`）——用静态盒当地板会得到空流（踩过）。
//! - `slide`（**S2**）：平滑地板 + **贴地盒、全带 +x 初速** ⇒ 落地即滑、摩擦减速到停；同一场
//!   既能验"滑时有 cue"，又能验"停下后静置不产循环音"。
//! - `bank`（**S3 例场**）：体素地板 + 盒雨，**逐体挂 3 种材质**（槽 1/2/3）⇒ 一条流里同时出现
//!   多个材质槽，用来考"表按材质分派音色组 id"。**必须给表**（不给 ⇒ 退出码 3）。
//! - `table`（可选，**S3**）：给 cue 流挂上**音色组 id**（行尾多一个 `"bank"` 字段），表由
//!   `<材质槽> -> <音色组 id>` 的外部文本给（示例见 `assets/sound_table.txt`）。**缺项显式拒绝**：
//!   表里没有的材质槽 ⇒ stderr 报错 + **退出码 2**（不回落默认音）；表读不到 ⇒ 退出码 3。
//!   不给 `table` 时 `impact`/`slide` 的输出与原样**逐字节相同**（两条基线不受影响）。
//! - `haptic`（**S4 震动设备输出**）：场景 A（体素盒雨 ⇒ 冲击）与场景 B（平滑地板滑移 ⇒ 持续
//!   接触）**并行**喂**同一个**驱动器 ⇒ stdout 是**设备帧流**（`tick`/`frame`/`intensity`，
//!   上钳在 `[0,1]`）。驱动量取两个源的 `max`（不叠加）；`update_hz` 取档位默认值，每 tick
//!   恰好 `round(update_hz·dt)` 帧 ⇒ **延迟 0 帧**（本 tick 的 cue 在本 tick 首帧就生效）。
//!   stderr 末行给 `frames_per_tick` 与 `last_cue_tick`：前者判"更新率口径"、后者划出**纯释放段**
//!   （门在那儿判包络单调与"释放到恰好 0.0"）。
use vxl_phys::{BodyBuildExt, Material, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{
    haptic_jsonl, jsonl, jsonl_banked, map_banks, slide_jsonl, slide_jsonl_banked, Cue,
    HapticConfig, HapticDriver, HapticFrame, MaterialSlot, SenseConfig, SenseExt, SlideCue,
    SoundTable,
};

fn arg(i: usize, dflt: &str) -> String {
    std::env::args().nth(i).unwrap_or_else(|| dflt.to_string())
}

fn num<T: std::str::FromStr>(i: usize, dflt: &str, fallback: T) -> T {
    arg(i, dflt).parse().ok().unwrap_or(fallback)
}

/// 载入音色表（S3）。读不到 ⇒ 退出码 3（无法判定）；解析失败 ⇒ 退出码 2（**显式拒绝**）。
fn load_table(path: &str) -> SoundTable {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("❌ sense_dump：音色表读不到 `{path}`：{e}");
            std::process::exit(3);
        }
    };
    match SoundTable::parse(&text) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("❌ sense_dump：{e}（表 `{path}`）");
            std::process::exit(2);
        }
    }
}

/// 把一条 cue 流映射成音色组 id 流；**任一 cue 缺项 ⇒ 报错 + 退出码 2**（S3 判据）。
fn banks_of<T: MaterialSlot>(table: &SoundTable, cues: &[T]) -> Vec<vxl_phys_sense::BankId> {
    match map_banks(table, cues) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("❌ sense_dump：{e}");
            std::process::exit(2);
        }
    }
}

/// S3 的 stderr 第二行（只在给了 `table` 时打 ⇒ S1/S2 的统计行不受影响）。
fn table_report(table: &SoundTable, banks: &[vxl_phys_sense::BankId]) -> String {
    let mut uniq: Vec<u32> = banks.iter().map(|b| b.0).collect();
    uniq.sort_unstable();
    uniq.dedup();
    format!(
        "{{\"table_entries\":{},\"mapped\":{},\"banks_used\":{},\"banks\":{uniq:?}}}",
        table.len(),
        banks.len(),
        uniq.len()
    )
}

/// 体素地板（provider）。**S1 冲击**的地板**必须**是 provider：冲击记录只对 `Shape::Provider`
/// 接触产生（见 `World::record_impacts`）——用静态盒当地板会得到空流（踩过）。
fn floor_voxel() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, -0.5, -4.0), 0.5, 16, 3, 16);
    vol.fill_box(Vec3::new(-4.0, -0.5, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    w
}

/// 平滑静态盒地板（top `y = 1.0`）。**S2 持续接触**用 `World::manifolds()`——它吃**任意**接触
/// （不要求 provider），且体素地板的棱会让盒子**翻滚**（实测 `ωz` 冲到 ±8 rad/s）、切向速度信号
/// 被搅乱；平滑地板给的是干净、稳态的滑移，正是"持续接触"要喂给音效师的量。
fn floor_smooth() -> World {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(4.0, 0.5, 4.0),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
    );
    w
}

/// 场景 A（S1 冲击）：体素地板 + 盒雨，`y ∈ [2.0, 3.0]` 错开初高 ⇒ 冲击成串。
fn scene_impact(bodies: usize) -> World {
    let mut w = floor_voxel();
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(
            -1.75 + 0.5 * ix,
            2.0 + 0.25 * (i % 5) as f32,
            -1.75 + 0.5 * iz,
        );
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

/// 场景 B（S2 持续接触）：**平滑地板** + 贴地盒（`y = 1.22`，落 0.02 触地）+ 全带 `+x 3 m/s`
/// ⇒ 落定即滑、μ=0.5 约 37 tick 减速到停（盒子 0.4³ 不翻：翻倒要 `a > 9.8 m/s²`）。
fn scene_slide(bodies: usize) -> World {
    let mut w = floor_smooth();
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(-1.75 + 0.5 * ix, 1.22, -1.75 + 0.5 * iz);
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
        w.bodies.linvel[id as usize] = Vec3::new(3.0, 0.0, 0.0);
    }
    w
}

/// 场景 C（**S3 例场**）：**多材质**冲击 —— 体素地板 + 盒雨，逐体轮流挂 3 种材质（槽 1/2/3）。
///
/// 为什么单开这个场景：S1/S2 的盒雨没显式挂材质 ⇒ `mat` 全是默认槽 0，拿它演示"表按材质分派
/// 音色组 id"**判不出对错**（一行表查谁都对）。这里让三种槽**同时**出现在一条流里 ⇒ "查错行 /
/// 永远返回第一条"这类实现会被工具链判据当场抓住。材质参数用默认值（本场景只考**映射标签**，
/// 不考摩擦差异）。
fn scene_banks(bodies: usize) -> World {
    let mut w = floor_voxel();
    let mats = [
        w.add_material(Material::default()),
        w.add_material(Material::default()),
        w.add_material(Material::default()),
    ];
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(
            -1.75 + 0.5 * ix,
            2.0 + 0.25 * (i % 5) as f32,
            -1.75 + 0.5 * iz,
        );
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
        w.bodies.set_material(id as usize, mats[i % mats.len()]);
    }
    w
}

/// 冲击流的公共循环（**场景 A 与场景 C 共用**）：`scene` 由调用方给。
fn dump_impact(
    scene: World,
    ticks: usize,
    bodies: usize,
    budget: usize,
    table: Option<&SoundTable>,
) {
    let cfg = SenseConfig {
        max_cues_per_tick: budget,
        ..SenseConfig::default()
    };
    let mut w = scene;
    let mut cues: Vec<Cue> = Vec::new();
    let (mut rec, mut emi, mut dr_budget, mut dr_weak, mut peak) = (0usize, 0, 0, 0, 0);
    for _ in 0..ticks {
        w.step();
        let before = cues.len();
        let st = w.sense_cues(&cfg, &mut cues);
        rec += st.records;
        emi += st.emitted;
        dr_budget += st.dropped_budget;
        dr_weak += st.dropped_weak;
        peak = peak.max(cues.len() - before);
    }
    match table {
        None => print!("{}", jsonl(&cues)),
        Some(t) => {
            let banks = banks_of(t, &cues);
            print!("{}", jsonl_banked(&cues, &banks));
            eprintln!("{}", table_report(t, &banks));
        }
    }
    eprintln!(
        "{{\"ticks\":{ticks},\"bodies\":{bodies},\"budget\":{budget},\"impacts\":{rec},\
         \"cues\":{emi},\"dropped_weak\":{dr_weak},\"dropped_budget\":{dr_budget},\
         \"peak_cues_per_tick\":{peak}}}"
    );
}

/// S1：场景 A（单材质冲击）→ cue 流。
fn run_impact(ticks: usize, bodies: usize, budget: usize, table: Option<&SoundTable>) {
    dump_impact(scene_impact(bodies), ticks, bodies, budget, table);
}

/// S3 例场：场景 C（**多材质**冲击）→ 带音色组 id 的 cue 流。**必须**给表（否则演示无意义）。
fn run_bank(ticks: usize, bodies: usize, budget: usize, table: &SoundTable) {
    dump_impact(scene_banks(bodies), ticks, bodies, budget, Some(table));
}

/// S2：持续接触（滑）→ cue 流。键名与 S1 区分（`contacts`/`slides`）以免基线口径混淆。
fn run_slide(ticks: usize, bodies: usize, budget: usize, table: Option<&SoundTable>) {
    let cfg = SenseConfig {
        max_slides_per_tick: budget,
        ..SenseConfig::default()
    };
    let mut w = scene_slide(bodies);
    let mut slides: Vec<SlideCue> = Vec::new();
    let (mut rec, mut emi, mut dr_budget, mut dr_weak, mut peak) = (0usize, 0, 0, 0, 0);
    for _ in 0..ticks {
        w.step();
        let before = slides.len();
        let st = w.slide_cues(&cfg, &mut slides);
        rec += st.records;
        emi += st.emitted;
        dr_budget += st.dropped_budget;
        dr_weak += st.dropped_weak;
        peak = peak.max(slides.len() - before);
    }
    match table {
        None => print!("{}", slide_jsonl(&slides)),
        Some(t) => {
            let banks = banks_of(t, &slides);
            print!("{}", slide_jsonl_banked(&slides, &banks));
            eprintln!("{}", table_report(t, &banks));
        }
    }
    eprintln!(
        "{{\"ticks\":{ticks},\"bodies\":{bodies},\"budget\":{budget},\"contacts\":{rec},\
         \"slides\":{emi},\"dropped_weak\":{dr_weak},\"dropped_budget\":{dr_budget},\
         \"peak_slides_per_tick\":{peak}}}"
    );
}

/// **S4：震动设备输出**（包络帧流）—— 两个场景**并行**喂**同一个**驱动器：场景 A（体素盒雨）
/// 给"那一下"（S1 冲击），场景 B（平滑地板滑移）给持续背景（S2 滑移）。
///
/// 为什么两条流合成一条：震动器件只有**一路**电平——"落地砰一下"与"贴着墙滑"在同一路上叠加。
/// 驱动量取 `max`（取最强的那一下；**不与** S2 求和）⇒ 与 cue 顺序无关、与预算无关，CLI 侧要
/// 把这条口径跑成**逐字可对拍**的基线（`scripts/gate_sense.sh` 的 `check_haptic`）。
///
/// stderr 末行是单行 JSON：`frames_per_tick` 供门判"更新率口径"，`last_cue_tick` 划出**纯释放段**
/// （门在那儿判包络逐帧非增 + 末帧恰 0.0），`clamped` / `peak` 判"上钳真的发生过"（非空洞）。
fn run_haptic(ticks: usize, bodies: usize, budget: usize) {
    let cfg = SenseConfig {
        max_cues_per_tick: budget,
        max_slides_per_tick: budget,
        ..SenseConfig::default()
    };
    let hp = HapticConfig::default();
    let dt = PhysConfig::default().dt;
    let mut wa = scene_impact(bodies);
    let mut wb = scene_slide(bodies);
    let mut driver = HapticDriver::new(hp, dt);

    let mut cues: Vec<Cue> = Vec::new();
    let mut slides: Vec<SlideCue> = Vec::new();
    let mut frames: Vec<HapticFrame> = Vec::new();
    let (mut ic, mut sc, mut last) = (0usize, 0usize, 0u64);
    let (mut clamped, mut silent, mut peak) = (0usize, 0usize, 0.0_f32);
    for t in 0..ticks {
        wa.step();
        wb.step();
        cues.clear();
        slides.clear();
        ic += wa.sense_cues(&cfg, &mut cues).emitted;
        sc += wb.slide_cues(&cfg, &mut slides).emitted;
        if !cues.is_empty() || !slides.is_empty() {
            last = t as u64;
        }
        let st = driver.step(t as u64, &cues, &slides, &mut frames);
        clamped += st.clamped;
        silent += st.silent;
        peak = peak.max(st.peak);
    }
    print!("{}", haptic_jsonl(&frames));
    eprintln!(
        "{{\"ticks\":{ticks},\"bodies\":{bodies},\"budget\":{budget},\"tick_dt\":{dt},\
         \"update_hz\":{},\"frames_per_tick\":{},\"impact_cues\":{ic},\"slide_cues\":{sc},\
         \"frames\":{},\"clamped\":{clamped},\"silent\":{silent},\"last_cue_tick\":{last},\
         \"peak\":{peak}}}",
        hp.update_hz,
        driver.frames_per_tick(),
        frames.len()
    );
}

fn main() {
    let ticks: usize = num(1, "300", 300);
    let bodies: usize = num(2, "64", 64);
    let budget: usize = num(3, "16", 16);
    let table = std::env::args().nth(5).map(|p| load_table(&p));
    match arg(4, "impact").as_str() {
        "slide" => run_slide(ticks, bodies, budget, table.as_ref()),
        "haptic" => run_haptic(ticks, bodies, budget),
        "bank" => {
            match table.as_ref() {
                Some(t) => run_bank(ticks, bodies, budget, t),
                None => {
                    eprintln!("❌ sense_dump：`bank` 模式必须给音色表（第 6 个参数）：多材质例场就是要考表");
                    std::process::exit(3);
                }
            }
        }
        _ => run_impact(ticks, bodies, budget, table.as_ref()),
    }
}
