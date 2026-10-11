//! `sense_dump`：把示例场景跑出**cue 流**（JSONL 到 stdout，统计到 stderr）。
//!
//! 工具链的第一件（`docs/ROUTE.md` §3.2）：音效师拿它对着场景调表，CI 拿它取基线
//! （`scripts/gate_sense.sh` 跑两遍逐字对拍 + 预算上界核对）。
//!
//! 用法：`sense_dump [ticks=300] [bodies=64] [budget=16]`
//!
//! **场景口径**（确定性、无随机器）：体素地板 + 一网格盒雨，落点错开 ⇒ 冲击成串、能同时触发
//! 预算截断与"轻碰不算响"两条规则。**地板必须是 provider**：冲击记录只对 `Shape::Provider`
//! 接触产生（见 `World::record_impacts`）——用静态盒当地板会得到空流（踩过）。
use vxl_phys::{PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{jsonl, Cue, SenseConfig, SenseExt};

fn arg(i: usize, dflt: &str) -> String {
    std::env::args().nth(i).unwrap_or_else(|| dflt.to_string())
}

fn num<T: std::str::FromStr>(i: usize, dflt: &str, fallback: T) -> T {
    arg(i, dflt).parse().ok().unwrap_or(fallback)
}

fn main() {
    let ticks: usize = num(1, "300", 300);
    let bodies: usize = num(2, "64", 64);
    let budget: usize = num(3, "16", 16);
    let cfg = SenseConfig {
        max_cues_per_tick: budget,
        ..SenseConfig::default()
    };

    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, -0.5, -4.0), 0.5, 16, 3, 16);
    vol.fill_box(Vec3::new(-4.0, -0.5, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
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
    print!("{}", jsonl(&cues));
    eprintln!(
        "{{\"ticks\":{ticks},\"bodies\":{bodies},\"budget\":{budget},\"impacts\":{rec},\
         \"cues\":{emi},\"dropped_weak\":{dr_weak},\"dropped_budget\":{dr_budget},\
         \"peak_cues_per_tick\":{peak}}}"
    );
}
