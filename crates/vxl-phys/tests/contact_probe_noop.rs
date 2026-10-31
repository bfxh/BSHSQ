//! **探针不得扰动动力学**：同一场景下开/关接触探针（含反事实预测趟），末态逐位相同。
//!
//! 为什么要有这条：反事实趟会在 tick 首子步**多跑一趟窄相**，一旦它的调用点从
//! "解算前"挪到"子步末"，量到的预测覆盖率会从 0.01% 虚高到 55%（位姿晚了一个子步），
//! 而编译、clippy、甚至"读数看起来合理"都发现不了。位同一性是对这种时序漂移唯一有效的锁。
//!
//! 另一半判据是**探针确实活着**：反事实档必须真的产出预测点，否则本测试会因为
//! "探针根本没开"而空洞地变绿。
use vxl_phys::world_step::contact_probe;
use vxl_phys::{BodyBuildExt, PhysConfig, Quat, Shape, Vec3, World};

const LAYERS: usize = 6;
const SIDE: usize = 3;
const TICKS: u64 = 30;

fn build() -> World {
    let mut w = World::new(PhysConfig {
        substeps: 4,
        ..PhysConfig::default()
    });
    w.add_static(
        Shape::Box {
            half: Vec3::new(6.0, 0.1, 6.0),
        },
        Vec3::new(0.0, -0.1, 0.0),
        Quat::IDENTITY,
    );
    for l in 0..LAYERS {
        for iz in 0..SIDE {
            for ix in 0..SIDE {
                let x = (ix as f32 - (SIDE as f32 - 1.0) * 0.5) * 0.52;
                let z = (iz as f32 - (SIDE as f32 - 1.0) * 0.5) * 0.52;
                w.add_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.25),
                    },
                    Vec3::new(x, 0.25 + 0.52 * l as f32, z),
                    Quat::IDENTITY,
                    1000.0,
                );
            }
        }
    }
    w
}

/// 末态位姿 + 速度（逐位比较用）。
fn run() -> Vec<u32> {
    let mut w = build();
    for _ in 0..TICKS {
        w.step();
    }
    let mut out = Vec::new();
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        let v = w.bodies.linvel[i];
        out.extend_from_slice(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        out.extend_from_slice(&[v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
    }
    out
}

#[test]
fn probe_does_not_perturb_dynamics() {
    let baseline = run();

    contact_probe::enable(true);
    contact_probe::counterfactual(true);
    let probed = run();
    let rows = contact_probe::take();
    let stats = contact_probe::loc_stats();
    contact_probe::enable(false);

    // 探针确实跑过：每个 tick 4 个采样，且反事实档产出了预测点、位置判定有样本。
    assert!(
        rows.len() as u64 == TICKS * 4,
        "采样数 {} ≠ {}（探针没跑起来 ⇒ 本测试会空洞变绿）",
        rows.len(),
        TICKS * 4
    );
    assert!(
        rows.iter().any(|s| !s.predicted.is_empty()),
        "反事实档没有任何预测点 ⇒ 预测趟没生效"
    );
    assert!(
        stats.same_loc + stats.new_loc + stats.new_pair > 0,
        "位置判定没样本 ⇒ 首子步落表/归类没跑"
    );

    assert_eq!(baseline.len(), probed.len(), "体数变了");
    for (i, (a, b)) in baseline.iter().zip(probed.iter()).enumerate() {
        assert_eq!(
            a, b,
            "体 {} 的第 {} 个分量位不一致：探针改变了动力学",
            i / 6,
            i % 6
        );
    }
}
