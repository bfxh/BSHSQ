//! **并列带种子分支**的定点对拍（P20 盲区）：主探针 `gpu_narrow_probe` 的场景不触发
//! `narrow.wgsl` 的「带内 >4 点」分支（同步种子前后读数逐字相同）⇒ 这里只摆**平放叠盒**：
//! 两块 0.8 m 盒零缝叠放、上层偏航 6.1° + 侧倾 0.02° ⇒ 裁剪点两两成对、深度并列且在
//! `TIE_TOL` 内（`docs/OPEN-PROBLEMS.md` §P20），正是该分支的入口（CPU 侧实测 `band=8`）。
//!
//! 判据：逐对**点数相同 + 点/深度/特征逐位相同**。
//! A/B 结论（2026-10-20）：把 wgsl 种子退回「带首」时本探针**转红**（两点集不同），
//! 用「坐标 (x,z) 定序种子」时逐位一致 ⇒ 该分支确实被走到，且与 CPU 同口径。
//!
//! 用法：`cargo run --release -p vxl-phys-gpu --example gpu_band_probe`
//! 无适配器 ⇒ 退出码 2（未判定，不是通过）。
use vxl_phys::{ContactPoint, DefaultNarrowPhase, NarrowPhase, Quat, Shape, Vec3, World};
use vxl_phys_core::interop::NoProviders;
use vxl_phys_core::schedule::ScopedPool;
use vxl_phys_gpu::narrow::{NarrowTier, Slot, BODY_WORDS, KIND_BOX, NOT_HANDLED};

const SKIN: f32 = 0.02;

fn pack(w: &World) -> Vec<u32> {
    let mut out = Vec::with_capacity(w.bodies.len() * BODY_WORDS);
    for i in 0..w.bodies.len() {
        let p = w.bodies.position[i];
        let r = w.bodies.rot(i);
        out.extend_from_slice(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
        out.extend_from_slice(&[r.x.to_bits(), r.y.to_bits(), r.z.to_bits(), r.w.to_bits()]);
        let Shape::Box { half } = w.bodies.shape[i] else {
            return Vec::new(); // 本探针只摆盒；出现别的形状说明场景被改坏了
        };
        out.push(KIND_BOX);
        for v in [half.x, half.y, half.z] {
            out.push(v.to_bits());
        }
        out.push(0);
    }
    out
}

/// 平放叠盒：地板 + 下层（平放）+ 上层（偏航 6.1°、侧倾 0.02°、零缝）。
fn stack_world() -> World {
    let mut w = World::new(Default::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 0.4, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.4),
        },
        Vec3::new(0.0, 1.2, 0.0),
        Quat::from_axis_angle(Vec3::Y, 6.1f32.to_radians())
            * Quat::from_axis_angle(Vec3::X, 0.02f32.to_radians()),
        1000.0,
    );
    w
}

/// 对一个对：CPU 侧点数/逐位 vs 卡上。返回是否一致。
fn check_pair(cpu: &[vxl_phys::Manifold], slot: &Slot, a: u32, b: u32) -> bool {
    let raw = slot.count();
    if raw == NOT_HANDLED {
        println!("对 ({a},{b})：卡上未接手（NOT_HANDLED）⇒ 本探针无效");
        return false;
    }
    let found = cpu.iter().find(|m| (m.a as u32, m.b as u32) == (a, b));
    let pts: &[ContactPoint] = match found {
        Some(m) => &m.points[..],
        None => &[],
    };
    let cnt = raw as usize;
    print!("对 ({a},{b})：CPU {} 点 / 卡上 {cnt} 点", pts.len());
    if pts.len() != cnt || cnt == 0 {
        println!("  ✗ 点数不符");
        return false;
    }
    let mut same = true;
    for k in 0..cnt {
        let (p, d, f) = slot.point(k);
        let cp = pts[k];
        if p != [cp.point.x, cp.point.y, cp.point.z]
            || d.to_bits() != cp.depth.to_bits()
            || f != cp.feature
        {
            same = false;
        }
    }
    println!(
        "  {}",
        if same {
            "逐位相同 ✓"
        } else {
            "✗ 点/深度/特征不同"
        }
    );
    if !same {
        for k in 0..cnt {
            let (p, d, f) = slot.point(k);
            println!(
                "    卡上 {k}: ({:.6},{:.6},{:.6}) d={d:e} f={f:#x}",
                p[0], p[1], p[2]
            );
            println!(
                "    CPU  {k}: ({:.6},{:.6},{:.6}) d={:e} f={:#x}",
                pts[k].point.x, pts[k].point.y, pts[k].point.z, pts[k].depth, pts[k].feature
            );
        }
    }
    same
}

fn main() {
    let w = stack_world();
    let pairs: Vec<(u32, u32)> = vec![(0, 1), (1, 2)];
    let flat: Vec<u32> = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
    let jobs = ScopedPool::new(1);
    let mut np = DefaultNarrowPhase::new(SKIN);
    let mut cpu = Vec::new();
    np.collide(&w.bodies, &pairs, &[], &NoProviders, &mut cpu, &jobs);

    let Ok(tier) = NarrowTier::new(0, w.bodies.len() as u32, pairs.len() as u32, SKIN) else {
        println!("⚠️ 本机无可用适配器 ⇒ **未判定**（不是通过）");
        std::process::exit(2);
    };
    println!("适配器：{}", tier.adapter());
    let Ok(run) = tier.run(&pack(&w), &flat) else {
        println!("❌ 卡上窄相跑不通");
        std::process::exit(1);
    };
    let mut ok = true;
    for (i, &(a, b)) in pairs.iter().enumerate() {
        ok &= check_pair(&cpu, &run.slots[i], a, b);
    }
    if ok {
        println!("✅ 并列带种子分支：CPU 与卡上逐位一致");
    } else {
        println!("❌ 存在不一致");
        std::process::exit(1);
    }
}
