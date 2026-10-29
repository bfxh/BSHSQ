//! **VBD 块求解**：粒子↔刚体接触的**块坐标下降**施加，**不依赖几何排序**。
//!
//! 现役做法（`rope.rs`）是**先按几何序排序、再就地 Gauss-Seidel**——顺序无关是"排出来的"，
//! 代价是每次施加前 m log m 排序（实测占子步 3.6%–6.8%，见 `tests/vbd_sort_cost.rs`）。
//! 本模块给另一条路：**Jacobi 块更新**（所有接触按同一迭代前状态解 λ，体的反作用一次性求和），
//! 迭代到定点 ⇒ 结果只依赖**接触集合**，与施加顺序无关（判据见本文件的置换测试）。
use std::sync::atomic::{AtomicU32, Ordering};

use vxl_phys_core::Vec3;

use crate::rigid::RigidProxy;
use crate::rope::{BodyHit, Rope};

/// 块求解的**遍数**（`0` = 关 = 现役"几何排序 + 就地 GS"）。进程级静态：由门面在**整次
/// step 之外**用 `with_block_solver` 设定，step 内不变 ⇒ 不影响岛级并行的确定性。
static BLOCK_ITERS: AtomicU32 = AtomicU32::new(0);

/// 当前线程的块求解遍数（`0` = 关）。
pub(crate) fn block_iters() -> u32 {
    BLOCK_ITERS.load(Ordering::Relaxed)
}

/// 在**块求解 N 遍**的上下文里跑 `f`（`0` = 关；退出时恢复原值，嵌套安全）。
pub fn with_block_solver<R>(iters: u32, f: impl FnOnce() -> R) -> R {
    let old = BLOCK_ITERS.swap(iters, Ordering::Relaxed);
    let r = f();
    BLOCK_ITERS.store(old, Ordering::Relaxed);
    r
}

/// 一次施加的上下文（体号 / 体代理 / 虚拟位姿 / 子步长）——打包成结构体是为了守住 args 门。
pub(crate) struct BodyCtx<'a> {
    pub j: usize,
    pub b: &'a RigidProxy,
    pub vpos: Vec3,
    pub h: f32,
}

/// **施加段的唯一入口**：默认 = 几何排序 + 逐 hit 就地 GS（现役，**逐位不变**）；
/// 块求解开 = **不排序** + Jacobi 块（带残余穿透口径，可多遍）。
pub(crate) fn apply_all(r: &mut Rope, j: usize, b: &RigidProxy, vpos: Vec3, h: f32) {
    let iters = block_iters();
    if iters == 0 {
        r.hits
            .sort_by(|a, c| a.ord.total_cmp(&c.ord).then(a.x.total_cmp(&c.x)));
    }
    let hits = std::mem::take(&mut r.hits);
    if iters > 0 {
        let ctx = BodyCtx { j, b, vpos, h };
        apply_block(r, &hits, &ctx, iters);
    } else {
        for hit in &hits {
            r.apply_body_hit(hit, j, b, vpos, h);
        }
    }
    r.hits = hits;
}

/// 对**同一个体**的一批命中做 Jacobi 块施加（带**残余穿透**口径，支持多遍）。
///
/// 残余口径：`hit.depth` 是收集时的**快照**；第 k 遍只施加**还没被消掉的那一份**
/// （`applied[i]` 记该接触已消掉的穿透）。没有它，多遍会把同一份穿透重复施加
/// （M1 实测 8 遍把盒子压穿，末 y = −7.72）。
pub(crate) fn apply_block(r: &mut Rope, hits: &[BodyHit], c: &BodyCtx, iters: u32) {
    let (j, b, vpos, h) = (c.j, c.b, c.vpos, c.h);
    let mut applied = vec![0.0f32; hits.len()];
    for _ in 0..iters.max(1) {
        let (dvb, dxb, dispb) = (r.body_dv[j], r.body_dx[j], r.body_disp[j]);
        let (mut acc_dv, mut acc_dx, mut acc_disp) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for (k, hit) in hits.iter().enumerate() {
            // 冻结：本遍内每个 hit 都从同一起点读体状态
            r.body_dv[j] = dvb;
            r.body_dx[j] = dxb;
            r.body_disp[j] = dispb;
            let resid = hit.depth - applied[k];
            if resid <= 0.0 {
                continue;
            }
            let mut sub = *hit;
            sub.depth = resid;
            let Some((lam, v_point)) = r.body_hit_solve(&sub, j, b, vpos, h) else {
                continue;
            };
            // `body_hit_solve` 里的**位置补足**（§8.4.9）也写 dx/disp ⇒ 先收进累计
            acc_dx += r.body_dx[j] - dxb;
            acc_disp += r.body_disp[j] - dispb;
            let before = r.pos[hit.i as usize];
            r.body_dv[j] = dvb;
            r.body_dx[j] = dxb;
            r.body_disp[j] = dispb;
            r.body_hit_project(&sub, h, lam, v_point);
            if b.inv_mass > 0.0 {
                r.body_hit_reaction(&sub, j, b, vpos, h, before);
                acc_dv += r.body_dv[j] - dvb;
                acc_dx += r.body_dx[j] - dxb;
                acc_disp += r.body_disp[j] - dispb;
            }
            applied[k] += lam * (r.inv_mass[hit.i as usize] + b.inv_mass);
        }
        r.body_dv[j] = dvb + acc_dv;
        r.body_dx[j] = dxb + acc_dx;
        r.body_disp[j] = dispb + acc_disp;
    }
}

/// 一次接触（与 `rope::BodyHit` 同口径的最小集；供 `solve_block` 的纯函数判据使用）。
#[derive(Clone, Copy, Debug)]
pub struct BlockContact {
    pub particle: u32,
    pub inv_mass_p: f32,
    pub n: Vec3,
    pub depth: f32,
}

/// 纯函数版块求解的读数。
#[derive(Clone, Debug)]
pub struct BlockSolve {
    pub iters: u32,
    pub body_dv: Vec3,
    pub corrections: Vec<Vec3>,
}

/// 纯函数版：对**同一体的整组接触**做 Jacobi 块下降（无引擎状态，用于顺序无关性判据）。
pub fn solve_block(contacts: &[BlockContact], inv_mass_b: f32, iters: u32) -> BlockSolve {
    let n = contacts.len();
    let mut p_disp = vec![0.0f32; n];
    let mut b_disp = Vec3::ZERO;
    for _ in 0..iters.max(1) {
        let mut lam = vec![0.0f32; n];
        for (k, c) in contacts.iter().enumerate() {
            let w = c.inv_mass_p + inv_mass_b;
            if w <= 0.0 {
                continue;
            }
            let resid = c.depth - p_disp[k] + b_disp.dot(c.n);
            lam[k] = (resid / w).max(0.0);
        }
        let mut sum = Vec3::ZERO;
        for (k, c) in contacts.iter().enumerate() {
            sum += c.n * lam[k];
        }
        b_disp -= sum * inv_mass_b;
        for (k, c) in contacts.iter().enumerate() {
            p_disp[k] += lam[k] * c.inv_mass_p;
        }
    }
    let corrections = contacts
        .iter()
        .enumerate()
        .map(|(k, c)| c.n * p_disp[k])
        .collect();
    BlockSolve {
        iters: iters.max(1),
        body_dv: b_disp,
        corrections,
    }
}

#[cfg(test)]
mod tests {
    use super::{solve_block, BlockContact};

    use vxl_phys_core::Vec3;

    fn contact_set() -> Vec<BlockContact> {
        (0..33u32)
            .map(|i| {
                let t = i as f32 / 32.0;
                BlockContact {
                    particle: i,
                    inv_mass_p: 1.0 / 0.02,
                    n: Vec3::new(0.0, 1.0, 0.0),
                    depth: 0.002 + 0.0004 * (t * std::f32::consts::PI).sin(),
                }
            })
            .collect()
    }

    fn permute(v: &[BlockContact], seed: u32) -> Vec<BlockContact> {
        let mut idx: Vec<u32> = (0..v.len() as u32).collect();
        let mut s = seed.wrapping_mul(1664525).wrapping_add(1013904223) | 1;
        for i in (1..idx.len()).rev() {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let j = (s >> 8) as usize % (i + 1);
            idx.swap(i, j);
        }
        idx.iter().map(|&i| v[i as usize]).collect()
    }

    /// **M0 判据**：任意置换 ⇒ 体反作用与逐粒子修正一致（≤1e-6），且**不需要几何排序**。
    #[test]
    fn block_solve_is_permutation_invariant_without_geometry_sort() {
        let base = contact_set();
        let r0 = solve_block(&base, 1.0, 24);
        let mut worst_b = 0.0f32;
        let mut worst_p = 0.0f32;
        for seed in 0..16u32 {
            let perm = permute(&base, seed);
            let r = solve_block(&perm, 1.0, 24);
            worst_b = worst_b.max((r.body_dv - r0.body_dv).length());
            for (k, c) in perm.iter().enumerate() {
                if let Some(o) = base.iter().position(|o| o.particle == c.particle) {
                    worst_p = worst_p.max((r.corrections[k] - r0.corrections[o]).length());
                }
            }
        }
        println!("[vbd] 置换最大差：体 {worst_b:.3e} / 粒子 {worst_p:.3e}（24 遍）");
        assert!(worst_b <= 1e-6, "体反作用随置换变化：{worst_b:.3e}");
        assert!(worst_p <= 1e-6, "粒子修正随置换变化：{worst_p:.3e}");
    }

    #[test]
    fn block_solve_converges_with_iterations() {
        let c = contact_set();
        let a = solve_block(&c, 1.0, 8);
        let b = solve_block(&c, 1.0, 32);
        let d = (a.body_dv - b.body_dv).length();
        println!("[vbd] 8 遍 vs 32 遍：体反作用差 {d:.3e}");
        assert!(d <= 1e-5, "未收敛到定点：{d:.3e}");
    }

    #[test]
    fn block_solve_shares_penetration_by_inverse_mass() {
        let c = vec![BlockContact {
            particle: 0,
            inv_mass_p: 50.0,
            n: Vec3::new(0.0, 1.0, 0.0),
            depth: 0.051,
        }];
        let r = solve_block(&c, 1.0, 32);
        let p = r.corrections[0].y;
        let b = -r.body_dv.y;
        println!(
            "[vbd] 单接触：粒子让 {p:.6} / 体让 {b:.6} / 合计 {:.6}",
            p + b
        );
        assert!((p + b - 0.051).abs() <= 1e-4, "未消掉穿透：{}", p + b);
        assert!((p / b - 50.0).abs() <= 0.5, "逆质量分担比例不对：{}", p / b);
    }
}
