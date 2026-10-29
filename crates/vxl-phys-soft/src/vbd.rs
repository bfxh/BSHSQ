//! **VBD 块求解（M0 原型）**：粒子↔刚体接触的**块坐标下降**施加，**不依赖几何排序**。
//!
//! 现役做法（`rope.rs` / `cloth/ext/body.rs`）是**先按几何序排序、再就地 Gauss-Seidel**
//! （`body_dv` 顺序推进）——顺序无关是"排出来的"，代价是每次施加前 m log m 排序。
//! 本模块给另一条路：**Jacobi 块更新**（所有接触按同一迭代前状态解 λ，体的反作用一次性求和），
//! 迭代到定点 ⇒ 结果只依赖**接触集合**，与施加顺序无关（顺序只影响浮点求和的舍入，见测试读数）。
//!
//! 与 VBD 论文的对应：体是"块"、每个接触粒子是"顶点"，块与顶点交替精确求解局部子问题；
//! 差别是本 M0 把顶点子问题退化成沿法线的 1 DOF（与现役 `body_hit_solve` 同口径），
//! 以便先只验证**顺序无关**这一条判据。

use vxl_phys_core::Vec3;

/// 一次接触（与 `rope::BodyHit` 同口径的最小集）。
#[derive(Clone, Copy, Debug)]
pub struct BlockContact {
    /// 粒子号（仅用于把修正写回对应粒子；求解本身不看它）。
    pub particle: u32,
    /// 粒子的逆质量。
    pub inv_mass_p: f32,
    /// 接触法线（由体指向粒子）。
    pub n: Vec3,
    /// 本子步的穿透深度（> 0 才需要修正）。
    pub depth: f32,
}

/// 块求解的配置与结果。
#[derive(Clone, Debug)]
pub struct BlockSolve {
    /// 迭代遍数（Jacobi 遍；定点收敛后与遍数无关，见测试的 §收敛）。
    pub iters: u32,
    /// 每遍的体反作用（累加后一次性施加，**不在遍内推进**）。
    pub body_dv: Vec3,
    /// 每个接触的粒子位置修正（与 `contacts` 同序）。
    pub corrections: Vec<Vec3>,
}

/// 对**同一体的整组接触**做块坐标下降：返回体反作用与逐粒子修正。
///
/// 关键性质（M0 判据）：`corrections` 与 `body_dv` 只依赖 `contacts` 的**集合**，
/// 把 `contacts` 任意置换，结果在浮点舍入内一致——因为每遍的 λ 都由**上一遍末状态**算，
/// 体的反作用在遍内是求和、不参与其它接触的 λ。
pub fn solve_block(contacts: &[BlockContact], inv_mass_b: f32, iters: u32) -> BlockSolve {
    let n = contacts.len();
    // 粒子沿各自法线的累计让位；体的累计位移（沿 −n 分担）。
    let mut p_disp = vec![0.0f32; n];
    let mut b_disp = Vec3::ZERO;
    for _ in 0..iters.max(1) {
        // ① 全部 λ 按**迭代前**的同一状态解出（Jacobi：本遍内互不影响 ⇒ 与施加顺序无关）
        let mut lam = vec![0.0f32; n];
        for (k, c) in contacts.iter().enumerate() {
            let w = c.inv_mass_p + inv_mass_b;
            if w <= 0.0 {
                continue;
            }
            // 残余穿透：初始穿透 − 粒子已让的 − 体已让的（体沿 −n 走 ⇒ 沿 +n 的闭合是 −b_disp·n）
            let resid = c.depth - p_disp[k] + b_disp.dot(c.n);
            lam[k] = (resid / w).max(0.0);
        }
        // ② 体的反作用**一次求和**（不在遍内推进 ⇒ 不与 ① 互为因果）
        let mut sum = Vec3::ZERO;
        for (k, c) in contacts.iter().enumerate() {
            sum += c.n * lam[k];
        }
        b_disp -= sum * inv_mass_b;
        // ③ 粒子各自让位
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
        // 33 个粒子压在一块 box 上（与 `rope` 判据同规模），法线朝上、深度按悬链形状递增
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

    /// 确定性 LCG 置换（不引入 rand 依赖）。
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
        let inv_b = 1.0f32; // 1 kg 盒
        let r0 = solve_block(&base, inv_b, 24);
        let mut worst_b = 0.0f32;
        let mut worst_p = 0.0f32;
        for seed in 0..16u32 {
            let perm = permute(&base, seed);
            let r = solve_block(&perm, inv_b, 24);
            worst_b = worst_b.max((r.body_dv - r0.body_dv).length());
            // 按粒子号对齐比较（置换后顺序不同）
            for (k, c) in perm.iter().enumerate() {
                let orig = base.iter().position(|o| o.particle == c.particle);
                if let Some(o) = orig {
                    let d = (r.corrections[k] - r0.corrections[o]).length();
                    worst_p = worst_p.max(d);
                }
            }
        }
        println!("[vbd] 置换最大差：体 {worst_b:.3e} / 粒子 {worst_p:.3e}（24 遍）");
        assert!(worst_b <= 1e-6, "体反作用随置换变化：{worst_b:.3e}");
        assert!(worst_p <= 1e-6, "粒子修正随置换变化：{worst_p:.3e}");
    }

    /// 迭代收敛：遍数翻倍后末态变化很小（定点存在）。
    #[test]
    fn block_solve_converges_with_iterations() {
        let c = contact_set();
        let a = solve_block(&c, 1.0, 8);
        let b = solve_block(&c, 1.0, 32);
        let d = (a.body_dv - b.body_dv).length();
        println!("[vbd] 8 遍 vs 32 遍：体反作用差 {d:.3e}");
        assert!(d <= 1e-5, "未收敛到定点：{d:.3e}");
    }

    /// 守恒向：体与粒子的反向位移按逆质量比例（w_p = 50、w_b = 1 ⇒ 粒子让 50/51）。
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
