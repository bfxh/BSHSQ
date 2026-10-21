//! **微探针：邻居相位的"访存收集 vs 算术"账**（`#[ignore]`，只打印）——决定第三刀
//! （按格序重排粒子存储）值不值得动**公开数组序**。
//!
//! 做法：同一份邻居表、同一遍历序，跑两个循环：
//! ① **收集版**：只碰 `pos/vel/dens/press/pmass` 五个数组并把值累加（几乎无算术）；
//! ② **算术版**：用同样收集到的值做真实力相位的那套运算（sqrt/乘加/分支）。
//! 两者之差 ≈ 纯算术成本；收集版本身 ≈ 访存成本。
//! ⇒ 若"收集 ≫ 算术" ⇒ 重排（缓存局部性）是杠杆；否则 SIMD 才是。
//!
//! **为什么单开文件**：`lib.rs` 的 `file_lines` 是 god 门的**棘轮**（只准减），
//! 而本探针与 crate 的公开 API 无关 ⇒ 放进独立文件只受**绝对阈值**管。
//!
//! **零 glob 导入**（glob-gate：新文件禁 `use …::*`）⇒ 全部显式列名。

/// 微探针本体（`#[ignore]`，只打印）。
#[test]
#[ignore = "微探针（只打印）：访存 vs 算术账；见上注"]
pub fn micro_probe_gather_vs_compute() {
    use crate::{FluidConfig, FluidSystem, Vec3};
    use std::time::Instant;
    let n = 50usize; // 125k 粒（够大又不至于跑很久）
    let spacing = 0.05f32;
    let mut f = FluidSystem::new(
        FluidConfig::default(),
        Vec3::new(
            -(n as f32) * spacing * 0.5,
            0.5,
            -(n as f32) * spacing * 0.5,
        ),
        [n, n, n],
        spacing,
    );
    f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders); // 建网格
    let h2 = f.dom.kern.h2;
    let (h, k6, ks) = (f.dom.kern.h, f.dom.kern.k6, f.dom.kern.ks);
    let alpha_c = f.cfg.artificial_viscosity * f.cfg.sound_speed;
    let nf = f.n_fluid;
    // 域②的五个数组借成局部（探针只读 ⇒ 借出后下面每行都短）。
    let (p, v, d, pr, m) = (
        &f.dom.ps.pos,
        &f.dom.ps.vel,
        &f.dom.ps.dens,
        &f.dom.ps.press,
        &f.dom.ps.pmass,
    );
    // ① 收集版
    let t0 = Instant::now();
    let mut sink = 0.0f32;
    for i in 0..nf {
        let vi = v[i];
        let rho_i = d[i];
        f.for_neighbors(i, |j, dj, r2| {
            sink += p[j].x + v[j].y + d[j] + pr[j] + m[j];
            sink += dj.x + r2;
            sink += vi.y + rho_i;
        });
    }
    let gather = t0.elapsed().as_secs_f64() * 1e3;
    // ② 算术版（真实力相位的算式，但结果丢弃）
    let t0 = Instant::now();
    let mut sink2 = 0.0f32;
    for i in 0..nf {
        let vi = v[i];
        let rho_i = d[i];
        let ci2 = pr[i] / (rho_i * rho_i);
        let mut a = Vec3::ZERO;
        let mut xs = Vec3::ZERO;
        f.for_neighbors(i, |j, dj, r2| {
            let rho_j = d[j];
            let cj2 = pr[j] / (rho_j * rho_j);
            let r = r2.sqrt();
            let t = h - r;
            let mj = m[j];
            let coef = mj * (ks * t * t) * (ci2 + cj2);
            let denom = r.max(1e-9);
            a += dj * (coef / denom);
            let vij = vi - v[j];
            let vdn = -vij.dot(dj);
            if vdn > 0.0 {
                let mu = vdn * h / (r2 + 0.01 * h2);
                let c = mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ks * t * t);
                a += dj * (c / denom);
            }
            let tt = h2 - r2;
            let w = k6 * tt * tt * tt;
            xs += (v[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
        });
        sink2 += a.x + xs.y;
    }
    let compute = t0.elapsed().as_secs_f64() * 1e3;
    println!(
        "微探针（{nf} 粒）：收集版 {gather:.1} ms | 算术版 {compute:.1} ms | 算术占比 {:.0}% | sink {sink:.0}/{sink2:.0}",
        100.0 * compute / (gather + compute)
    );
}
