//! carve：**铸装挖空**——按谓词移除流体粒子（`RECIPES.md` 那条"别把体直接放进已有水格"的正面解法）。
//!
//! 为什么需要它：把刚体直接放进已有水格 ⇒ 体与流体粒子**重合** ⇒ 近壁密度虚增 ⇒ 压力尖峰
//! （WCSPH 的经典开局陷阱，`RECIPES.md` 明确记为禁忌）。SPH 下**浮体按定义半浸没** ⇒ 想"零冲击
//! 就位"就必须先把体占位内的粒子挖掉（铸装口径）；`float_quiet_probe` 的"零冲击就位"正是踩了
//! 这条禁忌（实测把体弹到速度上限并穿透地板），见 `docs/OPEN-PROBLEMS.md` P7。
//!
//! 为什么这里是**自由函数**而不是 `FluidSystem` 的方法：该类型在 `god.gate.json` 里登记为
//! 「方法数只准减」的债务，+1 个方法就会顶红。
//!
//! 保序过滤 ⇒ 结果只由（谓词, 初始顺序）决定，**确定性**（§5）。
use super::{FluidSystem, UniformGrid, Vec3};

/// 按 `inside` 移除**流体粒子**（索引 `< n_fluid`），返回移除数。
///
/// 边界粒子（索引 `>= n_fluid`）**一个都不动**（它们是 2b 的介质表示），只整体前移；
/// `spans` 里的全局粒子索引同步重基 ⇒ **任意时刻**都可调用（含已挂边界粒子的系统）。
pub fn carve_fluid(fs: &mut FluidSystem, inside: impl Fn(Vec3) -> bool) -> usize {
    let n = fs.n_fluid;
    let keep: Vec<usize> = (0..n).filter(|&i| !inside(fs.pos[i])).collect();
    if keep.len() == n {
        return 0;
    }
    let removed = n - keep.len();
    // 前缀按 `keep` 保序取、尾巴（边界粒子）原样接上——8 条平行数组必须同序。
    fn pick<T: Copy>(v: &[T], keep: &[usize], n: usize) -> Vec<T> {
        keep.iter()
            .map(|&i| v[i])
            .chain(v[n..].iter().copied())
            .collect()
    }
    fs.pos = pick(&fs.pos, &keep, n);
    fs.vel = pick(&fs.vel, &keep, n);
    fs.acc = pick(&fs.acc, &keep, n);
    fs.xsph = pick(&fs.xsph, &keep, n);
    fs.bforce = pick(&fs.bforce, &keep, n);
    fs.dens = pick(&fs.dens, &keep, n);
    fs.press = pick(&fs.press, &keep, n);
    fs.pmass = pick(&fs.pmass, &keep, n);
    fs.n_fluid = keep.len();
    for s in &mut fs.spans {
        s.2 -= removed as u32;
        s.3 -= removed as u32;
    }
    // 邻居网格每子步按 `pos` 重填 ⇒ 置空即可，不留陈旧索引。
    fs.grid = UniformGrid::default();
    removed
}

/// [`carve_fluid`] 的常用形态：挖掉以 `center` 为心、半径 `r` 的球内流体粒子。
///
/// 存在的理由（小但硬）：调用点若直接写闭包，`rustfmt` 的 `fn_call_width` 会把整调用**拆成三行**——
/// 而"给就位体挖一个罩住它的空腔"是铸装的固定动作，值得一个一行的入口。
pub fn carve_sphere(fs: &mut FluidSystem, center: Vec3, r: f32) -> usize {
    carve_fluid(fs, |p| (p - center).length() < r)
}

/// [`carve_fluid`] 的另一种常用形态：挖掉以 `center` 为心、`half` 为半长的**轴对齐盒**内的粒子。
///
/// **尺寸怎么定（2026-10-04 实测，别凭直觉）**：挖空范围 ≈ **体 AABB + 核半径 h**，不是"越贴越好"。
/// `float_quiet_probe` 同一格里比过两档：`half = 体半长 + h`（0.06+0.05 = 0.11 球）⇒ 体停在平衡位
/// 附近；**收紧到 0.08**（只比体大半圈）⇒ 两个格被弹到速度上限并**穿透地板**（y≈−5000）
/// ⇒ 残留的"核带内"粒子仍在与体相互作用，反而是收紧后的主扰动源。切不可把 `half` 取成体半长本身。
pub fn carve_box(fs: &mut FluidSystem, center: Vec3, half: Vec3) -> usize {
    carve_fluid(fs, |p| {
        (p.x - center.x).abs() < half.x
            && (p.y - center.y).abs() < half.y
            && (p.z - center.z).abs() < half.z
    })
}

#[cfg(test)]
mod tests {
    use super::carve_fluid;
    use crate::{FluidConfig, FluidSystem, Vec3};

    /// 挖空只动流体前缀：计数、保序、边界段索引重基都对得上；谓词无命中时返回 0 且不动。
    #[test]
    fn carve_removes_only_fluid_prefix_and_rebases_spans() {
        let mut fs = FluidSystem::new(FluidConfig::default(), Vec3::ZERO, [4, 4, 4], 0.1);
        assert_eq!(fs.len(), 64);
        // 手工挂 4 个"边界粒子"（模拟 2b 的尾巴）+ 一条 span（全局 64..68）。
        fs.pos.extend([Vec3::new(9.0, 0.0, 0.0); 4]);
        fs.vel.extend([Vec3::ZERO; 4]);
        fs.acc.extend([Vec3::ZERO; 4]);
        fs.xsph.extend([Vec3::ZERO; 4]);
        fs.bforce.extend([Vec3::ZERO; 4]);
        fs.dens.extend([0.0; 4]);
        fs.press.extend([0.0; 4]);
        fs.pmass.extend([1.0; 4]);
        fs.spans.push((7, Vec3::ZERO, 64, 68));
        // 挖掉第一象限 2×2×2 的粒子（各轴 0.05/0.15 ⇒ 8 个）。
        let removed = carve_fluid(&mut fs, |p| p.x < 0.2 && p.y < 0.2 && p.z < 0.2);
        assert_eq!(removed, 8);
        assert_eq!(fs.len(), 56);
        assert_eq!(fs.pos.len(), 60);
        let s = fs.spans[0];
        assert_eq!((s.0, s.2, s.3), (7, 56, 60));
        // 剩下的流体粒子仍是原顺序的子序列：x 非减。
        assert!(fs.pos[..56].windows(2).all(|w| w[0].x <= w[1].x));
        // 谓词已无命中 ⇒ 0 且数组不变（用长度 + 首尾抽查：不引 `.clone()` 也不引 `.unwrap()`，两个棘轮都只准减）。
        let before_len = fs.pos.len();
        let (head, tail) = (fs.pos[0], fs.pos[before_len - 1]);
        assert_eq!(
            carve_fluid(&mut fs, |p| p.x < 0.2 && p.y < 0.2 && p.z < 0.2),
            0
        );
        assert_eq!(
            (fs.pos.len(), fs.pos[0], fs.pos[before_len - 1]),
            (before_len, head, tail)
        );
    }
}
