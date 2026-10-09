//! flow：**介质真双向**（`PLAN-COUPLING.md` §4.3 / D3 最小版**切片 1**）。
//!
//! 分工：`Splat` 的几何/密度口径不动（渲染桥与接触提供者是同一套参数）；本片给场
//! **加一维状态**——逐核速度 `kern_vel`（与 `splats` 同序同长）：
//!
//! - **`deposit`（体→介质）**：把注入动量按**同一核权重**（`exp(−α/2)·opacity`，与
//!   `density_grad` 逐字同式、同 `cut` 截断）分摊给 `x` 附近的核：
//!   `Δv_k = momentum·(e_k/Σe)/m_k`，核质量口径 `m_k = ρ·(4/3)π·σx σy σz`（1σ 椭球；
//!   只是动量分摊的口径，不动核的几何）；
//! - **`sample` 的速度项（介质→体）**：two_way 时读**核速度的 e 加权插值**
//!   （无近核 ⇒ 回落常值 `medium_velocity`）；
//! - **`advance(dt)`（域轮次加一格）**：逐核**平流**（`center += v·dt`）后按 `damping` 衰减；
//!   核一动均匀网格即脏（`grid = None` ⇒ 查询自动退回全扫，与网格逐位一致；`step_dynamics`
//!   每步开头重建一次 ⇒ 世界路径的压力/黏性吃到候选表）。提供者 AABB **随核漂移刷新**
//!   （`world_step/medium.rs` 对双向场每子步写回 `provider_bounds`；判据
//!   `tests/splat.rs::drifting_medium_bounds_follow_kernels`）。
//!
//! **默认关（`two_way = false`）**：`sample` 速度恒为常值、`deposit`/`advance` 空操作
//! ⇒ 默认档逐位不变（金样/四哈希守门）。确定性：候选走与 `density_grad` 同一
//! `candidate_ids()`（网格序 × 注册序）⇒ 同输入同结果、无浮点归约序变化。
//! （显式导入，不用 `use super::*`——glob-gate：新文件零通配。）
use crate::GaussianSplatField;
use vxl_phys_core::Vec3;

impl GaussianSplatField {
    /// 开启/关闭双向耦合（开启时惰性补齐 `kern_vel` 长度；**关闭不清零**——重开续用）。
    pub fn set_two_way(&mut self, on: bool) {
        self.two_way = on;
        if on {
            self.kern_vel.resize(self.splats.len(), Vec3::ZERO);
        }
    }

    pub fn two_way(&self) -> bool {
        self.two_way
    }

    /// 逐核速度场（诊断/判据；未开启为空）。
    pub fn kernel_velocities(&self) -> &[Vec3] {
        &self.kern_vel
    }

    /// 累计注入动量（审计：`deposit` 的向量和；N·s）。
    pub fn absorbed_momentum(&self) -> Vec3 {
        self.absorbed
    }

    /// 核质量口径：**单一来源在 [`Splat::mass`]**（介质密度场的积分质量，P11 定案）。
    /// 这里的 `.max(1e-9)` 只是**除零下限**（`medium_density = 0` 的场仍可开 two_way，
    /// 分摊公式不能出 NaN）—— 不是第二套质量口径。
    #[inline]
    fn kernel_mass(&self, k: usize) -> f32 {
        self.splats[k].mass(self.medium_density).max(1e-9)
    }

    /// 核速度的 e 加权插值（`MediumField::sample` 的速度项；two_way 关 ⇒ 常值回落）。
    pub(crate) fn flow_velocity(&self, x: Vec3) -> Vec3 {
        if !self.two_way || self.kern_vel.is_empty() {
            return self.medium_velocity;
        }
        let mut wsum = 0.0f32;
        let mut vsum = Vec3::ZERO;
        for k in self.candidate_ids(x) {
            let s = self.splats[k];
            let a = Self::alpha(&s, &s.axes(), x);
            if a > self.cut {
                continue;
            }
            let e = (-0.5 * a).exp() * s.opacity;
            if let Some(v) = self.kern_vel.get(k) {
                vsum += *v * e;
            }
            wsum += e;
        }
        if wsum > 1e-12 {
            vsum * (1.0 / wsum)
        } else {
            self.medium_velocity
        }
    }

    /// **体→介质的反作用沉积**（`MediumField::deposit` 的实现体；two_way 关 ⇒ 空操作）。
    /// `mass`/`pressure_work` 暂只做审计参数（未消费；留给对称记账片）。
    pub(crate) fn deposit_flow(&mut self, x: Vec3, momentum: Vec3, mass: f32, pressure_work: f32) {
        let _ = (mass, pressure_work);
        if !self.two_way || momentum == Vec3::ZERO {
            return;
        }
        if self.kern_vel.len() != self.splats.len() {
            self.kern_vel.resize(self.splats.len(), Vec3::ZERO);
        }
        // ① Σe（只读）；② 按 e 分摊（写 kern_vel——`take` 借出，避免与 `&self` 查询冲突）。
        let mut wsum = 0.0f32;
        for k in self.candidate_ids(x) {
            let s = self.splats[k];
            let a = Self::alpha(&s, &s.axes(), x);
            if a > self.cut {
                continue;
            }
            wsum += (-0.5 * a).exp() * s.opacity;
        }
        if wsum <= 1e-12 {
            return; // 无近核：动量不打空气
        }
        let mut kv = std::mem::take(&mut self.kern_vel);
        for k in self.candidate_ids(x) {
            let s = self.splats[k];
            let a = Self::alpha(&s, &s.axes(), x);
            if a > self.cut {
                continue;
            }
            let e = (-0.5 * a).exp() * s.opacity;
            let m = self.kernel_mass(k);
            if let Some(v) = kv.get_mut(k) {
                *v += momentum * (e / wsum / m);
            }
        }
        self.kern_vel = kv;
        self.absorbed += momentum;
    }

    /// **介质推进**（域轮次加一格；每 tick 一次）：逐核**平流**（`center += v·dt`）后按
    /// `damping` 衰减。零速核跳过（不产生位移、也不动几何）⇒ 无流时逐位不变。核一动，
    /// 均匀网格登记的中心即失效 ⇒ `grid = None`（查询退回全扫，与网格逐位一致）。
    pub fn advance(&mut self, dt: f32) {
        if !self.two_way || self.kern_vel.is_empty() {
            return;
        }
        let mut moved = false;
        for (k, v) in self.kern_vel.iter_mut().enumerate() {
            if *v == Vec3::ZERO {
                continue;
            }
            self.splats[k].center += *v * dt;
            *v *= self.damping;
            moved = true;
        }
        if moved {
            self.grid = None; // 脏（切片 2 由消费者按需 rebuild；全扫与网格逐位一致）
        }
    }

    /// 候核**索引**迭代（与 [`GaussianSplatField::candidates`] 同序：注册序 / 网格格内序）。
    /// `candidate_ids` 是唯一定义处，`candidates` 只是它的引用映射——两条路**序列逐条相同**。
    #[inline]
    pub(crate) fn candidate_ids(&self, p: Vec3) -> impl Iterator<Item = usize> + '_ {
        let empty: &[u32] = &[];
        let (list, all) = match &self.grid {
            Some(g) => {
                let cx = ((p.x - g.origin.x) * g.inv_bin).floor();
                let cy = ((p.y - g.origin.y) * g.inv_bin).floor();
                let cz = ((p.z - g.origin.z) * g.inv_bin).floor();
                if cx < 0.0 || cy < 0.0 || cz < 0.0 {
                    (empty, false)
                } else {
                    let (cx, cy, cz) = (cx as u32, cy as u32, cz as u32);
                    if cx >= g.dims[0] || cy >= g.dims[1] || cz >= g.dims[2] {
                        (empty, false)
                    } else {
                        let i = ((cx * g.dims[1] + cy) * g.dims[2] + cz) as usize;
                        (g.bins[i].as_slice(), false)
                    }
                }
            }
            None => (empty, true),
        };
        let len = if all { self.splats.len() } else { list.len() };
        (0..len).map(move |k| if all { k } else { list[k] as usize })
    }
}

#[cfg(test)]
mod tests {
    use crate::{GaussianSplatField, Splat};
    use vxl_phys_core::interop::MediumField as _;
    use vxl_phys_core::Vec3;

    /// 三核对称场（x 轴上等距；用于动量账与插值的解析对拍）。
    fn field3() -> GaussianSplatField {
        let mut f = GaussianSplatField::new(0.5);
        for k in 0..3 {
            f.push(Splat::isotropic(
                Vec3::new(k as f32 * 0.1, 0.0, 0.0),
                0.05,
                1.0,
            ));
        }
        f.medium_density = 2.0;
        f
    }

    /// ① **默认关金丝雀**：不开 two_way ⇒ deposit 空操作、sample 速度 = 常值、advance 零成本。
    #[test]
    fn two_way_off_is_a_no_op() {
        let mut f = field3();
        f.medium_velocity = Vec3::new(1.0, 0.0, 0.0);
        let before = f.sample(Vec3::ZERO);
        f.deposit(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0), 0.0, 0.0);
        f.advance(1.0 / 60.0);
        let after = f.sample(Vec3::ZERO);
        assert_eq!(before.velocity, after.velocity, "关档速度项不动");
        assert_eq!(after.velocity, Vec3::new(1.0, 0.0, 0.0), "关档 = 常值流速");
        assert!(f.kernel_velocities().is_empty(), "关档不建速度场");
        assert_eq!(f.absorbed_momentum(), Vec3::ZERO, "关档无沉积");
    }

    /// ② **动量账**：注入 `J` ⇒ `Σ m_k·Δv_k == J`（分摊口径自洽；容差用相对量）。
    #[test]
    fn deposit_conserves_momentum_by_kernel_mass() {
        let mut f = field3();
        f.set_two_way(true);
        let j = Vec3::new(0.7, -0.2, 0.05); // N·s
        f.deposit(Vec3::new(0.1, 0.0, 0.0), j, 0.0, 0.0);
        let mut sum = Vec3::ZERO;
        for (k, v) in f.kernel_velocities().iter().enumerate() {
            sum += *v * f.kernel_mass(k);
        }
        let rel = (sum - j).length() / j.length();
        assert!(rel < 1e-4, "Σ m_k·Δv_k 应 == J：rel={rel:.2e}（Σ={sum:?}）");
        assert_eq!(f.absorbed_momentum(), j, "审计量 = 注入动量");
    }

    /// ③ **速度插值**：单核场 ⇒ 核心处采样速度逐位 = 该核速度（e 加权唯一项）。
    /// （三核场里邻核权重 ≈0.135、插值是加权混合，**不会**退化为单核速度——断言要用单核场。）
    #[test]
    fn sampled_velocity_follows_kernel_velocity() {
        let mut f = GaussianSplatField::new(0.5);
        f.push(Splat::isotropic(Vec3::ZERO, 0.05, 1.0));
        f.medium_density = 2.0;
        f.set_two_way(true);
        f.deposit(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 0.0, 0.0);
        let v = f.sample(Vec3::ZERO).velocity;
        let vk = f.kernel_velocities()[0];
        assert_eq!(v, vk, "单核场核心处 ⇒ 插值退化为该核速度（逐位）");
        assert!(vk.x > 0.0, "该核确实被注入（用例非平凡）");
    }

    /// ④ **advance 衰减**：每 tick 乘 `damping`（解析对拍）。
    #[test]
    fn advance_decays_by_damping() {
        let mut f = field3();
        f.set_two_way(true);
        f.damping = 0.5;
        f.deposit(Vec3::new(0.1, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.0, 0.0);
        let v0 = f.kernel_velocities()[1].x;
        for _ in 0..3 {
            f.advance(1.0 / 60.0);
        }
        let v1 = f.kernel_velocities()[1].x;
        assert_eq!(v1, v0 * 0.5 * 0.5 * 0.5, "三次衰减 = ×0.125（f32 精确）");
    }

    /// ⑤ **平流精确**（切片 2）：一步后 `center += v₀·dt`、同一步内 `v ← v₀·damping`（逐位）。
    #[test]
    fn advance_advects_exactly_one_step() {
        let mut f = GaussianSplatField::new(0.5);
        f.push(Splat::isotropic(Vec3::ZERO, 0.05, 1.0));
        f.medium_density = 2.0;
        f.set_two_way(true);
        f.damping = 0.5;
        f.deposit(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 0.0, 0.0);
        let v0 = f.kernel_velocities()[0];
        let c0 = f.splats()[0].center;
        let dt = 1.0f32 / 60.0;
        f.advance(dt);
        assert_eq!(f.splats()[0].center, c0 + v0 * dt, "平流一步（逐位）");
        assert_eq!(f.kernel_velocities()[0], v0 * 0.5, "同一步内衰减 ×damping");
        assert!(v0.x > 0.0, "用例非平凡：该核确实有速度");
    }

    /// ⑥ **平流后网格一致性**（切片 2）：移动若干步 + `rebuild_grid()` ⇒ 加速路径与**全扫**
    /// 逐位相同（`candidate_ids` 是唯一定义处；网格只做候选裁剪，不改浮点求和序）。
    #[test]
    fn grid_agrees_with_brute_force_after_advection() {
        let mut f = GaussianSplatField::new(0.5);
        for k in 0..64 {
            let c = Vec3::new((k % 8) as f32 * 0.3, (k / 8) as f32 * 0.3, 0.0);
            f.push(Splat::isotropic(c, 0.35, 1.0));
        }
        f.medium_density = 1.0;
        f.set_two_way(true);
        for k in 0..64 {
            let c = Vec3::new((k % 8) as f32 * 0.3, (k / 8) as f32 * 0.3, 0.0);
            f.deposit(c, Vec3::new(0.02, -0.01, 0.0), 0.0, 0.0);
        }
        let c_before = f.splats()[0].center;
        for _ in 0..10 {
            f.advance(1.0 / 60.0);
        }
        assert_ne!(f.splats()[0].center, c_before, "平流确实发生（用例非平凡）");
        f.rebuild_grid();
        // 先取**网格路径**的读数，再借出网格走**全扫**对比，最后归还——免 `clone`（clone-gate：
        // 只准减）且两侧查询的是同一份 splat 状态（唯一差异 = 候选集来源）。
        let mut grid_got = Vec::new();
        for i in 0..40 {
            let p = Vec3::new(
                -0.4 + i as f32 * 0.11,
                0.35 + i as f32 * 0.06,
                i as f32 * 0.05 - 0.25,
            );
            grid_got.push((p, f.density_grad(p)));
        }
        let grid = f.grid.take(); // 借出（None ⇒ candidates 走全扫）
        for (p, (s1, g1)) in grid_got {
            let (s2, g2) = f.density_grad(p);
            assert_eq!(s1.to_bits(), s2.to_bits(), "σ 逐位（p={p:?}）");
            assert_eq!(
                (g1.x.to_bits(), g1.y.to_bits(), g1.z.to_bits()),
                (g2.x.to_bits(), g2.y.to_bits(), g2.z.to_bits()),
                "∇σ 逐位（p={p:?}）"
            );
        }
        f.grid = grid; // 归还
    }
}
