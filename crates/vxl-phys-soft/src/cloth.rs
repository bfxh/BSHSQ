//! cloth：**布料最小闭环**（三角网 + XPBD 距离约束）——软体域第二块（绳索之后的切片）。
//!
//! 与 [`crate::rope::Rope`] 同一套 XPBD 机械（预测 → 逐子步 Gauss-Seidel 投影 → 速度回写），
//! 差别只在**拓扑**：绳 = 链式邻居对，布 = **三角网的唯一边集**（结构 + 剪切：四边形网格的
//! 对角线本就是三角形的共享边 ⇒ 唯一边集已覆盖两组；**弯曲约束未做**，属后续切片）。
//!
//! **质量 = 薄壳均分**：`m_i = ρ·t·Σ(incident A_tri)/3`——与窄相 `MeshStore::shell_props`
//! 的薄壳总质量同口径（两者对同一张网给出的总质量一致）。
//!
//! **本片边界（写清，不是漏）**：无接触（提供者/刚体/自碰撞都属后续切片）、无撕裂、无弯曲、
//! 无气动消费（`vxl-phys-aero` 的 `face_force` 已备，接线属后续切片）。
use crate::params::Stiffness;
use std::collections::HashSet;
use vxl_phys_core::Vec3;

/// **布片**（XPBD 三角网）：粒子 + 唯一边距离约束 + 逐子步投影。
#[derive(Clone)]
pub struct ClothSheet {
    pub pos: Vec<Vec3>,
    pub(crate) prev: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    /// `1/m`；钉住/退化粒子 = 0。
    pub inv_mass: Vec<f32>,
    /// 粒子质量（薄壳均分；判据/渲染用）。
    pub mass: Vec<f32>,
    /// 三角（索引指向 `pos`；注册序 = 确定性）。
    pub tris: Vec<[u32; 3]>,
    /// 唯一边（`[min, max]` 有序对；插入序 = 三角扫描序 ⇒ 确定性）。
    pub(crate) cons: Vec<[u32; 2]>,
    pub(crate) rest: Vec<f32>,
    pub(crate) lambda: Vec<f32>,
    /// 距离约束 compliance（m/N；`Stiffness::alpha`）。
    pub compliance: f32,
    /// 每 tick 子步数（判据收敛旋钮）。
    pub substeps: u32,
    /// 每子步投影遍数（Gauss-Seidel）。
    pub iterations: u32,
    /// 速度回写阻尼（1.0 = 无阻尼；同 [`crate::rope::Rope::damping`] 口径）。
    pub damping: f32,
}

impl ClothSheet {
    /// 由顶点 + 三角构一张布片：**唯一边** = 距离约束（rest = 当前长度 ⇒ 注册态即零应变态）；
    /// 粒子质量按薄壳均分。`thickness ≤ 0` 兜底 1e-3（与 `MeshStore::shell_props` 同惯例）。
    pub fn new(
        points: Vec<Vec3>,
        tris: Vec<[u32; 3]>,
        density: f32,
        thickness: f32,
        stiffness: Stiffness,
    ) -> Self {
        let n = points.len();
        let rho = if density > 0.0 { density } else { 1.0 };
        let t = if thickness > 0.0 { thickness } else { 1e-3 };
        // 唯一边（插入序 = 三角扫描序；HashSet 只做去重）。
        let mut cons: Vec<[u32; 2]> = Vec::new();
        let mut seen: HashSet<[u32; 2]> = HashSet::new();
        for tri in &tris {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                let key = if a < b { [a, b] } else { [b, a] };
                if seen.insert(key) {
                    cons.push(key);
                }
            }
        }
        let rest = cons
            .iter()
            .map(|[a, b]| (points[*b as usize] - points[*a as usize]).length())
            .collect();
        // 薄壳均分质量：每三角面积 ×ρt/3 记到三个角上。
        let mut mass = vec![0.0f32; n];
        for tri in &tris {
            let (a, b, c) = (
                points[tri[0] as usize],
                points[tri[1] as usize],
                points[tri[2] as usize],
            );
            let area = (b - a).cross(c - a).length() * 0.5;
            let share = rho * t * area / 3.0;
            for v in tri {
                mass[*v as usize] += share;
            }
        }
        let inv_mass = mass
            .iter()
            .map(|&m| if m > 0.0 { 1.0 / m } else { 0.0 })
            .collect();
        Self {
            pos: points,
            prev: Vec::new(),
            vel: vec![Vec3::ZERO; n],
            inv_mass,
            mass,
            tris,
            cons,
            rest,
            lambda: Vec::new(),
            compliance: stiffness.alpha(),
            substeps: 8,
            iterations: 1,
            damping: 1.0,
        }
    }

    pub fn particle_count(&self) -> usize {
        self.pos.len()
    }

    pub fn edge_count(&self) -> usize {
        self.cons.len()
    }

    /// 钉住/释放一个粒子（钉住 = `inv_mass = 0`，同 [`crate::rope::Rope::set_pinned`]）。
    pub fn set_pinned(&mut self, i: usize, pinned: bool) {
        if i >= self.pos.len() {
            return;
        }
        self.inv_mass[i] = if pinned || self.mass[i] <= 0.0 {
            0.0
        } else {
            1.0 / self.mass[i]
        };
    }

    /// 推进一个 `dt`（内部按 `substeps` 细分；**无接触**——本片边界，见模块文档）。
    pub fn step(&mut self, dt: f32, gravity: Vec3) {
        if self.prev.len() != self.pos.len() {
            self.prev = self.pos.clone();
        }
        if self.lambda.len() != self.cons.len() {
            self.lambda = vec![0.0; self.cons.len()];
        }
        let h = dt / self.substeps.max(1) as f32;
        for _ in 0..self.substeps.max(1) {
            self.substep(h, gravity);
        }
    }

    /// 单个子步：预测 → 距离约束（XPBD）→ 速度回写 + 阻尼。
    fn substep(&mut self, h: f32, gravity: Vec3) {
        let n = self.pos.len();
        // ① 预测（钉住粒子原地不动、速度清零；与 rope 同款）。
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                self.prev[i] = self.pos[i];
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] += gravity * h;
            self.prev[i] = self.pos[i];
            self.pos[i] += self.vel[i] * h;
        }
        // ② 距离约束（Gauss-Seidel；λ 每子步清零、子步内按迭代累加 = XPBD 口径）。
        for l in self.lambda.iter_mut() {
            *l = 0.0;
        }
        let a_tilde = self.compliance / (h * h);
        for _ in 0..self.iterations.max(1) {
            for (k, [i, j]) in self.cons.iter().enumerate() {
                let (i, j) = (*i as usize, *j as usize);
                let w = self.inv_mass[i] + self.inv_mass[j];
                if w <= 0.0 {
                    continue;
                }
                let d = self.pos[j] - self.pos[i];
                let len = d.length();
                if len < 1e-9 {
                    continue;
                }
                let dir = d * (1.0 / len);
                let c = len - self.rest[k];
                let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
                self.lambda[k] += dl;
                self.pos[i] -= dir * (self.inv_mass[i] * dl);
                self.pos[j] += dir * (self.inv_mass[j] * dl);
            }
        }
        // ③ 速度回写 + 阻尼。
        let inv_h = 1.0 / h;
        for i in 0..n {
            self.vel[i] = if self.inv_mass[i] == 0.0 {
                Vec3::ZERO
            } else {
                (self.pos[i] - self.prev[i]) * inv_h * self.damping
            };
        }
    }

    /// 当前**最大边应变** `max |len − rest| / rest`（判据仪器：收敛/悬垂判据用）。
    pub fn max_strain(&self) -> f32 {
        let mut worst = 0.0f32;
        for (k, [a, b]) in self.cons.iter().enumerate() {
            let len = (self.pos[*b as usize] - self.pos[*a as usize]).length();
            let s = ((len - self.rest[k]) / self.rest[k]).abs();
            worst = worst.max(s);
        }
        worst
    }
}
