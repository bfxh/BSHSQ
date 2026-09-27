//! **软体域（绳索）通道** + 2b 边界的**成组状态**。
//!
//! **为什么 `FluidBoundary` 定义在这里**：`World` 是 god 门棘轮下的**记录型**结构（成员数只准减），
//! 加一个域就得腾一个成员位 ⇒ 把 2b 那一族的 3 个散字段收成一个结构（成员 23 → 22，再 +1 给绳索）。
//! 定义随域走、`World` 只留一个字段。
//!
//! **绳索通道**（T1，`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §8）：与液体域同款——每 tick **一次**
//! （体子步全部完成之后）推进；接触走**统一提供者通道**（`Providers` 实现 `ProviderColliders`，
//! 与窄相同一个 id 空间，见 `world_step.rs` 的 `fluid_pass` 同款段位）。**没有绳索的场景逐位不变**
//! （`ropes` 空 ⇒ `rope_pass` 首行短路）⇒ 默认档判据不受影响。
//!
//! **本片边界**：绳索的接触走**提供者**（地形）+ **刚体代理**（形状：Sphere/Box/Capsule）；
//! 自碰撞、`Compound`/`Cylinder`/`Cone`/`ConvexHull` 代理、体积/弯曲约束都属后续切片。
use super::*;

/// 2b（Akinci 边界粒子）那一族的成组状态（原为 `World` 的 3 个散字段：开关 / 暂存 / 覆盖集）。
#[derive(Default)]
pub struct FluidBoundary {
    /// 与 `fluids` 同序的 2b 开关：`true` = 该流体每 tick 重建边界粒子并回流反作用。
    pub two_b: Vec<bool>,
    /// 边界粒子生成的暂存 `(体 id, 形状, 位姿)`（复用免每 tick 分配）。
    pub scratch: Vec<(u32, Shape, vxl_phys_fluid::BodyPose)>,
    /// **覆盖集**（与 `bodies` 同序，每 tick 重建）：上次进了边界粒子集的体。
    pub covered: Vec<bool>,
}

impl FluidBoundary {
    /// 该流体是否开了 2b（越界一律 `false`）。**写成方法而不是在调用点展开链**：
    /// 展开式在原地超 `chain_width` 会被 rustfmt 折成 5 行 —— 而 `world_body.rs` 受尺寸棘轮
    /// （只准减），折行会让它"变胖"。
    pub fn is_two_b(&self, fluid: usize) -> bool {
        self.two_b.get(fluid).copied().unwrap_or(false)
    }
}

impl World {
    /// 注册一条绳索（`vxl_phys_soft::Rope`），返回其索引。
    pub fn add_rope(&mut self, rope: vxl_phys_soft::Rope) -> usize {
        self.ropes.push(rope);
        self.ropes.len() - 1
    }

    /// 已注册绳索（判据/渲染读 `pos` / `vel`）。
    pub fn ropes(&self) -> &[vxl_phys_soft::Rope] {
        &self.ropes
    }

    /// 第 `i` 条绳索。
    pub fn rope(&self, i: usize) -> Option<&vxl_phys_soft::Rope> {
        self.ropes.get(i)
    }

    /// **域通道**（每 tick 一次、体子步全部完成之后）：液体域 → 软体域。
    /// 顺序固定 ⇒ 确定性不受影响（两条通道互不读对方状态）。
    pub(crate) fn domain_pass(&mut self) {
        self.fluid_pass();
        self.rope_pass();
    }

    /// **软体域通道**：每条绳索按自身 `substeps` 推进一个 `config.dt`；接触走统一提供者通道
    /// （`0..providers.len()` 全量 id）与**刚体代理**（粒子↔刚体，Akinci 式最小实现）。
    /// 反作用回填：`bodies.linvel += body_dv`（速度增量）、`bodies.torque += τ/dt`（角冲量 → 力矩口径，
    /// 与 2b 流体反作用同段位）。**空集 ⇒ 零成本短路**。
    pub(crate) fn rope_pass(&mut self) {
        if self.ropes.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        // 刚体代理（每 tick 重建：体在动）。**地形类形状跳过**（`Provider`/`HeightField` 走提供者
        // 通道；`Compound` 本片不支持 ⇒ 直接跳过，别让它悄悄不清碰）。
        self.rope_proxies.clear();
        for i in 0..self.bodies.len() {
            let shape = self.bodies.shape[i];
            if matches!(
                shape,
                Shape::Provider(_) | Shape::HeightField(_) | Shape::Compound { .. }
            ) {
                continue;
            }
            self.rope_proxies.push(vxl_phys_soft::RigidProxy {
                body: i as u32,
                shape,
                pos: self.bodies.position[i],
                rot: self.bodies.rot(i),
                linvel: self.bodies.linvel[i],
                inv_mass: if self.bodies.is_dynamic(i) {
                    self.bodies.inv_mass[i]
                } else {
                    0.0
                },
            });
        }
        let proxies = std::mem::take(&mut self.rope_proxies);
        let providers = &self.providers;
        for rope in &mut self.ropes {
            rope.step(dt, gravity, providers, count, &proxies);
            for (j, p) in proxies.iter().enumerate() {
                if p.inv_mass <= 0.0 {
                    continue; // 静态体不收反作用
                }
                if let Some(dv) = rope.body_dv.get(j) {
                    self.bodies.linvel[p.body as usize] += *dv;
                }
                // **位置口径回填**（`Rope::body_dx`，§8.4.9）：只回速度会让体每 tick 按 `v·dt` 走过的
                // `g·dt²` 一去不回（实测下沉 ≈ 整漏的 83%）⇒ 把钳位压掉的那一份位置补上。
                // **速度侧不动**（`body_dx` 是从位置口径算出来的）⇒ 不会把修正反射成速度。
                if let Some(dx) = rope.body_dx.get(j) {
                    self.bodies.position[p.body as usize] += *dx;
                }
            }
            // **角反作用：本片不施加**（§8.4.9/§8.4.10 实测）。理由不是"力矩算错了"，而是
            // **接触模型看不见转动**：`body_disp` 只跟踪平移、摩擦的滑移用 `b.linvel` 而非
            // `linvel + ω×r`、`crossed_face` 用的是**冻结的** `rot` ⇒ 把角动量回填给体以后，
            // 体转起来的运动会**完全落在模型之外**。实测（1 kg 薄盒压在绳上）：`hit = 1`
            // （单点接触、力臂 ≈ 0.3 m、`I_zz ≈ 0.031`）⇒ `|ω|` 一 tick 就到 **5~8 rad/s**
            // ⇒ 接触立刻丢失（`hit = 0`）⇒ 盒子被甩下去（门面 1800 tick y = −2420）。
            // **自扮引擎侧一直不读 `reactions`**（只吃 `body_dv`）⇒ 它托得住（y@1800 = +0.995），
            // 这正是两侧差异的最后一块。**要恢复本行**必须先做"转动感知的代理"（见 §8.4.10）：
            // ⚠️ 恢复时口径要一并修：`integrate_velocities` 每**子步**消费并清零 `force/torque`，
            // 而这里在所有子步**之后**才注入 ⇒ 实收角冲量 = `τ/substeps`（默认 2 ⇒ **差 2×**）。
            let _ = &rope.reactions;
        }
        self.rope_proxies = proxies;
    }
}
