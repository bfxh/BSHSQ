//! **DestructionOpsExt**：`World` 的**破坏域（弹坑/碎裂/碎块）**——本域**不往 `World` 添方法**。
//!
//! **为什么**：`god.gate.json::god_pairs["crates/vxl-phys::World"]` 把 `World` 登记为
//! **已登记债务（非豁免）**，处置写的是「**域侧 API 拆成扩展 trait**（`impl FluidExt for World`
//! 等），门面只留管线组装」，判据是「新加一域**不必再往 `World` 添方法**」。
//!
//! **先例**（同一判据的既有落法）：`world_step/destruction.rs` 的 `DestructionExt`、
//! `world_step/conversion.rs` 的 `VoxelConversionExt`、`vxl-phys-sense` 的 `SenseExt`、
//! `world_aero_wake.rs` 的 `AeroWakeExt`、`world_kinematic.rs` 的 `KinematicExt`。
//!
//! **门的口径**：`scripts/god_gate.py::brace_metrics` 明确跳过 `impl Trait for Type`
//! （`god_gate.py:216`）⇒ 搬进 trait 后 `World` 的方法数**不计入本类型**。
//!
//! **数值与顺序一字不改**（纯搬移：方法体逐字搬过来，未改一个算式、一个遍历序）。
use crate::*;

/// **DestructionOpsExt**：`World` 的破坏域（弹坑/碎裂/碎块）。
pub trait DestructionOpsExt {
    fn spawn_box_debris(&mut self, id: u32, min: Vec3, max: Vec3, density: f32) -> usize;
    fn spawn_box_debris_vel(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        density: f32,
        vel: Vec3,
    ) -> usize;
    fn carve_sphere(&mut self, id: u32, center: Vec3, radius: f32, density: f32) -> usize;
    fn fracture_voronoi(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        seeds: &[Vec3],
        density: f32,
    ) -> usize;
    fn apply_impact_destruction(&mut self, id: u32, speed_threshold: f32, density: f32) -> usize;
}

impl DestructionOpsExt for World {
    /// **破坏（M3 第一块）**：把体素体盒域内的占据格转为**刚体碎块**
    /// （贪心合并成盒 → 逐个动态体），并从体素体里移除。返回碎块数。
    /// 确定性：提取顺序 = 固定扫描序（见 `VoxelVolume::extract_boxes`）；
    /// 碎块质量 = `density × 8·hx·hy·hz`（⚠️ 2026-09-27 前把该质量当**密度**传给 `push_dynamic`——它内部会再乘体积 ⇒ 实际质量是 `ρ·V²`（**尺寸相关**；V=1 恰好掩盖），故下面反解密度 `d`）。
    fn spawn_box_debris(&mut self, id: u32, min: Vec3, max: Vec3, density: f32) -> usize {
        self.spawn_box_debris_vel(id, min, max, density, Vec3::ZERO)
    }
    /// 同 [`spawn_box_debris`](Self::spawn_box_debris)，但碎块带初速 `vel`（冲击破坏用：碎块继承部分冲击速度）。
    fn spawn_box_debris_vel(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        density: f32,
        vel: Vec3,
    ) -> usize {
        // **I3 效应键**：本 tick 该域已被 `Conversion` 占用 ⇒ 拒绝提取（域原地不动），
        // 不静默"提了 0 个格"。反向顺序的冲突（先请求转换、后挖格）在这里显式挡住。
        if self.claim_extraction_effect(id).is_err() {
            return 0;
        }
        let Some(vol) = self.providers.voxel_mut(id) else {
            return 0;
        };
        let boxes = vol.extract_boxes(min, max);
        let n = boxes.len();
        for (c, h) in boxes {
            let d = (1e-3 / (8.0 * h.x * h.y * h.z)).max(density).max(1e-3);
            let b = self
                .bodies
                .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, d);
            self.bodies.linvel[b as usize] = vel;
        }
        self.refresh_provider_bounds();
        n
    }
    /// **球域挖洞**（任意形状切割第一步；爆炸/弹坑形态）：提取球内格 → 碎块 + 移除。
    /// 返回碎块数。确定性：格心判据 + 固定贪心扫描序（见 `extract_sphere`）。
    fn carve_sphere(&mut self, id: u32, center: Vec3, radius: f32, density: f32) -> usize {
        if self.claim_extraction_effect(id).is_err() {
            return 0;
        }
        let Some(vol) = self.providers.voxel_mut(id) else {
            return 0;
        };
        let boxes = vol.extract_sphere(center, radius);
        let n = boxes.len();
        for (c, h) in boxes {
            let d = (1e-3 / (8.0 * h.x * h.y * h.z)).max(density).max(1e-3);
            self.bodies
                .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, d);
        }
        self.refresh_provider_bounds();
        n
    }
    /// **Voronoi 预断裂（M3）**：把域内体素按「距最近种子」分块（划分，守恒），
    /// 每块提取为刚体碎块并移除。`seeds` 由调用方给（可用
    /// `vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered` 生成确定性抖动种子）。
    /// 返回碎块总数。
    ///
    /// **预算**（安全审计 F-14）：种子数与碎块数都封顶**默认档 `B1K` 的容量**（同破坏管线默认
    /// 档）；超预算的盒**回填成体素**（不丢几何），返回值如实给本次生成数。破坏策略路径
    /// （`DestructionExt`）用 `DestructionConfig::budget` 调 `voronoi_fracture::extract`，不受此处默认档限制。
    fn fracture_voronoi(
        &mut self,
        id: u32,
        min: Vec3,
        max: Vec3,
        seeds: &[Vec3],
        density: f32,
    ) -> usize {
        crate::world_body::voronoi_fracture::extract(
            self,
            id,
            vxl_phys_core::Aabb { min, max },
            seeds,
            density,
            crate::world_body::voronoi_fracture::DEFAULT_BUDGET,
        )
    }
    /// **M3 冲击破坏**（调用方在每次 `step()` 之后调用）：对指定 provider，
    /// 把本 tick **解算前**记录的冲击（沿接触法向的接近速度 ≥ `speed_threshold`）
    /// 转成弹坑：球心 = 接触点 + 冲击方向 × 1.05r，半径随冲击速度增长（0.25..0.9 m），
    /// 碎块**静止生成**（引擎不凭空造动量）。返回生成的碎块数。
    /// 确定性：按记录序处理、挖域为轴对齐盒、提取顺序固定（见 `extract_boxes`）。
    ///
    /// 记录在窄相之后、求解之前快照（`record_impacts`）：管线读"解算后速度"会
    /// 漏掉已被子步解掉的冲击（这正是 2 子步配置下炮弹不再挖洞的根因）。
    fn apply_impact_destruction(&mut self, id: u32, speed_threshold: f32, density: f32) -> usize {
        // 先在只读扫描里收集「挖点」（按冲击记录序），再逐个挖 —— 保持确定性。
        let mut digs: Vec<(Vec3, f32, Vec3)> = Vec::new();
        for rec in &self.impacts {
            if rec.provider != id || !self.bodies.is_dynamic(rec.body as usize) {
                continue;
            }
            // **冲击判据 = 沿接触法向的接近速度**（不是体速！）：贴地滑行是切向
            // 运动，体速很大但不该破坏（实测：8 m/s 滑行把脚下地板挖穿）。
            let approach = rec.approach;
            if approach < speed_threshold {
                continue;
            }
            let v = rec.velocity;
            let sp = approach;
            // 接触点 = 记录时的流形点均值（确定性；已在 record_impacts 算好）
            let c = rec.point;
            digs.push((c, sp, v));
        }
        let mut total = 0usize;
        for (c, sp, v) in digs {
            // 挖出半径随**实际冲击速度**增长（钳到 0.25..0.9 m）
            let r = (0.2 + 0.06 * sp).clamp(0.25, 0.9);
            let dir = if v.length_squared() > 1e-9 {
                v.normalize()
            } else {
                Vec3::ZERO
            };
            // **弹坑 = 球域**（任意形状切割第一步）：球心 = 接触点 + 冲击方向 × 1.05r
            // ⇒ 坑的近缘落在接触点**外侧 0.05r**、整体在材料里（薄墙会被打穿——物理如此），
            // 与冲击体**不重叠**⇒ 不需要再按冲击体的包围球半径外推。
            // （2026-10-06 删掉的那段 `reach` 正是死在这里：算完没用，注释还宣称了一个
            //   没实现的行为。）
            let center = c + dir * (r * 1.05);
            // 碎块**静止生成**（初速留给调用方用 `spawn_box_debris_vel` 显式给；
            // 引擎不凭空造动量——「继承半速」实测是能量源，已否）。
            total += self.carve_sphere(id, center, r, density);
        }
        total
    }
}
