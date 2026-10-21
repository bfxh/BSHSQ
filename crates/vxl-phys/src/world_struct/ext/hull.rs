//! **HullExt**：`World` 的**凸包/复合体面**——本域**不往 `World` 添方法**。
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

/// **HullExt**：`World` 的凸包/复合体面。
pub trait HullExt {
    fn add_hull(&mut self, points: Vec<Vec3>) -> u32;
    fn spawn_hull_pieces(
        &mut self,
        hull: u32,
        seeds: &[Vec3],
        pos: Vec3,
        rot: Quat,
        density: f32,
    ) -> Vec<u32>;
    fn hull_points(&self, hull: u32) -> &[Vec3];
    fn spawn_hull_body(&mut self, hull: u32, pos: Vec3, rot: Quat, density: f32) -> u32;
    fn add_compound(&mut self, children: Vec<CompoundChild>) -> u32;
    fn compound_half_extents(&self, compound: u32) -> Vec3;
    fn compound_children(&self, compound: u32) -> &[CompoundChild];
    fn spawn_compound_body(&mut self, compound: u32, pos: Vec3, rot: Quat, density: f32) -> u32;
    fn add_compound_static(&mut self, compound: u32, pos: Vec3, rot: Quat) -> u32;
}

impl HullExt for World {
    /// 注册凸体外壳（点云，局部坐标）→ hull id。
    fn add_hull(&mut self, points: Vec<Vec3>) -> u32 {
        self.narrow.add_hull(points)
    }
    /// **凸体预断裂**（「更一般的凸体/网格切割」的凸体侧）：原壳 ✕ 种子 ⇒
    /// 逐格生成凸碎块体（tiling 原体，局部坐标；缺口 = 种子在体外）。
    /// 典型用法：装载期把完整壳换成一堆碎块体，撞击即自然散架。
    fn spawn_hull_pieces(
        &mut self,
        hull: u32,
        seeds: &[Vec3],
        pos: Vec3,
        rot: Quat,
        density: f32,
    ) -> Vec<u32> {
        let Some(regions) = self.narrow.fracture_hull(hull, seeds) else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(regions.len());
        for region in regions {
            if region.len() < 4 {
                continue; // 退化格（无体积）
            }
            let id = self.narrow.add_hull(region);
            out.push(self.spawn_hull_body(id, pos, rot, density));
        }
        out
    }
    /// 外壳点云（局部；渲染/转储用）。
    fn hull_points(&self, hull: u32) -> &[Vec3] {
        self.narrow.hull_points(hull)
    }
    /// 生成**凸体外壳动态体**（多边形域）：点云已在 `add_hull` 注册。
    /// 半长取点云局部 AABB（宽相/惯量近似）；质量按 AABB 盒密度。
    fn spawn_hull_body(&mut self, hull: u32, pos: Vec3, rot: Quat, density: f32) -> u32 {
        let half = self.narrow.hull_half_extents(hull);
        self.bodies.push_dynamic(
            Shape::ConvexHull { hull, half },
            pos,
            rot,
            density.max(1e-3),
        )
    }
    /// 注册复合体（子形状 = 形状 + 局部平移/旋转）→ compound id。
    ///
    /// **入口拒绝**（安全审计 F-13）：子序号走 `u16` 通道（`tag_child_features` 的
    /// `space`/`feature` 两位共用它），子序号 ≥ 65536 回绕成 0 ⇒ 暖启动键与"非复合体"碰撞。
    fn add_compound(&mut self, children: Vec<CompoundChild>) -> u32 {
        assert!(
            children.iter().any(|c| !matches!(c.shape, Shape::Compound { .. })) && children.len() <= u16::MAX as usize,
            "复合体子形状数 {} 不合法（空 / 只含嵌套复合体（会被丢弃 ⇒ 空壳、AABB 退化 ⇒ 病态惯量，审计 F-10）；> u16::MAX 会回绕，审计 F-13）",
            children.len()
        );
        self.narrow.add_compound(children)
    }
    /// 子形状局部 AABB 并集半长（宽相/惯量近似用；空复合体 = ZERO）。
    fn compound_half_extents(&self, compound: u32) -> Vec3 {
        self.narrow.compound_half_extents(compound)
    }
    /// 复合体子形状表（诊断/外部管线用）。
    fn compound_children(&self, compound: u32) -> &[CompoundChild] {
        self.narrow.compound_children(compound)
    }
    /// 生成**复合体动态体**：子形状已在 `add_compound` 注册。
    /// 半长取子形状局部 AABB 并集（保守：子半长按包围球）；质量暂按该并集盒近似
    /// （精确并集 = 按子形状质量求和 + 平行轴，待接；见 `TECH-SURVEY.md` A9 ④）。
    fn spawn_compound_body(&mut self, compound: u32, pos: Vec3, rot: Quat, density: f32) -> u32 {
        let half = self.narrow.compound_half_extents(compound);
        let density = density.max(1e-3);
        let id = self
            .bodies
            .push_dynamic(Shape::Compound { compound, half }, pos, rot, density);
        // 质量/惯量按**子形状精确求和**覆写（`push_dynamic` 里只有并集 AABB 兜底近似）。
        if let Some((inv_mass, inv_inertia)) =
            compound_mass_props(self.narrow.compound_children(compound), density)
        {
            let i = id as usize;
            self.bodies.inv_mass[i] = inv_mass;
            self.bodies.local_inv_inertia[i] = inv_inertia;
        }
        id
    }
    /// 生成**复合体静态体**（同 `add_static` 语义）。
    fn add_compound_static(&mut self, compound: u32, pos: Vec3, rot: Quat) -> u32 {
        let half = self.narrow.compound_half_extents(compound);
        self.bodies
            .push_static(Shape::Compound { compound, half }, pos, rot)
    }
}
