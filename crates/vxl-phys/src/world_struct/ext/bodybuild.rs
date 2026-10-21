//! **BodyBuildExt**：`World` 的**刚体/场/液体注册面**——本域**不往 `World` 添方法**。
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
use crate::{BodyId, HeightField, Joint, Material, Quat, Shape, Vec3, World};

/// **BodyBuildExt**：`World` 的刚体/场/液体注册面。
pub trait BodyBuildExt {
    fn add_static(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId;
    fn add_dynamic(&mut self, shape: Shape, position: Vec3, rotation: Quat, density: f32)
        -> BodyId;
    fn add_material(&mut self, material: Material) -> vxl_phys_core::MaterialId;
    fn add_heightfield(&mut self, hf: HeightField) -> BodyId;
    fn add_voxel(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> BodyId;
    fn add_mesh(&mut self, mesh: vxl_phys_terrain::mesh::TriMesh) -> BodyId;
    fn add_splat_field(&mut self, field: vxl_phys_splat::GaussianSplatField) -> BodyId;
    fn add_joint(&mut self, joint: Joint) -> u32;
    fn provider_id_of(&self, body: BodyId) -> Option<u32>;
    fn add_fluid(&mut self, sys: vxl_phys_fluid::FluidSystem, boundaries: &[u32]) -> usize;
    fn add_fluid_with_boundary_coupling(
        &mut self,
        sys: vxl_phys_fluid::FluidSystem,
        boundaries: &[u32],
    ) -> usize;
}

impl BodyBuildExt for World {
    fn add_static(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId {
        self.bodies.push_static(shape, position, rotation)
    }
    fn add_dynamic(
        &mut self,
        shape: Shape,
        position: Vec3,
        rotation: Quat,
        density: f32,
    ) -> BodyId {
        self.bodies.push_dynamic(shape, position, rotation, density)
    }
    /// 注册材质并返回 id（§4.4/§4.5）；`BodySet::set_material` 绑定到体。
    fn add_material(&mut self, material: Material) -> vxl_phys_core::MaterialId {
        self.bodies.add_material(material)
    }
    /// 加入高度场：地形账本 + 静态 marker 体（宽相经 marker AABB 参与对生成）。
    fn add_heightfield(&mut self, hf: HeightField) -> BodyId {
        let id = self.terrain.add(hf);
        let bounds = self.terrain.bounds(id).expect("just inserted");
        self.hf_bounds.push(bounds);
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::HeightField(id), pos, rot)
    }
    /// 注册一个**体素体**为碰撞提供者（新域接入的第一条路径，ROUTE §3/§5）：
    /// 加入提供者集合 + 静态 Marker 体 `Shape::Provider(id)`；宽相 AABB 由
    /// 提供者的 bounds 供给（与高度场同机制）。返回 marker 体 id。
    fn add_voxel(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> BodyId {
        let id = self.providers.push(vol);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }
    /// 注册**三角网格**（网格域：静态关卡几何）：返回 provider id + 静态 marker 体。
    /// 接触语义默认 `MeshSemantics::Solid`（`depth = −sd`，与高度场族同族）；单页薄壳（布片/
    /// 薄板/碎片）须在**入世界之前** `set_semantics(MeshSemantics::Shell)` 显式标记 ⇒ 推出方向
    /// 改按"查询点在哪一侧"取（理由与精度前提见 `vxl_phys_terrain::mesh` 模块文档）。
    fn add_mesh(&mut self, mut mesh: vxl_phys_terrain::mesh::TriMesh) -> BodyId {
        mesh.build_grid(); // 加速结构（桶边长 = max(均边, 0.5)）
        let id = self.providers.push_mesh(mesh);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }
    /// 注册**高斯喷溅场**（喷溅域）：返回 provider id + 生成静态 marker 体。
    /// 刚体（盒/球/外壳）经统一提供者通道与喷溅体接触（隐式场 `(τ−σ)/|∇σ|`）。
    fn add_splat_field(&mut self, mut field: vxl_phys_splat::GaussianSplatField) -> BodyId {
        field.rebuild_grid(); // 加速结构（与全扫逐位一致；核数不足则内部退回全扫）
        let id = self.providers.push_splat(field);
        self.provider_bounds.push(
            self.providers
                .bounds(id)
                .expect("just inserted provider bounds"),
        );
        let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
        self.bodies.push_static(Shape::Provider(id), pos, rot)
    }
    /// 添加一条**关节**（§2.5 关节约束族：球/转动/固定/棱柱/距离）。
    /// 锚点/轴均为体局部量；返回关节 id（关节序即求解序 ⇒ 确定性）。
    /// 限位与马达**均已接线**（2026-09-27 更正原"v1 未接"）；见 `vxl_phys_solver::joints`。
    fn add_joint(&mut self, joint: Joint) -> u32 {
        let n = self.bodies.len() as u32; // 入口拒绝（安全审计 F-03）：越界体号旧实现要到下一次 step() 才 panic
        assert!(
            joint.a < n && joint.b < n,
            "关节端点体号越界：a={} b={}（体数 {n}）",
            joint.a,
            joint.b
        );
        self.joints.add(joint)
    }
    /// 由 provider marker 体（[`Self::add_voxel`]/`add_mesh`/`add_splat_field`
    /// 返回的静态体）反查 provider id（流体边界列表用，见 [`Self::add_fluid`]）。
    fn provider_id_of(&self, body: BodyId) -> Option<u32> {
        match self.bodies.shape.get(body as usize)? {
            Shape::Provider(id) => Some(*id),
            _ => None,
        }
    }
    /// 注册**流体系统**（液体域，WCSPH）：返回流体索引（取用见 `World::fluids` 访问器）。
    /// `boundaries` = 边界碰撞的 provider id 列表（与体素/网格同一 id 空间，
    /// 经 [`Self::provider_id_of`] 由 marker 体反查）；切片1为单向耦合——
    /// 流体被刚体几何约束（驻留投影），对刚体无反作用（水推箱属切片2）。
    fn add_fluid(&mut self, mut sys: vxl_phys_fluid::FluidSystem, boundaries: &[u32]) -> usize {
        sys.set_boundaries(boundaries);
        self.fluids.push((sys, boundaries.to_vec(), None));
        self.fluid_boundary.two_b.push(false);
        self.fluids.len() - 1
    }
    /// 注册流体系统并**开启 2b（Akinci 两层边界粒子）**：每 tick 按**近域体**
    /// （静态 + 动态，形状受支持）重建边界粒子（`FluidSystem::set_boundary_particles`），
    /// 并把**反作用（力 + 绕体原点的力矩）**回流到体上（`medium_pass` 段③）。
    /// 被覆盖的体由 2b 接管 ⇒ **2a 的介质场浮力/阻力对这些体让位**（不叠加）；
    /// 形状不受支持或不在近域的体仍走 2a 粗档（既不叠加、也不留空）。
    ///
    /// 与 [`Self::add_fluid`] 的唯一区别就是这一条 ⇒ 默认档位、既有场景逐位不变。
    fn add_fluid_with_boundary_coupling(
        &mut self,
        sys: vxl_phys_fluid::FluidSystem,
        boundaries: &[u32],
    ) -> usize {
        let id = self.add_fluid(sys, boundaries);
        if let Some(f) = self.fluid_boundary.two_b.get_mut(id) {
            *f = true;
        }
        id
    }
}
