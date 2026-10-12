//! `vxl_phys::prelude` —— **外部调用者的单一入口**（`use vxl_phys::prelude::*;` 一行为准）。
//!
//! 为什么要有它：本仓把能力按域拆成 20+ 个 crate，内部各有各的模块路径；集成方不该为了
//! "建世界 + 放个盒子 + 跑一步"去查五处 `use`。prelude 只列**稳定入口面**：
//!
//! - 世界与配置：[`World`]、[`PhysConfig`]、[`Preset`]；
//! - 数学与形状：[`Vec3`]、[`Quat`]、[`Shape`]、[`Aabb`]；
//! - 体与材质：[`BodyId`]、[`BodySet`]、[`BodyType`]、[`Material`]、[`FrictionModel`]；
//! - 关节：[`Joint`]、[`JointKind`]；
//! - 地形/网格：[`HeightField`]、[`CompoundChild`]；
//! - **域扩展 trait**（域 API 靠 trait 挂到 `World` 上，不往 `World` 加方法）：
//!   [`DestructionExt`]（破坏）、[`VoxelConversionExt`]（体素↔多边形）、[`KinematicExt`]（运动学体）。
//!
//! **不在 prelude**（有意）：内部策略类型（`ImpulseSolver`/`DefaultNarrowPhase`/`JointSet`…）、
//! 回放与哈希（`Recorder`/`StateHash`——要显式引入，见 `QUICKSTART.md` 的确定性一节）、
//! 每个域自己的调参类型。
//!
//! 写法约定：**全部具名再导出**（不用 `pub use ...::*`）——本仓的 `glob-gate` 对新文件的
//! glob 是硬判，而具名导出对调用者一样方便（调用方那一侧的 `use vxl_phys::prelude::*;` 才是
//! 他要的那颗星）。

pub use crate::world_kinematic::KinematicExt;
pub use crate::world_step::{conversion::VoxelConversionExt, destruction::DestructionExt};
pub use crate::{
    Aabb, BodyId, BodySet, BodyType, CompoundChild, FrictionModel, HeightField, Joint, JointKind,
    Material, PhysConfig, Preset, Quat, Shape, Vec3, World,
};
