//! providers：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

// **提供者条目**（从 `types.rs` 搬来：#6 迁入高度场后本枚举与 `providers.rs` 同生共死，
// 搬来让 `types.rs` 净缩 —— 那个文件**没有函数**，god 门"涨行必须缩函数"的账在那里无法支付）。
/// 提供者条目（统一 id 空间：体素体 / 高斯喷溅场…）。
pub enum ProviderEntry {
    Voxel(vxl_phys_terrain::voxel::VoxelVolume),
    /// **高斯喷溅场**（喷溅域的物理代理：隐式场提供者，见 `vxl-phys-splat`）。
    Splat(vxl_phys_splat::GaussianSplatField),
    /// **三角网格**（网格域：静态关卡几何，薄壳接触，见 `vxl-phys-terrain::mesh`）。
    Mesh(vxl_phys_terrain::mesh::TriMesh),
    /// **高度场**（地形：M2 余项迁入 provider 通道 —— 迁入后地形也拿到 provider 的
    /// **速度自适应接触带**，见 `OPEN-PROBLEMS.md` #6；`HeightField` 早已实现 `CollisionProvider`）。
    HeightField(vxl_phys_narrow::heightfield::HeightField),
}
use vxl_phys_core::interop::CollisionProvider;

/// 外部碰撞提供者集合（门面持有；实现 `interop::ProviderColliders` 供窄相查询）。
#[derive(Default)]
pub struct Providers {
    pub(crate) entries: Vec<ProviderEntry>,
    /// **转换窗口簿记**（效应键账 + 上一 tick 结果；定义见 `types.rs`，语义与入口见
    /// `world_step/conversion.rs`）。收在这里而不是 `World`：`world_struct.rs` 的成员位
    /// 顶在 god 门棘轮（23 成员、只准减）。
    pub(crate) conversion: crate::types::ConversionBook,
}

impl Providers {
    /// 注册体素体，返回其 id（= 注册序，全提供者共用一个 id 空间）。
    pub fn push(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Voxel(vol));
        id
    }

    /// 注册高斯喷溅场（同 id 空间）。
    pub fn push_splat(&mut self, field: vxl_phys_splat::GaussianSplatField) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Splat(field));
        id
    }

    /// 注册三角网格（静态关卡；同 id 空间）。
    pub fn push_mesh(&mut self, mesh: vxl_phys_terrain::mesh::TriMesh) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Mesh(mesh));
        id
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// provider(id) 的世界包围盒（宽相 AABB 供给；与 `CollisionProvider::bounds` 同义）。
    pub fn bounds(&self, id: u32) -> Option<Aabb> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => {
                use vxl_phys_core::interop::ProviderColliders;
                f.bounds(id)
            }
            ProviderEntry::HeightField(h) => Some(h.bounds()),
            ProviderEntry::Mesh(m) => {
                use vxl_phys_core::interop::ProviderColliders;
                m.bounds(id)
            }
        }
    }

    pub fn voxel(&self, id: u32) -> Option<&vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 网格只读视图（渲染/诊断）。
    pub fn mesh(&self, id: u32) -> Option<&vxl_phys_terrain::mesh::TriMesh> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Mesh(m) => Some(m),
            _ => None,
        }
    }

    pub fn voxel_mut(&mut self, id: u32) -> Option<&mut vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get_mut(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 喷溅场只读视图（渲染桥/诊断）。
    pub fn splat(&self, id: u32) -> Option<&vxl_phys_splat::GaussianSplatField> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Splat(f) => Some(f),
            _ => None,
        }
    }
}

// `ProviderColliders` 实现 + `push_heightfield`：整块搬到子模块（本文件受行数棘轮，
// #6 加第四类分派臂前必须先净缩）。
#[path = "providers_colliders.rs"]
mod colliders;
pub(crate) use colliders::add_heightfield;
