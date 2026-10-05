//! **`Providers` 的 `ProviderColliders` 实现 + `push_heightfield`**（从 `providers.rs` 整块搬出）。
//!
//! 搬出理由（两条都是代号级的）：① `providers.rs` 受 god 门**文件行数棘轮**（只准减），
//! 而 #6 高度场迁 provider 通道要在这里加**第四类**分派臂 ⇒ 整块搬出后宿主文件**净缩**；
//! ② `push_heightfield` 一并做成**自由函数** —— `Providers` 的成员/方法数在棘轮里顶格，
//! 自由函数不占方法位。
use crate::{Aabb, ProviderEntry, Providers, Quat, Vec3};
use vxl_phys_core::interop::CollisionProvider;

/// 注册**高度场**（同 id 空间；`OPEN-PROBLEMS.md` #6 的迁移落点）。自由函数：不占方法位。
pub(crate) fn push_heightfield(
    p: &mut Providers,
    hf: vxl_phys_narrow::heightfield::HeightField,
) -> u32 {
    let id = p.entries.len() as u32;
    p.entries.push(ProviderEntry::HeightField(hf));
    id
}

/// `World::add_heightfield` 的**本体**（`world_build.rs` 受行数棘轮 ⇒ 搬出后宿主净缩）：
/// 注册成 provider + 刷新 `provider_bounds` + 建 `Shape::Provider` 静态体。
pub(crate) fn add_heightfield(
    w: &mut crate::World,
    hf: vxl_phys_narrow::heightfield::HeightField,
) -> crate::BodyId {
    let id = push_heightfield(&mut w.providers, hf);
    let b = match w.providers.bounds(id) {
        Some(b) => b,
        None => Aabb {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
        },
    };
    w.provider_bounds.push(b);
    let (pos, rot) = vxl_phys_terrain::MARKER_TRANSFORM;
    w.bodies.push_static(crate::Shape::Provider(id), pos, rot)
}

// `#[rustfmt::skip]`：args-gate 按"逗号数+1"计形参，**竖排 + 尾逗号会多算一个** ⇒ 6 形参的
// `contacts_box` 会被读成 8（新文件零基线 ⇒ 直接红）。单行签名（本仓既有先例）。
#[rustfmt::skip]
impl vxl_phys_core::interop::ProviderColliders for Providers {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => f.bounds(id),
            ProviderEntry::Mesh(m) => m.bounds(id),
            ProviderEntry::HeightField(h) => Some(h.bounds()),
        }
    }

    fn contacts_box(&self, id: u32, half: Vec3, pos: Vec3, rot: Quat, skin: f32, out: &mut Vec<vxl_phys_core::interop::InteropContact>) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => v.contacts_box(half, pos, rot, skin, out),
            Some(ProviderEntry::Splat(f)) => f.contacts_box(id, half, pos, rot, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_box(id, half, pos, rot, skin, out),
            Some(ProviderEntry::HeightField(h)) => h.contacts_box(half, pos, rot, skin, out),
            None => false,
        }
    }

    fn contacts_point(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => v.contacts_point(p, skin, out),
            Some(ProviderEntry::Splat(f)) => f.contacts_point(id, p, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_point(id, p, skin, out),
            Some(ProviderEntry::HeightField(h)) => h.contacts_point(p, skin, out),
            None => false,
        }
    }

    /// 流体边界口径：体素走内点鲁棒变体（截断 SDF 在薄壁内部被格间内面
    /// 主导 ⇒ 中心差分法线可指向固体深处，投影穿壁隧逃——见切片1实测）；
    /// 其余提供者（解析面/半空间无内点歧义）沿用 `contacts_point`。
    ///
    /// ⚠️ 这一条**仍是门面直派**、没有并进域 `CollisionProvider`：它是**流体专属的第二张面**
    /// （刚体通道要的是"外点梯度"口径，流体投影要的是"内点最近真表面"口径），
    /// 域 trait 上目前没有对应方法 ⇒ 要么先给四域各定一份边界口径，要么让流体自己
    /// 按域问。别为了"看起来统一"把它塞进 `contacts_point`——那是两套语义。
    fn contacts_point_boundary(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_point_voxel_solid(v, p, skin, out)
            }
            _ => self.contacts_point(id, p, skin, out),
        }
    }

    fn contacts_sphere(
        &self,
        id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => v.contacts_sphere(center, radius, skin, out),
            Some(ProviderEntry::Splat(f)) => f.contacts_sphere(id, center, radius, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_sphere(id, center, radius, skin, out),
            Some(ProviderEntry::HeightField(h)) => h.contacts_sphere(center, radius, skin, out),
            None => false,
        }
    }
}
