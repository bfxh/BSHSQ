//! voxel_provider：从 voxel.rs 按域拆出（纯搬移，语义未改）。
use super::*;
use vxl_phys_core::interop::InteropContact;

impl CollisionProvider for VoxelVolume {
    fn bounds(&self) -> Aabb {
        // 有占据格 → 用占据包围盒（更紧）；否则退化为网格范围。
        self.occupied_bounds().unwrap_or(Aabb {
            min: self.origin,
            max: self.origin
                + Vec3::new(self.nx as f32, self.ny as f32, self.nz as f32) * self.step,
        })
    }

    /// 表面最近点：SDF + 有限差分法线（步长 = 半格，确定性固定序）。
    fn closest_point(&self, p: Vec3) -> Option<SurfaceHit> {
        closest_point_voxel(self, p)
    }

    /// 盒 / 球 / 点：**委托 `voxel_contacts`**（门面那条路用的同一份数学）⇒ 逐位一致。
    /// ⚠️ 不覆写 `contacts_box` 会退回「8 角点采样」，与体素专用「6 面 × 5 采样」不是一回事。
    fn contacts_box(
        &self,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        contacts_box_voxel(self, half, pos, rot, skin, out)
    }

    fn contacts_sphere(
        &self,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        contacts_sphere_voxel(self, center, radius, skin, out)
    }

    fn contacts_point(&self, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        contacts_point_voxel(self, p, skin, out)
    }
}
