//! 场景件（本探针自己的水箱 / 水块 / 姿态）—— 从 `float_quiet_probe.rs` 拆出（god 门 800 行硬线）。

use vxl_phys::{BodyBuildExt, Quat, Vec3, World};

pub(crate) fn tank(w: &mut World) -> u32 {
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.25, 0.0, -1.25), 0.5, 5, 3, 5);
    vol.fill_box(Vec3::new(-1.25, 0.0, -1.25), Vec3::new(1.25, 1.0, 1.25));
    for ix in 0..5u32 {
        for iz in 0..5u32 {
            if ix == 2 && iz == 2 {
                continue;
            }
            vol.set(ix, 2, iz, true);
        }
    }
    w.add_voxel(vol)
}

pub(crate) fn water(layers: u32) -> vxl_phys_fluid::FluidSystem {
    vxl_phys_fluid::FluidSystem::new(
        vxl_phys_fluid::FluidConfig {
            boundary_layers: layers,
            ..vxl_phys_fluid::FluidConfig::default()
        },
        Vec3::new(-0.2, 1.05, -0.2),
        [8, 8, 8],
        0.05,
    )
}

pub(crate) fn quat_y_45() -> Quat {
    Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_4)
}
