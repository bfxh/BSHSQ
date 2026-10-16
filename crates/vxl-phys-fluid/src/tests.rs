//! tests：从 lib.rs 拆出的单元测试 —— 本文件只留**测试替身与共享构件**，
//! 用例本体按主题分入 tests/ 子模块。
//!
//! 每个子模块声明都自带显式 #[path]：同一目录里 #[path] 会传染后续声明，
//! 漏写会让 rustc 去错误位置找文件（E0583）。
//!
//! 替身不随用例搬走，是门禁口径使然：contacts_box 是 trait 固定签名，
//! 形参数已登记在 args 棘轮基线（tests.rs = 2）——搬进新文件会计入「新文件」判红。

use super::*;
use vxl_phys_core::Aabb;

/// 测试替身：半空间 y<0 实心（表面 y=0，外法线 +y）。
/// 语义对齐体素点查询：depth = skin − sdf；depth < −skin 时仍返回支持。
/// x/z 取大范围（±50）：落水飞溅的角部粒子会在无压区滑行数米，
/// 面积太小会被「滑出测试地板边缘」误判成穿隧。
pub(crate) struct FloorY;
impl ProviderColliders for FloorY {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(Aabb {
            min: Vec3::new(-50.0, -1.0, -50.0),
            max: Vec3::new(50.0, 0.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: vxl_phys_core::Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        let depth = skin - p.y; // sdf = p.y
        if depth < -skin {
            return true; // 支持查询，但不在接触带内
        }
        out.push(InteropContact {
            point: Vec3::new(p.x, 0.0, p.z),
            normal: Vec3::Y,
            depth,
            feature: 0,
        });
        true
    }
}

/// 测试替身：矩形水槽——地板 y=0 + 四面墙（内壁 x/z = ±0.2；7³ 柱
/// 沉底摊平后仍保有 ~5 个核深的液柱，静水梯度可测）。
/// 墙体沿法线向外无限延伸（半空间式），无「翻墙」另一侧。
/// 面序：地板、左、右、后、前；sign = 内向法线的轴符号。
pub(crate) struct Tank;
impl Tank {
    const FACES: [(usize, f32, f32); 5] = [
        (1, 0.0, 1.0),
        (0, -0.2, 1.0),
        (0, 0.2, -1.0),
        (2, -0.2, 1.0),
        (2, 0.2, -1.0),
    ];
}
impl ProviderColliders for Tank {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        // 半空间几何的提示框必须覆盖固体全域：墙外/地板下仍是固体。
        // 若只框内腔，飞溅粒子越出提示框（如越过开放顶部落到 x<−0.31）
        // 后预滤会跳过一切查询——含地板——永久自由落体（实测 ESC：
        // x=−1.1 处 v=(·,−9.58,·) 匀加速坠穿）。对齐 FloorY 的 ±50。
        Some(Aabb {
            min: Vec3::new(-50.0, -50.0, -50.0),
            max: Vec3::new(50.0, 50.0, 50.0),
        })
    }
    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: vxl_phys_core::Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
    fn contacts_point(&self, _id: u32, p: Vec3, probe: f32, out: &mut Vec<InteropContact>) -> bool {
        for &(axis, plane, sign) in &Self::FACES {
            // 半空间墙在全部 y 生效：墙面下方仍是墙固体（无「墙脚缝」）。
            // 若 y < 0 关墙，贴壁底层（近压抖沉到 y ∈ [−pen, 0]）会在墙
            // 关闭的子步里从墙线下横滑出去，地板预滤（余量 h）一丢即自由落体。
            let comp = [p.x, p.y, p.z][axis];
            let sdf = (comp - plane) * sign;
            if sdf > 2.0 * probe {
                continue;
            }
            let point = match axis {
                0 => Vec3::new(plane, p.y, p.z),
                1 => Vec3::new(p.x, plane, p.z),
                _ => Vec3::new(p.x, p.y, plane),
            };
            let normal = match axis {
                0 => Vec3::new(sign, 0.0, 0.0),
                1 => Vec3::new(0.0, sign, 0.0),
                _ => Vec3::new(0.0, 0.0, sign),
            };
            out.push(InteropContact {
                point,
                normal,
                depth: probe - sdf,
                feature: 0,
            });
        }
        true
    }
}

/// 静止 5³ 晶格块（标定间距 = h/2）。
fn still_block() -> FluidSystem {
    let cfg = FluidConfig {
        gravity: Vec3::ZERO,
        substeps: 2,
        ..FluidConfig::default()
    };
    FluidSystem::new(cfg, Vec3::new(-0.1, 0.2, -0.1), [5, 5, 5], 0.05)
}

/// 体面速度静止的静态体（本文件只把"体"当几何用）。
fn still_pose(pos: Vec3) -> BodyPose {
    BodyPose {
        pos,
        rot: Quat::IDENTITY,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
    }
}

#[path = "tests/boundary.rs"]
mod boundary;
#[path = "tests/determinism.rs"]
mod determinism;
#[path = "tests/hydrostatics.rs"]
mod hydrostatics;
#[path = "tests/lattice.rs"]
mod lattice;
#[path = "tests/medium.rs"]
mod medium;
#[path = "tests/reaction_sampling.rs"]
mod reaction_sampling;
