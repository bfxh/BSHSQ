//! **气动域的空间尾流 API**（`wake.rs` 的域侧入口）—— 扩展 trait，`World` 零新方法。
//!
//! 为什么是 trait：`world_mesh.rs` 受 god 门 file_lines 与 max_type_members 双棘轮
//! （95 行 / 9 成员，只准减）；按 `KinematicExt` / `SenseExt` 的既有先例，域侧 API 走
//! `impl Trait for World`，不进 `World` 的方法账。
use crate::{Vec3, World};

/// 空间尾流（障碍物下游的速度亏损区）的门面 API。
///
/// 语义与模型见 `vxl_phys_aero::wake`：每个源是沿风向的圆柱形亏损区，多源按注册序叠加。
/// 未开启气动域（没调 `set_aero`）时所有方法都是**无操作/零**，不静默建槽。
pub trait AeroWakeExt {
    /// 设置尾流源（替换整组）。空 = 均匀风场。返回是否已开启气动域。
    fn set_aero_wakes(&mut self, wakes: Vec<vxl_phys_aero::WakeSource>) -> bool;

    /// 某点的**实际风矢量**（含尾流亏损；未开气动域 ⇒ 零）。
    fn aero_wind_at(&self, x: Vec3) -> Vec3;
}

impl AeroWakeExt for World {
    fn set_aero_wakes(&mut self, wakes: Vec<vxl_phys_aero::WakeSource>) -> bool {
        match self.aero.as_mut() {
            Some(st) => {
                st.wakes = wakes;
                true
            }
            None => false,
        }
    }

    fn aero_wind_at(&self, x: Vec3) -> Vec3 {
        match self.aero.as_ref() {
            Some(st) => st.wind_at(x),
            None => Vec3::ZERO,
        }
    }
}
