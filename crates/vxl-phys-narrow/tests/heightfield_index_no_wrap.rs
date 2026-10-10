//! **高度场下标不许 u32 回绕**（2026-10-10 安全审计 F-01 的回归判据）。
//!
//! 旧实现把下标写成 `(iz * nx + ix) as usize`：`u32` 里先乘、release 无溢出检查 ⇒
//! **静默回绕**。审计实测 `nx=65536, nz=65537` 时只分配 65536 格，`set_height(5, 65536)`
//! 改写了 `(5, 0)`（另一个格！）、其余行越界 panic、更大尺寸 OOM。
//!
//! 判据两条（都不真分配：`HeightField` 的字段是 `pub`，直接构造超大维度）：
//! ① **响亮**：越界写必须 panic —— 不许静默写到别的格；
//! ② **不静默写错格**：那一次写不许"成功"。
//! 旧实现在 ① 上会**静默通过**（回绕后落进小数组 ⇒ 本测试红），这正是它要钉的东西。
use vxl_phys_narrow::heightfield::HeightField;

/// 构造一个"维度巨大但数组故意只有 4 格"的高度场（只测下标算术，不测分配）。
fn tiny_backed(nx: u32, nz: u32) -> HeightField {
    HeightField {
        origin_x: 0.0,
        origin_z: 0.0,
        nx,
        nz,
        spacing: 0.5,
        heights: vec![0.0; 4],
    }
}

#[test]
#[should_panic]
fn out_of_range_write_is_loud_not_silently_aliased() {
    // 旧实现：`(65536 * 65536 + 2) as usize` = `(0 + 2)` = 2 ⇒ **静默写进第 2 格**（数组只有 4 格，不 panic）。
    // 新实现：`65536usize * 65536 + 2` = 4 294 967 298 ⇒ 越界 panic（响亮）。
    let mut f = tiny_backed(65_536, 65_537);
    f.set_height(2, 65_536, 1.0);
}

#[test]
fn in_range_index_matches_usize_product() {
    // 正常档：`height_ix` 就是 `iz * nx + ix`（usize），且确实读到那一格。
    let mut f = HeightField::flat(0.0, 0.0, 4, 3, 0.5, 0.0);
    f.set_height(1, 2, 7.0);
    assert_eq!(f.height_ix(1, 2), 7.0);
    assert_eq!(f.height_ix(0, 0), 0.0);
    assert_eq!(f.heights.len(), 12);
}
