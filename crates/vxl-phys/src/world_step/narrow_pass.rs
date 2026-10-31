//! 窄相调用点：宿主窄相 +（探针开着时）**反事实预测趟**。
//!
//! 单独成文件是门禁约束：`world_step.rs` 已卡在 god 门基线上（不许变胖），
//! 这段搬出来后调用点从 9 行降到 1 行。
//!
//! ⚠️ 反事实趟**必须与真窄相同点（解算前、位置积分前）执行**：挪到子步末会拿到
//! 晚一个子步的位姿，实测"新点覆盖率"从 0.01% 虚高到 55%（2026-10-31）。
use vxl_phys_narrow::NarrowPhase;

use super::contact_probe;
use crate::World;

pub(crate) fn run(w: &mut World, pairs: &[(u32, u32)], first: bool) {
    w.narrow.collide(
        &w.bodies,
        pairs,
        w.terrain.slice(),
        &w.providers,
        &mut w.manifolds,
        w.jobs.as_ref(),
    );
    if first {
        contact_probe::predicted_pass(w, pairs);
    }
}
