//! `world_step/splat_dynamics`：**高斯粒子域的门面轮次**（②物理代理接入，2026-10-08）。
//!
//! 门面只做三件事：**找开档的场 → 借出 → 跑 `step_dynamics` → 放回**；物理全在 `vxl-phys-splat`。
//!
//! - **为什么"借出"**（所有权结构决定的，不是花招）：喷溅场就住在 `providers` 里，而粒子步要拿
//!   **整套 providers**（`&dyn ProviderColliders`）做世界碰撞 ⇒ `&mut field` 与 `&providers` 不能
//!   同时借。把场临时借出（原位留空占位）后两者就分得开；查询 id 表**显式排除本场自己**
//!   （自己的隐式场不参与自己的碰撞 —— 那是自场压力的活）。
//! - **接口选型**：走 `ProviderColliders::contacts_point(id, p, skin, …)` —— **带探针半径**的那条
//!   通道，体素/网格/高度场/喷溅在门面侧都从它出来；`CollisionProvider::closest_point` 不带 skin
//!   ⇒ 对"粒子半径"查询是死路。
//! - **段位**：与 `medium_pass` / `aero_pass` 同段位（体子步开始处、积分之前）；**只在 `dynamics`
//!   开档的场上跑** ⇒ 默认档逐位不变（零代际）。
//! - **确定性**：按注册序遍历场、id 表升序、`step_dynamics` 内部同序。
use crate::types::ProviderEntry;
use crate::World;
use vxl_phys_splat::dynamics::step_dynamics;
use vxl_phys_splat::GaussianSplatField;

/// 门面轮次：推进所有开了动力学档的喷溅场。
pub(crate) fn pass(w: &mut World, dt: f32) {
    let n = w.providers.entries.len();
    for i in 0..n {
        let Some(mut field) = take_splat(w, i) else {
            continue;
        };
        let Some(cfg) = field.dynamics else {
            put_splat(w, i, field);
            continue;
        };
        // id 表 = 其余 provider（升序）；本场自己已借出、且显式排除。
        let ids: Vec<u32> = (0..n as u32).filter(|k| *k != i as u32).collect();
        step_dynamics(&mut field, dt, &w.providers, &ids, cfg);
        put_splat(w, i, field);
    }
}

/// 把第 `i` 个喷溅场**借出**（原位留空占位）；非喷溅条目 ⇒ `None`。
fn take_splat(w: &mut World, i: usize) -> Option<GaussianSplatField> {
    match w.providers.entries.get(i)? {
        ProviderEntry::Splat(_) => {}
        _ => return None,
    }
    let empty_slot = ProviderEntry::Splat(GaussianSplatField::new(0.5));
    match std::mem::replace(&mut w.providers.entries[i], empty_slot) {
        ProviderEntry::Splat(f) => Some(f),
        other => {
            w.providers.entries[i] = other; // 不可达（上面已判过）；防御性归还
            None
        }
    }
}

/// 放回借出的场。
fn put_splat(w: &mut World, i: usize, f: GaussianSplatField) {
    w.providers.entries[i] = ProviderEntry::Splat(f);
}
