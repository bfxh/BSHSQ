//! **高斯粒子域的运动步**（`ROUTE.md` §3.1 ②「物理代理」第一片，2026-10-08）。
//!
//! `flow::advance` 只把核当**运动学流场**（`center += v·dt`，没有外力、没有碰撞）。本文件让核
//! 变成**真粒子**：每核 = 一个质点（半径 = 各向异性椭球的支撑半径 `Splat::radius_along`、
//! 取向 = `rot`、质量 = `Splat::mass`），走半隐式欧拉 + **世界碰撞的非弹性投影**。
//!
//! 口径与边界：
//! - **一核一质点**，核间**暂无**相互作用（自场压力/黏性是下一片）；速度槽 = `kern_vel`
//!   （与介质双向共用；长度不符时惰性补齐）。
//! - 碰撞走 `ProviderColliders::contacts_point(id, p, skin, …)`（**带探针半径**的那条通道：
//!   体素/网格/高度场/喷溅场在门面侧都从它出来；`CollisionProvider::closest_point` 不带 skin
//!   ⇒ 对"粒子半径"查询是死路）。与流体边界同款非弹性投影：修正量用沿法线的**椭球支撑半径**
//!   `r(n)` ⇒ 各向异性核贴面时按朝向留出正确间隙。
//! - 确定性：核按注册序、provider 按传入序、接触按返回序；无 HashMap、无浮点归约顺序变化。
//! - `dt <= 0` ⇒ 空操作；`ids` 空 = **纯弹道**（不查世界）—— 判据里的金丝雀用它。
use crate::GaussianSplatField;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::Vec3;

/// 粒子步参数。
#[derive(Clone, Copy, Debug)]
pub struct ParticleStep {
    /// 重力（m/s²）。
    pub gravity: Vec3,
    /// 每步速度衰减（1 = 不衰减；与 `flow::advance` 的 `damping` 同口径）。
    pub damping: f32,
}

impl Default for ParticleStep {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            damping: 1.0,
        }
    }
}

/// 逐核推进一个 `dt`；返回**发生接触投影的次数**（诊断/判据）。
pub fn step_particles(
    field: &mut GaussianSplatField,
    dt: f32,
    providers: &dyn ProviderColliders,
    ids: &[u32],
    cfg: ParticleStep,
) -> usize {
    if dt.is_nan() || dt <= 0.0 || field.splats.is_empty() {
        return 0;
    }
    if field.kern_vel.len() != field.splats.len() {
        field.kern_vel.clear();
        field.kern_vel.resize(field.splats.len(), Vec3::ZERO);
    }
    let mut buf: Vec<InteropContact> = Vec::new();
    let mut hits = 0usize;
    for k in 0..field.splats.len() {
        let s = field.splats[k];
        let mut v = (field.kern_vel[k] + cfg.gravity * dt) * cfg.damping;
        let mut c = s.center + v * dt;
        for id in ids {
            buf.clear();
            if !providers.contacts_point(*id, c, s.max_radius(), &mut buf) {
                continue; // 该 provider 不支持点查询
            }
            for ct in &buf {
                // 约定 `depth = −sdf` ⇒ `push = r − sdf`：把核心推到"表面外 r(n) 处"。
                let push = s.radius_along(ct.normal) + ct.depth;
                if push.is_nan() || push <= 0.0 {
                    continue;
                }
                c += ct.normal * push;
                let vn = v.dot(ct.normal);
                if vn < 0.0 {
                    v -= ct.normal * vn; // 非弹性：去掉侵入法向分量
                }
                hits += 1;
            }
        }
        field.splats[k].center = c;
        field.kern_vel[k] = v;
    }
    field.grid = None; // 中心动了 ⇒ 均匀网格登记失效（查询自动退回全扫，逐位一致）
    hits
}
