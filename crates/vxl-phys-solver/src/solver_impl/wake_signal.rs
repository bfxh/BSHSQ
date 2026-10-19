//! **唤醒信号判据**（`sleep_pass_island` 的三条门）—— `solver_impl` 的子模块。
//!
//! 为什么单独成文件：`solver_impl/sleep.rs` 受 god 门**文件行数棘轮**（基线 190，只准减），
//! 判据本身是一个自洽的关注点（"什么算一次值得唤醒的接触"），搬出来既让 `sleep.rs` 回到基线，
//! 也让判据可以单独读、单独改。
use crate::{Island, SUBISLAND_WAKE_MULT, WAKE_GATE_FAST_MULT};
use vxl_phys_core::{BodySet, PhysConfig};
use vxl_phys_narrow::Manifold;

/// 运动学体的"非零运动"死区（m/s）。运动学速度是**宿主给的确定输入**，不是求解噪声 ⇒
/// 不需要 `SUBISLAND_WAKE_MULT` 那层滞回（滞回是为动态接触的闪烁体准备的）。这个死区只挡
/// 浮点尘埃（`set_kinematic_target` 反算出的 1e-9 级残余），不挡任何真实驱动。
const KIN_MOTION_EPS: f32 = 1e-6;

/// **入睡否决**：若 `m` 一侧是正在运动的运动学体、另一侧是动态体，返回该动态体。
///
/// 为什么需要：没有它时，被慢速支撑带着走的载荷会「睡 → 被叫醒 → 速度低于 `sleep_linear`
/// → 又睡」反复抖动（P24 实测 0.35–1.2 次/秒/体）。运动学体的驱动是**确定输入**，接触点在动
/// 就意味着载荷**正在被搬运**，与"静止到可以睡"互斥 —— 每 tick 把它的入睡计时清零即可。
///
/// 只看运动学体自己的点速度（睡着的动态体速度恒 0）：即便载荷已被带到同速，也必须保持清醒，
/// 否则它一睡就停止被模拟、平台会在它底下滑过去。
pub(crate) fn carried_by_moving_kinematic(bodies: &BodySet, m: &Manifold) -> Option<usize> {
    let (a, b) = (m.a as usize, m.b as usize);
    let (ka, kb) = (bodies.is_kinematic(a), bodies.is_kinematic(b));
    if ka == kb {
        return None;
    }
    let (kin, d) = if ka { (a, b) } else { (b, a) };
    let pos = bodies.position[kin];
    let moving = m
        .points
        .iter()
        .any(|p| bodies.velocity_at(kin, p.point - pos).length() > KIN_MOTION_EPS);
    moving.then_some(d)
}

/// 收集本岛**被正在运动的运动学支撑托着**的动态体（入睡否决名单）。
pub(crate) fn carried_bodies(
    bodies: &BodySet,
    island: &Island,
    manifolds: &[Manifold],
) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    for &mi in &island.manifs {
        if let Some(d) = carried_by_moving_kinematic(bodies, &manifolds[mi]) {
            out.push(d);
        }
    }
    out
}

/// 唤醒信号 `(kin_deep, hot, fast)`。
///
/// - `kin_deep`：运动学支撑的**深度门**（慢速下速度阈值永不成立）。几何信号不是抖动 ⇒ 它必须
///   绕过 `wake_gate_k`：接触数门防的是"被 1–2 个抖动邻居碰"，而一块平台托一个载荷天然只有
///   1 条流形，累计永远到不了 K（`kinematic_wake.rs` W3）。
/// - `hot`：相对运动取**接触点**速度（只对运动学体）：自转支撑的质心线速与角速都可以低于阈值，
///   而载荷所在半径处切向线速度并不低（W4：0.05 rad/s × 2 m）。动态体之间保持质心口径，
///   否则非运动学场景换代（实测 1:1 均匀堆冻结哈希即变）。
///   运动学体**任何非零**接触点相对速度都算 hot（W5）：滞回只该防抖动，宿主的驱动是确定量
///   —— 0.01 rad/s 慢转 10 s 也是 0.2 m 真实弧长，按 0.08 m/s 判"不算动"就是静默穿透。
/// - `fast`：强撞直通（不经接触数门）。
pub(crate) fn wake_signal(
    bodies: &BodySet,
    m: &Manifold,
    w: usize,
    s: usize,
    config: &PhysConfig,
) -> (bool, bool, bool) {
    let kin = bodies.is_kinematic(w);
    let kin_deep = kin && m.points.iter().any(|p| p.depth > 0.5 * config.contact_skin);
    let rel = if kin {
        m.points
            .iter()
            .map(|p| {
                (bodies.velocity_at(w, p.point - bodies.position[w])
                    - bodies.velocity_at(s, p.point - bodies.position[s]))
                .length()
            })
            .fold(0.0f32, f32::max)
    } else {
        (bodies.linvel[w] - bodies.linvel[s]).length()
    };
    let hot = if kin {
        kin_deep || rel > KIN_MOTION_EPS
    } else {
        rel > SUBISLAND_WAKE_MULT * config.sleep_linear
            || bodies.angvel(w).length() > SUBISLAND_WAKE_MULT * config.sleep_angular
    };
    (
        kin_deep,
        hot,
        rel > WAKE_GATE_FAST_MULT * config.sleep_linear,
    )
}
