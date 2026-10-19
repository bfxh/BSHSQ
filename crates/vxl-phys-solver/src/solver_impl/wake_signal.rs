//! **唤醒信号判据**（`sleep_pass_island` 的三条门）—— `solver_impl` 的子模块。
//!
//! 为什么单独成文件：`solver_impl/sleep.rs` 受 god 门**文件行数棘轮**（基线 190，只准减），
//! 判据本身是一个自洽的关注点（"什么算一次值得唤醒的接触"），搬出来既让 `sleep.rs` 回到基线，
//! 也让判据可以单独读、单独改。
use crate::{SUBISLAND_WAKE_MULT, WAKE_GATE_FAST_MULT};
use vxl_phys_core::{BodySet, PhysConfig};
use vxl_phys_narrow::Manifold;

/// 运动学体的"非零运动"死区（m/s）。运动学速度是**宿主给的确定输入**，不是求解噪声 ⇒
/// 不需要 `SUBISLAND_WAKE_MULT` 那层滞回（滞回是为动态接触的闪烁体准备的）。这个死区只挡
/// 浮点尘埃（`set_kinematic_target` 反算出的 1e-9 级残余），不挡任何真实驱动。
const KIN_MOTION_EPS: f32 = 1e-6;

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
