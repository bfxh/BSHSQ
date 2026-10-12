//! **运动学体域 API**（M2 · `docs/PLAN-kinematic.md` 的 K1）。
//!
//! 处置条目（`god_gate.py` 对 `World` 的**已登记债务**）明写「域侧 API 拆成扩展 trait」⇒
//! 本域的 `add_kinematic` / `set_kinematic_velocity` / `set_kinematic_target` 走
//! `impl KinematicExt for World`（`impl Trait for Type` 不计入该类型的方法账），
//! 与 `world_step/destruction.rs`、`world_step/conversion.rs` 同款先例。
//!
//! ⚠️ 别拿"挪静态体"当运动学体用：静态体的树代理盒**永不移动**、接触里速度恒 0
//! （三条断链与 ①–⑦ 实测见 `SURVEY-RIGID-HARD-CASES.md` §2 / `SURVEY-EXTREMES-AND-BORROW.md` §1）。

// ⚠️ 新文件不用 glob 导入（`glob-gate` 对新文件零基线）：显式列名。
use crate::{BodyId, Quat, Shape, Vec3, World};

/// 运动学体（宿主驱动：电梯 / 传送带 / 移动地形）的门面 API。
pub trait KinematicExt {
    /// **加运动学体**：位置由宿主驱动、质量无限、不受力，但**速度是输入**且**进岛**
    /// （A 档并岛，用户 2026-10-12 裁决）⇒ 摩擦因此携带载荷。
    ///
    /// 两种驱动方式（可混用）：
    /// - **速度驱动**：[`set_kinematic_velocity`](Self::set_kinematic_velocity)，位置由积分推进；
    /// - **位移驱动**：[`set_kinematic_target`](Self::set_kinematic_target)，引擎按 `config.dt`
    ///   反算速度 ⇒ 本 tick 末落在目标处（子步积分把它走完）。
    fn add_kinematic(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId;

    /// **速度驱动**：给定线/角速度，位置由积分器推进（`integrate_positions` 把运动学体一并推）。
    /// 它**不受力、不被接触冲量改写**，但它的速度**会被求解器看见** ⇒ 站在上面的载荷被摩擦带走。
    /// 非运动学体调用 = 明确拒绝（`assert`），不静默当动态体处理。
    fn set_kinematic_velocity(&mut self, id: BodyId, lin: Vec3, ang: Vec3);

    /// **位移驱动**：给目标位置，引擎按 `config.dt`（**tick 步长**，不是子步）**反算速度**
    /// ⇒ 本 tick 末落在目标处。角速度不受影响（转向请用 [`set_kinematic_velocity`](Self::set_kinematic_velocity)）。
    ///
    /// 为什么必须由引擎反算：宿主只改位置的话速度仍是 0 ⇒ 接触看"对面不动"（断链②，修好了也白修）。
    /// ⚠️ 这是"本 tick 的位移"，不是"持续目标"：每 tick 要移动就每 tick 调一次。
    fn set_kinematic_target(&mut self, id: BodyId, target: Vec3);
}

impl KinematicExt for World {
    fn add_kinematic(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId {
        self.bodies.push_kinematic(shape, position, rotation)
    }

    fn set_kinematic_velocity(&mut self, id: BodyId, lin: Vec3, ang: Vec3) {
        assert!(
            self.bodies.is_kinematic(id as usize),
            "set_kinematic_velocity 只接受运动学体（该体不是：请用 add_kinematic 建体）"
        );
        self.bodies.set_linvel(id as usize, lin);
        self.bodies.set_angvel(id as usize, ang);
    }

    fn set_kinematic_target(&mut self, id: BodyId, target: Vec3) {
        let i = id as usize;
        assert!(
            self.bodies.is_kinematic(i),
            "set_kinematic_target 只接受运动学体（该体不是：请用 add_kinematic 建体）"
        );
        let dt = self.config.dt;
        assert!(dt > 0.0 && dt.is_finite(), "config.dt 必须是有限正数");
        self.bodies.linvel[i] = (target - self.bodies.position[i]) * (1.0 / dt);
        self.bodies.wake(i);
    }
}
