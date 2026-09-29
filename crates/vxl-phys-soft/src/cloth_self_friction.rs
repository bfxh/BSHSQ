//! **布片自摩擦**（切向库仑锥）—— 自碰撞切片 T3 登记的边界（"两层之间可以自由滑"）。
//!
//! **口径**（与 rope / 刚体**同一条**库仑锥）：把一对自接触粒子的**相对切向滑移**限制在
//! `μ·depth` 以内，`depth` = 该对的**相对**法向修正量（`= (w_i+w_j)·λ`，即 `d_c − len`）。
//!
//! ⚠️ **与 rope 的体接触不同的那一点**（如实登记）：那边一侧是**刚体**、预算只对**粒子那一侧的
//! 滑移**生效（`μ·w_p·λ`，§8.4.25 的冲量口径）；这里是**两只都在动**的对，要限制的是**相对**
//! 滑移 ⇒ 相对法向修正量恰好回到 `depth`。⇒ **同一条物理（`|J_t| ≤ μ·J_n`），预算量按
//! "限制谁"取**（限制单个粒子的滑移 ⇒ `w_p·λ`；限制**相对**滑移 ⇒ `depth`）。
//!
//! **默认 `μ = 0`**（`SelfCollision::friction`；规格书没有规定自摩擦 ⇒ 本片选的默认是"关"）
//! ⇒ 本函数首行返回 ⇒ 切片 T3 的 4 条判据读数**逐位不变**。
//!
//! **为什么是自由函数而不是方法**：调用点里 `bucket` 正借着 `self.self_contacts`
//! （空间哈希的桶），方法调用会与它冲突；自由函数只借 `pos`/`prev` 两个切片 ✓。
use vxl_phys_core::Vec3;

/// 把粒子 `i`/`j` 的**相对切向滑移**按库仑锥扣掉：锥内整段吃掉（静摩擦），超出按动摩擦滑。
///
/// 分摊按逆质量（与法向那支同款）：两侧各走 `w/w_total` 的份额 ⇒ **相对**切向位移恰好减少
/// `removed`。`μ ≤ 0` ⇒ 直接返回（默认档零成本）。
#[allow(clippy::too_many_arguments)] // 对包：两粒子 + 逆质量 + 法线 + 深度 + μ
pub(crate) fn resist_slip(
    pos: &mut [Vec3],
    prev: &[Vec3],
    i: usize,
    j: usize,
    w_i: f32,
    w_j: f32,
    nrm: Vec3,
    depth: f32,
    mu: f32,
) {
    if mu <= 0.0 {
        return;
    }
    let rel = (pos[j] - prev[j]) - (pos[i] - prev[i]);
    let tan = rel - nrm * rel.dot(nrm);
    let slip = tan.length();
    if slip <= 0.0 {
        return;
    }
    let budget = mu * depth;
    let removed = if slip < budget { slip } else { budget };
    let k = removed / slip;
    let w = w_i + w_j;
    pos[i] += tan * (k * (w_i / w));
    pos[j] -= tan * (k * (w_j / w));
}
