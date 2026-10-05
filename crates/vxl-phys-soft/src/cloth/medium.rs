//! **布 × 介质的 scratch 组**（`cloth.rs` 的 `pub` 子模块）：逐面样本 + 逐面反作用冲量 +
//! 逐顶点有效质量倍率。
//!
//! 三者**同生共死**（都由"本 tick 采到了哪些介质样本"决定）⇒ 并成一组，`ClothSheet` 的成员棘轮
//! （只准减）因此从 24 降到 22。**默认档逐位不变**：无流体/未采样 ⇒ 三个 Vec 全空 ⇒ 首行短路。
//!
//! 分工：采样由门面填（`world_step::fill_cloth_medium`）；阻力与双向在 `cloth_medium`；
//! 湿质量在 `cloth/wet.rs`。本模块只放**数据**。
use vxl_phys_core::interop::MediumSample;
use vxl_phys_core::Vec3;

/// 布 × 介质的 scratch 组（公开数据面：门面直接读写，同 `ClothSheet::medium` 的既有口径）。
#[derive(Clone, Default)]
pub struct ClothMedium {
    /// 逐面介质样本（长度 = `tris.len()` 时逐面生效；**空 = 关**）。门面每 tick 按**面心**填。
    pub samples: Vec<MediumSample>,
    /// 逐面反作用冲量（N·s；布 → 介质）：`inject` 每子步累加 `−F·h`，门面 tick 末读走并**原地零化**。
    pub reaction: Vec<Vec3>,
    /// 逐顶点**有效质量倍率**（`m_eff = mass[i]·mass_scale[i]`；干 = `1.0`）：湿质量那半每个子步
    /// 从 `samples` 的 `occupied` 重算它并刷新 `inv_mass`（见 `cloth/wet.rs`）；兼作湿率累加器。
    pub mass_scale: Vec<f32>,
    /// **湿质量开关**（默认 `false` ⇒ 逐位不变）。开启后 `inv_mass = 1/(m·(1 + κ·wet))`。
    ///
    /// ⚠️ **为什么默认关**：吸水是**质量转移**，本实现不把被吸走的水的动量记回流体 ⇒ 布-液之间的
    /// **动量账会漏**（实测有湿质量时 `Δp布 + Δp水 ≈ +0.50`，无湿质量时 ≈ `-1.2e-5`）。
    /// 要默认开，得先补"吸收动量"那条通量（谁把水吸进来、水当时带多少动量）。
    pub wet_mass: bool,
}

impl ClothMedium {
    /// 清**逐 tick** 数据（样本 + 冲量）。`mass_scale` 是派生量、每个子步重算 ⇒ 不在这里清。
    pub fn clear(&mut self) {
        self.samples.clear();
        self.reaction.clear();
    }
}
