//! `SPEC.md` §4.6/§4.7 的**参数骨架**（从 `lib.rs` 拆出：`lib.rs` 受尺寸棘轮，且绳索落地后
//! 它只该留"crate 门面 = 模块声明 + 再导出"）。数值全部来自规格书，供消费方提前引用。

/// §4.6 软体刚度档（compliance α，m/N）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stiffness {
    NearRigid,
    Hard,
    Standard,
    Soft,
    Jelly,
    Custom(f32),
}

impl Stiffness {
    pub fn alpha(self) -> f32 {
        match self {
            Stiffness::NearRigid => 1e-7,
            Stiffness::Hard => 1e-6,
            Stiffness::Standard => 1e-5,
            Stiffness::Soft => 1e-4,
            Stiffness::Jelly => 3e-4,
            Stiffness::Custom(a) => a,
        }
    }
}

/// §4.6/§4.7 撕裂应变阈值。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TearStrain {
    E03,
    E05,
    E10,
    None,
}

impl TearStrain {
    pub fn eps(self) -> f32 {
        match self {
            TearStrain::E03 => 0.3,
            TearStrain::E05 => 0.5,
            TearStrain::E10 => 1.0,
            TearStrain::None => f32::INFINITY,
        }
    }
}

/// 布料三组约束的独立 compliance（§4.7）。
#[derive(Clone, Copy, Debug)]
pub struct ClothConstraints {
    pub structural: Stiffness,
    pub shear: Stiffness,
    pub bending: Stiffness,
    pub tear: TearStrain,
}

impl Default for ClothConstraints {
    fn default() -> Self {
        Self {
            structural: Stiffness::Standard,
            shear: Stiffness::Soft,
            bending: Stiffness::Jelly,
            tear: TearStrain::E05,
        }
    }
}

/// 自碰撞参数（空间哈希 + 粒子半径；开启代价 ≤50%〔目标〕）。
#[derive(Clone, Copy, Debug)]
pub struct SelfCollision {
    pub enabled: bool,
    pub particle_radius: f32,
    /// **自摩擦系数 μ**（切向库仑锥；`0` = 关）。
    ///
    /// ⚠️ **这个值不是规格书给的** —— 本骨架其余数值都取自 `SPEC.md`，而规格书**没有规定自摩擦**；
    /// 本值是本片选的默认（**`0` = 关**，与全仓"0=关字段"先例一致 ⇒ 打开它是有意的行为变化）。
    pub friction: f32,
}

impl Default for SelfCollision {
    fn default() -> Self {
        Self {
            enabled: false,
            particle_radius: 0.05,
            friction: 0.0,
        }
    }
}
