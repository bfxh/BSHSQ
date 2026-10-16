//! provider：从 lib.rs 按职责拆出（纯搬移，语义未改）。
//!
//! 本文件是 `GaussianSplatField` 的**外部 trait 实现**面：`ProviderColliders`（刚体侧
//! 的盒/球/外壳查询，口径见 lib.rs 模块头 ②）与 `vxl_phys_core::interop::MediumField`
//! （介质采样/沉积）。trait impl 内的方法不能写可见性，故这里全部靠**父模块的项**工作：
//! `Splat`、`GaussianSplatField` 的私有字段（子模块可见父模块私有项）与
//! `density_grad`/`sdf`/`world_bounds` 等固有方法。
use super::GaussianSplatField;
use std::vec::Vec;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Mat3, Vec3};

impl ProviderColliders for GaussianSplatField {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        let _ = id;
        Some(self.world_bounds())
    }

    /// 点查询（探针半径 = skin）：`depth = skin − sdf(p)`；法线 = −∇σ/|∇σ|（外向）。
    fn contacts_point(&self, id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        let _ = id;
        let (s, g) = self.density_grad(p);
        let gl = g.length();
        let f = if gl > 1e-6 {
            (self.iso - s) / gl
        } else if s > self.iso {
            -(s - self.iso).sqrt().min(1.0)
        } else {
            (self.iso - s).sqrt().min(1.0)
        };
        // depth = 穿透量（正 = 点已在等值面里侧）；带内判据 `depth > −skin`（等价 `f < skin`）
        // ⇒ 面外 skin 内仍生成**预期接触**。与体素（`−sdf`）、网格（2026-09-21 起 `−sd`）、
        // 体素的盒查询（`−d`）**同一口径**。
        //
        // **偏置修正**（2026-09-21，P5 ⑳/㉑ 同族）：旧式 `depth = skin − f` 在**外侧**也给正
        // depth ⇒ 盒（14 点采样走本函数）被顶到 `f ≈ skin` ⇒ 实测在平场上**悬空 ~1 cm**
        // （`splat_rest_probe`：半高 0.25/0.35 的最小离面间隙 +0.0086/+0.0115）。
        // 流体不读 `depth`（自算 sdf、按管道半径过滤）⇒ 该修正如体素那次一样无损。
        let depth = -f;
        if depth < -skin {
            return true; // 支持查询；该点不在带内
        }
        let gl = g.length();
        let n = if gl > 1e-6 {
            (-g) * (1.0 / gl)
        } else {
            Vec3::Y
        };
        out.push(InteropContact {
            point: p,
            normal: n,
            depth,
            feature: 0,
        });
        true
    }

    /// 盒查询：盒表面采样（6 面 × 中心 + 4 角，与体素盒路径同构）。
    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: vxl_phys_core::Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let r = Mat3::from_quat(rot);
        let axes = [
            r.mul_vec3(Vec3::X),
            r.mul_vec3(Vec3::Y),
            r.mul_vec3(Vec3::Z),
        ];
        let hs = [half.x, half.y, half.z];
        for i in 0..3 {
            let (u, hu) = (axes[(i + 1) % 3], hs[(i + 1) % 3]);
            let (v, hv) = (axes[(i + 2) % 3], hs[(i + 2) % 3]);
            for &side in &[-1.0f32, 1.0] {
                let face_center = pos + axes[i] * (side * hs[i]);
                self.contacts_point(id, face_center, skin, out);
                for &(su, sv) in &[(-1.0f32, -1.0f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                    let corner = face_center + u * (su * hu) + v * (sv * hv);
                    self.contacts_point(id, corner, skin, out);
                }
            }
        }
        true
    }

    /// 球查询：中心 SDF 解析深度（球面最近点方向 ≈ SDF 梯度）。
    fn contacts_sphere(
        &self,
        id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let _ = id;
        let f = self.sdf(center);
        let depth = radius - f;
        if depth < -skin {
            return true;
        }
        let (_, g) = self.density_grad(center);
        let gl = g.length();
        let n = if gl > 1e-6 {
            (-g) * (1.0 / gl)
        } else {
            Vec3::Y
        };
        out.push(InteropContact {
            point: center - n * (radius.min(f.max(0.0))),
            normal: n,
            depth,
            feature: 1,
        });
        true
    }
}

impl vxl_phys_core::interop::MediumField for GaussianSplatField {
    /// 采样：密度 ∝ σ(p)（每单位 σ 的 `medium_density`）、速度 = 场流速、
    /// 占用率 = σ/iso 钳到 0..1（iso 外为自由空间）。
    /// `medium_density == 0` 时直接返回真空（零成本短路）。
    fn sample(&self, x: Vec3) -> vxl_phys_core::interop::MediumSample {
        use vxl_phys_core::interop::MediumSample;
        if self.medium_density <= 0.0 || self.splats.is_empty() {
            return MediumSample::VACUUM;
        }
        let (sigma, _) = self.density_grad(x);
        if sigma <= 0.0 {
            return MediumSample::VACUUM;
        }
        MediumSample {
            density: sigma * self.medium_density,
            // 速度项：two_way 关 ⇒ 常值；开 ⇒ 核速度的 e 加权插值（flow.rs）。
            velocity: self.flow_velocity(x),
            viscosity: sigma * self.medium_viscosity,
            temperature: 0.0,
            occupied: if self.iso > 0.0 {
                (sigma / self.iso).clamp(0.0, 1.0)
            } else {
                0.0
            },
        }
    }

    /// 沉积（体→介质反作用）：`two_way` 开 ⇒ 按核权重分摊动量驱动场演化（`flow.rs`）；
    /// 关 ⇒ 空操作（旧"单向"语义，逐位不变）。
    fn deposit(&mut self, x: Vec3, momentum: Vec3, mass: f32, pressure_work: f32) {
        self.deposit_flow(x, momentum, mass, pressure_work);
    }
}
