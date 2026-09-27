//! Optional linearized aerodynamic load for CPU cloth triangles (SPEC section 4.7).

use crate::cloth::ClothSheet;
use vxl_phys_core::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct ClothWind {
    pub velocity: Vec3,
    pub density: f32,
    pub drag: f32,
}

impl ClothSheet {
    pub(crate) fn apply_wind(&mut self, h: f32) {
        let Some(wind) = self.wind else { return };
        assert!(wind.velocity.is_finite());
        assert!(wind.density.is_finite() && wind.density >= 0.0);
        assert!(wind.drag.is_finite() && wind.drag >= 0.0);
        if wind.density == 0.0 || wind.drag == 0.0 {
            return;
        }
        self.wind_delta.resize(self.pos.len(), Vec3::ZERO);
        self.wind_delta.fill(Vec3::ZERO);
        for &[a, b, c] in &self.triangles {
            let [a, b, c] = [a as usize, b as usize, c as usize];
            let twice_area = (self.pos[b] - self.pos[a]).cross(self.pos[c] - self.pos[a]);
            let len = twice_area.length();
            if len <= 1e-9 {
                continue;
            }
            let normal = twice_area * (1.0 / len);
            let relative = wind.velocity - (self.vel[a] + self.vel[b] + self.vel[c]) * (1.0 / 3.0);
            let impulse =
                normal * (0.25 * wind.density * wind.drag * len * relative.dot(normal) * h / 3.0);
            for i in [a, b, c] {
                self.wind_delta[i] += impulse * self.inv_mass[i];
            }
        }
        for (velocity, delta) in self.vel.iter_mut().zip(&self.wind_delta) {
            *velocity += *delta;
        }
    }
}
