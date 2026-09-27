//! Cloth world channel: fixed tick and stable particle/rigid proxy order.

use super::*;

#[derive(Default)]
pub(crate) struct SoftDomain {
    pub ropes: Vec<vxl_phys_soft::Rope>,
    pub cloths: Vec<vxl_phys_soft::ClothSheet>,
}

impl World {
    /// Register a cloth sheet. Indices remain stable for the lifetime of this world.
    pub fn add_cloth(&mut self, cloth: vxl_phys_soft::ClothSheet) -> usize {
        self.soft.cloths.push(cloth);
        self.soft.cloths.len() - 1
    }

    pub fn cloths(&self) -> &[vxl_phys_soft::ClothSheet] {
        &self.soft.cloths
    }

    pub fn cloth(&self, index: usize) -> Option<&vxl_phys_soft::ClothSheet> {
        self.soft.cloths.get(index)
    }

    pub(crate) fn cloth_pass(&mut self) {
        if self.soft.cloths.is_empty() {
            return;
        }
        let dt = self.config.dt;
        let gravity = self.config.gravity;
        let count = self.providers.len() as u32;
        self.refresh_soft_proxies();
        let mut proxies = std::mem::take(&mut self.rope_proxies);
        for cloth in &mut self.soft.cloths {
            cloth.step_with_bodies(dt, gravity, &self.providers, count, &proxies);
            for (j, p) in proxies.iter_mut().enumerate() {
                if p.inv_mass > 0.0 {
                    let body = p.body as usize;
                    let dv = cloth.body_dv[j];
                    let dx = cloth.body_dx[j];
                    self.bodies.linvel[body] += dv;
                    self.bodies.position[body] += dx;
                    p.linvel += dv;
                    p.pos += dx;
                }
            }
        }
        self.rope_proxies = proxies;
    }

    pub(crate) fn refresh_soft_proxies(&mut self) {
        self.rope_proxies.clear();
        for i in 0..self.bodies.len() {
            let shape = self.bodies.shape[i];
            if matches!(
                shape,
                Shape::Provider(_) | Shape::HeightField(_) | Shape::Compound { .. }
            ) {
                continue;
            }
            self.rope_proxies.push(vxl_phys_soft::RigidProxy {
                body: i as u32,
                shape,
                pos: self.bodies.position[i],
                rot: self.bodies.rot(i),
                linvel: self.bodies.linvel[i],
                inv_mass: if self.bodies.is_dynamic(i) {
                    self.bodies.inv_mass[i]
                } else {
                    0.0
                },
            });
        }
    }
}
