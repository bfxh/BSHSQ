//! Optional translational particle/rigid contact for CPU cloth.

use crate::{rigid::shape_penetration, ClothSheet, RigidProxy};
use vxl_phys_core::{interop::ProviderColliders, Shape, Vec3};

impl ClothSheet {
    pub(crate) fn project_contacts(&mut self, providers: &dyn ProviderColliders, count: u32) {
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            for id in 0..count {
                self.contacts.clear();
                if !providers.contacts_sphere(
                    id,
                    self.pos[i],
                    self.radius,
                    self.skin,
                    &mut self.contacts,
                ) {
                    continue;
                }
                for hit in &self.contacts {
                    if hit.depth > 0.0 {
                        self.pos[i] += hit.normal * hit.depth;
                    }
                }
            }
        }
    }

    /// Advance without rigid proxies; preserves the original provider-only path.
    pub fn step(&mut self, dt: f32, gravity: Vec3, providers: &dyn ProviderColliders, count: u32) {
        self.step_with_bodies(dt, gravity, providers, count, &[]);
    }

    /// Advance against static sphere/box/capsule and dynamic sphere proxies.
    /// Dynamic nonspheres need angular reaction and are skipped.
    /// `body_dv` and `body_dx` are indexed by proxy and reset on each call.
    pub fn step_with_bodies(
        &mut self,
        dt: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        count: u32,
        bodies: &[RigidProxy],
    ) {
        assert!(dt.is_finite() && dt > 0.0);
        self.body_dv.resize(bodies.len(), Vec3::ZERO);
        self.body_dv.fill(Vec3::ZERO);
        self.body_dx.resize(bodies.len(), Vec3::ZERO);
        self.body_dx.fill(Vec3::ZERO);
        let h = dt / self.substeps.max(1) as f32;
        for _ in 0..self.substeps.max(1) {
            self.substep(h, gravity, providers, count, bodies);
        }
    }

    pub(crate) fn project_body_contacts(&mut self, bodies: &[RigidProxy], h: f32) {
        for (j, b) in bodies.iter().enumerate() {
            if b.inv_mass > 0.0 && !matches!(b.shape, Shape::Sphere { .. }) {
                continue;
            }
            for i in 0..self.pos.len() {
                let wp = self.inv_mass[i];
                if wp <= 0.0 {
                    continue;
                }
                let wb = b.inv_mass;
                let Some((normal, depth, _)) = shape_penetration(
                    &b.shape,
                    b.pos + self.body_dx[j],
                    b.rot,
                    self.pos[i],
                    self.radius,
                ) else {
                    continue;
                };
                if depth <= 0.0 {
                    continue;
                }
                // Shared inverse-mass projection; both corrections come from the same lambda.
                let lambda = depth / (wp + wb);
                let particle_shift = normal * (wp * lambda);
                if wb > 0.0 {
                    let vp = (self.pos[i] - self.prev[i]) * (1.0 / h);
                    let approach = (b.linvel + self.body_dv[j] - vp).dot(normal).max(0.0);
                    let impulse_lambda = lambda.min(approach * h / (wp + wb));
                    self.prev[i] += normal * (wp * (lambda - impulse_lambda));
                    let body_shift = normal * (-wb * lambda);
                    self.body_dx[j] += body_shift;
                    self.body_dv[j] -= normal * (wb * impulse_lambda / h);
                }
                self.pos[i] += particle_shift;
            }
        }
    }
}
