//! CPU cloth: fixed-topology XPBD grid, static provider contact, optional triangle wind.
use crate::ClothConstraints;
use vxl_phys_core::{interop::ProviderColliders, Vec3};
#[derive(Clone, Copy, Debug)]
struct Edge {
    a: usize,
    b: usize,
    rest: f32,
    lambda: f32,
}

impl Edge {
    fn new(a: usize, b: usize, pos: &[Vec3]) -> Self {
        Self {
            a,
            b,
            rest: (pos[b] - pos[a]).length(),
            lambda: 0.0,
        }
    }
}

/// Row-major grid; triangles are render topology, not dynamic colliders. Pin nodes explicitly.
pub struct ClothSheet {
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub inv_mass: Vec<f32>,
    pub triangles: Vec<[u32; 3]>,
    pub(crate) prev: Vec<Vec3>,
    structural: Vec<Edge>,
    shear: Vec<Edge>,
    bending: Vec<Edge>,
    pub constraints: ClothConstraints,
    pub radius: f32,
    pub skin: f32,
    pub substeps: u32,
    pub iterations: u32,
    pub damping: f32,
    pub wind: Option<crate::ClothWind>,
    pub(crate) wind_delta: Vec<Vec3>,
    /// Proxy-indexed velocity change, reset on each coupled step.
    pub body_dv: Vec<Vec3>,
    /// Proxy-indexed position correction, reset on each coupled step.
    pub body_dx: Vec<Vec3>,
    pub(crate) contacts: Vec<vxl_phys_core::interop::InteropContact>,
}

impl ClothSheet {
    pub fn grid(origin: Vec3, col_step: Vec3, row_step: Vec3, cols: usize, rows: usize) -> Self {
        assert!(cols >= 2 && rows >= 2 && cols.checked_mul(rows).is_some());
        assert!(col_step.cross(row_step).length_squared() > 0.0);
        assert!(cols * rows <= u32::MAX as usize);
        let mut pos = Vec::with_capacity(cols * rows);
        for row in 0..rows {
            for col in 0..cols {
                pos.push(origin + col_step * col as f32 + row_step * row as f32);
            }
        }
        let mut structural = Vec::new();
        let mut shear = Vec::new();
        let mut bending = Vec::new();
        let mut triangles = Vec::with_capacity(2 * (cols - 1) * (rows - 1));
        for row in 0..rows {
            for col in 0..cols {
                let a = row * cols + col;
                if col + 1 < cols {
                    structural.push(Edge::new(a, a + 1, &pos));
                }
                if row + 1 < rows {
                    structural.push(Edge::new(a, a + cols, &pos));
                }
                if col + 2 < cols {
                    bending.push(Edge::new(a, a + 2, &pos));
                }
                if row + 2 < rows {
                    bending.push(Edge::new(a, a + 2 * cols, &pos));
                }
                if col + 1 < cols && row + 1 < rows {
                    shear.push(Edge::new(a, a + cols + 1, &pos));
                    shear.push(Edge::new(a + 1, a + cols, &pos));
                    triangles.push([a as u32, (a + cols) as u32, (a + 1) as u32]);
                    triangles.push([(a + 1) as u32, (a + cols) as u32, (a + cols + 1) as u32]);
                }
            }
        }
        let n = pos.len();
        Self {
            prev: pos.clone(),
            pos,
            vel: vec![Vec3::ZERO; n],
            inv_mass: vec![1.0; n],
            triangles,
            structural,
            shear,
            bending,
            constraints: ClothConstraints::default(),
            radius: 0.02,
            skin: 0.01,
            substeps: 8,
            iterations: 4,
            damping: 1.0,
            wind: None,
            wind_delta: Vec::new(),
            body_dv: Vec::new(),
            body_dx: Vec::new(),
            contacts: Vec::new(),
        }
    }

    pub fn set_pinned(&mut self, node: usize, pinned: bool) {
        self.inv_mass[node] = if pinned { 0.0 } else { 1.0 };
        if pinned {
            self.vel[node] = Vec3::ZERO;
        }
    }

    /// Structural, shear, bending edge counts in that order.
    pub fn edge_counts(&self) -> [usize; 3] {
        [self.structural.len(), self.shear.len(), self.bending.len()]
    }

    /// Largest absolute distance error in one constraint family (0, 1, 2 respectively).
    pub fn max_edge_error(&self, family: usize) -> f32 {
        let edges = match family {
            0 => &self.structural,
            1 => &self.shear,
            2 => &self.bending,
            _ => panic!("cloth constraint family must be 0..3"),
        };
        edges
            .iter()
            .map(|e| ((self.pos[e.b] - self.pos[e.a]).length() - e.rest).abs())
            .fold(0.0, f32::max)
    }

    /// One cloth substep, optionally coupled to rigid proxies.
    pub(crate) fn substep(
        &mut self,
        h: f32,
        gravity: Vec3,
        providers: &dyn ProviderColliders,
        count: u32,
        bodies: &[crate::RigidProxy],
    ) {
        self.apply_wind(h);
        for i in 0..self.pos.len() {
            self.prev[i] = self.pos[i];
            if self.inv_mass[i] == 0.0 {
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] += gravity * h;
            self.pos[i] += self.vel[i] * h;
        }
        for e in self
            .structural
            .iter_mut()
            .chain(&mut self.shear)
            .chain(&mut self.bending)
        {
            e.lambda = 0.0;
        }
        for _ in 0..self.iterations.max(1) {
            Self::project_edges(
                &mut self.pos,
                &self.inv_mass,
                &mut self.structural,
                self.constraints.structural.alpha(),
                h,
            );
            Self::project_edges(
                &mut self.pos,
                &self.inv_mass,
                &mut self.shear,
                self.constraints.shear.alpha(),
                h,
            );
            Self::project_edges(
                &mut self.pos,
                &self.inv_mass,
                &mut self.bending,
                self.constraints.bending.alpha(),
                h,
            );
        }
        self.project_contacts(providers, count);
        if !bodies.is_empty() {
            self.project_body_contacts(bodies, h);
        }
        for i in 0..self.pos.len() {
            self.vel[i] = if self.inv_mass[i] == 0.0 {
                Vec3::ZERO
            } else {
                (self.pos[i] - self.prev[i]) * (self.damping / h)
            };
        }
    }

    fn project_edges(pos: &mut [Vec3], masses: &[f32], edges: &mut [Edge], alpha: f32, h: f32) {
        let a_tilde = alpha / (h * h);
        for e in edges {
            let w = masses[e.a] + masses[e.b];
            if w == 0.0 {
                continue;
            }
            let delta = pos[e.b] - pos[e.a];
            let len = delta.length();
            if len < 1e-9 {
                continue;
            }
            let dl = (e.rest - len - a_tilde * e.lambda) / (w + a_tilde);
            e.lambda += dl;
            let shift = delta * (dl / len);
            pos[e.a] -= shift * masses[e.a];
            pos[e.b] += shift * masses[e.b];
        }
    }
}
