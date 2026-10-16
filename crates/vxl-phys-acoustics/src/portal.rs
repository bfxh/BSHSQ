//! **体素空洞自动识别（A10）**：从**占据位图**里认出**房间**与**开口**（portal），
//! 直接喂给 [`AcousticGeometry::portals`](crate::AcousticGeometry::portals)。
//!
//! 与 A4 的[自动认边](crate::AcousticGeometry::detect_edges)同一条「显式优先、自动兜底」思路
//! （设计文档 §8 待裁项①）：开口本该由作者显式标（[`Portal`]，A2），这里给一条从**体素占据**
//! 里自动认孔的兜底。口径：
//!
//! 1. **空 = `!占据`、实 = `占据`**；先按 **6 连通**把**空**格分成连通域 ⇒ 这就是「房间」；
//! 2. 分房前先按**切比雪夫半径 `neck_cells` 腐蚀**空域：窄于约 `2·neck_cells` 的通道会被切断，
//!    门洞两侧因此落进两个房间；再用**多源 BFS 膨胀**把房间号补满整个空域（同距按**小房间号**定序）；
//! 3. 两个房间号**相接**的那层空格就是**开口**：按 6 连通聚成一个 portal；法向取**标签变号最多**
//!    的那一轴，`quad` = 该轴居中、另两轴的角点（外扩半格覆盖格子）。
//!
//! 全流程**无哈希迭代、无随机**：种子按格索引序、膨胀按（距离 → 房间号 → 格索引）定序 ⇒
//! 同输入逐位可复现。判据 `tests/a10_portals.rs`。

use std::collections::VecDeque;
use vxl_phys::Vec3;

/// 一个**体素占据图**（A10）：均匀网格 + 逐格占据位。它是声学几何的另一种表示
/// （三角网 / 体素占据二选一），**只读**交进来——本层不读任何内部体素结构。
#[derive(Clone, Copy, Debug)]
pub struct VoxelOccupancy<'a> {
    /// 网格**最小角**（格 `(0,0,0)` 的外侧角）的世界坐标；格中心 = `origin + (i+0.5)·step`，
    /// 与 `vxl-phys-terrain` 的 `VoxelVolume` 同口径。
    pub origin: Vec3,
    /// 格边长（米，须 > 0）。
    pub step: f32,
    /// 格数 `[nx, ny, nz]`。
    pub dims: [u32; 3],
    /// 占据位（行主序 `ix + nx·(iy + ny·iz)`）；**短了按「实」（占据）算**。
    pub occupied: &'a [bool],
}

/// 自动认孔的筛选条件（A10）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PortalDetect {
    /// **颈宽阈值**（格）：空域先按切比雪夫半径这么腐蚀，窄于约 `2·neck_cells` 的通道会被切断 ⇒
    /// 两侧判成不同房间、其间出一扇门。`0` = 不腐蚀（整个连通空域并成一个房间 ⇒ 不产 portal）。
    pub neck_cells: u32,
    /// 最多产出多少条 portal（按区域对 + 起始格定序后截断）。
    pub max_portals: usize,
    /// 产出 portal 的**开启比例**（`1` = 全开；口径同 [`Portal::open`]）。
    pub open: f32,
}

impl Default for PortalDetect {
    fn default() -> Self {
        Self {
            neck_cells: 1,
            max_portals: 64,
            open: 1.0,
        }
    }
}

/// 一个**开口**（A2）：四边形（四个角，顶点顺序无要求）+ **开启比例**。
///
/// `open ∈ [0,1]`：`0` = 关死（这条开口不传声，只剩墙的透射泄漏）、`1` = 全开；中间值就是
/// 「开了一条缝」——增益按 `open` 线性衰减 ⇒ **缝宽 → 衰减单调**（文档 §5 的 A2 判据）。
/// 开口推荐**叠在实心建模面上声明**（几何照常封闭，开口由本表给出）；几何里真挖了空洞也能用。
///
/// `regions`（A10）：自动认孔时填该开口**连接的两侧房间 id**（升序 `[小, 大]`）；作者显式标的
/// 多半是 `None`（不标区域）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Portal {
    /// 四边形的四个角（世界坐标）。
    pub quad: [Vec3; 4],
    /// 开启比例 `∈ [0,1]`。
    pub open: f32,
    /// 两侧区域 id（`[小, 大]`）；`None` = 未标。
    pub regions: Option<[u32; 2]>,
}

impl VoxelOccupancy<'_> {
    /// 从占据位图**自动认孔**（A10）：返回按区域对定序的 [`Portal`]，其 `regions` 已填两侧房间 id。
    /// 少于两个房间 ⇒ 空（没有内部接口）。
    pub fn detect_portals(&self, cfg: &PortalDetect) -> Vec<Portal> {
        let n = self.cells();
        if n == 0 {
            return Vec::new();
        }
        let free = self.free_mask();
        let surv = self.erode_free(&free, cfg.neck_cells);
        let (seed_room, rooms) = self.label_rooms(&surv);
        if rooms < 2 {
            return Vec::new();
        }
        let dist = self.bfs_dist(&free, &surv);
        let room = self.fill_rooms(&free, &dist, seed_room);
        let is_portal = self.mark_interface(&free, &room);
        let mut groups = self.group_portals(&free, &room, &is_portal);
        groups.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.cmp(&b.1))
                .then(a.3.first().cmp(&b.3.first()))
        });
        groups.truncate(cfg.max_portals);
        groups
            .iter()
            .map(|g| Portal {
                quad: self.portal_quad(&g.3, g.2 as usize),
                open: cfg.open,
                regions: Some([g.0, g.1]),
            })
            .collect()
    }

    #[inline]
    fn cells(&self) -> usize {
        self.dims[0] as usize * self.dims[1] as usize * self.dims[2] as usize
    }

    #[inline]
    fn idx(&self, ix: usize, iy: usize, iz: usize) -> usize {
        let nx = self.dims[0] as usize;
        let ny = self.dims[1] as usize;
        ix + nx * (iy + ny * iz)
    }

    #[inline]
    fn solid_at(&self, i: usize) -> bool {
        self.occupied.get(i).copied().unwrap_or(true)
    }

    #[inline]
    fn solid(&self, ix: usize, iy: usize, iz: usize) -> bool {
        self.solid_at(self.idx(ix, iy, iz))
    }

    fn center(&self, i: usize) -> Vec3 {
        let nx = self.dims[0] as usize;
        let ny = self.dims[1] as usize;
        let plane = nx * ny;
        let iz = i / plane;
        let rem = i % plane;
        let iy = rem / nx;
        let ix = rem % nx;
        self.origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * self.step
    }

    /// 6 连通邻居 + 每个邻居所在的轴（0=x, 1=y, 2=z）+ 数量。
    fn nbrs6_axis(&self, i: usize) -> ([usize; 6], [u8; 6], usize) {
        let nx = self.dims[0] as usize;
        let ny = self.dims[1] as usize;
        let nz = self.dims[2] as usize;
        let plane = nx * ny;
        let iz = i / plane;
        let rem = i % plane;
        let iy = rem / nx;
        let ix = rem % nx;
        let mut out = [0usize; 6];
        let mut ax = [0u8; 6];
        let mut m = 0;
        if ix > 0 {
            out[m] = i - 1;
            ax[m] = 0;
            m += 1;
        }
        if ix + 1 < nx {
            out[m] = i + 1;
            ax[m] = 0;
            m += 1;
        }
        if iy > 0 {
            out[m] = i - nx;
            ax[m] = 1;
            m += 1;
        }
        if iy + 1 < ny {
            out[m] = i + nx;
            ax[m] = 1;
            m += 1;
        }
        if iz > 0 {
            out[m] = i - plane;
            ax[m] = 2;
            m += 1;
        }
        if iz + 1 < nz {
            out[m] = i + plane;
            ax[m] = 2;
            m += 1;
        }
        (out, ax, m)
    }

    #[inline]
    fn nbrs6(&self, i: usize) -> ([usize; 6], usize) {
        let (out, _ax, m) = self.nbrs6_axis(i);
        (out, m)
    }

    fn free_mask(&self) -> Vec<bool> {
        (0..self.cells()).map(|i| !self.solid_at(i)).collect()
    }

    /// 切比雪夫半径 `k` 的腐蚀：`k` 邻域须**全在范围内且全空**才留下。越界按「实」⇒ 边界也蚀。
    fn erode_free(&self, free: &[bool], k: u32) -> Vec<bool> {
        let n = self.cells();
        if k == 0 {
            return free.to_vec();
        }
        let mut surv = vec![false; n];
        let (nx, ny, nz) = (
            self.dims[0] as i64,
            self.dims[1] as i64,
            self.dims[2] as i64,
        );
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let i = self.idx(ix as usize, iy as usize, iz as usize);
                    if free[i] && self.clear_neighborhood(ix, iy, iz, k as i64) {
                        surv[i] = true;
                    }
                }
            }
        }
        surv
    }

    fn clear_neighborhood(&self, ix: i64, iy: i64, iz: i64, k: i64) -> bool {
        let (nx, ny, nz) = (
            self.dims[0] as i64,
            self.dims[1] as i64,
            self.dims[2] as i64,
        );
        let mut dz = -k;
        while dz <= k {
            let mut dy = -k;
            while dy <= k {
                let mut dx = -k;
                while dx <= k {
                    let (cx, cy, cz) = (ix + dx, iy + dy, iz + dz);
                    if cx < 0 || cy < 0 || cz < 0 || cx >= nx || cy >= ny || cz >= nz {
                        return false;
                    }
                    if self.solid(cx as usize, cy as usize, cz as usize) {
                        return false;
                    }
                    dx += 1;
                }
                dy += 1;
            }
            dz += 1;
        }
        true
    }

    /// 给幸存格按 6 连通连通域打房间号（按格索引序 ⇒ 确定性）。返回 (标签, 房间数)。
    fn label_rooms(&self, surv: &[bool]) -> (Vec<u32>, u32) {
        let n = self.cells();
        let mut label = vec![u32::MAX; n];
        let mut rooms = 0u32;
        for s in 0..n {
            if !surv[s] || label[s] != u32::MAX {
                continue;
            }
            label[s] = rooms;
            let mut stack = vec![s];
            while let Some(c) = stack.pop() {
                let (nb, m) = self.nbrs6(c);
                for &q in &nb[..m] {
                    if surv[q] && label[q] == u32::MAX {
                        label[q] = rooms;
                        stack.push(q);
                    }
                }
            }
            rooms += 1;
        }
        (label, rooms)
    }

    /// 多源 BFS：每个空格的「到最近幸存格的距离」。种子按格索引序入队 ⇒ 层序确定。
    fn bfs_dist(&self, free: &[bool], surv: &[bool]) -> Vec<u32> {
        let n = self.cells();
        let mut dist = vec![u32::MAX; n];
        let mut queue: VecDeque<usize> = VecDeque::new();
        for s in 0..n {
            if surv[s] {
                dist[s] = 0;
                queue.push_back(s);
            }
        }
        while let Some(c) = queue.pop_front() {
            let d = dist[c];
            let (nb, m) = self.nbrs6(c);
            for &q in &nb[..m] {
                if free[q] && dist[q] == u32::MAX {
                    dist[q] = d + 1;
                    queue.push_back(q);
                }
            }
        }
        dist
    }

    /// 把房间号补满整个空域：每个空格取**距离 = 自己距离 − 1** 的邻居里**最小房间号**。
    fn fill_rooms(&self, free: &[bool], dist: &[u32], seed_room: Vec<u32>) -> Vec<u32> {
        let mut room = seed_room;
        let mut order: Vec<usize> = (0..self.cells())
            .filter(|&i| free[i] && dist[i] != u32::MAX)
            .collect();
        order.sort_by_key(|&i| (dist[i], i));
        for &c in &order {
            if dist[c] == 0 {
                continue;
            }
            let (nb, m) = self.nbrs6(c);
            let mut best = u32::MAX;
            for &q in &nb[..m] {
                if free[q] && dist[q] != u32::MAX && dist[q] + 1 == dist[c] {
                    best = best.min(room[q]);
                }
            }
            room[c] = best;
        }
        room
    }

    /// 开口格 = 空格里「有邻居房间号与己不同」的那些（两房标签相接的一层皮）。
    fn mark_interface(&self, free: &[bool], room: &[u32]) -> Vec<bool> {
        let n = self.cells();
        let mut out = vec![false; n];
        for i in 0..n {
            if !free[i] || room[i] == u32::MAX {
                continue;
            }
            let (nb, m) = self.nbrs6(i);
            for &q in &nb[..m] {
                if free[q] && room[q] != u32::MAX && room[q] != room[i] {
                    out[i] = true;
                    break;
                }
            }
        }
        out
    }

    /// 开口格按 6 连通聚成 portal：返回 (区域小, 区域大, 法向轴, 成员格)。
    fn group_portals(
        &self,
        free: &[bool],
        room: &[u32],
        is_portal: &[bool],
    ) -> Vec<(u32, u32, u8, Vec<usize>)> {
        let n = self.cells();
        let mut visited = vec![false; n];
        let mut groups: Vec<(u32, u32, u8, Vec<usize>)> = Vec::new();
        for s in 0..n {
            if !is_portal[s] || visited[s] {
                continue;
            }
            visited[s] = true;
            let mut stack = vec![s];
            let mut members = Vec::new();
            let mut trans = [0u32; 3];
            let (mut lo, mut hi) = (u32::MAX, 0u32);
            while let Some(c) = stack.pop() {
                members.push(c);
                lo = lo.min(room[c]);
                hi = hi.max(room[c]);
                let (nb, ax, m) = self.nbrs6_axis(c);
                for k in 0..m {
                    let q = nb[k];
                    if free[q] && room[q] != u32::MAX {
                        lo = lo.min(room[q]);
                        hi = hi.max(room[q]);
                        if room[q] != room[c] {
                            trans[ax[k] as usize] += 1;
                        }
                    }
                    if is_portal[q] && !visited[q] {
                        visited[q] = true;
                        stack.push(q);
                    }
                }
            }
            let mut normal = 0usize;
            if trans[1] > trans[normal] {
                normal = 1;
            }
            if trans[2] > trans[normal] {
                normal = 2;
            }
            groups.push((lo, hi, normal as u8, members));
        }
        groups
    }

    /// 用成员格的 AABB 造 quad：法向轴居中、另两轴取角点并外扩半格（覆盖整格）。
    fn portal_quad(&self, members: &[usize], normal: usize) -> [Vec3; 4] {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for &c in members {
            let p = self.center(c);
            let a = [p.x, p.y, p.z];
            for k in 0..3 {
                lo[k] = lo[k].min(a[k]);
                hi[k] = hi[k].max(a[k]);
            }
        }
        let wide: [usize; 2] = match normal {
            0 => [1, 2],
            1 => [0, 2],
            _ => [0, 1],
        };
        let mid_n = (lo[normal] + hi[normal]) * 0.5;
        let w0 = (lo[wide[0]] + hi[wide[0]]) * 0.5;
        let w1 = (lo[wide[1]] + hi[wide[1]]) * 0.5;
        let h0 = (hi[wide[0]] - lo[wide[0]]) * 0.5 + self.step * 0.5;
        let h1 = (hi[wide[1]] - lo[wide[1]]) * 0.5 + self.step * 0.5;
        let corner = |s0: f32, s1: f32| {
            let mut p = [0f32; 3];
            p[normal] = mid_n;
            p[wide[0]] = w0 + s0 * h0;
            p[wide[1]] = w1 + s1 * h1;
            Vec3::new(p[0], p[1], p[2])
        };
        [
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ]
    }
}
