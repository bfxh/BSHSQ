//! mesh_provider：从 mesh.rs 按职责拆出（纯搬移，语义未改）。
use super::*;
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Quat, Vec3};
impl ProviderColliders for TriMesh {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        Some(self.world_bounds())
    }

    /// 点查询：`depth = −sd`（**带符号**面距离 `sd = (p − q)·n`：外侧 sd>0、面里侧 sd<0），
    /// 即 **depth = 穿透量**（正 = 点已在面里侧）；法线 = 面法线。
    /// 带内判据 `depth > −skin`（等价 `sd < skin`）⇒ 地表上方 skin 内仍产生**预期接触**
    /// （speculative margin，与高度场路径 `depth = h − v.y` + `depth > −skin` 同一口径）。
    ///
    /// **符号修正**（2026-09-15）：旧式用无符号距离 `d`，体穿到面里侧时 `depth = skin − d`
    /// 仍为**负**（被求解器当间隙）⇒ 单向屏障失效、体一路下沉（实测 arena 三角网场景
    /// 穿透 0.17 m）。改用带符号 `sd`：内侧变正 ⇒ 正常顶出。
    ///
    /// **偏置修正**（2026-09-21，P5）：上式一度写成 `depth = skin − sd`——它在**外侧**也给出
    /// 正 depth（点到面之间还差 skin 时就算"穿透"），求解器据此把体顶到 `sd ≈ skin`
    /// ⇒ **静置时体比地表高约一个 skin**（实测：平地三角网上盒体高 3.5 cm、起伏网 1.4–2.0 cm；
    /// 而同档的高度场路径贴住 ⇒ 两条地形路口径不一致）。改成 `depth = −sd` 后
    /// **外侧 depth ≤ 0**（不再有虚假穿透），内侧仍为正 ⇒ 可恢复性不变（见 `shape-trimesh-sunk`）。
    ///
    /// ⚠️ **本查询不看 `MeshSemantics`**（`Shell` 只在 `contacts_sphere` 折法线，见那里）：圆柱/
    /// 圆锥这一族的"体心在哪一侧"折法在**采样层**（`provider.rs::ring_provider_contacts`），
    /// 这里再折一次会把那层的折法**折回去**（两层互消）。
    fn contacts_point(&self, _id: u32, p: Vec3, skin: f32, out: &mut Vec<InteropContact>) -> bool {
        let Some((_, q, n, ti)) = self.closest(p, skin) else {
            return true;
        };
        let sd = (p - q).dot(n);
        let depth = -sd;
        if depth < -skin {
            return true; // 支持查询；不在带内
        }
        TRI_PUSHES.fetch_add(1, Ordering::Relaxed);
        if self.tri_aabb_lb2(ti, p) > skin * skin {
            TRI_PUSH_FAR.fetch_add(1, Ordering::Relaxed);
        }
        out.push(InteropContact {
            point: q,
            normal: n,
            depth,
            feature: (ti as u32) + 1,
        });
        true
    }

    /// 球查询：`depth = r − sd(球心)`（**带符号**面距离；同点查询的符号修正）。
    ///
    /// **法线口径按逐网语义取**（`oriented`）：`Solid`（默认）用**环绕法线**定符号 —— 球心在地表
    /// **下方**时 `depth = r + |sd|` = **真的穿透量**（`rest_height_contract.rs` 的
    /// `mesh_sphere_inside_is_ejected` 就靠它把陷网球顶出去：y = −0.20 → ≈ r）。`Shell` 则把法线
    /// 折向**查询球心那一侧**、深度改用 `|sd|` ⇒ 从**背面**来的球/胶囊（`ccd_thin_shell.rs` ⑥）
    /// 被推回**来处**，而不是算出一个指向"继续穿过去"的深度与法线（那会让 CCD 的闭合判据
    /// `vn < 0` 永不成立：探针实测逐子步 360 采样 0 命中、直穿 395 m）。
    ///
    /// 实测（2026-10-15 复跑，⑥ 形状矩阵 60 m/s × 4×4 m 单页薄板标 `Shell`）：球停在 **−0.400**、
    /// 胶囊 **−0.198**（各自前缘贴住壳面、`vx` 归零）；不标记（`Solid`）时两族都 **x = 395 直穿**。
    /// 正面来的场景与 `Solid` **逐位相同** ⇒ 换代账零。本条把 P19 的"待裁决"落成"逐网标记"。
    fn contacts_sphere(
        &self,
        _id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let Some((_, q, n, ti)) = self.closest(center, radius + skin) else {
            return true;
        };
        let sd_raw = (center - q).dot(n);
        let (n, sd) = self.oriented(n, sd_raw);
        let depth = radius - sd;
        if depth < -skin {
            return true;
        }
        TRI_PUSHES.fetch_add(1, Ordering::Relaxed);
        if self.tri_aabb_lb2(ti, center) > (radius + skin) * (radius + skin) {
            TRI_PUSH_FAR.fetch_add(1, Ordering::Relaxed);
        }
        out.push(InteropContact {
            point: q,
            normal: n,
            depth,
            feature: (ti as u32) + 1,
        });
        true
    }

    /// 盒查询：8 顶点 + 6 面心 + **12 棱相交**（"面穿过体"，P19）—— 口径与实现见 `mesh_cross`。
    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        mesh_cross::contacts_box(
            self,
            &mesh_cross::BoxQuery {
                id,
                half,
                pos,
                rot,
                skin,
            },
            out,
        )
    }
}
