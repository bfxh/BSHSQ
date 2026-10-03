//! world_step/conversion：**体素↔多边形转换窗口（V2）**——`PLAN-CONVERSION.md` §3.2 的落地。
//!
//! 契约卡（同 `PLAN-COUPLING.md` §3）：**通道 = Position（表示切换）**、**时间戳 = tick 末
//! 各域轮次之后**、**频次 = 事件级且同一体素域同 tick 至多一次**、**对称对 = 旧表示 ↔ 新表示**。
//!
//! ## 窗口四步（本文件的全部语义）
//!
//! 1. **请求**：[`World::request_voxel_conversion`] 只登记意图（事件级），不改几何、不建体。
//! 2. **影子**：本 pass 在 tick 末从源体素域提取零等值面（`surface_mesh`，**整个体积**），
//!    建成一张 `TriMesh`——它**不注册进 `Providers`**，因此不参与求解、不写速度/位置
//!    （防双计账）。窗口期物理照走源表示，**不挖格**。
//! 3. **对拍**：对受影响的动态体各取一个**包围球探针**，分别向源 provider 与影子网格查同一批
//!    接触，比流形摘要（点数 / 最大穿透 / 法向和夹角余弦）。阈值见 [`RECON_DEPTH_TOL`] 等
//!    （**先量后写**，锚写在常量注释里）。
//! 4. **提交式切换**：对拍通过 ⇒ 影子升为正式表示（`add_mesh` 注册静态网格 provider +
//!    静态 marker 体）、源表示退场（整体移格）、`refresh_provider_bounds()`。对拍不过 ⇒
//!    **拒绝切换**（保持源表示）并把原因写进 [`ConversionReport`]——fail-loud、无静默。
//!
//! ### ⚠️ 与计划文本的一处**如实偏差**（不是静默改动）
//!
//! `PLAN-CONVERSION.md` §3.2 ④ 写"影子升为**正式动态体**"、`spawn_box_debris` 先例。
//! 但本仓**现状的体素域是静态标记体**（`Shape::Provider(id)`、`MARKER_TRANSFORM`；
//! §5 的 V5+ 行自己登记了这个差异："动态体素体需要体素域带位姿"）。源表示没有位姿、
//! 没有速度 ⇒ 把它切成动态体等于**凭空造出一个会下落的刚体**（地形/关卡几何会直接掉走），
//! 与"引擎不凭空造动量"（`world_body.rs:154-155`）同一条护栏的方向相反。
//!
//! 因此本片按代码事实走**静态 → 静态**：源是静态 provider，目标也是静态 mesh provider。
//! 交接档 (i)"不继承"在这里是**恒等**（静态体本来就没有可继承的动量，`Δp = ΔL = 0` 逐位）；
//! 动态体素体（V5+）与 (ii) 速度/角速度继承（V3）落地时，本 pass 的提交段要换成
//! `add_trimesh` + `spawn_trimesh_body`（薄壳质量口径 `m = ρ·t·ΣA` 已就位，V2 用不到）。
//!
//! ## 效应键（I3）
//!
//! [`EffectKey::Conversion`] 与破坏域提取（[`EffectKey::VoxelExtraction`]）**互斥**：两者都
//! 消费体素格，同一域同一 tick 占据两个键 ⇒ `Err`（[`ConversionError::EffectConflict`] /
//! 提取侧不再重复登记）。账目按 tick 过滤 ⇒ 体量与运行时长无关。
//!
//! ## 边界（如实）
//!
//! - **不做区域裁剪**：本片转换整个 provider（裁剪面要不要封口属 V3，见 §3.2 与 `surface.rs` 注）。
//! - **不做交接档 (ii)/(iii)**：速度/角速度继承与 warm 残量属 V3/C5；本片只做 (i)。
//! - **不做预算降级**：超顶点预算只拒绝、不降级到凸包/盒（P4 属 V4）。
//! - **默认关**：`PhysConfig::conversion.enabled == false` 时本 pass 首行短路、请求路径直接
//!   `Err` ⇒ 零代际（I0）。
//!
//! 显式导入（不用 `use super::*`）——glob-gate：新文件零通配。
use crate::types::{ConversionError, ConversionReject, ConversionReport, EffectKey};
use crate::{Aabb, Providers, Vec3, World};
use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_terrain::mesh::TriMesh;
use vxl_phys_terrain::voxel::surface_mesh;

/// 对拍用的接触带（m）：与 `PhysConfig::contact_skin` 默认值同量级——探针只在**接近接触**的
/// 位置取值，远离表面的体不参与对拍（否则"两侧都没接触"会稀释摘要）。
const RECON_SKIN: f32 = 0.02;

/// I5 点数容差（个）：两侧流形点数之差的上界。
///
/// **先量后写**：初测锚见本文件 `recon_threshold_anchor` 测试（盒地板场景，源 SDF 路径 vs
/// 影子逐顶点路径，实测 `|Δpoints| = 0`、`|Δmax_depth| = 0.000 m`、`normal_dot = 1.0000`）。
/// 容差取实测 + 一格离散余量（`h/2 = 0.01 m` 的深度口径）⇒ 留 1 个点、2 cm 深度、0.99 余弦。
const RECON_POINTS_TOL: usize = 1;

/// I5 最大穿透深度容差（m）：源 SDF 路径与影子逐顶点采样路径的**几何离散误差**口径
/// （不是逐点相等）——两者生成器不同，差值先量后写。
const RECON_DEPTH_TOL: f32 = 0.02;

/// I5 法向一致性下界（法向和的夹角余弦）。法向和是**粗摘要**（只看整体朝向），
/// 分辨"影子被错位/翻面"这类破坏——金丝雀正是靠它变红。
const RECON_NORMAL_TOL: f32 = 0.99;

/// 一次对拍的摘要（全是**两侧同批实测**，不是期望值）。
#[derive(Clone, Copy, Debug, PartialEq)]
#[doc(hidden)]
pub struct ReconSummary {
    /// 参与对拍的探针数（与源域 AABB 相交的动态体数）。
    pub probes: usize,
    /// 两侧流形点数（同批探针）。
    pub source_points: usize,
    pub shadow_points: usize,
    /// 两侧最大穿透深度（m）。
    pub source_max_depth: f32,
    pub shadow_max_depth: f32,
    /// 法向和夹角余弦（粗摘要；无探针 = 1.0）。
    pub normal_dot: f32,
}

impl Providers {
    /// **I3 认领效应键**：登记"本 tick 该域已被 `key` 消费"。
    ///
    /// 返回 `Err(already)` = 该域本 tick 已被**另一个键**占用（双消费 ⇒ 调用方 fail-loud）。
    /// 顺带裁掉非本 tick 的旧条目 ⇒ 体量 = 本 tick 的消费型效应数（与运行时长无关）。
    pub(crate) fn claim_effect(
        &mut self,
        id: u32,
        tick: u64,
        key: EffectKey,
    ) -> Result<(), EffectKey> {
        self.conversion.effects.retain(|&(_, t, _)| t == tick);
        if let Some(&(_, _, prev)) = self
            .conversion
            .effects
            .iter()
            .find(|&&(p, t, _)| p == id && t == tick)
        {
            return if prev == key { Ok(()) } else { Err(prev) };
        }
        self.conversion.effects.push((id, tick, key));
        Ok(())
    }

    /// 本 tick 已登记的**其它**域数（同一域的重复请求不算新事件 ⇒ 事件预算与重复请求分开判）。
    pub(crate) fn conversion_events_excluding(&self, tick: u64, id: u32) -> usize {
        self.conversion
            .effects
            .iter()
            .filter(|&&(p, t, _)| t == tick && p != id)
            .count()
    }

    /// 本 tick 待转换的域 id（升序 ⇒ 固定扫描序 ⇒ 确定性；同一域至多一条）。
    pub(crate) fn pending_conversions(&self, tick: u64) -> Vec<u32> {
        let mut v: Vec<u32> = self
            .conversion
            .effects
            .iter()
            .filter(|&&(_, t, k)| t == tick && k == EffectKey::Conversion)
            .map(|&(p, _, _)| p)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// **V2 转换窗口的域侧 API（扩展 trait）**。
///
/// 为什么不是 `impl World`：`god.gate.json` 把 `World` 的方法数登记为**只准减**的债务
/// （门面对每个新域都添 `add_*` / `*_pass` 会失控），处置条目明写"域侧 API 拆成扩展 trait
/// （impl FluidExt for World 等），门面只留管线组装"。本片照此办理——`impl ... for World`
/// 块按 `god_gate.py` 的规则不计入该类型（`impl Trait for Type` 记的是 trait 的实现）。
///
/// `#[doc(hidden)]` 的六个是**内部实现**（pass 本体与助手）：它们必须与公开 API 同在一个
/// trait 上，才能既不占 `World` 的方法位、又不给门面添新方法。
pub trait VoxelConversionExt {
    /// 请求把第 `id` 个体素域在**本 tick 末**转成网格表示（事件级；同一域同一 tick 至多一次）。
    fn request_voxel_conversion(&mut self, id: u32) -> Result<(), ConversionError>;
    /// 上一 tick 的转换结果（诊断 / 判据出口；无转换 = `None`）。
    fn last_conversion(&self) -> Option<&ConversionReport>;
    #[doc(hidden)]
    fn claim_extraction_effect(&mut self, id: u32) -> Result<(), EffectKey>;
    #[doc(hidden)]
    fn conversion_pass(&mut self);
    #[doc(hidden)]
    fn convert_one(&mut self, id: u32, tick: u64);
    #[doc(hidden)]
    fn store_report(&mut self, rep: ConversionReport, reject: Option<ConversionReject>);
    #[doc(hidden)]
    fn clear_voxel(&mut self, id: u32);
    #[doc(hidden)]
    fn reconcile(&self, id: u32, shadow: &TriMesh) -> ReconSummary;
}

impl VoxelConversionExt for World {
    /// **请求转换**（事件级）：把第 `id` 个体素域标记为"本 tick 末转成网格表示"。
    ///
    /// 只登记意图——不提取、不建体、不动几何；实际动作在本 tick 末的 `conversion_pass`。
    /// 默认关（`PhysConfig::conversion.enabled == false`）⇒ `Err(Disabled)`：**不静默丢弃**。
    fn request_voxel_conversion(&mut self, id: u32) -> Result<(), ConversionError> {
        let cfg = self.config.conversion;
        if !cfg.enabled {
            return Err(ConversionError::Disabled);
        }
        if self.providers.voxel(id).is_none() {
            return Err(ConversionError::NotVoxel(id));
        }
        let tick = self.tick;
        if cfg.max_events != 0
            && self.providers.conversion_events_excluding(tick, id) >= cfg.max_events
        {
            return Err(ConversionError::EventBudget {
                tick,
                max: cfg.max_events,
            });
        }
        // 认领是**一步**完成的（先查后认领会有 TOCTOU 缝隙）：失败即按占用者 fail-loud。
        self.providers
            .claim_effect(id, tick, EffectKey::Conversion)
            .map_err(|already| match already {
                EffectKey::Conversion => ConversionError::Duplicate { provider: id, tick },
                other => ConversionError::EffectConflict {
                    provider: id,
                    tick,
                    already: other,
                },
            })
    }

    /// 破坏域提取侧的效应键登记（同一 key 重复登记是恒等；撞上 `Conversion` ⇒ 调用方 fail-loud）。
    fn claim_extraction_effect(&mut self, id: u32) -> Result<(), EffectKey> {
        let tick = self.tick;
        self.providers
            .claim_effect(id, tick, EffectKey::VoxelExtraction)
    }

    /// 上一 tick 的转换结果（诊断 / 判据出口；无转换 = `None`）。
    fn last_conversion(&self) -> Option<&ConversionReport> {
        self.providers.conversion.last.as_ref()
    }

    /// **转换窗口 pass**（tick 末、各域轮次之后）。默认关 ⇒ 首行短路（零代际，I0）。
    fn conversion_pass(&mut self) {
        if !self.config.conversion.enabled {
            return;
        }
        let tick = self.tick;
        let pending = self.providers.pending_conversions(tick);
        for id in pending {
            self.convert_one(id, tick);
        }
    }

    /// 单个域的窗口四步：提取影子 → 对拍 → 提交或拒绝（拒绝也留档）。
    fn convert_one(&mut self, id: u32, tick: u64) {
        let mut rep = ConversionReport {
            provider: id,
            tick,
            min_normal_dot: 1.0,
            ..Default::default()
        };
        let Some(extracted) = self.providers.voxel(id).map(surface_mesh) else {
            self.store_report(rep, Some(ConversionReject::EmptyMesh));
            return;
        };
        rep.vertices = extracted.points.len();
        rep.tris = extracted.tris.len();
        if rep.vertices == 0 || rep.tris == 0 {
            self.store_report(rep, Some(ConversionReject::EmptyMesh));
            return;
        }
        // **零丢弃**：`TriMesh::new` 只丢越界索引 ⇒ 先自检，丢一个就拒绝（不悄悄少三角）。
        let n = extracted.points.len() as u32;
        if extracted
            .tris
            .iter()
            .any(|t| t[0] >= n || t[1] >= n || t[2] >= n)
        {
            self.store_report(rep, Some(ConversionReject::DegenerateMesh));
            return;
        }
        let cap = self.config.conversion.max_vertices;
        if cap != 0 && rep.vertices > cap {
            self.store_report(rep, Some(ConversionReject::VertexBudget));
            return;
        }
        let shadow = TriMesh::new(extracted.points.clone(), extracted.tris.clone());
        let rec = self.reconcile(id, &shadow);
        rep.source_points = rec.source_points;
        rep.shadow_points = rec.shadow_points;
        rep.source_max_depth = rec.source_max_depth;
        rep.shadow_max_depth = rec.shadow_max_depth;
        rep.min_normal_dot = rec.normal_dot;
        if !recon_passed(&rec) {
            self.store_report(rep, Some(ConversionReject::Reconciliation));
            return;
        }
        // 提交式切换（静态 → 静态；模块头"如实偏差"一节写清了为什么不切动态体）。
        // `add_mesh` 内部：`build_grid()` → `providers.push_mesh` → `provider_bounds.push` →
        // 静态 marker 体；因此目标表示的位姿与源表示同为 `MARKER_TRANSFORM` ⇒ 位姿不跳。
        let marker = self.add_mesh(TriMesh::new(extracted.points, extracted.tris));
        let mesh = self.provider_id_of(marker);
        self.clear_voxel(id);
        self.refresh_provider_bounds();
        rep.committed = true;
        rep.mesh = mesh;
        rep.body = Some(marker);
        self.store_report(rep, None);
    }

    /// 落盘一次结果（`rejected` = 拒绝原因；`None` + `committed=false` 表示空域等前置失败）。
    fn store_report(&mut self, mut rep: ConversionReport, reject: Option<ConversionReject>) {
        rep.rejected = reject;
        self.providers.conversion.last = Some(rep);
    }

    /// 整体移格（提交式切换的第 4 步）：把该域的全部占据格清空。
    ///
    /// 走现成的 `extract_where`（整范围全真）——**不新增 terrain API**；其返回值（贪心盒）
    /// 在这里是副产物、直接丢弃：本片要的是"源表示退场"，不是盒碎块。
    fn clear_voxel(&mut self, id: u32) {
        let Some(vol) = self.providers.voxel_mut(id) else {
            return;
        };
        let (o, s) = (vol.origin(), vol.step());
        let (nx, ny, nz) = vol.dims();
        let max = o + Vec3::new(nx as f32, ny as f32, nz as f32) * s;
        let _ = vol.extract_where(o, max, |_, _, _| true);
    }

    /// **影子对拍**（I5）：对每颗与源域 AABB 相交的动态体取一个包围球探针，两侧各查一次。
    fn reconcile(&self, id: u32, shadow: &TriMesh) -> ReconSummary {
        let mut rec = ReconSummary {
            probes: 0,
            source_points: 0,
            shadow_points: 0,
            source_max_depth: 0.0,
            shadow_max_depth: 0.0,
            normal_dot: 1.0,
        };
        let source_box = self.providers.bounds(id).map(|b| b.grown(RECON_SKIN));
        let mut src_n = Vec3::ZERO;
        let mut shd_n = Vec3::ZERO;
        for i in 0..self.bodies.len() {
            if !self.bodies.is_dynamic(i) {
                continue;
            }
            let (pos, _) = self.bodies.pose(i);
            let r = self.bodies.shape[i].bounding_sphere_radius();
            if let Some(sb) = source_box {
                let probe = Aabb {
                    min: pos - Vec3::splat(r),
                    max: pos + Vec3::splat(r),
                };
                if !sb.overlaps(&probe) {
                    continue;
                }
            }
            let mut a: Vec<InteropContact> = Vec::new();
            let mut b: Vec<InteropContact> = Vec::new();
            let _ = self
                .providers
                .contacts_sphere(id, pos, r, RECON_SKIN, &mut a);
            let _ = shadow.contacts_sphere(0, pos, r, RECON_SKIN, &mut b);
            rec.probes += 1;
            rec.source_points += a.len();
            rec.shadow_points += b.len();
            for c in &a {
                rec.source_max_depth = rec.source_max_depth.max(c.depth);
                src_n += c.normal;
            }
            for c in &b {
                rec.shadow_max_depth = rec.shadow_max_depth.max(c.depth);
                shd_n += c.normal;
            }
        }
        rec.normal_dot = normal_dot(src_n, shd_n);
        rec
    }
}

/// 法向和的夹角余弦（粗摘要；任一侧为空 = 1.0，表示"没有可比的朝向信息"）。
fn normal_dot(a: Vec3, b: Vec3) -> f32 {
    let (la, lb) = (a.length(), b.length());
    if la <= 1e-9 || lb <= 1e-9 {
        return 1.0;
    }
    let d = (a.dot(b)) / (la * lb);
    d.clamp(-1.0, 1.0)
}

/// I5 判定：两侧流形摘要是否在阈值内（阈值先量后写，见各常量注释）。
fn recon_passed(r: &ReconSummary) -> bool {
    let dp = r.source_points.abs_diff(r.shadow_points);
    let dd = (r.source_max_depth - r.shadow_max_depth).abs();
    dp <= RECON_POINTS_TOL && dd <= RECON_DEPTH_TOL && r.normal_dot >= RECON_NORMAL_TOL
}

#[cfg(test)]
mod tests {
    // 显式导入（glob-gate：新文件零通配）。
    use super::{
        recon_passed, ReconSummary, VoxelConversionExt, RECON_DEPTH_TOL, RECON_NORMAL_TOL,
        RECON_POINTS_TOL,
    };
    use crate::{PhysConfig, Quat, Shape, Vec3, World};
    use vxl_phys_terrain::mesh::TriMesh;
    use vxl_phys_terrain::voxel::{surface_mesh, VoxelVolume};

    /// 与 `tests/voxel_mesh_extract_rest.rs` 同源的地板（8×2×8、边长 0.5、顶面 y=1.0）。
    fn floor() -> VoxelVolume {
        let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
        v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
        v
    }

    /// 建场：体素地板 + 一只盒（半高 0.5）落在上面 → 静置 → (world, provider id, 影子网格)。
    fn resting_scene() -> (World, u32, TriMesh) {
        let mut w = World::new(PhysConfig::default());
        let marker = w.add_voxel(floor());
        let pid = w.provider_id_of(marker).unwrap_or(u32::MAX);
        assert!(pid != u32::MAX, "体素 marker 应带 provider id");
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::new(0.25, 1.6, 0.25),
            Quat::IDENTITY,
            1000.0,
        );
        for _ in 0..240 {
            w.step();
        }
        let m = surface_mesh(&floor());
        (w, pid, TriMesh::new(m.points, m.tris))
    }

    /// **阈值锚（先量后写的现场）**：打印静置场景下源 provider 与影子的同批摘要。
    /// 判据只打印不设阈——设阈在常量处（本文件头 `RECON_*` 的注释引这条读数）。
    #[test]
    fn recon_threshold_anchor() {
        let (w, pid, shadow) = resting_scene();
        let r = w.reconcile(pid, &shadow);
        println!(
            "  I5 锚：探针 {} · 点数 源 {} / 影子 {} · 最大穿透 源 {:.5} / 影子 {:.5} · normal_dot {:.5}",
            r.probes,
            r.source_points,
            r.shadow_points,
            r.source_max_depth,
            r.shadow_max_depth,
            r.normal_dot
        );
        assert!(r.probes > 0, "静置场景应至少命中一个探针（否则判据空过）");
        assert!(
            recon_passed(&r),
            "同一位姿的源/影子应当通过对拍：{r:?}（阈值 {RECON_POINTS_TOL} 点 / {RECON_DEPTH_TOL} m / {RECON_NORMAL_TOL}）"
        );
    }

    /// **金丝雀**：人为把影子整体抬高 0.5 m ⇒ 对拍判据**必红**（防"空过"）。
    /// 期望值锚到规则之外的观测：0.5 m 位移 ≫ 阈值 0.02 m，且探针取自源 provider 的真实接触。
    #[test]
    fn misplaced_shadow_makes_reconciliation_red() {
        let (w, pid, shadow) = resting_scene();
        let lifted = surface_mesh(&floor());
        let points: Vec<Vec3> = lifted
            .points
            .iter()
            .map(|p| *p + Vec3::new(0.0, 0.5, 0.0))
            .collect();
        let bad = TriMesh::new(points, lifted.tris);
        let good = w.reconcile(pid, &shadow);
        let misplaced = w.reconcile(pid, &bad);
        println!(
            "  金丝雀：正确影子 depths {:.5}/{:.5} dot {:.5}；错位影子 depths {:.5}/{:.5} dot {:.5}",
            good.source_max_depth,
            good.shadow_max_depth,
            good.normal_dot,
            misplaced.source_max_depth,
            misplaced.shadow_max_depth,
            misplaced.normal_dot
        );
        assert!(recon_passed(&good), "同一位姿的影子本该通过：{good:?}");
        assert!(
            !recon_passed(&misplaced),
            "错位 0.5 m 的影子必须判红（否则判据空过）：{misplaced:?}"
        );
    }

    /// 摘要判据自身的分辨力（不依赖场景）：点数/深度/法向任一项越界都要红。
    #[test]
    fn recon_predicate_has_resolution() {
        let ok = ReconSummary {
            probes: 1,
            source_points: 4,
            shadow_points: 4,
            source_max_depth: 0.01,
            shadow_max_depth: 0.01,
            normal_dot: 1.0,
        };
        assert!(recon_passed(&ok), "基线摘要应通过");
        let mut more = ok;
        more.shadow_points += RECON_POINTS_TOL + 1;
        assert!(!recon_passed(&more), "点数越界必须红");
        let mut deep = ok;
        deep.shadow_max_depth += RECON_DEPTH_TOL + 0.001;
        assert!(!recon_passed(&deep), "深度越界必须红");
        let mut flip = ok;
        flip.normal_dot = RECON_NORMAL_TOL - 0.001;
        assert!(!recon_passed(&flip), "法向越界必须红");
    }
}
