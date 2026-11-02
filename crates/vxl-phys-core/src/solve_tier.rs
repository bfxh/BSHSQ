//! 求解**卡上档**的接口契约（`PLAN-gpu.md` §30 B 片：把 B1+B3+B4 接进 `World` 的求解位置）。
//!
//! **与窄相档（`narrow_tier`）的分工**：窄相档把「几何求交」搬上卡，实测**单对慢 4–6×**
//! （§3.23：卡上 155–254 ns/对 vs 主机 40 ns）⇒ 路线修正为「**窄相留主机、构建+求解上卡**」。
//! 本文件就是后者的契约：主机给**裸字**（体表/接触表/CSR/参数），卡上回**速度表**。
//!
//! **为什么只过裸字**（同 `narrow_tier` 的理由）：`Manifold` 在 `vxl-phys-narrow`，core 不能反向
//! 依赖；而 GPU 侧只认布局 ⇒ 打包/解包都留在调用方（`vxl-phys-solver::tier_phase`）。
//!
//! **布局三处同源**：本文件的常量 = `solve_tier.wgsl` 的偏移 = `vxl-phys-gpu::solve_tier` 的常量
//! （后者 `pub use` 本文件）。改一处要同步另两处。
//!
//! ⚠️ **口径**：卡上是 **Jacobi**（跨接触读同一份速度表）+ **无暖启动**（冲量表每次求解清零）+
//! 切向基由法线现算，与默认档的「顺序 GS + 暖启动 + 各向异性基」**不同源**⇒ 只作显式 opt-in 档，
//! 质量判据另立（`PLAN-gpu.md` B6），不进金样逐位门。

use crate::BodySet;

/// 逐体记录（44 B）：`pos3 · quat4 · inv_mass · local_inv_i3`（= 探针 `bd` 的 11 字）。
pub const BD_WORDS: usize = 11;
/// 逐体速度（24 B）：`lin3 · ang3`。
pub const VEL_WORDS: usize = 6;
/// 逐体世界逆惯量（36 B，列主序）——卡上 `make_iw` 的产物，主机不传。
pub const IW_WORDS: usize = 9;
/// 接触输入（48 B）：`a · b · point3 · normal3 · depth · mu · e · pad`。
pub const CT_WORDS: usize = 12;
/// 构建产物（48 B）：`ra3 · rb3 · kn · kt1 · kt2 · kc · rhs · cfm`（卡上 `build` 的产物）。
pub const CON_WORDS: usize = 12;
/// 每条接触的冲量（法向 + 两个切向）。
pub const IMP_WORDS: usize = 3;

/// 一次求解的标量参数（由 `vxl-phys-solver` 的 `SolverParams` 导出，见 `tier_params`）。
#[derive(Clone, Copy, Debug)]
pub struct SolveTierParams {
    /// 子步长（秒）。
    pub dt: f32,
    /// `1/dt`（speculative 项用）。
    pub inv_dt: f32,
    /// 去穿透偏置速率：动态对 / 静态侧。
    pub erp_dyn: f32,
    pub erp_static: f32,
    /// CFM 正则化：动态对 / 静态侧。
    pub cfm_dyn: f32,
    pub cfm_static: f32,
    /// 允许穿透余量。
    pub slop: f32,
    /// 修正速度上限。
    pub max_corr: f32,
    /// 弹性生效阈值（`PhysConfig::restitution_threshold`）。
    pub e_threshold: f32,
    /// 偏置开关（`1.0` = 带偏置趟，`0.0` = 无偏置趟 = CPU `solve_unbiased` 同位）。
    pub bias: f32,
    /// Jacobi 扫描次数（每次调用内 `dispatch` 的轮数）。
    pub sweeps: u32,
}

impl SolveTierParams {
    /// 卡上 `Params` uniform 块的字节序（16 槽 × 4 B = 64 B；与 `solve_tier.wgsl` 逐字对应）。
    pub fn to_words(&self) -> [u32; 16] {
        let mut w = [0u32; 16];
        // `[0..4]` 的规模字由后端填（`n_c`/`n_b`/`n_upd`/`sweeps`）。
        w[4] = self.dt.to_bits();
        w[5] = self.inv_dt.to_bits();
        w[6] = self.erp_dyn.to_bits();
        w[7] = self.erp_static.to_bits();
        w[8] = self.slop.to_bits();
        w[9] = self.max_corr.to_bits();
        w[10] = self.cfm_dyn.to_bits();
        w[11] = self.cfm_static.to_bits();
        w[12] = self.e_threshold.to_bits();
        w[13] = self.bias.to_bits();
        w[3] = self.sweeps;
        w
    }
}

/// 一次求解的输入（裸字表）。
pub struct SolveTierIn<'a> {
    /// 逐体记录（`BD_WORDS` 字/体，含**睡眠体清零**后的质量）。
    pub bd: &'a [f32],
    /// 逐体速度（`VEL_WORDS` 字/体）。
    pub vel: &'a [f32],
    /// 逐接触输入（`CT_WORDS` 字/接触）。
    pub ct: &'a [f32],
    /// CSR：`off[b]..off[b+1]` → `items` 里的接触序号（升序，确定性）。
    pub off: &'a [u32],
    pub items: &'a [u32],
    pub params: SolveTierParams,
}

/// **卡上求解后端**：吃 `SolveTierIn`，回**同一长度**的速度表。
/// 失败（无适配器 / 超容量 / 核跑不通）⇒ `Err`，调用方**整趟回退 CPU**（不得半张表）。
pub trait SolveTierBackend {
    /// 一次求解 = 一次提交（散写 → `make_iw` → `build` → `sweeps × (sweep, apply)`）+ 速度回读。
    ///
    /// **体表差量（B1）由后端自己管**：后端留着上一份 `bd`，每趟只把**变动记录**散写进卡上常驻表
    /// （首趟全量）。放在后端而不是调用方，是因为"卡上现在有多少行、哪些变了"只有后端知道。
    fn solve_run(&self, inp: &SolveTierIn) -> Result<Vec<f32>, String>;

    /// 后端名（诊断用；默认空）。
    fn tier_name(&self) -> &str {
        ""
    }
}

/// 逐体打包（与探针 `extract` 同式）：`pos3 · quat4 · inv_mass · local_inv_i3` + 独立速度表。
///
/// ⚠️ **睡眠体按"质量无限"打包**（`inv_mass`/`local_inv_i` 归零）：卡上既不推它的速度，也不让它
/// 分担冲量——与 CPU「睡着的体不吃冲量、但能把对面顶开」同口径。写回侧同样只认清醒动体
/// （见 `tier_phase::write_back`）。
pub fn pack_bodies(bodies: &BodySet) -> (Vec<f32>, Vec<f32>) {
    let n = bodies.len();
    let mut bd = Vec::with_capacity(n * BD_WORDS);
    let mut vel = Vec::with_capacity(n * VEL_WORDS);
    for i in 0..n {
        let p = bodies.position[i];
        let q = bodies.rot(i);
        let li = if bodies.awake[i] && bodies.is_dynamic(i) {
            bodies.local_inv_inertia[i]
        } else {
            crate::Vec3::ZERO
        };
        let im = if bodies.awake[i] && bodies.is_dynamic(i) {
            bodies.inv_mass[i]
        } else {
            0.0
        };
        bd.extend_from_slice(&[p.x, p.y, p.z, q.x, q.y, q.z, q.w, im, li.x, li.y, li.z]);
        let lv = bodies.linvel[i];
        let av = bodies.angvel(i);
        vel.extend_from_slice(&[lv.x, lv.y, lv.z, av.x, av.y, av.z]);
    }
    (bd, vel)
}

/// 一条接触的输入（形参棘轮 >7 ⇒ 走结构体；`vxl-phys-solver::tier_phase` 逐点填）。
pub struct ContactRow {
    pub a: u32,
    pub b: u32,
    pub point: crate::Vec3,
    pub normal: crate::Vec3,
    pub depth: f32,
    pub mu: f32,
    pub e: f32,
}

/// 逐接触打包（`ct` 表）：体号以 **f32 数值**存（< 2^24 精确，与探针同法；别存位模式——
/// 非规格化会被卡上冲刷，见 `PLAN-gpu.md` §17.2 踩坑）。
pub fn pack_contact(out: &mut Vec<f32>, r: &ContactRow) {
    out.extend_from_slice(&[
        r.a as f32, r.b as f32, r.point.x, r.point.y, r.point.z, r.normal.x, r.normal.y,
        r.normal.z, r.depth, r.mu, r.e, 0.0,
    ]);
}

/// **CSR 计数排序**（主机侧，O(接触数)）：每体一段升序接触号 ⇒ 卡上体累加的序确定。
/// 返回 `(off, items)`；`off` 长度 `n_b + 1`。
pub fn build_csr(ct: &[f32], n_b: usize) -> (Vec<u32>, Vec<u32>) {
    let mut counts = vec![0u32; n_b + 1];
    for c in ct.as_chunks::<CT_WORDS>().0 {
        for &idx in &c[..2] {
            let b = idx as usize;
            if b < n_b {
                counts[b + 1] += 1;
            }
        }
    }
    for i in 0..n_b {
        counts[i + 1] = counts[i + 1].saturating_add(counts[i]);
    }
    let mut items = vec![0u32; counts[n_b] as usize];
    let mut cursor: Vec<usize> = counts[..n_b].iter().map(|&v| v as usize).collect();
    for (ci, c) in ct.as_chunks::<CT_WORDS>().0.iter().enumerate() {
        for &idx in &c[..2] {
            let b = idx as usize;
            if b < n_b {
                items[cursor[b]] = ci as u32;
                cursor[b] += 1;
            }
        }
    }
    (counts, items)
}

/// **差量比对**（B1）：与上一份 `bd` 逐记录比，返回变动的体号；缓存顺手更新。
/// 缓存缺失或记录数变了 ⇒ 本趟按"全部变动"（首帧 / 体数变化）。
pub fn diff_records(cache: &mut Option<Vec<f32>>, bd: &[f32]) -> Vec<u32> {
    let n_rec = bd.len() / BD_WORDS;
    let mut changed: Vec<u32> = Vec::new();
    match cache.as_mut() {
        Some(c) if c.len() == bd.len() => {
            for i in 0..n_rec {
                let o = i * BD_WORDS;
                if c[o..o + BD_WORDS] != bd[o..o + BD_WORDS] {
                    changed.push(i as u32);
                    c[o..o + BD_WORDS].copy_from_slice(&bd[o..o + BD_WORDS]);
                }
            }
        }
        _ => {
            changed.extend(0..n_rec as u32);
            *cache = Some(bd.to_vec());
        }
    }
    changed
}

#[cfg(test)]
mod tests {
    // glob 门对新文件零基线 ⇒ 显式导入（含测试模块）。
    use super::{
        build_csr, diff_records, pack_contact, ContactRow, SolveTierParams, BD_WORDS, CT_WORDS,
    };
    use crate::Vec3;

    fn row(a: u32, b: u32) -> ContactRow {
        ContactRow {
            a,
            b,
            point: Vec3::ZERO,
            normal: Vec3::Z,
            depth: 0.0,
            mu: 0.5,
            e: 0.0,
        }
    }

    #[test]
    fn csr_orders_contacts_ascending_per_body() {
        // 三条接触：0-2、1-2、0-1 => 体 2 的段应是 [0, 1]（升序，确定性）。
        let mut ct = Vec::new();
        for (a, b) in [(0u32, 2u32), (1, 2), (0, 1)] {
            pack_contact(&mut ct, &row(a, b));
        }
        let (off, items) = build_csr(&ct, 3);
        assert_eq!(off.len(), 4);
        assert_eq!(
            items[off[2] as usize..off[3] as usize],
            [0, 1],
            "体 2 的接触段必须升序"
        );
        assert_eq!(off[3] as usize, 6, "两端各记一次 => 条目数 = 2x接触数");
    }

    #[test]
    fn diff_reports_only_moved_records() {
        // 不用 `.clone()`（clone 门按文件计数，测试模块也算）=> 每份数据各自现造。
        let mut cache: Option<Vec<f32>> = None;
        let first = vec![1.0f32; BD_WORDS * 3];
        assert_eq!(diff_records(&mut cache, &first), [0, 1, 2], "首帧全量");
        let mut second = vec![1.0f32; BD_WORDS * 3];
        second[BD_WORDS * 2] = 9.0;
        assert_eq!(diff_records(&mut cache, &second), [2], "只报动了的那条");
        let shrunk = vec![1.0f32; BD_WORDS * 2];
        assert_eq!(
            diff_records(&mut cache, &shrunk),
            [0, 1],
            "记录数变了要全量重传"
        );
    }

    #[test]
    fn params_land_on_the_shader_slots() {
        let p = SolveTierParams {
            dt: 1.0 / 240.0,
            inv_dt: 240.0,
            erp_dyn: 11.0,
            erp_static: 12.0,
            cfm_dyn: 13.0,
            cfm_static: 14.0,
            slop: 15.0,
            max_corr: 16.0,
            e_threshold: 17.0,
            bias: 1.0,
            sweeps: 12,
        };
        let w = p.to_words();
        assert_eq!(w[3], 12, "sweeps 必须落在槽 3（核里读它）");
        assert_eq!(f32::from_bits(w[5]), 240.0);
        assert_eq!(f32::from_bits(w[13]), 1.0, "bias 落在槽 13");
    }

    #[test]
    fn contact_row_lands_ct_words_per_contact() {
        let mut ct = Vec::new();
        pack_contact(&mut ct, &row(3, 7));
        pack_contact(&mut ct, &row(0, 1));
        assert_eq!(ct.len(), 2 * CT_WORDS, "每条接触恰好 CT_WORDS 字");
        assert_eq!(ct[0], 3.0, "体号以 f32 数值存（别存位模式）");
    }
}
