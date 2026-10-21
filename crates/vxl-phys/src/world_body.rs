//! world_body：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

pub(crate) mod voronoi_fracture;

impl World {
    /// 窄相之后、求解之前：把「动体 × provider」对的接近速度与接触点快照进 `self.impacts`。
    ///
    /// **口径 = 本 tick 的峰值，不是最后一个子步的瞬时值**（2026-10-10 更正）：旧实现每子步
    /// `clear` 重记，而快撞通常在第 1 个子步就被解掉 ⇒ 末子步只剩 ~0.1–0.2 m/s ⇒ 按"每次
    /// `step` 之后调用"设计的破坏管线在默认 2 子步下**一块碎块都挖不出**（实测 12 m/s 撞击：
    /// `substeps=1` 108 块 vs 默认 **0** 块）。现在首子步清空、其余子步**取峰值**（见
    /// `merge_impact`）；速度仍是各子步**解算前**的值。判据 `destruction_substep_invariance`。
    pub(crate) fn record_impacts(&mut self, first_substep: bool) {
        if first_substep {
            self.impacts.clear();
        }
        for m in &self.manifolds {
            let (sa, sb) = (
                self.bodies.shape[m.a as usize],
                self.bodies.shape[m.b as usize],
            );
            let (other, prov, other_is_a) = match (sa, sb) {
                (Shape::Provider(p), _) => (m.b, p, false),
                (_, Shape::Provider(p)) => (m.a, p, true),
                _ => continue,
            };
            if !self.bodies.is_dynamic(other as usize) {
                continue;
            }
            let v = self.bodies.linvel[other as usize];
            // 法线 a→b：other 在 a 侧 ⇒ 接近速度 = v·n；在 b 侧 ⇒ −v·n。
            let approach = if other_is_a {
                v.dot(m.normal)
            } else {
                -v.dot(m.normal)
            };
            let n = m.points.len().max(1) as f32;
            let mut c = Vec3::ZERO;
            for p in m.points.iter() {
                c += p.point;
            }
            merge_impact(
                &mut self.impacts,
                ImpactRecord {
                    provider: prov,
                    body: other,
                    point: c * (1.0 / n),
                    approach,
                    velocity: v,
                },
            );
        }
    }
}

/// 合并一条冲击记录：同一 `(provider, body)` 只留**接近速度最大**的那条（并列留先出现的）。
/// 顺序 = 首次出现序 ⇒ 确定性。抽成自由函数是为了把 `record_impacts` 压回 god 门棘轮内。
///
/// ⚠️ **成本界**：线性查找 ⇒ 每子步 `O(k²)`（`k` = provider 接触数）。实测（体素地板 + N 盒、
/// 关睡眠、`impacts = 200/800/1600`）：合并的净成本 **≈0.25–0.4 ms/tick**（对照关合并的旧行为），
/// 落在本仓自报的噪声带（±3–5%）里；本仓四哈希/金样场景的 `k` 恒为 0（无 provider 接触）。
/// `k ≫ 10⁴` 的场景（如 10 万盒压在体素地形上）需要改成带键索引——届时那是**已知可量**的一刀，
/// 不是现在的猜测优化。
pub(crate) fn merge_impact(impacts: &mut Vec<ImpactRecord>, rec: ImpactRecord) {
    if let Some(slot) = impacts
        .iter_mut()
        .find(|r| r.provider == rec.provider && r.body == rec.body)
    {
        if rec.approach > slot.approach {
            *slot = rec;
        }
        return;
    }
    impacts.push(rec);
}
