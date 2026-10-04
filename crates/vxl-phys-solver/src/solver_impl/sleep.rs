//! **岛级休眠与唤醒**（§4.11 / §3 稳定性）—— `solver_impl` 的**子模块**。
//!
//! **为什么拆出来**：`solver_impl.rs` 受 god 门**文件行数棘轮**（阈值 800，它已经 824 行），而休眠族
//! （`sleep_pass` + `sleep_pass_island` + `sleep_pass_whole`，~165 行）是一块独立关注点。选**子模块**
//! 而不是 crate 级新文件，理由与 `narrow` 的 `mesh_mesh.rs`/`mesh_grid.rs` 相同：`solver/lib.rs` 只剩
//! 0 行预算（99 行 / 基线 99）⇒ 加 `mod` 声明就顶棘轮；**子模块还能直接看见父模块的私有项**。
//!
//! ⚠️ 两处只为过门禁、语义零变化的写法：本文件**不用 glob 导入**（`glob-gate` 对**新文件**零基线）；
//! 三个方法由 `fn` 改成 **`pub(crate) fn`**（子模块的私有项父模块看不见，调用方 `solve_phase` 在父模块）。
use crate::{ImpulseSolver, Island};
use crate::{
    SLEEP_DIAG_DEEP, SLEEP_D_BODY_ALL, SLEEP_D_FAST, SLEEP_D_FAST_BODY, SLEEP_D_SLEPT,
    SLEEP_D_WAIT, SLEEP_D_WAIT_MAX_MS, SUBISLAND_SLEEP, SUBISLAND_WAKE_MULT, WAKE_GATE_FAST_MULT,
};
use vxl_phys_core::{BodySet, PhysConfig, Vec3};
use vxl_phys_narrow::Manifold;

impl ImpulseSolver {
    /// 5) 岛级休眠与唤醒（§4.11 / §3 稳定性）。
    ///    - 建岛阶段已只收「与清醒体连通」的岛（含被牵连的睡眠体），
    ///      遗漏的睡眠体天然保持冻结（不解算不积分）；
    ///    - 清醒岛：任一成员 awake → 全岛同步为 awake（外部唤醒传播）；
    ///    - 全员速度低于阈值持续 sleep_time → 岛内**原子**入睡（同帧全员睡），
    ///      不存在"部分睡部分醒"状态，从机制上排除反复唤醒；
    ///    - 无接触的孤立清醒动体 = 单成员岛，走同一套休眠判定。
    ///
    ///    无偏置趟不做休眠判定（同一子步内带偏置趟已判过；重复判定会让
    ///    sleep_timer 每次子步双倍累积 ⇒ 入睡提前，属非本意行为改变）。
    pub(crate) fn sleep_pass(
        &mut self,
        bodies: &mut BodySet,
        islands: &[Island],
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        cleanup: bool,
    ) {
        let sleep_islands: &[Island] = if cleanup { &[] } else { islands };
        // 唤醒接触数门：**每次本函数调用清零**（寿命 = 语义，见 `wake_streak` 字段注）。
        // `k == 0` 时整段不参与 ⇒ 与现行行为**逐位一致**（不含任何算术语义改动）。
        let wake_gate_k = config.wake_gate_k;
        if wake_gate_k > 0 && !sleep_islands.is_empty() {
            if self.wake_streak.len() < bodies.len() {
                self.wake_streak.resize(bodies.len(), 0);
            }
            self.wake_streak[..bodies.len()].fill(0);
        }
        for island in sleep_islands {
            if SUBISLAND_SLEEP {
                self.sleep_pass_island(bodies, island, manifolds, config, dt, wake_gate_k);
                continue;
            }
            self.sleep_pass_whole(bodies, island, config, dt);
        }
    }

    /// 5a) —— 实验：子块睡眠（见 `SUBISLAND_SLEEP` 注）——
    ///     ① 唤醒 = "实质相互作用"：与**清醒**邻居的相对运动显著才唤醒（连通本身不唤醒），
    ///        并带**滞回**（阈值 ×`SUBISLAND_WAKE_MULT`），否则边界体被闪烁体每子步叫醒。
    ///     ② 逐体计时 + 逐体入睡（不要求整岛齐）。
    pub(crate) fn sleep_pass_island(
        &mut self,
        bodies: &mut BodySet,
        island: &Island,
        manifolds: &[Manifold],
        config: &PhysConfig,
        dt: f32,
        wake_gate_k: u32,
    ) {
        for &mi in &island.manifs {
            let m = &manifolds[mi];
            let (a, b) = (m.a as usize, m.b as usize);
            let (sa, sb) = (bodies.awake[a], bodies.awake[b]);
            if sa == sb {
                continue; // 双醒：无需唤醒；双睡：不在清醒岛内
            }
            let (s, w) = if sa { (b, a) } else { (a, b) };
            let rel = (bodies.linvel[w] - bodies.linvel[s]).length();
            let hot = rel > SUBISLAND_WAKE_MULT * config.sleep_linear
                || bodies.angvel(w).length() > SUBISLAND_WAKE_MULT * config.sleep_angular;
            let fast = rel > WAKE_GATE_FAST_MULT * config.sleep_linear;
            if wake_gate_k == 0 || fast {
                // 现行（或强撞直通）：任一 hot 邻居立即唤醒
                if hot {
                    bodies.awake[s] = true;
                    bodies.sleep_timer[s] = 0.0;
                }
            } else if hot {
                // 接触数门：同一次调用内累计 ≥K 条 hot 观测才唤醒；断一次清零。
                let n = &mut self.wake_streak[s];
                *n = n.saturating_add(1);
                if *n >= wake_gate_k {
                    bodies.awake[s] = true;
                    bodies.sleep_timer[s] = 0.0;
                    *n = 0;
                }
            } else {
                self.wake_streak[s] = 0;
            }
        }
        for &bi in &island.bodies {
            let i = bi as usize;
            if !bodies.awake[i] {
                continue; // 睡着的：本轮不动它（上面的唤醒已判过）
            }
            let lin = bodies.linvel[i].length();
            let ang = bodies.angvel(i).length();
            if lin < config.sleep_linear && ang < config.sleep_angular {
                bodies.sleep_timer[i] += dt;
                if bodies.sleep_timer[i] >= config.sleep_time {
                    bodies.awake[i] = false;
                    bodies.linvel[i] = Vec3::ZERO;
                    bodies.set_angvel_raw(i, Vec3::ZERO);
                }
            } else {
                bodies.sleep_timer[i] = 0.0;
            }
        }
    }

    /// 5b) 整岛原子入睡/唤醒（`SUBISLAND_SLEEP = false` 时的路径）。
    pub(crate) fn sleep_pass_whole(
        &mut self,
        bodies: &mut BodySet,
        island: &Island,
        config: &PhysConfig,
        dt: f32,
    ) {
        for &bi in &island.bodies {
            let i = bi as usize;
            if !bodies.awake[i] {
                bodies.awake[i] = true;
                bodies.sleep_timer[i] = 0.0;
            }
        }
        let mut all_slow = true;
        let mut fast_n = 0u64;
        let mut body_n = 0u64;
        for &bi in &island.bodies {
            let i = bi as usize;
            body_n += 1;
            let lin = bodies.linvel[i].length();
            let ang = bodies.angvel(i).length();
            if lin >= config.sleep_linear || ang >= config.sleep_angular {
                all_slow = false;
                fast_n += 1;
                if !SLEEP_DIAG_DEEP {
                    break;
                }
            }
        }
        if SLEEP_DIAG_DEEP {
            use std::sync::atomic::Ordering::Relaxed;
            SLEEP_D_FAST_BODY.fetch_add(fast_n, Relaxed);
            SLEEP_D_BODY_ALL.fetch_add(body_n, Relaxed);
        }
        if all_slow {
            let mut min_timer = f32::MAX;
            for &bi in &island.bodies {
                let i = bi as usize;
                bodies.sleep_timer[i] += dt;
                min_timer = min_timer.min(bodies.sleep_timer[i]);
            }
            use std::sync::atomic::Ordering::Relaxed;
            SLEEP_D_WAIT.fetch_add(1, Relaxed);
            SLEEP_D_WAIT_MAX_MS.fetch_max((min_timer * 1000.0) as u64, Relaxed);
            if min_timer >= config.sleep_time {
                SLEEP_D_SLEPT.fetch_add(1, Relaxed);
                for &bi in &island.bodies {
                    let i = bi as usize;
                    bodies.awake[i] = false;
                    bodies.linvel[i] = Vec3::ZERO;
                    bodies.set_angvel_raw(i, Vec3::ZERO);
                }
            }
        } else {
            SLEEP_D_FAST.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            for &bi in &island.bodies {
                let i = bi as usize;
                bodies.sleep_timer[i] = 0.0;
                self.sleep_resets += 1;
            }
        }
    }
}
