//! `check` 子命令：同场景跑两遍、逐观测点比对哈希（CI 自检；纯搬移自 `lib.rs`）。

use crate::scene::scene;
use vxl_phys_replay::Recorder;

/// 跑两遍同一场景，逐观测点比对 `state_hash`（`Recorder` 是公共 API 里最小的"观测点"容器）。
/// 不一致 ⇒ `Err`（CLI 退出码 2 ⇒ 可直接当 CI 门用）。
pub(super) fn check(name: &str, ticks: usize, json: bool) -> Result<String, String> {
    let period = 60u64;
    let sample = |name: &str| -> Result<(Recorder, u128), String> {
        let mut w = scene(name)?;
        let mut rec = Recorder::new(period);
        for t in 0..ticks as u64 {
            w.step();
            if (t + 1) % period == 0 {
                rec.observe(t + 1, w.state_hash());
            }
        }
        Ok((rec, w.state_hash()))
    };
    let (a, ha) = sample(name)?;
    let (b, hb) = sample(name)?;
    let samples = (ticks as u64 / period) as usize;
    let pass = a.matches(&b) && ha == hb;
    if json {
        return Ok(format!(
            "{{\"scene\":\"{name}\",\"ticks\":{ticks},\"samples\":{samples},\"pass\":{pass},\
             \"state_hash\":\"0x{ha:032x}\"}}\n"
        ));
    }
    if !pass {
        return Err(format!(
            "FAIL：{name} / {ticks} tick 两次跑不一致（观测点 {samples} 个，末态 0x{ha:032x} vs 0x{hb:032x}）"
        ));
    }
    Ok(format!(
        "PASS：{name} / {ticks} tick / {samples} 个观测点逐位一致（0x{ha:032x}）\n"
    ))
}
