//! `replay` 子命令：与基线逐行对拍（纯搬移自 `lib.rs`）。

use vxl_phys_replay::Recorder;

use crate::record::record_text;

/// `replay`：基线文件是**必需**的（缺了就是用法错误，不该静默什么都不比）。
pub(super) fn replay_cmd(name: &str, ticks: usize, path: Option<&str>) -> Result<String, String> {
    match path {
        Some(p) => replay(name, ticks, p),
        None => Err("replay 需要基线文件：`replay <场景> [ticks] <基线.jsonl>`".to_string()),
    }
}

/// 从一行 JSONL（`{"tick":..,"state_hash":"0x.."}`）取出 `(tick, hash)`。
fn parse_line(line: &str) -> Option<(u64, u128)> {
    let t0 = line.find("\"tick\":")? + "\"tick\":".len();
    let t1 = line[t0..].find(',')? + t0;
    let tick: u64 = line[t0..t1].trim().parse().ok()?;
    let h = "\"state_hash\":\"0x";
    let h0 = line.find(h)? + h.len();
    let h1 = line[h0..].find('"')? + h0;
    let hash = u128::from_str_radix(line[h0..h1].trim(), 16).ok()?;
    Some((tick, hash))
}

/// 解析整份 JSONL。**任何一行解析失败都必须报错**——曾经用 `filter_map` 静默丢弃，
/// 结果"篡改一个哈希"被丢掉后两边都"一致"，对拍假 PASS（被 `cli` 的篡改用例抓到）。
fn to_recorder(text: &str, who: &str) -> Result<Recorder, String> {
    let mut hashes = Vec::new();
    for (i, l) in text.lines().enumerate() {
        if l.trim().is_empty() {
            continue;
        }
        match parse_line(l) {
            Some(v) => hashes.push(v),
            None => {
                return Err(format!(
                    "FAIL：{who} 第 {} 行与观测点格式不一致（不是合法观测点）：{l}",
                    i + 1
                ))
            }
        }
    }
    Ok(Recorder { period: 1, hashes })
}

/// 重跑场景并与基线**按 tick 对齐**比对：不一致时报**首个发散 tick**（基线/本次两侧哈希）。
fn replay(name: &str, ticks: usize, path: &str) -> Result<String, String> {
    let want = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => return Err(format!("读不了 {path}：{e}")),
    };
    let got = record_text(name, ticks)?;
    let (wl, gl) = (want.lines().count(), got.lines().count());
    if wl != gl {
        return Err(format!(
            "FAIL：观测点行数不同（基线 {wl} vs 本次 {gl}）——tick 数或档位不一样？"
        ));
    }
    let (rw, rg) = (to_recorder(&want, "基线")?, to_recorder(&got, "本次")?);
    // **逐字比**（严格）：`0x0b71…` 与 `0xb71…` 数值相等但文本不同，篡改必须被抓到。
    // 差分只用来把"第一个不同的行"翻译成 **tick**，不改变判定口径。
    for (i, (w, g)) in want.lines().zip(got.lines()).enumerate() {
        if w != g {
            let tick = parse_line(w)
                .map(|(t, _)| t)
                .or_else(|| parse_line(g).map(|(t, _)| t));
            let d = rw.divergence(&rg);
            return Err(format!(
                "FAIL：首个发散观测点 tick={:?}（第 {} 行，逐字不一致）
  基线 {w}
  本次 {g}
  {}",
                tick,
                i + 1,
                match d {
                    Some(d) => d.report(),
                    None => "（两侧哈希数值相同 ⇒ 纯文本差异，如前导零/大小写）".to_string(),
                }
            ));
        }
    }
    Ok(format!(
        "PASS：{name} / {ticks} tick / {gl} 个观测点与 {path} 逐字一致\n"
    ))
}

#[cfg(test)]
mod tests {
    use super::parse_line;

    #[test]
    fn parses_record_line() {
        let l = "{\"tick\":120,\"state_hash\":\"0x0000000000000000000000000000abcd\"}";
        let (t, h) = match parse_line(l) {
            Some(v) => v,
            None => {
                assert!(false, "应当解析成功");
                return;
            }
        };
        assert_eq!(t, 120);
        assert_eq!(h, 0xabcd);
    }

    #[test]
    fn rejects_other_lines() {
        assert!(parse_line("hello").is_none());
    }
}
