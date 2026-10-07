//! GPU 测试的公共诊断件。
//!
//! **为什么有它**（issue #30 的「抓启动期栈」第 1 条）：CI 的 MSVC 步早就设了
//! `RUST_LOG=wgpu_core=info,wgpu_hal=info,naga=warn`，但 `RUST_LOG` 只是环境变量 ——
//! **测试侧没人初始化 logger**，所以 wgpu 的适配器枚举 / 设备请求日志**一行都没出来**，
//! 现场只剩一个 `0xc0000005`（或"挂住不返回"）。这里补上最小门面：
//! `RUST_LOG` 为空 ⇒ 不装（本地/其它 job 保持安静、零日志噪声）；非空 ⇒ Info 档全量转发到
//! stderr（`cargo test` 默认捕获，失败/`--nocapture` 时随日志落盘 ⇒ 进 CI 的 flake artifact）。
//!
//! ⚠️ 它**不改任何测试语义**：只读环境变量、只写 stderr；不参与断言、不进任何哈希。

/// 最小 stderr logger：不做 target 过滤（`RUST_LOG` 的解析留给使用者；CI 那边只开 info 档）。
struct StderrLogger;

impl log::Log for StderrLogger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, rec: &log::Record<'_>) {
        // 前缀带 `gpu-diag` 便于在 CI artifact 里 grep（tests/ 目录不受 print-gate 管）。
        eprintln!("[gpu-diag {} {}] {}", rec.level(), rec.target(), rec.args());
    }

    fn flush(&self) {}
}

static LOGGER: StderrLogger = StderrLogger;

/// 开关 = `RUST_LOG` 非空。幂等：重复调用只生效一次（第二个调用拿到 `Err` 就跳过）。
pub fn init_logger() {
    let enabled = std::env::var("RUST_LOG")
        .map(|v| !v.trim().is_empty())
        .unwrap_or(false);
    if !enabled {
        return;
    }
    // `set_logger` 只在**首次**成功；重复调用返回 `Err`（已注册）——两种情况下 max_level 都要设。
    match log::set_logger(&LOGGER) {
        Ok(()) | Err(_) => log::set_max_level(log::LevelFilter::Info),
    }
}
