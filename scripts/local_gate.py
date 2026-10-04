#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BSHSQ 门禁统一入口（单一来源）。

CI 的 ratchet 作业只调本文件；本文件列出全部门禁步骤。改门禁 = 改这里 + 对应脚本 +
基线，三处同源（gate_selftest 的 S1 校验 ci.yml 是否调用本文件）。

硬判（存量须为 0）：dag-gate / rayon-gate / cpp-gate / fastmath-gate / f64-gate /
asm-gate / staticmut-gate / license-gate / dbg-gate / print-gate / memfind-core-gate /
engine-dep-gate。
棘轮（存量进基线，只准减，新增即红）：unwrap / unsafe / linelen / glob / todo / god /
cyc / nest / args / clone。
元门：gate-selftest（门禁自检）/ gate-probe（逐门注入验证是真门）。

注意：子进程一律用 sys.executable，避免「python」不在子进程 PATH 上导致静默失败。

入口分工（2026-09-30 合并 feat/solver-limits 时定的口径）：
  · 本文件 = CI ratchet 作业入口，只跑静态细粒度门（纯 stdlib、无计时、秒级）。
  · `bash scripts/gate_all.sh` = 本地全量入口，另含行为门（determinism / m0_gates）、
    金样门与计时独占锁；那些在 CI 是独立作业，不进本文件。
  两者范围不同、不是重复入口。分支曾把一份与 gate_all 功能重合的自用 local_gate 撤成
  指路牌，那是另一回事，不冲突。

stdout 编码：中文 Windows 下重定向会用 cp936，打不出 ✅ 直接 UnicodeEncodeError（deps_lock /
god --selftest 因此在 gate_all 里假红过一次）⇒ 本进程 reconfigure + 给子进程注入 PYTHONUTF8。
"""
import os
import sys
import subprocess

sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # 中文 Windows：stdout 默认 cp936，打 ✅ 会崩
os.environ["PYTHONUTF8"] = "1"   # 子门脚本继承（曾 25/25 假红；gate_probe 读子进程输出也靠它）

STEPS = [
    ("dag-gate",     ["scripts/dag_gate.py"]),
    ("rayon-gate",   ["scripts/rayon_gate.py"]),
    ("cpp-gate",     ["scripts/cpp_gate.py"]),
    ("fastmath-gate",["scripts/fastmath_gate.py"]),
    ("f64-gate",     ["scripts/f64_gate.py"]),
    ("asm-gate",     ["scripts/asm_gate.py"]),
    ("staticmut-gate",["scripts/staticmut_gate.py"]),
    ("license-gate", ["scripts/license_gate.py"]),
    ("dbg-gate",     ["scripts/dbg_gate.py"]),
    ("print-gate",   ["scripts/print_gate.py"]),
    ("memfind-core-gate", ["scripts/memfind_core_gate.py"]),
    ("engine-dep-gate", ["scripts/engine_dep_gate.py"]),
    ("clone-gate",   ["scripts/clone_gate.py", "--git-tracked"]),
    ("unwrap-gate",  ["scripts/unwrap_gate.py", "--git-tracked"]),
    ("unsafe-gate",  ["scripts/unsafe_gate.py", "--git-tracked"]),
    ("linelen-gate", ["scripts/linelen_gate.py", "--git-tracked"]),
    ("glob-gate",    ["scripts/glob_gate.py", "--git-tracked"]),
    ("todo-gate",    ["scripts/todo_gate.py", "--git-tracked"]),
    # 本仓 god_gate 是可移植那版（配置驱动 + 棘轮基线 + --selftest 金丝雀），不认
    # --git-tracked ⇒ 按它自己的口径调 --root，自检金丝雀单列一步。
    ("god-gate",         ["scripts/god_gate.py", "--root", "."]),
    ("god-gate-selftest",["scripts/god_gate.py", "--selftest"]),

    ("cyc-gate",     ["scripts/cyc_gate.py", "--git-tracked"]),
    ("nest-gate",    ["scripts/nest_gate.py", "--git-tracked"]),
    ("args-gate",    ["scripts/args_gate.py", "--git-tracked"]),
    ("gate-selftest",["scripts/gate_selftest.py"]),
    ("gate-probe",   ["scripts/gate_probe.py"]),
]


def main():
    parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fails = []
    for name, args in STEPS:
        print(f"--- {name} ---")
        rc = subprocess.run([sys.executable] + args, cwd=parent).returncode
        if rc != 0:
            fails.append(name)
            print(f"    ❌ {name} 退出码 {rc}")
        else:
            print(f"    ✅ {name}")
    print("=" * 50)
    if fails:
        print(f"门禁失败：{len(fails)} 道 —— {', '.join(fails)}")
        sys.exit(1)
    print(f"全部 {len(STEPS)} 道门禁通过 ✅")
    sys.exit(0)


if __name__ == "__main__":
    main()
