#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""print-gate：禁引擎核心直接写 stdout/stderr（硬判，引擎核心纪律）。

物理核心是确定性、headless 可跑的库；直接 `println!`/`eprintln!`/`print!`/`eprint!`
会污染跑分/回归日志、破坏「核心场景禁用可选 feature 仍可运行」的清洁性，也泄露内部状态。
调试输出应走 tracing/日志门面或在 CLI/bin 层做。tests/examples/benches 允许，故只扫 src/。
存量须为 0。
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

PRT_RE = re.compile(r"\b(?:e?print)(ln)?!\s*\(")
# tests/examples/benches 允许（见文件头）。注意本仓的测试码有两种形态：
#   · 目录式 `crates/x/tests/*.rs`、`.../src/tests.rs`（文件名就叫 tests.rs）
#   · 文件内 `#[cfg(test)] mod tests { … }`
# 此前 SRC_EXCL 只认目录形态与 `src/bin`，于是 `src/tests.rs` 与 `mod tests` 里的打印
# 都被当成"引擎核心直接写终端"判红（实测 7 处命中里 5 处属此类）。
SRC_EXCL = re.compile(
    r"(?:/tests/|/examples/|/benches/|/src/bin/|^tests/|^examples/|^benches/|(?:^|/)tests\.rs$)"
)


def strip_cfg_test(src):
    """把 `#[cfg(test)]` 紧随的 mod/函数整体挖空（保持长度 ⇒ 行号不变）。

    只做属性 → 下一个 `{` 块的括号配平；解析不了的形态保守不挖（宁可多报不误放）。
    """
    out = list(src)
    i = 0
    while True:
        j = src.find("#[cfg(test)]", i)
        if j < 0:
            break
        head = src[j : j + 400]
        cut_brace = head.find("{")
        cut_semi = head.find(";")
        # `#[cfg(test)] mod tests;` 这种**无体声明**必须跳过：否则"找下一个 {"会落到
        # 后面不相干的那个 item 上，把它的体整段挖掉 ⇒ 真违规被静默放过（实测踩过：
        # vxl-phys-fluid/src/lib.rs 的 #[ignore] 微探针 println 就被这样吞掉过）。
        if cut_brace < 0 or (0 <= cut_semi < cut_brace):
            i = j + len("#[cfg(test)]")
            continue
        k = j + cut_brace
        depth, p = 0, k
        while p < len(src):
            if src[p] == "{":
                depth += 1
            elif src[p] == "}":
                depth -= 1
                if depth == 0:
                    break
            p += 1
        if depth != 0:  # 括号不配平，形态没吃透 ⇒ 保守：不挖
            i = k + 1
            continue
        for q in range(j, p + 1):
            if out[q] != "\n":
                out[q] = " "
        i = p + 1
    return "".join(out)


def main():
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, git_tracked=False):
        reln = rel.replace("\\", "/")
        if not reln.endswith(".rs") or SRC_EXCL.search(reln):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = strip_cfg_test(gc.mask(f.read()))
        for m in PRT_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            hits.append((reln, line))
    if not hits:
        print("✅ print-gate: src 内无直接 stdout/stderr 写（引擎核心应走日志门面/CLI 层）")
        sys.exit(0)
    print("❌ print-gate: src 内发现 println!/eprintln!/print!/eprint!（引擎核心不该直接写终端）")
    for rel, line in hits[:40]:
        print(f"   {rel}:{line}")
    sys.exit(1)


if __name__ == "__main__":
    main()
