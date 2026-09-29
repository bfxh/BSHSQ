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
ALLOW_MARK = re.compile(r"print[:_-]allow")
# tests/examples/benches 允许（见文件头）。注意本仓的测试码有三种形态：
#   · 目录式 `crates/x/tests/*.rs`、`.../src/tests.rs`（文件名就叫 tests.rs）
#   · 文件内 `#[cfg(test)] mod tests { … }` / `#[cfg(test)] mod tests;`
#   · 直接标在函数上的 `#[test]`（常配 `#[ignore]`，如 fluid 的微探针）
# 此前 SRC_EXCL 只认目录形态与 `src/bin`，于是后两种测试码里的打印都被当成
# "引擎核心直接写终端"判红（实测 7 处命中里 5 处属此类）。
SRC_EXCL = re.compile(
    r"(?:/tests/|/examples/|/benches/|/src/bin/|^tests/|^examples/|^benches/|(?:^|/)tests\.rs$)"
)

# 需要整项跳过的测试形态属性：cfg(test) 块 与 单个 #[test] 项
TEST_ATTRS = ("#[cfg(test)]", "#[test]")


def _skip_item(src, out, j):
    """从属性位置 j 起把紧随的 item 挖空（保持长度 ⇒ 行号不变）。返回继续扫描的位置。"""
    head = src[j : j + 400]
    cut_brace = head.find("{")
    cut_semi = head.find(";")
    # `#[cfg(test)] mod tests;` 这种**无体声明**必须跳过：否则"找下一个 {"会落到后面
    # 不相干的那个 item 上，把它的体整段挖掉 ⇒ 真违规被静默放过（实测踩过：
    # vxl-phys-fluid/src/lib.rs 的 #[ignore] 微探针 println 就被这样吞掉过）。
    if cut_brace < 0 or (0 <= cut_semi < cut_brace):
        return -1
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
        return k + 1
    for q in range(j, p + 1):
        if out[q] != "\n":
            out[q] = " "
    return p + 1


def strip_test_regions(src):
    """把测试形态（cfg(test) 块 / #[test] 项）挖空，长度不变。解析不了就保守不挖。"""
    out = list(src)
    i = 0
    while True:
        cand = [(src.find(a, i), a) for a in TEST_ATTRS]
        cand = [(p, a) for p, a in cand if p >= 0]
        if not cand:
            break
        j, attr = min(cand)
        nxt = _skip_item(src, out, j)
        i = (j + len(attr)) if nxt < 0 else nxt
    return "".join(out)


def main():
    root = gc.repo_root()
    hits = []
    skipped = []
    for rel in gc.list_rs(root, git_tracked=False):
        reln = rel.replace("\\", "/")
        if not reln.endswith(".rs") or SRC_EXCL.search(reln):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            raw = f.read()
        src = strip_test_regions(gc.mask(raw))
        raw_lines = raw.split("\n")
        for m in PRT_RE.finditer(src):
            line = src.count("\n", 0, m.start()) + 1
            # 行级显式豁免：`print:allow` 写在同一行或紧邻上一行才生效（与 naming:allow、
            # zizmor ignore 同惯例——豁免必须落在评审看得见的地方，没有静默白名单）。
            # 查标记要用**原文**：gc.mask 会把注释抹成空白，从 masked 里永远看不见标记。
            window = "\n".join(raw_lines[max(0, line - 2) : line])
            if ALLOW_MARK.search(window):
                skipped.append((reln, line))
                continue
            hits.append((reln, line))
    if not hits:
        extra = f"；显式豁免 {len(skipped)} 处：" + ", ".join(f"{r}:{l}" for r, l in skipped) if skipped else ""
        print(f"✅ print-gate: src 内无未登记豁免的直接 stdout/stderr 写（引擎核心应走日志门面/CLI 层）{extra}")
        sys.exit(0)
    print("❌ print-gate: src 内发现 println!/eprintln!/print!/eprint!（引擎核心不该直接写终端）")
    for rel, line in hits[:40]:
        print(f"   {rel}:{line}")
    if skipped:
        print(f"（另有 {len(skipped)} 处带 print:allow 显式豁免，逐条见源码）")
    sys.exit(1)


if __name__ == "__main__":
    main()
