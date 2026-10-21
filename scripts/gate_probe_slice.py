# -*- coding: utf-8 -*-
"""slice-gate 的注入验证：改一个分片文件的内容 ⇒ slice-gate 必须转红。

为什么独立成模块（2026-10-21）：`scripts/gate_probe.py` 受 god 棘轮约束
（基线 153 行 / 最长函数 28 行，**只准减**）⇒ 往里加代码会撞「不许变胖」。
⇒ 本模块只做 slice-gate 这一道，探针（`gate_probe.py`）调它。

与 .rs 探针的差别：slice-gate 判的是**派生文档**（`docs/slices/*.md`）与原文的
逐字节一致性 ⇒ 注入物是「改一个分片文件里的一行」，不是新建 .rs。
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(ROOT, "docs", "slices", "OPEN-PROBLEMS.p01.md")
SCRIPT = "scripts/reslice_docs.py"


def _run():
    return subprocess.run(
        [sys.executable, SCRIPT, "--check"], cwd=ROOT,
        capture_output=True, text=True,
        encoding="utf-8", errors="replace",
    )


def probe_slice_gate():
    """返回失败列表（空 = 通过）。"""
    fails = []
    if not os.path.isfile(TARGET):
        print(f"⚠️ slice-gate: 探针目标不存在（{TARGET}），跳过")
        return fails
    with open(TARGET, "rb") as f:
        original = f.read()
    try:
        pre = _run()
        # 注入：第 3 行起（跳过片头行与片首行——片首行是下一片的边界锚点，
        # 改它会报「锚点找不到」而不是内容漂移）的第一个数字
        lines = original.decode("utf-8").split("\n")
        hit = None
        for i in range(2, len(lines)):
            for ch in "0123456789":
                if ch in lines[i]:
                    lines[i] = lines[i].replace(ch, "9" if ch != "9" else "8", 1)
                    hit = i
                    break
            if hit is not None:
                break
        if hit is None:
            print("⚠️ slice-gate: 找不到可注入的数字行，跳过")
            return fails
        with open(TARGET, "w", encoding="utf-8", newline="") as f:
            f.write("\n".join(lines))
        out = _run()
        if pre.returncode != 0:
            print("⚠️ slice-gate: 注入前已红（基线未绿）——本次无法判定它是否真门")
            fails.append("slice-gate(基线未绿)")
        elif out.returncode != 0:
            print("✅ slice-gate: 注入最小漂移后转红（是真门）")
        else:
            print("❌ slice-gate: 注入后仍绿 —— 门可能为空门！")
            print("   rc:", out.returncode, "| stdout:", repr((out.stdout or "")[:300]))
            fails.append("slice-gate")
    finally:
        with open(TARGET, "wb") as f:
            f.write(original)
    return fails


if __name__ == "__main__":
    bad = probe_slice_gate()
    sys.exit(1 if bad else 0)
