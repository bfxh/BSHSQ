#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""usize-gate：`… * … as usize` —— **先转 usize 再算**的硬判（安全审计 S2 的 lint 侧）。

**为什么需要它**（审计 §4 系统簇 S2）：网格/体素/高度场把维度存成 `u32`/`u16`，下标与容量
在窄类型里乘完再转 `usize`；release 档**没有**溢出检查 ⇒ 静默回绕（F-01/F-02/F-06/F-13 都是
这一族）。clippy 抓不到这条（`as usize` 是合法转换，乘法本身也合法），所以它必须是一条
**自定义门**。已知点位已在 2026-10-10 的 S2 修复里改完；本门把"不许再长回来"钉住。

**判据（精确到只抓真违规）**：一个 `as usize` 转换，若它的**操作数**里含有**二元乘法** ⇒ 违规。

**关键一句**：Rust 里 `as` 比 `*` **结合得更紧** ⇒ `x * y as usize` 是 `x * (y as usize)`，
所以**深度 0 的裸 `*` 永远不会属于 `as` 的操作数**。操作数里真出现乘积，只可能是**括号组内**
（`(a * b) as usize`、`arr[a * b] as usize`、`f((a * b) as usize)`）。于是扫描规则是：

  · 向左扫到**未配对的开括号**（操作数边界）或**深度 0 的二元运算符/`;`/`,`/关键字**就停；
  · 只在**组内（深度 ≥ 1）**把 `*` 记为乘法；且它的左邻必须是一个"值结尾"的记号
    （标识符/数字/`)`/`]`）——这样 `(*p) as usize`、`(*const T) as usize` 的解引用与裸指针
    不算乘法（实测这两类就在仓里）。

**边界（如实写）**：`let n = nx * ny;` 之后 `n as usize` 这种"先存进窄类型变量再转"的形态
**不在面内**（要类型信息才判得准）；本门抓的是 cast 当场可见的乘积，也就是审计列举的全部点位。
两条**去噪**规则（都来自实测假阳性，见下）：① 组内 `*` 的任一邻是**浮点字面量**（`2.0`）
⇒ 那是浮点乘法，不是整数回绕；② 走到 `(` 时若它的前一个记号是**标识符/`)`/`]`** ⇒ 那是
**函数调用**的参数表（`f(cap * 8) as usize`：乘积在实参里、cast 作用在返回值上），到此为止；
`[`（索引）不适用这条——`arr[a * b] as usize` 的乘积**在**操作数里，要报。

**存量须为 0**（不是棘轮：这类回绕没有"可接受的存量"，只有"还没改到"）。
tests/examples/benches 与 `src/bin` 排除（同 `clone-gate` 的口径：纪律面 = 引擎源码）。

用法：`python scripts/usize_gate.py [--git-tracked] [--list]`；退出码 0 = 通过、1 = 有违规。
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_common as gc

SRC_EXCL = re.compile(r"(?:/tests/|/examples/|/benches/|/src/bin/|^tests/|^examples/|^benches/)")

# Rust 词法近似：标识符 / 数字字面量 / 多字符运算符 / 单字符记号。
_TOK = re.compile(r"[A-Za-z_][A-Za-z0-9_]*|[0-9][0-9A-Za-z_.]*|::|->|=>|&&|\|\||[^\sA-Za-z0-9_]")
_AS_USIZE = re.compile(r"\bas\s+usize\b")
_IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
# 深度 0 上"操作数到此为止"的记号：二元运算符与语句/表达式分界。
_STOP = {
    ";", "=", ",", "+", "-", "*", "/", "%", "|", "&", "^", "<", ">", "!", "?",
    "&&", "||", "=>", "->",
    "let", "return", "if", "else", "match", "fn", "while", "for", "in", "as",
    "{", "}",
}


def _is_float_lit(tok):
    """浮点字面量（`2.0` / `1e-3`）：它参与的乘法是浮点乘法，不是整数回绕。"""
    return bool(re.match(r"^[0-9]", tok)) and ("." in tok or "e" in tok or "E" in tok)


def _is_value(tok):
    """该记号能否作为乘法**左**操作数的结尾（决定组内那个 `*` 是不是二元乘法）。"""
    if tok in (")", "]", "}"):
        return True
    if _IDENT.match(tok):
        # 关键字/类型名不是"值"：`*const` / `*mut` 里的 `*` 走这条被排除。
        return tok not in ("const", "mut", "ref", "in", "as", "fn", "dyn", "where", "usize")
    return bool(re.match(r"^[0-9]", tok))


def cast_operand_has_mul(toks):
    """`toks` 是 `as usize` **之前**的全部记号；返回该转换的操作数里有没有二元乘法。"""
    depth = 0
    saw_mul = False
    float_seen = False
    stack = []  # 进入右括号时压栈：记下"进组前"的 saw_mul，供配对左括号决定是否丢弃
    for i in range(len(toks) - 1, -1, -1):
        t = toks[i]
        if _is_float_lit(t):
            float_seen = True
        if t in (")", "]", "}"):
            stack.append(saw_mul)
            depth += 1
            continue
        if t in ("(", "[", "{"):
            if depth == 0:
                # `f(x * y) as usize`：实参表里的乘积属于**函数**，cast 作用在返回值上 ⇒ 到此为止。
                # `(x * y) as usize`（前一个记号不是标识符/`)`/`]`）：括号表达式 ⇒ 继续往左扫。
                if t == "(" and i > 0 and (
                    _IDENT.match(toks[i - 1]) or toks[i - 1] in (")", "]")
                ):
                    saw_mul = False
                    break
                if t == "{":
                    break
                continue
            depth -= 1
            before = stack.pop() if stack else False
            # 组内是**调用实参表** ⇒ 组里扫到的乘积不算；而且整个操作数到此为止
            # （形如 `x.ceil() as usize` / `f(…) as usize`：类型由调用给出——浮点、长度——
            #  不是"窄整数乘积"，本门不管，宁可漏也不误报）。
            if t == "(" and i > 0 and (
                _IDENT.match(toks[i - 1]) or toks[i - 1] in (")", "]")
            ):
                saw_mul = before
                break
            continue
        if depth > 0:
            if t == "*" and i > 0 and _is_value(toks[i - 1]):
                right = toks[i + 1] if i + 1 < len(toks) else ""
                if not _is_float_lit(toks[i - 1]) and not _is_float_lit(right):
                    saw_mul = True
            continue
        if t in _STOP:
            break
    # 操作数里出现过浮点字面量 ⇒ 这是浮点表达式（Rust 不做隐式转换），不是整数回绕。
    return saw_mul and not float_seen


def violations(src):
    """返回 [(行号, 行文本)]：本文件里"窄类型先乘再转"的位置。"""
    lines = src.splitlines()
    out = []
    for m in _AS_USIZE.finditer(src):
        head = src[: m.start()]
        if cast_operand_has_mul(_TOK.findall(head)):
            ln = head.count("\n") + 1
            out.append((ln, lines[ln - 1].strip() if ln - 1 < len(lines) else ""))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--git-tracked", action="store_true",
                   help="只扫 git ls-files 跟踪的 *.rs（CI/提交口径）")
    p.add_argument("--list", action="store_true", help="只列出违规（不判红）")
    a = p.parse_args()
    root = gc.repo_root()
    hits = []
    for rel in gc.list_rs(root, a.git_tracked):
        reln = rel.replace("\\", "/")
        if SRC_EXCL.search(reln):
            continue
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
            src = gc.mask(f.read())
        for ln, text in violations(src):
            hits.append(f"{reln}:{ln}: {text[:120]}")
    for h in hits:
        print("   " + h)
    if hits and not a.list:
        print(f"❌ usize-gate: {len(hits)} 处「先乘再转 usize」（窄类型里溢出会静默回绕——"
              f"先转 usize 再算，见安全审计 S2）")
        sys.exit(1)
    if hits:
        print(f"ℹ️ usize-gate: {len(hits)} 处（--list 模式，不判红）")
        sys.exit(0)
    print("✅ usize-gate: 无「先乘再转 usize」的窄类型算术")
    sys.exit(0)


if __name__ == "__main__":
    main()
