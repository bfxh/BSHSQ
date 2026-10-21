# -*- coding: utf-8 -*-
"""重切 `docs/slices/<NAME>.pNN.md` + 重建 `<NAME>.INDEX.md`（5 份大文档通吃）。

为什么自己写（不直接用 `E:\\bundles\\reslice-2026-10-21\\reslice.py`）：那份脚本的 `build()`
断言「旧片首行必须在原文里逐字命中」，而 **p01 的首行是读法行**——它内含「整篇估算 token」，
这个数一变，首行就变 ⇒ 断言必然红。本脚本改成「锚点只用于**定边界**（p02..pNN 的标题行），
p01 的首行**不做锚点断言**」，其余口径与 `verify.py` **逐条对齐**：

- 原文 = `docs/<NAME>.md`（`newline=''` 读取，**不转行尾**）
- `--check` 比较时**归一行尾**（CRLF ≡ LF）⇒ 只因行尾不同不判漂移（假红）；
  **写盘**仍按原文行尾写（保持「文件行尾与原文同构」这条不变量）
- 分片 = 1 行 HTML 注释头 + 原文逐行副本；文件行尾与原文**同构**（CRLF/LF 不许混）
- 边界 = p02..pNN 的**旧首行**（标题行）在新原文里的唯一命中；p01 从第 1 行起
- 估算 token = `round(CJK 字符数 + 其它字符数/4)`（`verify.py::est`）
- INDEX 两表：分片表（起始行 + 估算 token）、小节表（§编号 + 标题 + 分片 + 原文行）
- **整篇估算三处必须相等**：原文读法行、INDEX 散文、分片估算之和

覆盖的 5 份（`docs/slices/` 下的分片必须与原文逐字节一致）：
`OPEN-PROBLEMS` / `EXPERIMENTS` / `M1-PLAN` / `PLAN-gpu` / `SURVEY-SOFT-CLOTH-AND-CONVERSION`

用法：
  `python scripts/reslice_docs.py`            # 全部重切并写盘
  `python scripts/reslice_docs.py --check`    # 只校验不写盘（退出 0 = 一致，1 = 有漂移）
  `python scripts/reslice_docs.py --check OPEN-PROBLEMS`   # 只查一份
"""

import difflib
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")
SL = os.path.join(DOCS, "slices")

# 5 份大文档：分片必须与原文逐字节一致（口径见 `E:\bundles\reslice-2026-10-21\verify.py`）
NAMES = [
    "OPEN-PROBLEMS",
    "EXPERIMENTS",
    "M1-PLAN",
    "PLAN-gpu",
    "SURVEY-SOFT-CLOTH-AND-CONVERSION",
]

# 模糊锚点的两条阈值（宁可不写，也不能把边界写歪）
FUZZY_MIN = 0.80  # 与最佳候选的相似度下限
FUZZY_MARGIN = 0.05  # 最佳与第二名的最小差距（防「多个候选都像」）

CJK = lambda ch: (
    "\u4e00" <= ch <= "\u9fff"
    or "\u3000" <= ch <= "\u303f"
    or "\uff00" <= ch <= "\uffef"
)


def est(text):
    c = sum(1 for ch in text if CJK(ch))
    return round(c + (len(text) - c) / 4)


def norm(s):
    return re.sub(r"\s+", "", s.replace("**", "").replace("`", ""))


def read_text(path):
    return open(path, encoding="utf-8", newline="").read()


def write_text(path, text, eol="\n"):
    """写盘：`text` 是 **LF 空间**的文本 ⇒ 按 `eol` 转成目标行尾。

    为什么要转：仓内 canonical blob 是 LF，但本机 `core.autocrlf=true`
    ⇒ 检出的工作树是 CRLF ⇒ 「文件行尾与原文同构」这条不变量要求分片跟着原文走。
    """
    out = text.replace("\n", eol) if eol != "\n" else text
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(out)


def norm_eol(text):
    """把 CRLF / 裸 CR 归一成 LF（**只用于比较**）⇒ 行尾不同不算漂移。"""
    return text.replace("\r\n", "\n").replace("\r", "\n")


def split_lines(raw):
    """切成行（**内部一律 LF**）：先归一 CRLF，否则每行都带结尾 `\r`
    ⇒ 后续 `join` + 补 eol 会产出 `\r\r\n`（实测在 Windows worktree 里假红）。"""
    lines = raw.replace("\r\n", "\n").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    return lines


def detect_eol(raw):
    """返回 ('\\r\\n' | '\\n')，按首个换行符判定（原文为准）。"""
    i = raw.find("\n")
    if i > 0 and raw[i - 1] == "\r":
        return "\r\n"
    return "\n"


def slice_files(name):
    """该文档已有的分片文件名（按序号）。"""
    pat = re.compile(re.escape(name) + r"\.p(\d\d)\.md$")
    out = []
    for fn in os.listdir(SL):
        m = pat.match(fn)
        if m:
            out.append((int(m.group(1)), fn))
    out.sort()
    return [fn for _, fn in out]


def slice_head(name, idx, total, start, end):
    return (
        f"<!-- 分片 {idx:02d}/{total:02d}：本文的连续切片副本，原文未改。"
        f"源第 {start}-{end} 行。索引：{name}.INDEX.md -->"
    )


def find_boundary_anchor(name, idx):
    """取第 idx 片旧文件的首行作为边界锚点。"""
    fn = os.path.join(SL, f"{name}.p{idx:02d}.md")
    raw = read_text(fn)
    m = re.match(r"^<!-- 分片 \d+/\d+：.*?源第 (\d+)-(\d+) 行。索引：\S+ -->\r?\n", raw)
    if not m:
        raise SystemExit(f"{fn}: 头格式不匹配")
    body = raw[m.end() :]
    bl = body.split("\n")
    if bl and bl[-1] == "":
        bl.pop()
    return bl[0]


def locate(lines, anchor, after):
    """锚点在原文里的定位：精确 → 归一化 → **模糊（标题相似度）**。

    为什么要有模糊层（2026-10-21）：边界锚点 = 下一片的首行 = 一条**标题行**。
    有人改了那条标题的正文（例如补一句「（补充说明）」）⇒ 精确/归一化都找不到
    ⇒ 旧行为直接报错，人只能手工重切。模糊层把「改了标题」这种常见编辑救回来：
    只在**标题行**里找相似度最高的那条，阈值/领先幅度不够就**仍然报错**
    （宁可不写，也不能把边界写歪 —— 边界写歪 ⇒ 分片内容整体错位）。
    """
    hits = [j + 1 for j, l in enumerate(lines) if l == anchor]
    if hits:
        after_hits = [h for h in hits if h > after]
        if after_hits:
            return after_hits[0]
        raise SystemExit(f"锚点只出现在边界之前: {anchor[:70]}")
    na = norm(anchor)
    hits = [j + 1 for j, l in enumerate(lines) if norm(l) == na]
    if hits:
        after_hits = [h for h in hits if h > after]
        if after_hits:
            return after_hits[0]
        raise SystemExit(f"锚点只出现在边界之前: {anchor[:70]}")
    # 模糊：只在标题行里找，且要求相似度够高 + 明显领先第二名
    best_r, best_ln, second_r = 0.0, None, 0.0
    for j, l in enumerate(lines):
        if j + 1 <= after:
            continue
        if not re.match(r"^(#{1,6})[ \t]+", l):
            continue
        r = difflib.SequenceMatcher(None, na, norm(l)).ratio()
        if r > best_r:
            best_r, best_ln, second_r = r, j + 1, best_r
        elif r > second_r:
            second_r = r
    if (
        best_ln is not None
        and best_r >= FUZZY_MIN
        and best_r - second_r >= FUZZY_MARGIN
    ):
        return best_ln
    raise SystemExit(f"锚点找不到: {anchor[:70]}")


def build(name, doc_lines, starts, eol):
    out = []
    total = len(starts)
    for i, st in enumerate(starts):
        en = (starts[i + 1] - 1) if i + 1 < len(starts) else len(doc_lines)
        # ⚠️ **不要 rstrip**：权威 `verify.py` 的口径是
        # `want = '\n'.join(doc[st-1:en]) + tail_nl` ⇒ 片末行若是空行，
        # `join` 产生的尾换行**必须保留**（去掉就会与原文差一行 ⇒ verify.py 判红）。
        body = "\n".join(doc_lines[st - 1 : en])
        head = slice_head(name, i + 1, total, st, en)
        tok = est(body)
        out.append(
            dict(
                fn=f"{name}.p{i + 1:02d}.md",
                start=st,
                end=en,
                body=body,
                tok=tok,
                # LF 空间拼（行尾在 write_text 时统一转成原文的 eol ⇒ 「行尾与原文同构」不变）
                text=head + "\n" + body + "\n",
            )
        )
    return out


def build_index(name, idx_lines, slices, headings, nums, total_num):
    frag = [f"| `{s['fn']}` | 第 {s['start']} 行起 | {s['tok']:,} |" for s in slices]

    def slice_of(line):
        for s in slices:
            if s["start"] <= line <= s["end"]:
                return s["fn"]
        return slices[-1]["fn"]

    new_rows = [
        f"| {num} | {'  ' * (lvl - 1)}{title} | `{slice_of(ln)}` | {ln} |"
        for (ln, lvl, title), num in zip(headings, nums)
    ]

    out = []
    seen_slice_tbl = False
    i, n = 0, len(idx_lines)
    while i < n:
        l = idx_lines[i]
        s = l.strip()
        if re.match(r"^\| `\S+\.p\d\d\.md` \| 第 \d+ 行起 \|", s):
            if not seen_slice_tbl:
                out.extend(frag)
                seen_slice_tbl = True
            i += 1
            continue
        if s.startswith("| §编号 |"):
            out.append(l)
            i += 1
            if i < n and idx_lines[i].strip().startswith("|---"):
                out.append(idx_lines[i])
                i += 1
            else:
                out.append("|" + "---|" * len(s.strip("|").split("|")))
            out.extend(new_rows)
            while (
                i < n
                and idx_lines[i].strip().startswith("|")
                and idx_lines[i].count("|") >= 5
            ):
                i += 1
            continue
        out.append(l)
        i += 1
    for i, l in enumerate(out):
        if "≈" in l and "估算 token" in l:
            out[i] = re.sub(r"≈[\d,]+ 估算 token", f"≈{total_num} 估算 token", l)
            break
    return "\n".join(out)


def process(name, check_only):
    doc = os.path.join(DOCS, name + ".md")
    idx_path = os.path.join(SL, name + ".INDEX.md")
    raw = read_text(doc)
    eol = detect_eol(raw)
    doc_lines = split_lines(raw)

    fns = slice_files(name)
    n = len(fns)
    if n == 0:
        raise SystemExit(f"{name}: 没有分片文件")

    # 边界：p02..pNN 用旧片首行锚定；p01 固定从第 1 行起
    starts = [1]
    for i in range(2, n + 1):
        anchor = find_boundary_anchor(name, i)
        starts.append(locate(doc_lines, anchor, starts[-1]))

    slices = build(name, doc_lines, starts, eol)
    total = sum(s["tok"] for s in slices)
    total_num = f"{total:,}"

    # 原文读法行的整篇估算 = 分片和
    m = re.search(r"≈([\d,]+) 估算 token", doc_lines[0])
    if not m:
        raise SystemExit(f"{name}: 原文首行没有「≈N 估算 token」")
    new_doc_lines = list(doc_lines)
    new_doc_lines[0] = new_doc_lines[0].replace(m.group(1), total_num, 1)
    # 估算必须稳定（改估算不改 token 量）
    slices2 = build(name, new_doc_lines, starts, eol)
    if [s["tok"] for s in slices2] != [s["tok"] for s in slices]:
        raise SystemExit(f"{name}: 估算漂移：改读法行后 token 量变了")
    slices = slices2

    # INDEX 小节表：标题 = 原文所有 markdown 标题
    headings = []
    for i, l in enumerate(new_doc_lines):
        mm = re.match(r"^(#{1,6})[ \t]+(.*)$", l)
        if mm:
            headings.append((i + 1, len(mm.group(1)), mm.group(2)))

    # §编号：沿用旧 INDEX 行的编号（按标题文本匹配），缺失则从标题里猜
    idx_raw = read_text(idx_path)
    idx_lines = split_lines(idx_raw)
    old_rows = []
    in_sec = False
    for l in idx_lines:
        if l.startswith("## "):
            in_sec = True
            continue
        if in_sec and l.startswith("|") and l.count("|") >= 5:
            c = [x.strip() for x in l.strip().strip("|").split("|")]
            if len(c) >= 4 and c[0] != "§编号" and not set(c[0]) <= set("- "):
                old_rows.append(c)
    by_title = {}
    for c in old_rows:
        by_title.setdefault(norm(c[1]), c[0])
    nums = []
    for ln, lvl, title in headings:
        num = by_title.get(norm(title))
        if num is None:
            mm = re.match(r"^(\d+(?:\.\d+)*)[.、\s]", title)
            num = mm.group(1) if mm else "—"
        nums.append(num)

    idx_text = build_index(name, idx_lines, slices, headings, nums, total_num)
    if not idx_text.endswith("\n"):
        idx_text += "\n"

    # 原文回写：同样在 LF 空间拼（写盘时统一转 eol）
    new_doc_text = "\n".join(new_doc_lines) + ("\n" if raw.replace("\r\n", "\n").endswith("\n") else "")

    files = {doc: new_doc_text, idx_path: idx_text}
    for s in slices:
        files[os.path.join(SL, s["fn"])] = s["text"]

    if check_only:
        bad = 0
        for p, t in files.items():
            # 行尾归一化后比较：内容漂移仍红；只是 CRLF/LF 不同不红（假红，见函数注释）。
            if norm_eol(read_text(p)) != norm_eol(t):
                bad += 1
                print(f"✗ 漂移: {os.path.relpath(p, ROOT)}")
        if bad:
            print(f"  {name}: FAIL（{bad} 个文件漂移）")
            return 1
        print(
            f"✓ {name}: {len(new_doc_lines)} 行 / {len(slices)} 片 / 整篇 {total_num}"
        )
        return 0

    for p, t in files.items():
        write_text(p, t, eol)
    print(
        f"已写入 {len(files)} 个文件；{name} 原文 {len(new_doc_lines)} 行；"
        f"边界 {starts}；整篇估算 {total_num}"
    )
    for s in slices:
        print(
            f"   {s['fn']}: 第 {s['start']}-{s['end']} 行"
            f"（{s['end'] - s['start'] + 1} 行）  估算 {s['tok']:,}"
        )
    return 0


def main():
    args = list(sys.argv[1:])
    check_only = "--check" in args
    args = [a for a in args if not a.startswith("--")]
    names = args if args else NAMES

    fail = 0
    for name in names:
        if name not in NAMES:
            raise SystemExit(f"未知文档 {name}；可选：{', '.join(NAMES)}")
        fail |= process(name, check_only)
    if check_only:
        if fail:
            print("\nFAIL（存在漂移）")
        else:
            print(
                f"\nALL PASS（{len(names)} 份文档：分片 + INDEX + 原文读法行 三处估算一致）"
            )
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
