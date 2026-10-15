#!/usr/bin/env python3
"""本地 Python 脚本入库前的安全评审门。

只读评审：不改写任何被审文件，只输出发现清单与结论。
判据口径：违规条数与覆盖面并列报告——覆盖面为 0 时不得判绿（fail-closed）。
退出码：0 = 无阻断项；1 = 有阻断项或无法评审；2 = 用法错误。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

BLOCK = "BLOCK"
WARN = "WARN"

# 值看起来像占位符就不算密钥字面量，否则每个示例文档都要开例外
BENIGN_SECRET_HINT = re.compile(
    r"(?i)^(x+|\*+|-+|_+|<[^>]*>|\{\{[^}]*\}\}|\$\{[^}]*\})$"
    # 下行含被检测的占位类词本身 ⇒ 该行带行级自指豁免
    r"|example|sample|placeholder|changeme|your[_-]?|dummy|fake|redacted|\bnone\b"  # naming:allow
)

RULES = [
    (
        "SEC-01 走 shell 执行",
        BLOCK,
        re.compile(r"shell\s*=\s*True|\bos\.system\s*\(|\bos\.popen\s*\("),
    ),
    (
        "SEC-02 动态代码执行",
        BLOCK,
        re.compile(
            r"(?<![\w.])eval\s*\(|(?<![\w.])exec\s*\(|(?<![\w.])compile\s*\(|__import__\s*\("
        ),
    ),
    (
        "SEC-03 不安全反序列化",
        BLOCK,
        re.compile(
            r"pickle\.loads?|marshal\.loads?|\byaml\.load\s*\((?![^)]*Safe)[^)]*\)"
            r"|torch\.load\s*\((?![^)]*weights_only\s*=\s*True)"
        ),
    ),
    (
        "SEC-04 密钥字面量",
        BLOCK,
        re.compile(
            r"sk-[A-Za-z0-9]{16,}|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}"
            r"|AKIA[0-9A-Z]{16}|xox[baprs]-[A-Za-z0-9-]{10,}"
            r"|-----BEGIN[ A-Z]*PRIVATE KEY-----"
            r"|eyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}"
        ),
    ),
    (
        "SEC-05 密钥赋值字面量",
        BLOCK,
        re.compile(
            r"(?i)\b(api[_-]?key|secret|access[_-]?token|auth[_-]?token|password|passwd"
            r"|client[_-]?secret|private[_-]?key)\b\s*[:=]\s*[\"'][^\"'\s][^\"']{7,}[\"']"
        ),
    ),
    (
        "SEC-06 泄出密钥到输出",
        BLOCK,
        re.compile(
            r"(?i)(print|log(?:_info|_debug|s)?|write|echo)\s*\(.*(environ|getenv)\s*[\(\[]"
            r".{0,80}(KEY|TOKEN|SECRET|PASSWORD)"
        ),
    ),
    (
        "SEC-07 密钥拼接网络出口",
        BLOCK,
        re.compile(
            r"(?i)(requests|urlopen|httpx|aiohttp|urllib)\.(post|put|open|request).{0,120}"
            r"(key|token|secret|authorization|bearer)"
        ),
    ),
    (
        "SEC-08 字节解码后执行",
        BLOCK,
        re.compile(
            r"base64\.(b64decode|decodebytes).{0,40}(exec|eval)|exec\s*\(\s*base64"
        ),
    ),
    (
        "SEC-09 直接改写宿主配置",
        BLOCK,
        re.compile(
            r"(?i)open\s*\(\s*[\"'][^\"']*\.(mcp\.json|ssh[\\/]authorized_keys|netrc|bashrc)"
        ),
    ),
    ("WARN-A 弱哈希", WARN, re.compile(r"hashlib\.(md5|sha1)\s*\(")),
    (
        "WARN-B 网络出口",
        WARN,
        re.compile(
            r"\bimport\s+(requests|urllib|urllib3|httpx|socket|aiohttp)\b"
            r"|\bfrom\s+(requests|urllib|urllib3|httpx|socket|aiohttp)\b"
        ),
    ),
    (
        "WARN-C 破坏性文件操作",
        WARN,
        re.compile(
            r"shutil\.rmtree|os\.remove|os\.unlink|send2trash|\bos\.rmdir"
            r"|[\"']rm\s+-rf|[\"']del\s+/[sq]"
        ),
    ),
    (
        "WARN-D 子进程执行面",
        WARN,
        re.compile(r"subprocess\.(run|call|check_output|Popen|check_call)"),
    ),
    (
        "WARN-E 吞异常",
        WARN,
        re.compile(r"except\s*(Exception)?\s*:\s*(pass|continue|\.\.\.)"),
    ),
]

SCAN_EXTS = {".py"}


def find_files(targets):
    out = []
    for t in targets:
        if os.path.isfile(t):
            out.append(t)
        elif os.path.isdir(t):
            for dp, dn, fn in os.walk(t):
                dn[:] = [
                    d
                    for d in dn
                    if d not in ("__pycache__", ".git", "target", "node_modules")
                ]
                for f in fn:
                    if os.path.splitext(f)[1].lower() in SCAN_EXTS:
                        out.append(os.path.join(dp, f))
        else:
            out.append(t)  # 不存在也进列表，交给 per-file 判定成无法评审
    return sorted(set(out))


def review_file(path):
    """返回 (findings, 行数)。无法读取必须产出 BLOCK——读不到不等于通过。"""
    if not os.path.exists(path):
        return [
            {
                "rule": "SEC-00 无法评审",
                "severity": BLOCK,
                "line": 0,
                "text": "路径不存在：" + path,
            }
        ], 0
    try:
        raw = open(path, "rb").read()
    except OSError as exc:
        return [
            {
                "rule": "SEC-00 无法评审",
                "severity": BLOCK,
                "line": 0,
                "text": "读取失败：%s" % exc,
            }
        ], 0
    if b"\x00" in raw[:4096]:
        return [
            {
                "rule": "SEC-00 无法评审",
                "severity": BLOCK,
                "line": 0,
                "text": "二进制/含 NUL，无法按文本评审",
            }
        ], 0
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        return [
            {
                "rule": "SEC-00 无法评审",
                "severity": BLOCK,
                "line": 0,
                "text": "非 UTF-8，无法按文本评审：%s" % exc,
            }
        ], 0

    lines = text.splitlines()
    findings = []
    for lineno, line in enumerate(lines, 1):
        for rule, sev, rx in RULES:
            m = rx.search(line)
            if not m:
                continue
            hit = m.group(0)
            if rule.startswith("SEC-05") and BENIGN_SECRET_HINT.search(hit):
                continue
            if rule.startswith("SEC-04") and BENIGN_SECRET_HINT.search(hit):
                continue
            findings.append(
                {
                    "rule": rule,
                    "severity": sev,
                    "line": lineno,
                    "text": line.strip()[:160],
                }
            )
    crlf = text.count("\r\n")
    if crlf:
        findings.append(
            {
                "rule": "WARN-F 行尾 CRLF %d 处" % crlf,
                "severity": WARN,
                "line": 0,
                "text": "入库前建议归一为 LF",
            }
        )
    return findings, len(lines)


def _rc_of_main(argv):
    """静默跑一遍 main，只取退出码——自检要断言的是门的结论，不是中间量。"""
    import contextlib
    import io

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return main(argv)


def selftest():
    """逐条反证：每条 BLOCK 规则必须真能咬到自己造的违规；干净样本必须不咬。

    返回 (通过数, 失败清单)。这道门报绿以前要先过这里，否则"装了工具"会被读成"给了判定"。
    """
    import shutil
    import tempfile

    cases = [
        ("SEC-01", "import subprocess\nsubprocess.run(cmd, shell=True)\n"),
        ("SEC-02", "def f(src):\n    return eval(src)\n"),
        ("SEC-03", "import pickle\nobj = pickle.loads(raw)\n"),
        ("SEC-04", 'H = "sk-ABCDEFGHIJKLMNOPQRSTUV"\n'),
        ("SEC-05", 'api_key = "Zm9vYmFyYmF6cXV1eDEyMzQ1"\n'),
        ("SEC-06", 'print("boot", os.environ["OPENAI_API_KEY"])\n'),
        ("SEC-07", 'requests.post(url, json={"authorization": bearer})\n'),
        ("SEC-08", "exec(base64.b64decode(payload))\n"),
        ("SEC-09", 'open("C:\\\\Users\\\\me\\\\.mcp.json", "w").write(x)\n'),
    ]
    tmp = tempfile.mkdtemp(prefix="sec-review-selftest-")
    ok = 0
    fails = []
    try:
        for want, body in cases:
            p = os.path.join(tmp, "inj_%s.py" % want.replace("-", "_"))
            with open(p, "w", encoding="utf-8", newline="\n") as w:
                w.write(body)
            f, _ = review_file(p)
            hit = [x for x in f if x["severity"] == BLOCK and want in x["rule"]]
            if hit:
                ok += 1
            else:
                fails.append("%s 未判红（实际：%s）" % (want, [x["rule"] for x in f]))
        # 反向：占位符不该算密钥，否则每个示例文档都要开例外
        clean = os.path.join(tmp, "clean.py")
        with open(clean, "w", encoding="utf-8", newline="\n") as w:
            w.write(
                'api_key = "your_api_key_here"\ntoken = "<PUT_TOKEN_HERE>"\n'
                "data = [i for i in range(3)]\nprint(sum(data))\n"
            )
        f, _ = review_file(clean)
        if [x for x in f if x["severity"] == BLOCK]:
            fails.append("占位符被误判为密钥：%s" % [x["rule"] for x in f])
        else:
            ok += 1
        # 覆盖面：空目录必须走 main 判红（没在看 ≠ 通过），要看真退出码
        empty = os.path.join(tmp, "empty")
        os.makedirs(empty, exist_ok=True)
        # 默认根不存在 ⇒ 必须判"通过（无可评审对象）"，否则 CI 干净检出必然咬人
        elsewhere = os.path.join(tmp, "nowhere")
        os.makedirs(elsewhere, exist_ok=True)
        prev = os.getcwd()
        try:
            os.chdir(elsewhere)
            rc_default = _rc_of_main([])
        finally:
            os.chdir(prev)
        if rc_default == 0:
            ok += 1
        else:
            fails.append("默认根不存在时被误判红（rc 应为 0，实为 %s）" % rc_default)
        # 反向：显式指定空目录仍须判红（fail-closed 只在显式口径上生效）
        if _rc_of_main([empty]) == 1:
            ok += 1
        else:
            fails.append("空目录被读成通过（rc 应为 1）")
        # 反向：全干净目录走 main 必须 rc=0
        alldone = os.path.join(tmp, "cleanpkg")
        os.makedirs(alldone, exist_ok=True)
        shutil.copy(clean, os.path.join(alldone, "clean.py"))
        if _rc_of_main([alldone]) == 0:
            ok += 1
        else:
            fails.append("干净目录被判红（rc 应为 0）")
        # 无法评审：二进制必须判红
        blob = os.path.join(tmp, "blob.py")
        with open(blob, "wb") as w:
            w.write(b"\x00\x01\x02binary")
        f, _ = review_file(blob)
        if [x for x in f if x["rule"].startswith("SEC-00")]:
            ok += 1
        else:
            fails.append("二进制文件没被判无法评审")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok, fails


def main(argv=None):
    ap = argparse.ArgumentParser(description="本地 Python 脚本入库前安全评审（只读）")
    ap.add_argument(
        "targets", nargs="*", help="文件或目录；缺省时评审 .codex-local-python/"
    )
    ap.add_argument("--json", action="store_true", help="机器可读输出")
    ap.add_argument("--report", metavar="PATH", help="把结论写成 markdown 报告")
    ap.add_argument("--selftest", action="store_true", help="逐条反证本门的判定能力")
    ap.add_argument(
        "--allow-warn",
        action="store_true",
        help="WARN 不写入报告首屏计数（默认写入，便于对账）",
    )
    args = ap.parse_args(argv)

    if args.selftest:
        n, fails = selftest()
        print("自检：%d 项通过" % n)
        for x in fails:
            print("  FAIL %s" % x)
        if fails:
            print("结论：判红（门自身不可信）")
            return 1
        print("结论：门可用")
        return 0

    targets = args.targets or [".codex-local-python"]
    using_default = not args.targets
    files = find_files(targets)
    if using_default and not any(os.path.isfile(p) for p in files):
        # 默认根不存在＝这台机器上没有本地脚本可评审（CI 干净检出即如此），
        # 不能据此判红；只有用户显式指定目标却评审不到东西才算 fail-closed。
        print("安全评审 · 通过（无可评审对象）")
        print("覆盖面：默认根 %s 不存在或不含 .py ⇒ 本次无脚本可评审" % targets[0])
        if args.report:
            with open(args.report, "w", encoding="utf-8", newline="\n") as w:
                w.write(
                    "# 本地 Python 安全评审报告\n\n- 结论：**通过（无可评审对象）**\n"
                    "- 默认根 `%s` 不存在或不含 .py\n" % targets[0]
                )
        return 0
    per_file = []
    for p in files:
        f, n = review_file(p)
        per_file.append({"path": p, "lines": n, "findings": f})

    blocks = sum(1 for r in per_file for f in r["findings"] if f["severity"] == BLOCK)
    warns = sum(1 for r in per_file for f in r["findings"] if f["severity"] == WARN)
    reviewed = sum(1 for r in per_file if r["lines"] or not r["findings"])
    cannot = sum(
        1 for r in per_file for f in r["findings"] if f["rule"].startswith("SEC-00")
    )
    # 覆盖面为 0 的门不许报绿：否则"没在看"会被读成"通过了"
    empty_run = reviewed == 0
    if empty_run:
        blocks += 1

    verdict = "通过" if blocks == 0 else "判红"
    if args.json:
        print(
            json.dumps(
                {
                    "verdict": verdict,
                    "block": blocks,
                    "warn": warns,
                    "files_reviewed": reviewed,
                    "cannot_review": cannot,
                    "results": per_file,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        print("安全评审 · %s" % verdict)
        print(
            "覆盖面：文件 %d（其中无法评审 %d）｜发现：阻断 %d｜提示 %d"
            % (reviewed, cannot, blocks, warns)
        )
        for r in per_file:
            if not r["findings"]:
                continue
            print("- %s（%d 行）" % (r["path"], r["lines"]))
            for f in r["findings"]:
                print(
                    "    %s %s:%d  %s"
                    % (f["severity"], f["rule"], f["line"], f["text"])
                )
        if empty_run:
            print("    BLOCK SEC-00 覆盖面：本次没有评审到任何文件 ⇒ 不得判绿")

    if args.report:
        with open(args.report, "w", encoding="utf-8", newline="\n") as w:
            w.write("# 本地 Python 安全评审报告\n\n")
            w.write(
                "- 结论：**%s**\n- 覆盖面：评审 %d 个文件，无法评审 %d 个\n"
                "- 阻断项 %d ｜ 提示项 %d\n- 目标：%s\n\n"
                % (verdict, reviewed, cannot, blocks, warns, " ".join(targets))
            )
            w.write("| 文件 | 行 | 级别 | 规则 | 命中 |\n|---|---|---|---|---|\n")
            for r in per_file:
                for f in r["findings"]:
                    cell = f["text"].replace("|", "\\|")
                    w.write(
                        "| `%s` | %d | %s | %s | `%s` |\n"
                        % (r["path"], f["line"], f["severity"], f["rule"], cell[:90])
                    )
        print("报告已写：%s" % args.report)

    return 1 if blocks else 0


if __name__ == "__main__":
    sys.exit(main())
