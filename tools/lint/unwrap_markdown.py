#!/usr/bin/env python3
"""把硬换行的 Markdown 段落合并为单行（CJK-aware），不改变渲染语义。

用法: unwrap_markdown.py [--check] FILE [FILE...]

规则（只做"去掉段落内软换行"这一件事）：
- fenced 代码块（```/~~~）、表格行、ATX 标题、水平线、frontmatter：原样保留。
- 列表项：同一项的连续非空行合并为一行（保留项目符号与缩进）；新的项目符
  号行、缩进不足的行、空行都结束当前项。
- blockquote：按 "> " 前缀分组，组内合并；空引用行结束组。
- 其余普通段落：连续非空行合并为一行。
- 合并时丢弃行尾硬换行空格，插入单个空格；合并点两侧都是 CJK 字符时不补
  空格（中文排版习惯）。
- setext 标题（下一行是 ===/---）：标题文本行原样保留。
- --check 只报告哪些文件会变，不写回。
"""
import argparse
import re
import sys

CJK = (
    "⺀-⻿　-〿぀-ヿ㄰-㆏"
    "㐀-䶿一-鿿豈-﫿︰-﹏"
    "＀-￯‘-”—―…、。！（），：；？"
)
CJK_RE = re.compile(f"^[{CJK}]$")

LIST_RE = re.compile(r"^(\s*)([-*+]\s+|\d+[.)]\s+)(.*)$")
QUOTE_RE = re.compile(r"^(\s*>\s?)(.*)$")
HR_RE = re.compile(r"^\s{0,3}((-\s*){3,}|(\*\s*){3,}|(_\s*){3,})$")
SETEXT_NEXT_RE = re.compile(r"^\s{0,3}(=+|-+)\s*$")
ATX_RE = re.compile(r"^\s{0,3}#{1,6}\s")
TABLE_RE = re.compile(r"^\s{0,3}\|")
FENCE_OPEN_RE = re.compile(r"^\s*(```+|~~~+)\s*")


CJK_TIGHT_PUNCT = "，。；：？！、）】》”’』〕…—·"
CJK_PUNCT_END = re.compile(f"[{CJK_TIGHT_PUNCT}]$")


def join_lines(parts):
    out = ""
    in_span = False  # 是否处在未闭合的 `...` 行内代码 span 中
    for p in parts:
        p = p.strip()
        if not out:
            out = p
        elif not p:
            continue
        elif in_span and out.endswith("_"):
            # span 内的连接符断行（如 `sonar_camera_`\n`reconstruction/` 的笔误）：
            # 下划线是标识符的一部分，不加空格
            out += p
        elif CJK_PUNCT_END.search(out) or (CJK_RE.match(out[-1]) and CJK_RE.match(p[0])):
            # 前一行以中文标点结尾，或两侧都是 CJK 字符：不加空格（中文排版习惯）
            out += p
        else:
            out += " " + p
        in_span = out.count("`") % 2 == 1
    return out


def unwrap(text: str) -> str:
    lines = text.split("\n")
    n = len(lines)
    result = []
    i = 0
    para = []          # 普通段落缓冲
    quote_parts = []   # 引用缓冲：("prefix", "body") 元组列表，每条 "> " 行一项
    cur_item = []      # 当前列表项缓冲（含符号行的完整行）
    list_indent = None # 当前列表项符号行的缩进长度
    quote = False      # 当前是否在引用块内
    prev_blank = True

    def flush():
        nonlocal para, quote, quote_parts
        if para:
            result.append(join_lines(para))
            para = []
        for prefix, body in quote_parts:
            result.append(prefix + body)
        quote_parts = []
        quote = False

    def flush_item():
        nonlocal cur_item, list_indent
        if cur_item:
            result.append(join_lines(cur_item))
            cur_item = []
        list_indent = None

    while i < n:
        line = lines[i].rstrip()
        stripped = line.strip()

        # fenced code block（含列表项内缩进的 fence）
        m = FENCE_OPEN_RE.match(line)
        if m:
            flush(); flush_item()
            result.append(line)
            marker = m.group(1)[:3]
            i += 1
            while i < n:
                result.append(lines[i].rstrip())
                if FENCE_OPEN_RE.match(lines[i]) and lines[i].strip().startswith(marker):
                    i += 1
                    break
                i += 1
            continue

        if not stripped:
            flush(); flush_item()
            result.append(line)
            prev_blank = True
            i += 1
            continue

        # frontmatter（仅文件首行 --- 开始）
        if i == 0 and stripped == "---":
            result.append(line)
            i += 1
            while i < n and lines[i].strip() != "---":
                result.append(lines[i].rstrip())
                i += 1
            if i < n:
                result.append("---")
                i += 1
            continue

        # setext 标题：当前缓冲只有一行且下一物理行是 ===/--- 时，那是标题
        if SETEXT_NEXT_RE.match(line) and len(para) == 1 and not quote:
            # para 里这行其实是 setext 标题文本
            result.append(para.pop())
            result.append(line)
            i += 1
            continue

        if ATX_RE.match(line) or HR_RE.match(line) or TABLE_RE.match(line):
            flush(); flush_item()
            result.append(line)
            prev_blank = True
            i += 1
            continue

        qm = QUOTE_RE.match(line)
        if qm:
            flush_item()
            body = qm.group(2).rstrip()
            if not quote:
                flush()
                quote = True
            prefix = qm.group(1)
            if body.strip():
                quote_parts.append((prefix, body))
            else:
                # "> " 空行：引用内段落分隔，原样保留
                quote_parts.append((prefix.rstrip(), ""))
            i += 1
            continue

        lm = LIST_RE.match(line)
        if lm:
            flush()
            indent_s, marker_s, body = lm.groups()
            ind = len(indent_s.expandtabs(4))
            # 同级或更深层的新列表项都开新行；缩进只影响"该项是否结束"
            flush_item()
            cur_item = [line]
            list_indent = ind
            prev_blank = False
            i += 1
            continue

        ind = len(line) - len(line.lstrip())
        if list_indent is not None and ind > list_indent:
            cur_item.append(line)
        elif list_indent is not None:
            # 缩进回落的普通行：先结算列表项，当作普通段落
            flush_item()
            para.append(line.rstrip())
        else:
            flush_item()
            para.append(line)
        prev_blank = False
        i += 1

    flush(); flush_item()
    # 规范化：去掉文末多余空行之外不动
    out = "\n".join(result)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    changed = []
    for path in args.files:
        with open(path, encoding="utf-8") as f:
            src = f.read()
        dst = unwrap(src)
        if dst != src:
            changed.append(path)
            if not args.check:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(dst)
    if changed:
        print("needs unwrap:" if args.check else "rewrote:")
        for p in changed:
            print(f"  {p}")
        sys.exit(1 if args.check else 0)
    print("clean")


if __name__ == "__main__":
    main()
