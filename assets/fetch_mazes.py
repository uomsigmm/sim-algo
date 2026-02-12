#!/usr/bin/env python3
# /// script
# dependencies = [
#   "requests>=2.31",
#   "beautifulsoup4>=4.12",
# ]
# ///
"""Fetch micromouse maze files from public sources in MMS-compatible format.

Downloads maze files from:
  1. http://www.tcp4me.com/mmr/mazes/ (~95 classic competition mazes)
  2. https://github.com/micromouseonline/mazefiles (~570 mazes across categories)

All mazes are saved in MMS MAP format (text-based wall representation).
Naming convention:
  - tcp4me_{name}.txt       -- mazes from tcp4me.com
  - mmo_{name}.txt          -- classic 16x16 from micromouseonline
  - mmo_half_{name}.txt     -- halfsize 32x32 from micromouseonline
  - mmo_train_{name}.txt    -- training mazes from micromouseonline

Usage:
  uv run assets/fetch_mazes.py [output_dir]

The default output directory is 'assets/mazes'. Existing files are skipped
so the script can be re-run safely to resume interrupted downloads.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import requests
from bs4 import BeautifulSoup

TCP4ME_BASE = "http://www.tcp4me.com/mmr/mazes/"
MMO_RAW_BASE = "https://raw.githubusercontent.com/micromouseonline/mazefiles/master"
MMO_API_BASE = "https://api.github.com/repos/micromouseonline/mazefiles/contents"
SESSION = requests.Session()
SESSION.headers["User-Agent"] = "mms-maze-fetch/2.0"


# -- data types ---------------------------------------------------------------


@dataclass
class MazeInfo:
    name: str
    url: str
    source: str
    category: str = "classic"


# -- tcp4me source ------------------------------------------------------------


def fetch_tcp4me_list() -> list[MazeInfo]:
    """Scrape tcp4me.com index page for .maze file links."""
    resp = SESSION.get(TCP4ME_BASE, timeout=30)
    resp.raise_for_status()
    soup = BeautifulSoup(resp.text, "html.parser")
    mazes = []
    for link in soup.find_all("a", href=True):
        href = link["href"]
        if href.endswith(".maze"):
            name = href.removesuffix(".maze")
            url = TCP4ME_BASE + href if not href.startswith("http") else href
            mazes.append(MazeInfo(name=name, url=url, source="tcp4me"))
    return mazes


def convert_tcp4me_to_mms(content: str) -> str:
    """Convert tcp4me compact maze format to MMS MAP format.

    tcp4me format uses 2-char-wide cells:  +-+-+  and  | |
    MMS MAP format uses 4-char-wide cells: +---+---+ and |       |

    The conversion expands each character in post rows and cell rows.
    tcp4me post row chars: '+' '-' ' '
    tcp4me cell row chars: '|' ' '

    In a post row (starts with '+'):
      '+' -> '+'  (post marker)
      '-' -> '---' (horizontal wall)
      ' ' -> '   ' (no horizontal wall)

    In a cell row (starts with '|' or ' '):
      '|' at even positions -> '|' (vertical wall)
      ' ' at even positions -> ' ' (no vertical wall)
      content between verticals expands from 1 char to 3 chars
    """
    lines = content.splitlines()
    if not lines:
        return content

    # strip trailing empty lines
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        return content

    # detect if this is already MMS format (4-char cells)
    # a 16x16 MMS maze has 33 rows and the first row is 65+ chars
    first_line = lines[0]
    if len(first_line) >= 60:
        return content  # already wide format, return as-is

    result = []
    for line in lines:
        if not line.strip():
            continue
        # replace any leading/trailing dots or special chars with + for corners
        # tcp4me sometimes uses '.' for the bottom-left corner
        line = line.replace(".", "+")
        if line.lstrip().startswith("+"):
            # post row: expand each segment
            out = []
            for ch in line:
                if ch == "+":
                    out.append("+")
                elif ch == "-":
                    out.append("---")
                elif ch == " ":
                    out.append("   ")
                else:
                    out.append(ch)
            result.append("".join(out))
        else:
            # cell row: characters at even positions are wall markers,
            # odd positions are cell content
            out = []
            for i, ch in enumerate(line):
                if i % 2 == 0:
                    # wall position (| or space)
                    out.append(ch)
                else:
                    # cell content: expand 1 char to 3 chars
                    out.append(ch * 3 if ch == " " else f" {ch} ")
            result.append("".join(out))

    return "\n".join(result) + "\n"


# -- micromouseonline source --------------------------------------------------


def fetch_mmo_file_list(subdir: str) -> list[str]:
    """List .txt files in a micromouseonline/mazefiles subdirectory via GitHub API."""
    url = f"{MMO_API_BASE}/{subdir}"
    resp = SESSION.get(url, timeout=30)
    resp.raise_for_status()
    entries = resp.json()
    return [e["name"] for e in entries if e["name"].endswith(".txt")]


def fetch_mmo_list() -> list[MazeInfo]:
    """Get list of all maze files from micromouseonline/mazefiles."""
    mazes = []
    categories = {
        "classic": "mmo",
        "halfsize": "mmo_half",
        "training": "mmo_train",
    }
    for subdir, prefix in categories.items():
        try:
            files = fetch_mmo_file_list(subdir)
            for fname in files:
                name = fname.removesuffix(".txt")
                url = f"{MMO_RAW_BASE}/{subdir}/{fname}"
                mazes.append(
                    MazeInfo(name=name, url=url, source="mmo", category=subdir)
                )
        except Exception as e:
            print(f"  WARN: failed to list mmo/{subdir}: {e}", file=sys.stderr)
    return mazes


# -- format validation --------------------------------------------------------


def validate_mms_maze(content: str) -> tuple[bool, str]:
    """Basic validation that content looks like a valid MMS MAP format maze.

    Returns (is_valid, reason).
    """
    lines = [l for l in content.splitlines() if l.strip()]
    if len(lines) < 3:
        return False, "too few lines"
    # check first line starts with a post character
    if lines[0][0] not in ("+", "o"):
        return False, f"first line starts with '{lines[0][0]}', expected '+' or 'o'"
    # check rectangular-ish: all lines should be roughly the same length
    lengths = [len(l) for l in lines]
    max_len = max(lengths)
    min_len = min(lengths)
    if max_len - min_len > 2:
        return False, f"line lengths vary too much ({min_len}-{max_len})"
    # check minimum dimensions (at least 2x2 maze = 5 lines, 9 chars)
    if len(lines) < 5:
        return False, f"only {len(lines)} lines, need at least 5"
    if max_len < 9:
        return False, f"lines only {max_len} chars wide, need at least 9"
    return True, "ok"


# -- output naming ------------------------------------------------------------


def output_filename(maze: MazeInfo) -> str:
    """Generate semantic output filename for a maze."""
    name = maze.name.lower()
    name = re.sub(r"[^a-z0-9._-]+", "_", name).strip("_")
    if not name:
        name = "unnamed"
    if maze.source == "tcp4me":
        return f"tcp4me_{name}.txt"
    elif maze.category == "halfsize":
        return f"mmo_half_{name}.txt"
    elif maze.category == "training":
        return f"mmo_train_{name}.txt"
    else:
        return f"mmo_{name}.txt"


# -- download logic -----------------------------------------------------------


def download_maze(maze: MazeInfo, out_dir: Path) -> tuple[bool, str]:
    """Download a single maze file. Returns (success, message)."""
    fname = output_filename(maze)
    out_path = out_dir / fname

    if out_path.exists():
        return True, f"SKIP {fname} (exists)"

    try:
        resp = SESSION.get(maze.url, timeout=30)
        resp.raise_for_status()
    except Exception as e:
        return False, f"FAIL {fname}: {e}"

    content = resp.text

    # convert tcp4me format
    if maze.source == "tcp4me":
        content = convert_tcp4me_to_mms(content)

    # validate
    valid, reason = validate_mms_maze(content)
    if not valid:
        return False, f"INVALID {fname}: {reason}"

    out_path.write_text(content)
    return True, f"OK   {fname}"


# -- main ---------------------------------------------------------------------


def process_source(
    label: str,
    mazes: list[MazeInfo],
    out_dir: Path,
    stats: dict,
    dry_run: bool,
    limit: int | None,
    polite_delay: float = 0.0,
):
    """Download mazes from a single source, updating stats."""
    print(f"\n=== {label} ===")
    print(f"  found {len(mazes)} mazes")
    if limit:
        mazes = mazes[:limit]
        print(f"  (limited to {limit})")
    for maze in mazes:
        if dry_run:
            fname = output_filename(maze)
            exists = (out_dir / fname).exists()
            tag = "EXISTS" if exists else "WOULD"
            print(f"  {tag}  {fname}  <-  {maze.url}")
            continue
        success, msg = download_maze(maze, out_dir)
        print(f"  {msg}")
        if "SKIP" in msg:
            stats["skip"] += 1
        elif success:
            stats["ok"] += 1
        else:
            stats["fail"] += 1
        if polite_delay > 0 and success and "SKIP" not in msg:
            time.sleep(polite_delay)


def main():
    parser = argparse.ArgumentParser(description="Fetch micromouse mazes in MMS format")
    parser.add_argument(
        "output",
        nargs="?",
        default="assets/mazes",
        help="output directory (default: assets/mazes)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="list mazes without downloading"
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="max mazes to download per source (for testing)",
    )
    parser.add_argument(
        "--source",
        choices=["tcp4me", "mmo", "all"],
        default="all",
        help="which source to fetch from (default: all)",
    )
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    stats = {"ok": 0, "skip": 0, "fail": 0}

    # source 1: tcp4me.com
    if args.source in ("tcp4me", "all"):
        try:
            tcp4me_mazes = fetch_tcp4me_list()
            tcp4me_mazes.sort(key=lambda m: m.name)
        except Exception as e:
            print(f"ERROR: tcp4me fetch failed: {e}", file=sys.stderr)
            tcp4me_mazes = []
        process_source(
            "tcp4me.com",
            tcp4me_mazes,
            out_dir,
            stats,
            args.dry_run,
            args.limit,
            polite_delay=0.2,
        )

    # source 2: micromouseonline/mazefiles
    if args.source in ("mmo", "all"):
        try:
            mmo_mazes = fetch_mmo_list()
            mmo_mazes.sort(key=lambda m: (m.category, m.name))
        except Exception as e:
            print(f"ERROR: mmo fetch failed: {e}", file=sys.stderr)
            mmo_mazes = []
        process_source(
            "micromouseonline/mazefiles",
            mmo_mazes,
            out_dir,
            stats,
            args.dry_run,
            args.limit,
        )

    # summary
    if not args.dry_run:
        total = stats["ok"] + stats["skip"] + stats["fail"]
        print(f"\n=== Summary ===")
        print(f"  total:      {total}")
        print(f"  downloaded: {stats['ok']}")
        print(f"  skipped:    {stats['skip']}")
        print(f"  failed:     {stats['fail']}")
        print(f"  output:     {out_dir.resolve()}")


if __name__ == "__main__":
    main()
