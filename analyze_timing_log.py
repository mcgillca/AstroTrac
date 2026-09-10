#!/usr/bin/env python3
"""
Analyze an AstroTrac driver debug log (AstroTrac.cpp's LogDebug output, PLUGIN_DEBUG>=1) for
command timing - round-trip time per command and, if present, write-only time - and plot the
distribution.

Usage:
    python3 analyze_timing_log.py [logfile] [--threshold-ms N] [--ext png]

    logfile         Path to the log file. Defaults to AstroTracLog.txt in the current directory
                    (run from wherever the log actually is - the Pi's home dir when run there,
                    or the mounted share's path when run from the Mac).
    --threshold-ms  Duration (ms) above which a command counts as "long". Default 30.
    --ext           Output image format/extension. Default png.

Output: an image written next to the input file, same basename - e.g. example1.txt ->
example1.png. Reads "succeeded first try" / "succeeded after N retries" lines for round-trip
time, "writeFile+flushTx" lines for write-only time (if the log was captured with that logging
present - older logs without it just show an empty write-time distribution), and counts
"TIMED OUT" / "FAILED after" lines separately as failed attempts.
"""

import argparse
import datetime
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TS_RE = re.compile(r"^\[(\w{3} \w{3} +\d+ \d{2}:\d{2}:\d{2}(?:\.\d{3})? \d{4})\]")
ROUNDTRIP_RE = re.compile(r"Cmd: (\S+) succeeded (?:first try|after \d+ retries), ([\d.]+) seconds total")
WRITE_RE = re.compile(r"writeFile\+flushTx for Cmd: (\S+) took ([\d.]+) seconds")

BLUE = "#2a78d6"
ORANGE = "#eb6834"
GRAY_TEXT = "#52514e"
GRAY_GRID = "#d8d7d0"
SURFACE = "#fcfcfb"


def parse_ts(s):
    # Millisecond precision was added to the driver's log timestamps later than some logs in the
    # wild predate it - handle both.
    fmt = "%a %b %d %H:%M:%S.%f %Y" if "." in s else "%a %b %d %H:%M:%S %Y"
    return datetime.datetime.strptime(s, fmt)


def analyze(path: Path):
    roundtrip_ms = []
    write_ms = []
    n_timed_out = 0
    n_failed = 0
    first_ts = None
    last_ts = None

    with open(path, errors="replace") as f:
        for line in f:
            m = TS_RE.match(line)
            if m:
                ts = parse_ts(m.group(1))
                if first_ts is None:
                    first_ts = ts
                last_ts = ts

            if "succeeded" in line:
                m = ROUNDTRIP_RE.search(line)
                if m:
                    roundtrip_ms.append(float(m.group(2)) * 1000.0)
                continue
            if "writeFile+flushTx" in line:
                m = WRITE_RE.search(line)
                if m:
                    write_ms.append(float(m.group(2)) * 1000.0)
                continue
            if "TIMED OUT" in line:
                n_timed_out += 1
                continue
            if "FAILED after" in line:
                n_failed += 1
                continue

    return {
        "roundtrip_ms": np.array(roundtrip_ms),
        "write_ms": np.array(write_ms),
        "n_timed_out": n_timed_out,
        "n_failed": n_failed,
        "first_ts": first_ts,
        "last_ts": last_ts,
    }


def make_plot(data, threshold_ms, out_path: Path, title_source: str):
    rt = data["roundtrip_ms"]
    wr = data["write_ms"]
    n_rt = len(rt)
    n_over = int((rt > threshold_ms).sum()) if n_rt else 0
    n_long_total = n_over + data["n_timed_out"] + data["n_failed"]
    n_attempts_total = n_rt + data["n_timed_out"] + data["n_failed"]
    pct_long = (n_long_total / n_attempts_total * 100.0) if n_attempts_total else 0.0

    span_min = None
    rate_per_min = None
    if data["first_ts"] and data["last_ts"]:
        span_s = (data["last_ts"] - data["first_ts"]).total_seconds()
        if span_s > 0:
            span_min = span_s / 60.0
            rate_per_min = n_long_total / span_min

    fig, ax = plt.subplots(figsize=(10, 7), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    if n_rt:
        max_val = max(rt.max(), wr.max() if len(wr) else 0)
        bins = np.arange(0, max_val + 3, 2)
        ax.hist(rt, bins=bins, color=BLUE, alpha=0.75, label=f"round-trip (n={n_rt})", zorder=2)
        if len(wr):
            ax.hist(wr, bins=bins, color=ORANGE, alpha=0.7, label=f"write only (n={len(wr)})", zorder=3)
        ax.set_yscale("log")
        ax.axvline(threshold_ms, color="#b23a24", linewidth=1.5, linestyle="--", zorder=4,
                   label=f"{threshold_ms:.0f}ms threshold")
    else:
        ax.text(0.5, 0.5, "No timing data found in this log", ha="center", va="center",
                transform=ax.transAxes, color=GRAY_TEXT, fontsize=13)

    ax.set_xlabel("Command duration (ms)", color=GRAY_TEXT, fontsize=12)
    ax.set_ylabel("Count (log scale)", color=GRAY_TEXT, fontsize=12)
    ax.grid(True, axis="y", color=GRAY_GRID, linewidth=0.6, which="both", alpha=0.5, zorder=0)
    ax.tick_params(colors=GRAY_TEXT)
    for spine in ax.spines.values():
        spine.set_color(GRAY_GRID)
    if n_rt:
        legend = ax.legend(loc="upper right", fontsize=9.5, facecolor=SURFACE, edgecolor=GRAY_GRID)
        for t in legend.get_texts():
            t.set_color(GRAY_TEXT)

    span_str = f"{span_min:.1f} min" if span_min is not None else "unknown span"
    rate_str = f"{rate_per_min:.3f}/min" if rate_per_min is not None else "n/a"
    stats_lines = [f"{title_source}  -  {span_str}"]
    if n_rt:
        stats_lines.append(
            f"n={n_attempts_total}  median={np.median(rt):.1f}ms  p95={np.percentile(rt, 95):.1f}ms  max={rt.max():.1f}ms"
        )
    stats_lines.append(
        f">{threshold_ms:.0f}ms: {n_long_total}/{n_attempts_total} ({pct_long:.4f}%, rate {rate_str})"
        f"  [{n_over} slow round-trips, {data['n_timed_out']} timed out, {data['n_failed']} failed]"
    )
    ax.set_title("\n".join(stats_lines), color="#0b0b0b", fontsize=12, pad=14)

    fig.tight_layout()
    fig.savefig(out_path, facecolor=SURFACE)
    return {
        "n_attempts_total": n_attempts_total,
        "n_long_total": n_long_total,
        "pct_long": pct_long,
        "rate_per_min": rate_per_min,
        "span_min": span_min,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", nargs="?", default="AstroTracLog.txt",
                        help="Path to the log file (default: AstroTracLog.txt in the current directory)")
    parser.add_argument("--threshold-ms", type=float, default=30.0,
                        help="Duration in ms above which a command counts as 'long' (default: 30)")
    parser.add_argument("--ext", default="png", help="Output image format/extension (default: png)")
    args = parser.parse_args()

    in_path = Path(args.logfile)
    if not in_path.exists():
        print(f"error: {in_path} does not exist", file=sys.stderr)
        sys.exit(1)

    out_path = in_path.with_suffix("." + args.ext.lstrip("."))

    data = analyze(in_path)
    summary = make_plot(data, args.threshold_ms, out_path, in_path.name)

    print(f"Parsed {in_path}")
    print(f"  round-trip samples: {len(data['roundtrip_ms'])}")
    print(f"  write-only samples: {len(data['write_ms'])}")
    print(f"  timed out: {data['n_timed_out']}   failed: {data['n_failed']}")
    if summary["span_min"] is not None:
        print(f"  span: {summary['span_min']:.1f} min")
    print(f"  >{args.threshold_ms:.0f}ms: {summary['n_long_total']}/{summary['n_attempts_total']} "
          f"({summary['pct_long']:.4f}%)"
          + (f", rate {summary['rate_per_min']:.3f}/min" if summary["rate_per_min"] is not None else ""))
    print(f"saved {out_path}")


if __name__ == "__main__":
    main()
