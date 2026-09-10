#!/usr/bin/env python3
"""
Compare a TheSkyX autoguiding session (Autoguider.NNN.log, under
~/Pictures/TSX Guider/<Month DD YYYY>/) against the AstroTrac X2 driver's own
debug log (AstroTracLog.txt) to check:

  1. Guide-pulse timing - does the driver execute each RA/DEC correction for
     close to the duration TheSkyX actually commanded (derived from GuideErrX/Y,
     the calibrated velocity vectors and the aggressiveness factors), or is
     there a systematic/occasional mismatch?
  2. Comms timing during the session - round-trip times, write times, and any
     timed-out/failed commands (same check as analyze_timing_log.py, but
     restricted to the guiding session's own time window).

Usage:
    python3 analyze_guiding_session.py [txt_file] [date_folder] [--threshold-ms N]

    txt_file      AstroTrac driver debug log. Defaults to ~/AstroTracLog.txt.
    date_folder   Folder name under ~/Pictures/TSX Guider, e.g. "September 03 2026".
                  Defaults to the latest such folder (by date, not mtime).
    --threshold-ms  Comms "long command" threshold in ms. Default 60.

There may be more than one Autoguider.NNN.log in a date folder (TheSkyX starts
a new one roughly every 15 minutes) - all of them are analysed and pooled into
one report.

Output (written into the date folder): guide_cycle_timing.png, comms_timing.png.
"""

import argparse
import datetime
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
ORANGE = "#eb6834"
TEAL = "#2f6f5e"
RUST = "#b23a24"
GRAY_TEXT = "#52514e"
GRAY_GRID = "#d8d7d0"
SURFACE = "#fcfcfb"

DRIVER_TS_RE = r"\w{3} \w{3} +\d+ \d{2}:\d{2}:\d{2}(?:\.\d{3})? \d{4}"


def parse_driver_ts(s):
    # Millisecond precision was added to the driver's log timestamps later than some logs in the
    # wild predate it - handle both.
    fmt = "%a %b %d %H:%M:%S.%f %Y" if "." in s else "%a %b %d %H:%M:%S %Y"
    return datetime.datetime.strptime(s, fmt)


# ---------------------------------------------------------------------------
# Guide log parsing
# ---------------------------------------------------------------------------

def parse_guide_log(path):
    """Returns (start_dt, vectors, speeds, agg, limits, rows). rows is a list
    of dicts with elapsed_s, abs_dt, guide_err_x/y, and the four relay
    columns. vectors holds each calibration push's own (X, Y) pixel-space
    components (not just its magnitude) - needed to properly decompose a
    guide error into X-pair/Y-pair pulse durations when the two calibrated
    push directions aren't aligned with the image X/Y axes (see
    commanded_ms_pair)."""
    start_dt = None
    vectors = {}
    speeds = {}
    agg = {}
    limits = {"min_move": 0.0, "max_move": None, "calib_dec": None, "current_dec": None}
    rows = []

    with open(path, errors="replace") as f:
        for line in f:
            if start_dt is None:
                m = re.match(r"Local Start Date Time = (\d{2}/\d{2}/\d{4} \d{2}:\d{2}:\d{2}\.\d+ [AP]M)", line)
                if m:
                    start_dt = datetime.datetime.strptime(m.group(1), "%d/%m/%Y %I:%M:%S.%f %p")
                    continue

            m = re.match(r"Aggressiveness Factor (X|Y) (Plus|Minus)\s*=\s*([\d.]+)", line)
            if m:
                agg[m.group(1) + m.group(2)] = float(m.group(3)) / 10.0
                continue

            m = re.match(r"\s*(XPlus|XMinus|YPlus|YMinus)\s+Speed=\s*([\d.]+)\s+\(\1\s+X=\s*(-?[\d.]+),\s*\1\s+Y=\s*(-?[\d.]+)\)", line)
            if m:
                name, spd, vx, vy = m.groups()
                speeds[name] = float(spd)
                vectors[name] = (float(vx), float(vy))
                continue

            m = re.match(r"Minimum Move\s*=\s*([\d.]+)", line)
            if m:
                limits["min_move"] = float(m.group(1))
                continue

            m = re.match(r"Maximum Move\s*=\s*([\d.]+)", line)
            if m:
                limits["max_move"] = float(m.group(1))
                continue

            m = re.match(r"Calibration declination\s*=\s*([-\d.]+)", line)
            if m:
                limits["calib_dec"] = float(m.group(1))
                continue

            m = re.match(r"Declination now\s*=\s*([-\d.]+)", line)
            if m:
                limits["current_dec"] = float(m.group(1))
                continue

            if not line.startswith("|") or line.startswith("|Elapsed"):
                continue
            parts = [p.strip() for p in line.strip().strip("|").split("|")]
            if len(parts) < 12:
                continue
            try:
                elapsed = float(parts[0])
                gex = float(parts[5])
                gey = float(parts[6])
                xp, xm, yp, ym = (float(parts[8]), float(parts[9]), float(parts[10]), float(parts[11]))
            except ValueError:
                continue
            rows.append({
                "elapsed_s": elapsed,
                "guide_err_x": gex,
                "guide_err_y": gey,
                "x_plus_relay": xp,
                "x_minus_relay": xm,
                "y_plus_relay": yp,
                "y_minus_relay": ym,
            })

    if start_dt is None:
        raise ValueError(f"{path}: could not find 'Local Start Date Time' header line")
    for row in rows:
        row["abs_dt"] = start_dt + datetime.timedelta(seconds=row["elapsed_s"])

    return start_dt, vectors, speeds, agg, limits, rows


def build_calibration_matrix(vectors):
    """A 2x2 matrix whose columns are the unit directions of the XPlus/YPlus
    calibration pushes. Needed because a naive per-letter pairing
    (GuideErrX/XSpeed for the X-pair, GuideErrY/YSpeed for the Y-pair) is
    only correct if the two calibrated push directions happen to be aligned
    with the image X/Y axes - which is NOT true whenever the guide camera is
    rotated relative to RA/DEC. On the 2026-08-13 session, for example,
    XPlus/XMinus's own vectors point almost entirely along image Y (e.g.
    "XPlus X=0.00138, XPlus Y=-0.41600"), not image X. Confirmed empirically:
    the naive pairing gave a driver-executed/commanded ratio of 1.60 for RA
    and 0.84 for DEC; decomposing through this matrix instead brought both to
    ~1.0 (r=0.93 for both). Returns None if the vectors are missing or
    degenerate (near-parallel, singular matrix)."""
    vx, vy = vectors.get("XPlus"), vectors.get("YPlus")
    if vx is None or vy is None:
        return None
    vx, vy = np.array(vx), np.array(vy)
    nx, ny = np.linalg.norm(vx), np.linalg.norm(vy)
    if nx < 1e-9 or ny < 1e-9:
        return None
    M = np.column_stack([vx / nx, vy / ny])
    if abs(np.linalg.det(M)) < 1e-6:
        return None
    return np.linalg.inv(M)


def commanded_ms_pair(err_x, err_y, speeds, agg, minv):
    """Decompose a (GuideErrX, GuideErrY) pixel error into how long the
    X-pair and Y-pair relays each need to fire, via the calibration matrix
    (see build_calibration_matrix) - not a simple per-letter
    GuideErrX/XSpeed, GuideErrY/YSpeed pairing, which only happens to be
    correct when the calibration is already axis-aligned. Returns
    (x_pair_ms, y_pair_ms), each uncapped (see cap_to_max_move) and possibly
    None if the corresponding speed is missing."""
    dx, dy = minv @ (-np.array([err_x, err_y]))
    x_speed = speeds.get("XPlus") if dx > 0 else speeds.get("XMinus")
    y_speed = speeds.get("YPlus") if dy > 0 else speeds.get("YMinus")
    x_agg = agg.get("XPlus", 1.0) if dx > 0 else agg.get("XMinus", 1.0)
    y_agg = agg.get("YPlus", 1.0) if dy > 0 else agg.get("YMinus", 1.0)
    tx = abs(dx) / x_speed * x_agg * 1000.0 if x_speed else None
    ty = abs(dy) / y_speed * y_agg * 1000.0 if y_speed else None
    return tx, ty


def cap_to_max_move(cmd_ms, max_move):
    """Minimum Move and Maximum Move in the guide log header are in SECONDS
    for this PulseGuide/"Direct Guide" configuration, not pixels - they cap
    the actual pulse DURATION TheSkyX will send, not the pixel error that
    feeds the duration formula. (Units vary by guider connection type - a
    camera-relay/ST4 guider would use pixels instead; always confirm which
    applies before reusing this.) Confirmed on 2026-08-13: Maximum Move=2.00
    matches the ~2.0-2.07s ceiling actually executed for DEC almost exactly."""
    if cmd_ms is None or not max_move:
        return cmd_ms
    return min(cmd_ms, max_move * 1000.0)


def dec_compensation_factor(limits):
    """A calibrated RA-pair speed (px/sec) is only valid at the declination it
    was measured at - RA/HA motion's apparent rate on the sky, and so its
    pixel-space speed, scales with cos(dec). If the current target is at a
    different declination than the calibration was, the calibrated RA speed
    needs scaling by cos(calib_dec)/cos(current_dec) before it's used to
    convert a pixel error into a commanded duration. DEC motion isn't
    foreshortened this way, so this factor is only ever applied to the
    RA-mapped axis. Returns 1.0 (no-op) if either declination is missing or
    current_dec is too close to +/-90 to trust."""
    calib_dec, current_dec = limits.get("calib_dec"), limits.get("current_dec")
    if calib_dec is None or current_dec is None:
        return 1.0
    cos_current = math.cos(math.radians(current_dec))
    if abs(cos_current) < 1e-6:
        return 1.0
    return math.cos(math.radians(calib_dec)) / cos_current


def ra_max_move_scale(limits):
    """Maximum Move is a cap on real angular correction (matches the "arcsec
    version" of this same setting on other guider connection types) - it
    isn't itself tied to the calibration's declination, just to whatever
    fixed real-motion limit the user configured. But converting that fixed
    real-motion cap into RA/HA rotation TIME depends on the CURRENT
    declination: the same real angular correction needs more hour-angle
    rotation at higher declination, since RA motion is foreshortened by
    cos(dec) (a smaller diurnal circle near the pole). So the RA-mapped
    axis's time cap should be Maximum Move / cos(current_dec), not the raw
    value - DEC isn't foreshortened this way, so this never applies there.
    Returns 1.0 (no-op) if current_dec is missing or too close to +/-90."""
    current_dec = limits.get("current_dec")
    if current_dec is None:
        return 1.0
    cos_current = math.cos(math.radians(current_dec))
    if abs(cos_current) < 1e-6:
        return 1.0
    return 1.0 / cos_current


def annotate_commanded(rows, vectors, speeds, agg, limits):
    min_move_ms = limits["min_move"] * 1000.0
    max_move = limits["max_move"]
    dec_factor = dec_compensation_factor(limits)
    ra_move_scale = ra_max_move_scale(limits)
    minv = build_calibration_matrix(vectors)
    for row in rows:
        if minv is not None:
            tx, ty = commanded_ms_pair(row["guide_err_x"], row["guide_err_y"], speeds, agg, minv)
        else:
            tx, ty = None, None
        row["cmd_x_ms"] = tx
        row["cmd_y_ms"] = ty
        row["cmd_x_ms_capped"] = cap_to_max_move(row["cmd_x_ms"], max_move)
        row["cmd_y_ms_capped"] = cap_to_max_move(row["cmd_y_ms"], max_move)
        # A relay can fire on a below-minimum-move correction too (e.g. small
        # housekeeping/backlash pulses independent of the reported guide
        # error) - only trust it as a genuine error-driven correction once
        # the computed duration itself clears TheSkyX's own Minimum Move
        # threshold (seconds, not pixels - see cap_to_max_move).
        row["x_commanded"] = (row["x_plus_relay"] > 0 or row["x_minus_relay"] > 0) and (row["cmd_x_ms"] or 0) >= min_move_ms
        row["y_commanded"] = (row["y_plus_relay"] > 0 or row["y_minus_relay"] > 0) and (row["cmd_y_ms"] or 0) >= min_move_ms
        # Constant per file (declination/Maximum Move don't change row-to-row
        # within one guide log) - applied later, once we know which of X/Y is
        # RA (dec_factor and ra_move_scale only ever apply to the RA-mapped
        # axis).
        row["dec_factor"] = dec_factor
        row["ra_move_scale"] = ra_move_scale
        row["max_move"] = max_move


# ---------------------------------------------------------------------------
# Driver log parsing - open-loop-move cycles, plus comms timing
# ---------------------------------------------------------------------------

START_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\] \[AstroTrac::startOpenSlew\] setting to Dir (\d+)")
STOP_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\] \[AstroTrac::stopOpenLoopMove\] (RA \(East/West\)|DEC \(North/South\)) duration ([\d.]+) seconds")
ROUNDTRIP_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\].*Cmd: (\S+) succeeded (?:first try|after \d+ retries), ([\d.]+) seconds total")
WRITE_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\].*writeFile\+flushTx for Cmd: (\S+) took ([\d.]+) seconds")
TIMEDOUT_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\].*TIMED OUT")
FAILED_RE = re.compile(r"^\[(" + DRIVER_TS_RE + r")\].*FAILED after")

RA_DIRS = {2, 3}   # MD_EAST, MD_WEST
DEC_DIRS = {0, 1}  # MD_NORTH, MD_SOUTH


def parse_driver_log(path):
    """Returns (cycles, comms) where cycles is a list of dicts with end_dt,
    total_ra_ms, total_dec_ms, and comms is a dict of timestamped comms events."""
    pending = {}       # axis -> start_dt (most recent open startOpenSlew for that axis)
    segments = []       # closed (axis, start_dt, stop_dt, duration_s), in log order

    roundtrips = []     # (dt, cmd, seconds)
    writes = []         # (dt, cmd, seconds)
    n_timed_out = 0
    n_failed = 0

    with open(path, errors="replace") as f:
        for line in f:
            m = START_RE.match(line)
            if m:
                ts, dir_code = m.groups()
                dt = parse_driver_ts(ts)
                axis = "RA" if int(dir_code) in RA_DIRS else "DEC"
                pending[axis] = dt
                continue

            m = STOP_RE.match(line)
            if m:
                ts, axis_label, dur = m.groups()
                dt = parse_driver_ts(ts)
                axis = "RA" if axis_label.startswith("RA") else "DEC"
                start_dt = pending.pop(axis, dt)
                segments.append((axis, start_dt, dt, float(dur)))
                continue

            m = ROUNDTRIP_RE.match(line)
            if m:
                ts, cmd, secs = m.groups()
                roundtrips.append((parse_driver_ts(ts), cmd, float(secs) * 1000.0))
                continue

            m = WRITE_RE.match(line)
            if m:
                ts, cmd, secs = m.groups()
                writes.append((parse_driver_ts(ts), cmd, float(secs) * 1000.0))
                continue

            if TIMEDOUT_RE.match(line):
                n_timed_out += 1
                continue
            if FAILED_RE.match(line):
                n_failed += 1
                continue

    # Group segments into cycles: a gap of more than 1s between one segment's
    # stop and the next segment's start starts a new cycle (see
    # astrotrac-guide-pulse-timing-analysis memory, step 4).
    cycles = []
    current = None
    for axis, start_dt, stop_dt, dur in segments:
        if current is not None and (start_dt - current["last_stop"]).total_seconds() > 1.0:
            cycles.append(current)
            current = None
        if current is None:
            current = {"end_dt": stop_dt, "last_stop": stop_dt, "total_ra_ms": 0.0, "total_dec_ms": 0.0}
        current["end_dt"] = stop_dt
        current["last_stop"] = stop_dt
        current[f"total_{axis.lower()}_ms"] += dur * 1000.0
    if current is not None:
        cycles.append(current)

    comms = {
        "roundtrips": roundtrips,
        "writes": writes,
        "n_timed_out": n_timed_out,
        "n_failed": n_failed,
    }
    return cycles, comms


# ---------------------------------------------------------------------------
# Matching guide rows to driver cycles
# ---------------------------------------------------------------------------

def match_rows_to_cycles(rows, cycles, max_match_s=5.0):
    """For each guide row with at least one commanded axis, find the nearest
    cycle by end time. Annotates row['cycle'] and row['match_distance_s']."""
    if not cycles:
        return
    cycle_ends = [c["end_dt"] for c in cycles]
    for row in rows:
        if not (row["x_commanded"] or row["y_commanded"]):
            continue
        best_i, best_dist = None, None
        for i, end_dt in enumerate(cycle_ends):
            dist = abs((end_dt - row["abs_dt"]).total_seconds())
            if best_dist is None or dist < best_dist:
                best_dist, best_i = dist, i
        if best_dist is not None and best_dist <= max_match_s:
            row["cycle"] = cycles[best_i]
            row["match_distance_s"] = best_dist


def detect_axis_mapping(rows):
    """Determine whether the guide log's X/Y pair drives the driver's RA/DEC
    axis, by correlating commanded X/Y duration against executed RA/DEC
    duration across all matched rows (see astrotrac-guide-pulse-timing-analysis
    memory, step 2 - never assume this, confirm it per session). Uses the
    Maximum-Move-capped commanded value, since executed durations are
    themselves capped there - this makes the large-correction rows (which
    dominate a raw, uncapped comparison) match cleanly instead of just adding
    noise."""
    x_cmd, y_cmd, ra_exec, dec_exec = [], [], [], []
    for row in rows:
        if "cycle" not in row:
            continue
        x_cmd.append(row["cmd_x_ms_capped"] if row["x_commanded"] else 0.0)
        y_cmd.append(row["cmd_y_ms_capped"] if row["y_commanded"] else 0.0)
        ra_exec.append(row["cycle"]["total_ra_ms"])
        dec_exec.append(row["cycle"]["total_dec_ms"])

    if len(x_cmd) < 8:
        return "X=RA, Y=DEC (assumed - too few matched rows to verify)", "X", "Y", None, None

    x_cmd, y_cmd, ra_exec, dec_exec = map(np.array, (x_cmd, y_cmd, ra_exec, dec_exec))

    def safe_corr(a, b):
        if np.std(a) == 0 or np.std(b) == 0:
            return 0.0
        return float(np.corrcoef(a, b)[0, 1])

    r_x_ra = safe_corr(x_cmd, ra_exec)
    r_x_dec = safe_corr(x_cmd, dec_exec)

    if r_x_ra >= r_x_dec:
        label = f"X=RA (r={r_x_ra:.4f}), Y=DEC (r={safe_corr(y_cmd, dec_exec):.4f})"
        return label, "X", "Y", r_x_ra, safe_corr(y_cmd, dec_exec)
    else:
        label = f"X=DEC (r={r_x_dec:.4f}), Y=RA (r={safe_corr(y_cmd, ra_exec):.4f})"
        return label, "Y", "X", safe_corr(y_cmd, ra_exec), r_x_dec


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_guide_timing(rows, mapping_label, x_is, y_is, out_path):
    commanded_ra, executed_ra = [], []
    commanded_dec, executed_dec = [], []
    for row in rows:
        if "cycle" not in row:
            continue
        cmd_x = row["cmd_x_ms"] if row["x_commanded"] else 0.0
        cmd_y = row["cmd_y_ms"] if row["y_commanded"] else 0.0
        # Correct order: declination compensation adjusts the underlying
        # calibrated speed (so it must apply before capping), then Maximum
        # Move caps the resulting duration - applying it in the other order
        # could let a dec-scaled value slip back above (or below) the real
        # ceiling TheSkyX actually enforces.
        raw_ra = cmd_x if x_is == "X" else cmd_y
        ra_max_move = row["max_move"] * row["ra_move_scale"] if row["max_move"] else row["max_move"]
        cmd_ra = cap_to_max_move(raw_ra * row["dec_factor"], ra_max_move) if raw_ra else 0.0
        raw_dec = cmd_y if x_is == "X" else cmd_x
        cmd_dec = cap_to_max_move(raw_dec, row["max_move"]) if raw_dec else 0.0
        if cmd_ra > 0:
            commanded_ra.append(cmd_ra)
            executed_ra.append(row["cycle"]["total_ra_ms"])
        if cmd_dec > 0:
            commanded_dec.append(cmd_dec)
            executed_dec.append(row["cycle"]["total_dec_ms"])

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    for ax, commanded, executed, color, name in (
        (axes[0], commanded_ra, executed_ra, BLUE, "RA"),
        (axes[1], commanded_dec, executed_dec, ORANGE, "DEC"),
    ):
        ax.set_facecolor(SURFACE)
        if commanded:
            lim = max(max(commanded), max(executed) if executed else 0) * 1.1
            ax.plot([0, lim], [0, lim], color=GRAY_GRID, linewidth=1.5, linestyle="--", zorder=1)
            ax.scatter(commanded, executed, s=18, color=color, alpha=0.6, zorder=2)
            ratios = np.array(executed) / np.array(commanded)
            ax.set_title(f"{name}: n={len(commanded)}  ratio median={np.median(ratios):.2f} "
                         f"p95={np.percentile(ratios, 95):.2f}", color="#0b0b0b", fontsize=11)
        else:
            ax.text(0.5, 0.5, "No matched corrections", ha="center", va="center",
                    transform=ax.transAxes, color=GRAY_TEXT)
        ax.set_xlabel("Commanded (ms)", color=GRAY_TEXT)
        ax.set_ylabel("Executed (ms)", color=GRAY_TEXT)
        ax.tick_params(colors=GRAY_TEXT)
        ax.grid(True, color=GRAY_GRID, linewidth=0.6, alpha=0.5, zorder=0)
        for spine in ax.spines.values():
            spine.set_color(GRAY_GRID)

    fig.suptitle(f"Guide-pulse commanded vs. executed duration - axis mapping: {mapping_label}",
                 color="#0b0b0b", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, facecolor=SURFACE)


def plot_comms_timing(comms, threshold_ms, window_start, window_end, out_path):
    rt = np.array([ms for dt, cmd, ms in comms["roundtrips"] if window_start <= dt <= window_end])
    wr = np.array([ms for dt, cmd, ms in comms["writes"] if window_start <= dt <= window_end])
    n_over = int((rt > threshold_ms).sum()) if len(rt) else 0
    n_total = len(rt) + comms["n_timed_out"] + comms["n_failed"]

    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    if len(rt):
        max_val = max(rt.max(), wr.max() if len(wr) else 0)
        bins = np.arange(0, max_val + 3, 2)
        ax.hist(rt, bins=bins, color=BLUE, alpha=0.75, label=f"round-trip (n={len(rt)})", zorder=2)
        if len(wr):
            ax.hist(wr, bins=bins, color=ORANGE, alpha=0.7, label=f"write only (n={len(wr)})", zorder=3)
        ax.set_yscale("log")
        ax.axvline(threshold_ms, color=RUST, linewidth=1.5, linestyle="--", zorder=4,
                   label=f"{threshold_ms:.0f}ms threshold")
        legend = ax.legend(loc="upper right", fontsize=9.5, facecolor=SURFACE, edgecolor=GRAY_GRID)
        for t in legend.get_texts():
            t.set_color(GRAY_TEXT)
    else:
        ax.text(0.5, 0.5, "No comms trace found in this window\n(log needs PLUGIN_DEBUG >= 2)",
                ha="center", va="center", transform=ax.transAxes, color=GRAY_TEXT, fontsize=12)

    ax.set_xlabel("Command duration (ms)", color=GRAY_TEXT, fontsize=12)
    ax.set_ylabel("Count (log scale)", color=GRAY_TEXT, fontsize=12)
    ax.grid(True, axis="y", color=GRAY_GRID, linewidth=0.6, which="both", alpha=0.5, zorder=0)
    ax.tick_params(colors=GRAY_TEXT)
    for spine in ax.spines.values():
        spine.set_color(GRAY_GRID)

    title = f"Comms timing during guiding session ({window_start:%H:%M:%S}-{window_end:%H:%M:%S})"
    if len(rt):
        title += f"\nmax={rt.max():.1f}ms  >{threshold_ms:.0f}ms: {n_over}/{n_total}  timed out: {comms['n_timed_out']}  failed: {comms['n_failed']}"
    ax.set_title(title, color="#0b0b0b", fontsize=12, pad=14)

    fig.tight_layout()
    fig.savefig(out_path, facecolor=SURFACE)
    return n_over, n_total, rt.max() if len(rt) else None


# ---------------------------------------------------------------------------
# File discovery
# ---------------------------------------------------------------------------

def find_latest_date_folder(base):
    candidates = []
    for d in base.iterdir():
        if not d.is_dir():
            continue
        try:
            dt = datetime.datetime.strptime(d.name, "%B %d %Y")
        except ValueError:
            continue
        candidates.append((dt, d))
    if not candidates:
        raise SystemExit(f"error: no date-named folders (e.g. 'September 03 2026') found under {base}")
    candidates.sort()
    return candidates[-1][1]


def find_guide_logs(date_dir):
    def sort_key(p):
        m = re.search(r"(\d+)", p.stem)
        return int(m.group(1)) if m else 0
    files = sorted(date_dir.glob("Autoguider.*.log"), key=sort_key)
    if not files:
        raise SystemExit(f"error: no Autoguider.*.log files found in {date_dir}")
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("txt_file", nargs="?", default=None,
                        help="AstroTrac driver debug log (default: ~/AstroTracLog.txt)")
    parser.add_argument("date_folder", nargs="?", default=None,
                        help="Folder name under ~/Pictures/TSX Guider (default: latest by date)")
    parser.add_argument("--threshold-ms", type=float, default=60.0,
                        help="Comms 'long command' threshold in ms (default: 60)")
    args = parser.parse_args()

    driver_log = Path(args.txt_file) if args.txt_file else Path.home() / "AstroTracLog.txt"
    if not driver_log.exists():
        sys.exit(f"error: {driver_log} does not exist")

    guider_base = Path.home() / "Pictures" / "TSX Guider"
    if args.date_folder:
        date_dir = Path(args.date_folder)
        if not date_dir.is_dir():
            date_dir = guider_base / args.date_folder
    else:
        date_dir = find_latest_date_folder(guider_base)
    if not date_dir.is_dir():
        sys.exit(f"error: {date_dir} does not exist")

    guide_logs = find_guide_logs(date_dir)
    print(f"Driver log: {driver_log}")
    print(f"Guide session folder: {date_dir}")
    print(f"Guide log file(s): {', '.join(p.name for p in guide_logs)}")

    print("\nParsing driver log...")
    cycles, comms = parse_driver_log(driver_log)
    print(f"  {len(cycles)} open-loop-move cycles, {len(comms['roundtrips'])} round-trip samples, "
          f"{comms['n_timed_out']} timed out, {comms['n_failed']} failed")

    all_rows = []
    session_start, session_end = None, None
    for guide_log in guide_logs:
        start_dt, vectors, speeds, agg, limits, rows = parse_guide_log(guide_log)
        annotate_commanded(rows, vectors, speeds, agg, limits)
        match_rows_to_cycles(rows, cycles)
        n_matched = sum(1 for r in rows if "cycle" in r)
        n_commanded = sum(1 for r in rows if r["x_commanded"] or r["y_commanded"])
        print(f"  {guide_log.name}: {len(rows)} rows, {n_commanded} with a commanded correction, "
              f"{n_matched} matched to a driver cycle")
        if limits["calib_dec"] is not None and limits["current_dec"] is not None:
            factor = dec_compensation_factor(limits)
            note = "" if abs(factor - 1.0) < 0.001 else "  (RA commanded durations scaled by this)"
            print(f"    calibration dec={limits['calib_dec']:.2f}, current dec={limits['current_dec']:.2f}, "
                  f"RA dec-compensation factor={factor:.4f}{note}")
        all_rows.extend(rows)
        if rows:
            first_dt, last_dt = rows[0]["abs_dt"], rows[-1]["abs_dt"]
            session_start = first_dt if session_start is None else min(session_start, first_dt)
            session_end = last_dt if session_end is None else max(session_end, last_dt)

    mapping_label, x_is, y_is, r1, r2 = detect_axis_mapping(all_rows)
    print(f"\nDetected axis mapping: {mapping_label}")

    match_dists = [r["match_distance_s"] for r in all_rows if "cycle" in r]
    if match_dists:
        print(f"Match distance to nearest cycle: median {np.median(match_dists):.2f}s, "
              f"max {max(match_dists):.2f}s")

    guide_png = date_dir / "guide_cycle_timing.png"
    plot_guide_timing(all_rows, mapping_label, x_is, y_is, guide_png)
    print(f"saved {guide_png}")

    if session_start and session_end:
        comms_png = date_dir / "comms_timing.png"
        n_over, n_total, max_ms = plot_comms_timing(comms, args.threshold_ms, session_start, session_end, comms_png)
        print(f"saved {comms_png}")
        if max_ms is not None:
            print(f"  comms during session: max {max_ms:.1f}ms, >{args.threshold_ms:.0f}ms: {n_over}/{n_total}, "
                  f"timed out: {comms['n_timed_out']}, failed: {comms['n_failed']}")


if __name__ == "__main__":
    main()
