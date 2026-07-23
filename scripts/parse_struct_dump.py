#!/usr/bin/env python3
"""
parse_struct_dump.py - STM32 chip-cooling controller: story-driven analysis.

Accepts EITHER input format (auto-detected per file):
  A) RAW binary dumps of thermal_log / thermal_events
     (GDB: dump binary memory run1.bin &thermal_log (char*)&thermal_log+73728)
     (     dump binary memory run1_events.bin &thermal_events (char*)&thermal_events+768)
  B) GDB/CubeIDE Expressions-view text: {time_s = 433.2, temp_c = 41.7, ...}

Figures:
  <stem>_overview.png     The whole run: temp, load (with real heater_req), fan + RPM.
  <stem>_mechanism.png    One throttle cycle, annotated step by step (arrows).
  <stem>_detail.png       FSM with live counts + dynamic per-fault zoom panels.
Data:
  <stem>_log.csv          Full log export, one row per sample (--csv).
                          NOTE: deliberately not <stem>.csv - scripts/run1.csv
                          is an older, differently-shaped tracked file.

Usage:
  python parse_struct_dump.py run1.bin --events run1_events.bin -o run1
  python parse_struct_dump.py run1.txt --events run1_events.txt --marks run1_marks.txt -o run1
  python parse_struct_dump.py run1.bin --events run1_events.bin --no-show   # write files only
  python parse_struct_dump.py run1.bin --events run1_events.bin --csv         # + CSV
  python parse_struct_dump.py run1.bin --csv-only                             # no figures

Marks file (optional): time,label[,color] per line, e.g.
  433,FAN MOVED AWAY,#d97706
  687,FAN RESTORED,#15803d

SENSOR-CHAIN NOTE (differential rework, contract A):
  Firmware logs vnode as a RECONSTRUCTION from signed counts:
      vnode = 5.0*(counts/5120 + 10/340)
  Trend-faithful, absolute-approximate. Fault thresholds in this domain:
      short (-140 cnt) -> 0.010 V ; open (+400 cnt) -> 0.538 V
  NTC-open now SATURATES at ~0.65 V (not 5 V as in the single-ended era).

MISMATCHED-DUMP GUARD (see validate_events):
  Pairing a short log with an events file from a different run used to draw
  state rectangles thousands of seconds outside the axes. bbox_inches='tight'
  measures every artist, so a 20 s run + 1630 s events produced a 695-inch
  canvas -> 111,195 x 1,057 px -> MemoryError. Events beyond the log are now
  dropped with a warning, and every span/rect is clamped to the axes.
"""

import argparse
import csv
import os
import re
import sys

import numpy as np
import matplotlib

# Backend must be chosen before pyplot is imported. Scan argv directly: the
# real parse happens later, but by then pyplot is already bound to a backend.
if any(a in ("--no-show", "--no-plot", "--csv-only") for a in sys.argv[1:]):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt                                    # noqa: E402
from matplotlib.patches import (Rectangle, FancyBboxPatch,         # noqa: E402
                                FancyArrowPatch)

# ----- Thresholds & Constants -----------------------------------------------
TEMP_VALID_LO, TEMP_VALID_HI = -10.0, 100.0
THROTTLE_FAN_SAT, THROTTLE_MARGIN_C, THROTTLE_ENGAGE_N = 0.98, 0.3, 5
FAN_STALL_RPM = 300

# Reconstructed-vnode fault thresholds (contract A count domain -> volts):
#   v(counts) = 5.0*(counts/5120 + 10/340)
VNODE_SHORT_V = 5.0 * (-140.0 / 5120.0 + 10.0 / 340.0)   # ~0.010 V
VNODE_OPEN_V = 5.0 * (400.0 / 5120.0 + 10.0 / 340.0)     # ~0.538 V
VNODE_YLIM = (-0.05, 0.80)

# Guard limits. A figure wider than this many inches is a bug, not a run.
MAX_FIG_INCHES = 60.0
# Events later than t_end*EV_SLACK_K + EV_SLACK_S are treated as another run.
EV_SLACK_K, EV_SLACK_S = 1.5, 60.0

# ----- Palette --------------------------------------------------------------
C_BG, C_INK, C_INK_SOFT = '#ffffff', '#1f2933', '#6b7280'
C_GRID = '#e5e7eb'
C_TEMP_RAW, C_TEMP_EMA, C_SETPOINT = '#d1d5db', '#d97706', '#1f2933'
C_HEATER, C_FAN, C_REQUEST = '#ea580c', '#0369a1', '#7f1d1d'
C_VNODE, C_FAULT = '#7c3aed', '#b91c1c'
C_PID, C_COOLING, C_IDLE, C_THROTTLE = '#15803d', '#0369a1', '#9ca3af', '#f59e0b'
C_BADGE_BG, C_BADGE_ED = '#fef3c7', '#92400e'
C_DENIED = '#dc2626'

# ----- Firmware enums / layouts ---------------------------------------------
STATE_NAMES = {0: 'IDLE', 1: 'PID', 2: 'THROTTLE', 3: 'COOLING', 4: 'FAULT'}
NAME_TO_ID = {v: k for k, v in STATE_NAMES.items()}
STATE_COLORS = {0: C_IDLE, 1: C_PID, 2: C_THROTTLE, 3: C_COOLING, 4: C_FAULT}
EV_STATE_CHANGE, EV_FAULT_RAISED, EV_FAULT_CLEARED = 1, 2, 3
EV_SETPOINT_CHG, EV_START_REQ, EV_STOP_REQ = 4, 5, 6
EVENT_NAMES = {1: 'STATE_CHANGE', 2: 'FAULT_RAISED', 3: 'FAULT_CLEARED',
               4: 'SETPOINT_CHG', 5: 'START_REQ', 6: 'STOP_REQ'}
FAULT_NAMES = {0: 'NONE', 1: 'NTC_OPEN', 2: 'NTC_SHORT',
               3: 'HEATER_OPEN', 4: 'FAN_OPEN', 5: 'ALL_DISCONNECTED',
               6: 'NODE_OFFLINE'}

EVT_DTYPE = np.dtype([('time_s', '<f4'), ('kind', 'u1'),
                      ('u8_payload', 'u1'), ('_pad', '<u2'),
                      ('f_payload', '<f4')])

# MUST match sizeof(ThermalLogEntry) == 36. If the firmware struct changes,
# this dtype changes with it or every field misaligns from entry 2 onward.
LOG_DTYPE = np.dtype([('time_s', '<f4'), ('temp_c', '<f4'), ('temp_ema', '<f4'),
                      ('vnode', '<f4'), ('fan_duty', '<f4'), ('heater_duty', '<f4'),
                      ('heater_req', '<f4'), ('setpoint', '<f4'), ('fan_rpm', '<u4')])

STATE_GLOSSARY = [
    ('IDLE',     'Outputs off.\nWaiting for start.'),
    ('PID',      'Fan loop holds setpoint.\nHeater = user load\n(simulated chip workload).'),
    ('THROTTLE', 'Fan at 100% & still hot:\n2nd PID overrides user,\ncaps load at sustainable max.'),
    ('COOLING',  'Heater off, fan full.\nPurge residual heat\nbefore idle.'),
    ('FAULT',    'Hardware failure:\nheater killed, fan full.\nAuto-recover when sane.'),
]
TRANSITION_TRIGGERS = {
    ('IDLE', 'PID'): 'start request',
    ('PID', 'THROTTLE'): 'fan maxed &\nstill hot (5 s)',
    ('THROTTLE', 'PID'): 'load sustainable /\nrequest changed',
    ('PID', 'COOLING'): 'stop request',
    ('THROTTLE', 'COOLING'): 'stop request',
    ('PID', 'FAULT'): 'hardware fault',
    ('THROTTLE', 'FAULT'): 'hardware fault',
    ('FAULT', 'COOLING'): 'hardware sane again',
    ('COOLING', 'IDLE'): 'temp below threshold',
}


# =============================================================================
# PARSING - format auto-detection
# =============================================================================
def looks_binary(path):
    """RAW dumps contain NULs and no '=' in the first block; text dumps don't."""
    head = open(path, 'rb').read(512)
    if b'\x00' in head:
        return True
    return b'=' not in head


def parse_binary_log(path):
    rawbytes = open(path, 'rb').read()
    n = len(rawbytes) // LOG_DTYPE.itemsize
    if n == 0:
        raise ValueError(f"{path}: smaller than one ThermalLogEntry "
                         f"({LOG_DTYPE.itemsize} B)")
    if len(rawbytes) % LOG_DTYPE.itemsize:
        print(f"[warn] {path}: {len(rawbytes) % LOG_DTYPE.itemsize} trailing "
              f"bytes ignored - check export started at &thermal_log",
              file=sys.stderr)
    arr = np.frombuffer(rawbytes[:n * LOG_DTYPE.itemsize], dtype=LOG_DTYPE)
    fields = list(LOG_DTYPE.names)
    data = np.column_stack([arr[f].astype(np.float64) for f in fields])
    return fields, data


def parse_binary_events(path):
    rawbytes = open(path, 'rb').read()
    n = len(rawbytes) // EVT_DTYPE.itemsize
    if n == 0:
        return None
    if len(rawbytes) % EVT_DTYPE.itemsize:
        print(f"[warn] {path}: {len(rawbytes) % EVT_DTYPE.itemsize} trailing "
              f"bytes ignored - check export started at &thermal_events",
              file=sys.stderr)
    arr = np.frombuffer(rawbytes[:n * EVT_DTYPE.itemsize], dtype=EVT_DTYPE).copy()
    arr = arr[arr['kind'] != 0]
    return arr[np.argsort(arr['time_s'])] if len(arr) else None


def parse_expressions_dump(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    if not entries:
        raise ValueError("No {field = value, ...} blocks found "
                         "(and file did not look binary).")
    parsed = []
    for entry in entries:
        pairs = re.findall(r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if pairs:
            parsed.append(dict(pairs))
    if not parsed:
        raise ValueError("Found {...} blocks but no field = number pairs inside.")
    fields = list(parsed[0].keys())
    rows = [[float(d[f]) for f in fields] for d in parsed if all(f in d for f in fields)]
    if not rows:
        raise ValueError("No block contained the full field set.")
    seen, unique = set(), []
    for row in rows:
        key = tuple(round(v, 6) for v in row)
        if key not in seen:
            seen.add(key)
            unique.append(row)
    return fields, np.array(unique, dtype=np.float64)


def parse_events_from_expressions(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    parsed, req = [], ('time_s', 'kind', 'u8_payload', 'f_payload')
    for entry in entries:
        pairs = re.findall(r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if not pairs:
            continue
        d = dict(pairs)
        if all(k in d for k in req):
            parsed.append((float(d['time_s']), int(float(d['kind'])),
                           int(float(d['u8_payload'])), 0, float(d['f_payload'])))
    if not parsed:
        return None
    arr = np.array(parsed, dtype=EVT_DTYPE)
    arr = arr[arr['kind'] != 0]
    if len(arr) == 0:
        return None
    keys = {(float(e['time_s']), int(e['kind']), int(e['u8_payload']),
             float(e['f_payload'])): e for e in arr}
    arr = np.array(list(keys.values()), dtype=EVT_DTYPE)
    return arr[np.argsort(arr['time_s'])]


def load_log(path):
    if looks_binary(path):
        print(f"[in] {path}: RAW binary", file=sys.stderr)
        return parse_binary_log(path)
    print(f"[in] {path}: Expressions text", file=sys.stderr)
    with open(path, encoding="utf-8", errors="replace") as f:
        return parse_expressions_dump(f.read())


def load_events(path):
    if path is None:
        return None
    if looks_binary(path):
        print(f"[in] {path}: RAW binary (events)", file=sys.stderr)
        return parse_binary_events(path)
    print(f"[in] {path}: Expressions text (events)", file=sys.stderr)
    with open(path, encoding="utf-8", errors="replace") as f:
        return parse_events_from_expressions(f.read())


def validate_events(events, t_end, path):
    """Reject an events file that plainly belongs to a different run.

    Symptom this prevents: a 20 s log paired with events from a 1630 s run
    drew state rectangles far outside the axes; bbox_inches='tight' then sized
    the canvas to the artists and matplotlib tried to allocate ~0.5 GB.

    Events *before* the log window are kept - a ring that wrapped legitimately
    starts mid-run, and the last prior state change establishes the state at
    t[0]. Only events after the log ends are dropped.
    """
    if events is None or len(events) == 0:
        return None

    ev_lo = float(events['time_s'].min())
    ev_hi = float(events['time_s'].max())
    limit = t_end * EV_SLACK_K + EV_SLACK_S

    if ev_hi > limit:
        print("", file=sys.stderr)
        print(f"[MISMATCH] {path}: events span {ev_lo:.1f}..{ev_hi:.1f}s but the "
              f"log ends at {t_end:.1f}s.", file=sys.stderr)
        print("[MISMATCH] These look like dumps from two different runs. "
              "Re-export both", file=sys.stderr)
        print("[MISMATCH] from the same halt, or the figures below will be "
              "meaningless.", file=sys.stderr)
        print("", file=sys.stderr)

    keep = events[events['time_s'] <= t_end]
    dropped = len(events) - len(keep)
    if dropped:
        print(f"[warn] dropped {dropped} event(s) past the end of the log",
              file=sys.stderr)
    return keep if len(keep) else None


def load_marks(path):
    marks = []
    if not path:
        return marks
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.startswith("#"):
                    continue
                parts = [p.strip() for p in line.strip().split(",")]
                if len(parts) >= 2:
                    try:
                        marks.append((float(parts[0]), parts[1],
                                      parts[2] if len(parts) > 2 else C_INK_SOFT))
                    except ValueError:
                        print(f"[warn] marks: skipping unparseable line: {line.strip()}",
                              file=sys.stderr)
    except FileNotFoundError:
        print(f"[warn] marks file {path} not found - continuing without marks",
              file=sys.stderr)
    return marks


def col(data, fields, name):
    return data[:, fields.index(name)] if name in fields else None


# =============================================================================
# DERIVED VIEWS
# =============================================================================
def state_intervals(events, t_end):
    if events is None or len(events) == 0:
        return []
    sc = events[events['kind'] == EV_STATE_CHANGE]
    out = []
    for i, e in enumerate(sc):
        t0 = float(e['time_s'])
        t1 = float(sc[i + 1]['time_s']) if i + 1 < len(sc) else t_end
        out.append((t0, t1, int(e['u8_payload'])))
    return out


def clamp_span(t0, t1, lo, hi):
    """Clip an interval to the axes. Returns None if nothing is left.

    Every axvspan/Rectangle goes through this. Without it, one out-of-range
    interval inflates the tight bbox and the render allocation explodes.
    """
    a, b = max(lo, min(t0, hi)), max(lo, min(t1, hi))
    return None if b <= a else (a, b)


def fault_windows(events):
    if events is None or len(events) == 0:
        return []
    wins, t0, reason = [], None, None
    for e in events:
        k = int(e['kind'])
        if k == EV_FAULT_RAISED:
            t0, reason = float(e['time_s']), FAULT_NAMES.get(int(e['u8_payload']), '?')
        elif k == EV_FAULT_CLEARED and t0 is not None:
            wins.append((t0, float(e['time_s']), reason))
            t0, reason = None, None
    return wins


def fault_mask(t, events):
    """True where a fault window is active - sensors read garbage here."""
    m = np.zeros_like(t, dtype=bool)
    for (t0, t1, _) in fault_windows(events):
        m |= (t >= t0) & (t <= t1)
    return m


def mask_during_faults(x, t, events, fill=np.nan):
    """Blank a channel inside fault windows (RPM/vnode are meaningless there)."""
    if x is None:
        return None
    x = x.astype(float).copy()
    x[fault_mask(t, events)] = fill
    return x


def throttle_mask(t, intervals):
    """True only inside THROTTLE intervals.

    The denied-load hatch MUST be gated on this. Heater is also below the
    request during FAULT and COOLING - but that is a safety cutoff, not the
    throttle refusing load. Hatching it credits the throttle with work it
    did not do, which is the kind of overclaim a reviewer will catch.
    """
    m = np.zeros_like(t, dtype=bool)
    for (t0, t1, st) in intervals:
        if STATE_NAMES.get(st) == 'THROTTLE':
            m |= (t >= t0) & (t <= t1)
    return m


def _finite(series_list):
    vals = []
    for s in series_list:
        if s is None:
            continue
        v = np.asarray(s, dtype=float)
        v = v[np.isfinite(v)]
        if len(v):
            vals.append(v)
    return np.concatenate(vals) if vals else None


def nice_ylim(series_list, pad_frac=0.12, min_span=2.0, hard_lo=None, hard_hi=None):
    """Plain min/max limits from finite data only."""
    allv = _finite(series_list)
    if allv is None:
        return None
    lo, hi = float(allv.min()), float(allv.max())
    if hi - lo < min_span:
        mid = 0.5 * (lo + hi)
        lo, hi = mid - min_span / 2, mid + min_span / 2
    pad = (hi - lo) * pad_frac
    lo, hi = lo - pad, hi + pad
    if hard_lo is not None:
        lo = max(lo, hard_lo)
    if hard_hi is not None:
        hi = min(hi, hard_hi)
    return lo, hi


def zoom_ylim(series_list, pad_frac=0.18, min_span=2.0, q=(1.0, 99.0),
              max_stretch=2.2, hard_lo=None, hard_hi=None):
    """Limits framed on the BULK of the data, not on its extremes.

    A single 48 C excursion in an otherwise 34 C run forces a 25..50 axis and
    squashes 1600 s of regulation into a few pixels. Frame on the 1..99th
    percentile instead, then allow the window to grow by at most `max_stretch`
    to swallow real excursions. Anything still outside is clipped - the caller
    marks that with clip_markers() so nothing is silently hidden.

    Returns (lo, hi, clipped_lo, clipped_hi).
    """
    allv = _finite(series_list)
    if allv is None:
        return None
    p_lo, p_hi = np.percentile(allv, q)
    core = max(float(p_hi - p_lo), 1e-9)
    if core < min_span:
        mid = 0.5 * (p_lo + p_hi)
        p_lo, p_hi = mid - min_span / 2, mid + min_span / 2
        core = p_hi - p_lo
    pad = core * pad_frac
    lo, hi = p_lo - pad, p_hi + pad
    limit = core * max_stretch
    d_lo, d_hi = float(allv.min()), float(allv.max())
    lo = max(lo, min(d_lo - pad, p_lo - limit))
    hi = min(hi, max(d_hi + pad, p_hi + limit))
    lo = min(lo, d_lo - pad) if (p_lo - d_lo) <= limit else lo
    hi = max(hi, d_hi + pad) if (d_hi - p_hi) <= limit else hi
    if hard_lo is not None:
        lo = max(lo, hard_lo)
    if hard_hi is not None:
        hi = min(hi, hard_hi)
    return lo, hi, bool(d_lo < lo - 1e-9), bool(d_hi > hi + 1e-9)


def clip_markers(ax, t, series, lo, hi, color, label_hi=None, label_lo=None):
    """Draw arrows where a trace leaves the zoomed window, so it is never
    silently cropped. Honest zoom: the reader sees that data went off-scale."""
    if series is None:
        return
    v = np.asarray(series, dtype=float)
    for m, y, va, txt in ((v > hi, hi, 'top', label_hi), (v < lo, lo, 'bottom', label_lo)):
        if not m.any():
            continue
        xs = t[m]
        # one marker per contiguous excursion
        brk = np.where(np.diff(xs) > (t[1] - t[0]) * 3)[0] if len(xs) > 1 else []
        for seg in np.split(xs, np.array(brk) + 1):
            if not len(seg):
                continue
            xm = float(seg[len(seg) // 2])
            ax.annotate('', xy=(xm, y), xytext=(xm, y - (hi - lo) * 0.07
                                                if va == 'top' else y + (hi - lo) * 0.07),
                        arrowprops=dict(arrowstyle='-|>', color=color, lw=1.6,
                                        mutation_scale=11), zorder=9,
                        annotation_clip=False)
        if txt:
            peak = float(np.nanmax(v)) if va == 'top' else float(np.nanmin(v))
            ax.text(0.995, 0.965 if va == 'top' else 0.035, txt.format(peak=peak),
                    transform=ax.transAxes, ha='right',
                    va='top' if va == 'top' else 'bottom', fontsize=8,
                    color=color, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=color,
                              lw=0.7, alpha=0.95), zorder=10)


def mask_invalid_temps(x, t, events):
    if x is None:
        return None
    x = x.copy()
    x[(x < TEMP_VALID_LO) | (x > TEMP_VALID_HI)] = np.nan
    for (t0, t1, _) in fault_windows(events):
        x[(t >= t0) & (t <= t1)] = np.nan
    return x


def throttle_cycles(intervals):
    out = []
    for i, (t0, t1, st) in enumerate(intervals):
        if STATE_NAMES.get(st) != 'THROTTLE':
            continue
        prev_st = STATE_NAMES.get(intervals[i - 1][2]) if i > 0 else None
        next_st = STATE_NAMES.get(intervals[i + 1][2]) if i + 1 < len(intervals) else None
        out.append((t0, t1, prev_st, next_st))
    return out


def handover_delta(t, heater, t_enter):
    if heater is None:
        return None
    before = heater[(t < t_enter)][-1:]
    after = heater[(t >= t_enter)][:1]
    if len(before) and len(after):
        return float(after[0] - before[0])
    return None


def count_state_visits(events):
    out = {}
    if events is None:
        return out
    for e in events[events['kind'] == EV_STATE_CHANGE]:
        st = int(e['u8_payload'])
        out[st] = out.get(st, 0) + 1
    return out


def count_transitions(events):
    out, prev = {}, None
    if events is None:
        return out
    for e in events[events['kind'] == EV_STATE_CHANGE]:
        st = int(e['u8_payload'])
        if prev is not None:
            out[(prev, st)] = out.get((prev, st), 0) + 1
        prev = st
    return out


# =============================================================================
# STYLE HELPERS
# =============================================================================
def apply_style():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'axes.titlesize': 11, 'axes.titleweight': 'bold',
        'axes.labelsize': 10, 'axes.edgecolor': C_INK_SOFT, 'axes.linewidth': 0.8,
        'axes.spines.top': False, 'axes.spines.right': False, 'grid.color': C_GRID,
        'grid.linewidth': 0.6, 'legend.frameon': False, 'legend.fontsize': 9,
        'xtick.color': C_INK_SOFT, 'ytick.color': C_INK_SOFT, 'xtick.labelsize': 9,
        'ytick.labelsize': 9,
    })


def compute_kpis(data, fields, events):
    """Headline numbers for the banner. All measured, none asserted."""
    t = col(data, fields, "time_s")
    t_end = float(t[-1])
    iv = state_intervals(events, t_end)
    temp = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    setp = col(data, fields, "setpoint")
    heater = col(data, fields, "heater_duty")
    req = col(data, fields, "heater_req")
    k = {}
    k['dur'] = f"{int(t_end)//60}:{int(t_end)%60:02d}"

    # hold error: PID only, setpoint valid, temperature valid
    if temp is not None and setp is not None:
        pid_m = np.zeros_like(t, dtype=bool)
        for (a, b, st) in iv:
            if STATE_NAMES.get(st) == 'PID':
                pid_m |= (t >= a) & (t <= b)
        m = pid_m & np.isfinite(temp) & (setp > 0)
        k['err'] = f"{np.abs(temp[m] - setp[m]).mean():.2f} °C" if m.sum() > 10 else "n/a"
    else:
        k['err'] = "n/a"

    cyc = throttle_cycles(iv)
    k['thr'] = f"{len(cyc)}" if cyc else "0"
    if cyc and heater is not None:
        dv = handover_delta(t, heater, cyc[0][0])
        k['step'] = f"{dv:+.3f}" if dv is not None else "n/a"
    else:
        k['step'] = "n/a"

    if req is not None and heater is not None:
        d = (np.isfinite(req) & (req - heater > 0.01) & throttle_mask(t, iv))
        k['den'] = f"{np.trapezoid((req - heater)[d], t[d]):.0f} duty·s" if d.any() else "0"
    else:
        k['den'] = "n/a"

    fw = fault_windows(events)
    k['flt'] = f"{len(fw)} / {len(fw)} cleared" if fw else "none"
    return k


def draw_kpi_banner(ax, k):
    ax.axis('off')
    cells = [("RUN LENGTH", k['dur'], C_INK),
             ("MEAN |ERROR| IN PID", k['err'], C_PID),
             ("THROTTLE CYCLES", k['thr'], C_THROTTLE),
             ("HANDOVER STEP", k['step'], C_THROTTLE),
             ("LOAD REFUSED", k['den'], C_DENIED),
             ("FAULTS", k['flt'], C_FAULT)]
    n = len(cells)
    for i, (lab, val, c) in enumerate(cells):
        x = i / n
        ax.add_patch(Rectangle((x + 0.004, 0.05), 1.0 / n - 0.008, 0.9,
                               transform=ax.transAxes, facecolor='#f8fafc',
                               edgecolor='#dde3e8', lw=0.9, clip_on=False))
        ax.text(x + 0.5 / n, 0.66, lab, transform=ax.transAxes, ha='center',
                va='center', fontsize=7.4, color=C_INK_SOFT)
        fs = 12.5 if len(val) <= 9 else (10.5 if len(val) <= 14 else 9.0)
        ax.text(x + 0.5 / n, 0.30, val, transform=ax.transAxes, ha='center',
                va='center', fontsize=fs, color=c, fontweight='bold')


def draw_title(ax, main, subtitle, suffix=""):
    ax.axis('off')
    ax.text(0.0, 0.85, main, fontsize=20, fontweight='bold', color=C_INK,
            ha='left', va='top', transform=ax.transAxes)
    ax.text(0.0, 0.22, subtitle + (" - " + suffix if suffix else ""), fontsize=10,
            color=C_INK_SOFT, ha='left', va='top', transform=ax.transAxes)


def shade_states(ax, intervals, t_end, alpha_scale=1.0, t_lo=0.0):
    shade = {1: (C_PID, 0.05), 2: (C_THROTTLE, 0.13),
             3: (C_COOLING, 0.07), 4: (C_FAULT, 0.12)}
    for (t0, t1, st) in intervals:
        if st not in shade:
            continue
        span = clamp_span(t0, t1, t_lo, t_end)
        if span is None:
            continue
        c, a = shade[st]
        ax.axvspan(span[0], span[1], color=c, alpha=a * alpha_scale, zorder=0)


def state_strip(ax, intervals, t_end, label_min_frac=0.05):
    ax.set_xlim(0, t_end)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.grid(False)
    for s in ('left', 'top', 'right'):
        ax.spines[s].set_visible(False)
    for (t0, t1, st) in intervals:
        span = clamp_span(t0, t1, 0.0, t_end)
        if span is None:
            continue
        a, b = span
        ax.add_patch(Rectangle((a, 0.25), b - a, 0.5,
                               facecolor=STATE_COLORS.get(st, C_IDLE),
                               edgecolor='white', linewidth=0.5))
        if (b - a) > label_min_frac * t_end:
            nm = STATE_NAMES.get(st, '?')
            ax.text((a + b) / 2, 0.5, 'THROT.' if nm == 'THROTTLE' else nm,
                    ha='center', va='center', fontsize=6.5,
                    fontweight='bold', color='white')


def draw_marks(marks, label_ax, line_axes, t_lo=None, t_hi=None):
    for (tm, lbl, mc) in (marks or []):
        if t_lo is not None and not (t_lo <= tm <= t_hi):
            continue
        for ax in line_axes:
            ax.axvline(tm, color=mc, lw=1.1, ls='--', alpha=0.8, zorder=6)
        label_ax.annotate(lbl, xy=(tm, 0.99), xycoords=('data', 'axes fraction'),
                          ha='center', va='top', fontsize=7.5, color=mc,
                          fontweight='bold',
                          bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=mc,
                                    lw=0.8, alpha=0.95), zorder=11)


def save_figure(fig, png_path, dpi=160):
    """Save with a tight bbox, but refuse to allocate an absurd canvas.

    bbox_inches='tight' sizes to the union of all artists. A stray artist far
    outside the axes turns that into gigabytes. Measure first, fall back to the
    plain figure box if the tight box is unreasonable.
    """
    bbox = 'tight'
    try:
        fig.canvas.draw()
        tb = fig.get_tightbbox(fig.canvas.get_renderer())
        if tb.width > MAX_FIG_INCHES or tb.height > MAX_FIG_INCHES:
            print(f"[warn] tight bbox is {tb.width:.0f}x{tb.height:.0f} in - "
                  f"an artist is far outside the axes.", file=sys.stderr)
            print("[warn] saving without bbox_inches='tight'. "
                  "Check that the log and events are from the same run.",
                  file=sys.stderr)
            bbox = None
    except Exception as exc:                                  # pragma: no cover
        print(f"[warn] could not measure tight bbox ({exc}); saving plain",
              file=sys.stderr)
        bbox = None

    fig.savefig(png_path, dpi=dpi, bbox_inches=bbox, facecolor=C_BG)
    print(f"[png] wrote {png_path}", file=sys.stderr)


# =============================================================================
# CSV EXPORT
# =============================================================================
def export_csv(data, fields, csv_path):
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for row in data:
            w.writerow([f"{v:.6g}" for v in row])
    print(f"[csv] wrote {csv_path}  ({len(data)} rows)", file=sys.stderr)


def export_events_csv(events, csv_path):
    if events is None or len(events) == 0:
        return
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "kind", "kind_name", "u8_payload",
                    "payload_name", "f_payload"])
        for e in events:
            kind = int(e['kind'])
            u8 = int(e['u8_payload'])
            if kind == EV_STATE_CHANGE:
                pname = STATE_NAMES.get(u8, '?')
            elif kind in (EV_FAULT_RAISED, EV_FAULT_CLEARED):
                pname = FAULT_NAMES.get(u8, '?')
            else:
                pname = ''
            w.writerow([f"{float(e['time_s']):.3f}", kind,
                        EVENT_NAMES.get(kind, '?'), u8, pname,
                        f"{float(e['f_payload']):.6g}"])
    print(f"[csv] wrote {csv_path}  ({len(events)} events)", file=sys.stderr)


# =============================================================================
# FIGURE 1 - OVERVIEW
# =============================================================================
def make_overview_plot(data, fields, events, png_path, show, title_suffix="",
                       marks=None, trange=None):
    apply_style()
    t = col(data, fields, "time_s")
    t_end = float(t[-1])
    temp_raw = mask_invalid_temps(col(data, fields, "temp_c"), t, events)
    temp_ema = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    setp = col(data, fields, "setpoint")
    fan = col(data, fields, "fan_duty")
    fan_rpm = mask_during_faults(col(data, fields, "fan_rpm"), t, events)
    heater = col(data, fields, "heater_duty")
    request = col(data, fields, "heater_req")
    intervals = state_intervals(events, t_end)

    fig = plt.figure(figsize=(13.5, 9.0), facecolor=C_BG)
    gs = fig.add_gridspec(6, 1, height_ratios=[0.50, 0.52, 2.5, 1.5, 1.1, 0.38],
                          hspace=0.34, left=0.07, right=0.94, top=0.955, bottom=0.055)

    ax_title = fig.add_subplot(gs[0])
    ax_kpi = fig.add_subplot(gs[1])
    ax_temp = fig.add_subplot(gs[2])
    ax_load = fig.add_subplot(gs[3], sharex=ax_temp)
    ax_fan = fig.add_subplot(gs[4], sharex=ax_temp)
    ax_strip = fig.add_subplot(gs[5], sharex=ax_temp)
    draw_kpi_banner(ax_kpi, compute_kpis(data, fields, events))

    draw_title(ax_title, "STM32 Chip-Cooling Controller",
               "Fan PID holds temperature; when cooling authority runs out, "
               "a second PID overrides the requested load", title_suffix)

    ax_temp.set_xlim(0, t_end)

    shade_states(ax_temp, intervals, t_end)
    if temp_raw is not None:
        ax_temp.plot(t, temp_raw, lw=0.7, color=C_TEMP_RAW, label='temp (raw)')
    if temp_ema is not None:
        ax_temp.plot(t, temp_ema, lw=1.9, color=C_TEMP_EMA, label='temp (EMA, loop input)')
    if setp is not None:
        sp = setp.copy()
        sp[sp <= 0] = np.nan
        ax_temp.step(t, sp, where='post', lw=1.3, ls='--', color=C_SETPOINT,
                     label='setpoint')
    ax_temp.set_ylabel("Temp (°C)")
    ax_temp.grid(True, axis='y')
    ax_temp.legend(loc='lower right', ncol=3, bbox_to_anchor=(1.0, 1.005))
    if trange:
        ax_temp.set_ylim(*trange)
    else:
        sp_v = setp.copy() if setp is not None else None
        if sp_v is not None:
            sp_v[sp_v <= 0] = np.nan
        z = zoom_ylim([temp_ema, sp_v], pad_frac=0.30, min_span=4.0,
                      q=(2.0, 98.0), max_stretch=0.9)
        if z:
            lo, hi, _, _ = z
            ax_temp.set_ylim(lo, hi)
            clip_markers(ax_temp, t, temp_ema, lo, hi, C_TEMP_EMA,
                         label_hi='peak {peak:.1f} °C (off-scale)')

    shade_states(ax_load, intervals, t_end)
    if heater is not None:
        ax_load.fill_between(t, 0, heater, color=C_HEATER, alpha=0.40,
                             label='granted load (heater PWM)')
        ax_load.plot(t, heater, lw=1.0, color=C_HEATER)
    if request is not None and heater is not None:
        ax_load.plot(t, request, lw=1.4, ls=':', color=C_REQUEST,
                     label='user request (g_heater_request)')
        # Gate on THROTTLE: heater < request during FAULT/COOLING too, but that
        # is the safety cutoff, not the throttle. See throttle_mask().
        denied = (np.isfinite(request) & (request - heater > 0.01)
                  & throttle_mask(t, intervals))
        ax_load.fill_between(t, heater, request, where=denied, facecolor='none',
                             edgecolor=C_DENIED, hatch='///', lw=0.0, alpha=0.85,
                             label='load DENIED by throttle')
        e_den = float(np.trapezoid((request - heater)[denied], t[denied])) if denied.any() else 0.0
        if e_den > 0:
            ax_load.text(0.995, 0.90, f"{e_den:.0f} duty·s refused",
                         transform=ax_load.transAxes, ha='right', va='top',
                         fontsize=8.5, color=C_DENIED, fontweight='bold',
                         bbox=dict(boxstyle='round,pad=0.3', fc='white',
                                   ec=C_DENIED, lw=0.8, alpha=0.95))
    z = zoom_ylim([heater, request], pad_frac=0.30, min_span=0.12,
                  max_stretch=3.0, hard_lo=-0.02, hard_hi=1.08)
    if z:
        ax_load.set_ylim(z[0], z[1])
    else:
        ax_load.set_ylim(-0.05, 1.12)
    ax_load.set_ylabel("Load [0..1]")
    ax_load.grid(True, axis='y')
    ax_load.legend(loc='upper left', ncol=3, fontsize=8, bbox_to_anchor=(0.0, 1.28))

    shade_states(ax_fan, intervals, t_end)
    lines_f, labels_f = [], []
    if fan is not None:
        l1 = ax_fan.plot(t, fan, lw=1.5, color=C_FAN, label='fan cmd (0..1)')
        ax_fan.axhline(THROTTLE_FAN_SAT, color=C_FAULT, lw=0.8, ls=':')
        lines_f += l1
        labels_f.append('fan cmd (0..1)')
    if fan_rpm is not None:
        ax_rpm = ax_fan.twinx()
        l2 = ax_rpm.plot(t, fan_rpm, lw=1.2, ls='-', color=C_INK_SOFT, alpha=0.5,
                         label='tachometer (RPM)')
        ax_rpm.set_ylabel("Speed (RPM)", color=C_INK_SOFT)
        ax_rpm.tick_params(axis='y', labelcolor=C_INK_SOFT)
        yl = nice_ylim([fan_rpm], hard_lo=0.0)
        if yl:
            ax_rpm.set_ylim(*yl)
        lines_f += l2
        labels_f.append('tachometer (RPM)')
    ax_fan.set_ylim(-0.05, 1.12)
    ax_fan.set_ylabel("Fan Duty")
    ax_fan.grid(True, axis='y')
    if lines_f:
        ax_fan.legend(lines_f, labels_f, loc='upper left', fontsize=8)

    state_strip(ax_strip, intervals, t_end)
    ax_strip.set_xlabel("Time (s)   /   total run: {}:{:02d}".format(
        int(t_end) // 60, int(t_end) % 60))

    for ax in (ax_temp, ax_load, ax_fan):
        plt.setp(ax.get_xticklabels(), visible=False)
    draw_marks(marks, ax_temp, (ax_temp, ax_load, ax_fan), 0.0, t_end)

    save_figure(fig, png_path)
    if show:
        plt.show()
    plt.close(fig)


# =============================================================================
# FIGURE 2 - MECHANISM
# =============================================================================
def make_mechanism_plot(data, fields, events, png_path, show, title_suffix="",
                        marks=None):
    apply_style()
    t = col(data, fields, "time_s")
    t_end = float(t[-1])
    temp_ema = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    setp = col(data, fields, "setpoint")
    fan = col(data, fields, "fan_duty")
    heater = col(data, fields, "heater_duty")
    request = col(data, fields, "heater_req")
    intervals = state_intervals(events, t_end)
    cycles = throttle_cycles(intervals)
    if not cycles:
        print("[mechanism] no THROTTLE interval in this run - figure skipped "
              "(need an over-load or degraded-cooling episode)", file=sys.stderr)
        return

    complete = [c for c in cycles if c[3] == 'PID'] or cycles

    def mark_score(c):
        return sum(1 for (tm, _, _) in (marks or []) if c[0] - 90.0 <= tm <= c[1] + 90.0)

    t_in, t_out, _, next_st = max(complete, key=lambda c: (mark_score(c), c[1] - c[0]))

    pre = min(90.0, max(30.0, t_in * 0.9))
    post = min(90.0, max(0.0, t_end - t_out))
    w = (t >= t_in - pre) & (t <= t_out + post)
    if w.sum() < 2:
        print("[mechanism] throttle cycle falls outside the logged samples - "
              "figure skipped (log and events may be from different runs)",
              file=sys.stderr)
        return
    tw = t[w]
    t_lo, t_hi = float(tw[0]), float(tw[-1])
    if t_hi <= t_lo:
        print("[mechanism] degenerate time window - figure skipped", file=sys.stderr)
        return
    dv = handover_delta(t, heater, t_in)

    fig = plt.figure(figsize=(13.5, 7.8), facecolor=C_BG)
    gs = fig.add_gridspec(4, 1, height_ratios=[0.55, 1.9, 1.5, 1.7], hspace=0.34,
                          left=0.07, right=0.97, top=0.95, bottom=0.07)
    ax_title = fig.add_subplot(gs[0])
    ax_t = fig.add_subplot(gs[1])
    ax_f = fig.add_subplot(gs[2], sharex=ax_t)
    ax_h = fig.add_subplot(gs[3], sharex=ax_t)
    draw_title(ax_title, "Throttle Mechanism - one cycle, step by step",
               "The controller reacts to the *effect* (no cooling headroom), never the "
               "cause - fan degraded, blocked, or load too high.", title_suffix)

    for ax in (ax_t, ax_f, ax_h):
        span = clamp_span(t_in, t_out, t_lo, t_hi)
        if span:
            ax.axvspan(span[0], span[1], color=C_THROTTLE, alpha=0.13, zorder=0)
        span = clamp_span(t_in - THROTTLE_ENGAGE_N, t_in, t_lo, t_hi)
        if span:
            ax.axvspan(span[0], span[1], color=C_THROTTLE, alpha=0.28, zorder=0)
        if t_lo <= t_in <= t_hi:
            ax.axvline(t_in, color=C_BADGE_ED, lw=1.2)
        if t_lo <= t_out <= t_hi:
            ax.axvline(t_out, color=C_PID, lw=1.2, ls='-.')
        ax.set_xlim(t_lo, t_hi)
        ax.grid(True, axis='y')

    if temp_ema is not None:
        ax_t.plot(tw, temp_ema[w], lw=1.8, color=C_TEMP_EMA, label='temp (EMA)')
        spw0 = setp[w].copy() if setp is not None else None
        if spw0 is not None:
            spw0[spw0 <= 0] = np.nan
        z = zoom_ylim([temp_ema[w], spw0], pad_frac=0.25, min_span=2.5,
                      max_stretch=2.0)
        if z:
            lo, hi, _, _ = z
            ax_t.set_ylim(lo, hi + 0.40 * (hi - lo))
            clip_markers(ax_t, tw, temp_ema[w], lo, hi, C_TEMP_EMA,
                         label_hi='peak {peak:.1f} °C')
    if setp is not None:
        spw = setp[w].copy()
        spw[spw <= 0] = np.nan
        ax_t.step(tw, spw, where='post', lw=1.2, ls='--', color=C_SETPOINT,
                  label='setpoint')
        ax_t.fill_between(tw, spw, spw + THROTTLE_MARGIN_C, color=C_FAULT, alpha=0.10,
                          label=f'+{THROTTLE_MARGIN_C:.1f}°C engage margin')
    ax_t.set_ylabel("Temp (°C)")
    ax_t.legend(loc='upper right', ncol=3, fontsize=8)

    if fan is not None:
        ax_f.plot(tw, fan[w], lw=1.6, color=C_FAN)
        ax_f.axhline(THROTTLE_FAN_SAT, color=C_FAULT, lw=0.8, ls=':')
    ax_f.set_ylim(-0.05, 1.12)
    ax_f.set_ylabel("Fan [0..1]")

    if heater is not None:
        ax_h.fill_between(tw, 0, heater[w], color=C_HEATER, alpha=0.40)
        ax_h.plot(tw, heater[w], lw=1.2, color=C_HEATER, label='granted load')
    if request is not None and heater is not None:
        rw = request[w]
        ax_h.plot(tw, rw, lw=1.4, ls=':', color=C_REQUEST, label='user request')
        denied = (np.isfinite(rw) & (rw - heater[w] > 0.01)
                  & throttle_mask(tw, intervals))
        ax_h.fill_between(tw, heater[w], rw, where=denied, facecolor='none',
                          edgecolor=C_DENIED, hatch='///', lw=0.0, alpha=0.85,
                          label='denied')
        duty_vals = np.concatenate((heater[w], rw[np.isfinite(rw)]))
        if len(duty_vals):
            dlo, dhi = float(duty_vals.min()), float(duty_vals.max())
            pad = max(0.15 * (dhi - dlo), 0.02)
            ax_h.set_ylim(max(-0.02, dlo - pad), dhi + pad * 2.5)

    ax_h.set_ylabel("Load (zoomed)")
    ax_h.set_xlabel("Time (s)")
    ax_h.legend(loc='upper right', ncol=3, fontsize=8)

    def callout(ax, num, text, xy, xytext_frac, color=C_BADGE_ED):
        ax.annotate(f"{num}  {text}", xy=xy, xycoords='data', xytext=xytext_frac,
                    textcoords='axes fraction', ha='center', va='top', fontsize=7.8,
                    color=C_INK, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc=C_BADGE_BG, ec=color,
                              lw=1.0, alpha=0.97),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.2,
                                    shrinkA=2, shrinkB=3), zorder=12)

    def interp(arr, tv):
        if arr is None:
            return 0.0
        aw, m = arr[w], np.isfinite(arr[w])
        return float(np.interp(tv, tw[m], aw[m])) if m.any() else 0.0

    def xf(tv):
        return (tv - t_lo) / (t_hi - t_lo)

    if fan is not None:
        sat_mask = (fan[w] >= 0.999) & (tw <= t_in)
        t_sat = float(tw[sat_mask][0]) if sat_mask.any() else t_in - 2.0
        callout(ax_f, "1", "fan hits 100% - cooling authority\nexhausted, whatever the cause",
                (t_sat, 1.0), (max(0.13, xf(t_sat) - 0.13), 0.55), color=C_FAN)
    callout(ax_t, "2", "temp stuck above\nsetpoint + margin",
            (t_in - 2.0, interp(temp_ema, t_in - 2.0)),
            (max(0.10, xf(t_in) - 0.22), 0.30), color=C_FAULT)
    callout(ax_h, "3",
            f"handover to throttle PID\nstep = {dv:+.3f} duty" if dv is not None else "handover",
            (t_in + 1.0, interp(heater, t_in + 1.0)),
            (min(0.85, xf(t_in) + 0.14), 0.97))
    t_mid = (t_in + t_out) / 2.0
    callout(ax_h, "4", "user request DENIED (hatched);\nPID tracks sustainable max",
            (t_mid, interp(heater, t_mid)), (xf(t_mid), 0.30), color=C_DENIED)

    # Callout 5 only exists when a recovery mark is present (physical fan-move test).
    rec = [(tm, lbl) for (tm, lbl, _) in (marks or []) if t_in < tm < t_out]
    if rec:
        t_rec = rec[-1][0]
        callout(ax_t, "5", "cooling recovers ->\ntemp falls toward setpoint",
                (t_rec + 5.0, interp(temp_ema, t_rec + 5.0)),
                (min(0.88, xf(t_rec) + 0.13), 0.92), color=C_PID)

    if next_st == 'PID':
        callout(ax_h, "6" if rec else "5",
                "throttle output climbs to request ->\nexit, user regains control",
                (t_out, interp(heater, min(t_out + 2.0, t_hi))),
                (min(0.88, xf(t_out) + 0.10), 0.62), color=C_PID)

    draw_marks(marks, ax_t, (ax_t, ax_f, ax_h), t_lo, t_hi)
    save_figure(fig, png_path)
    if show:
        plt.show()
    plt.close(fig)


# =============================================================================
# FIGURE 3 - DETAIL (FSM + dynamic fault panels)
# =============================================================================
def draw_zoom_panel(ax, t, temp_raw, temp_ema, vnode, fan_rpm, t_trip, t_clear,
                    reason_name, idx):
    margin = max(30.0, (t_clear - t_trip) * 1.5)
    t_lo, t_hi = max(0.0, t_trip - margin), t_clear + margin
    m = (t >= t_lo) & (t <= t_hi)
    if not m.any():
        ax.axis('off')
        return
    tl = t[m]
    ax2 = ax.twinx()

    if temp_raw is not None:
        ax.plot(tl, temp_raw[m], lw=0.9, color=C_TEMP_RAW, label='temp (raw)', alpha=0.7)
    if temp_ema is not None:
        ax.plot(tl, temp_ema[m], lw=1.6, color=C_TEMP_EMA, label='temp_ema')

    # Dynamic secondary axis: FAN fault -> RPM ; NTC fault -> reconstructed vnode.
    if 'FAN' in reason_name and fan_rpm is not None:
        ax2.plot(tl, fan_rpm[m], lw=1.5, color=C_FAN, ls='--', label='fan (RPM)')
        ax2.axhline(FAN_STALL_RPM, color=C_FAULT, lw=0.8, ls=':', label='Stall Threshold')
        ax2.set_ylabel("Speed (RPM)", color=C_FAN, fontsize=9)
        ax2.tick_params(axis='y', labelcolor=C_FAN, labelsize=8)
    elif vnode is not None:
        # SENSOR-CHAIN v2: vnode is RECONSTRUCTED from differential counts.
        # Open saturates ~0.65 V (PGA rail), NOT 5 V as in the single-ended era.
        ax2.plot(tl, vnode[m], lw=1.5, color=C_VNODE, ls='--',
                 label='vnode (reconstructed)')
        ax2.set_ylim(*VNODE_YLIM)
        ax2.axhline(VNODE_OPEN_V, color=C_FAULT, lw=0.8, ls=':',
                    label=f'Open thresh (+400 cnt = {VNODE_OPEN_V:.3f} V)')
        ax2.axhline(VNODE_SHORT_V, color=C_FAULT, lw=0.8, ls=':',
                    label=f'Short thresh (-140 cnt = {VNODE_SHORT_V:.3f} V)')
        ax2.set_ylabel("vnode (V, from counts)", color=C_VNODE, fontsize=9)
        ax2.tick_params(axis='y', labelcolor=C_VNODE, labelsize=8)

    ax.axvspan(t_trip, t_clear, color=C_FAULT, alpha=0.15, zorder=0)
    if reason_name == 'NODE_OFFLINE':
        ax.text((t_trip + t_clear) / 2, 0.06, 'no telemetry', transform=None if False else
                ax.get_xaxis_transform(), ha='center', va='bottom', fontsize=7.5,
                color=C_FAULT, style='italic')
    ax.axvline(t_trip, color=C_FAULT, lw=1.2)
    ax.axvline(t_clear, color=C_PID, lw=1.2, ls='-.')

    z = zoom_ylim([temp_raw[m] if temp_raw is not None else None,
                   temp_ema[m] if temp_ema is not None else None],
                  pad_frac=0.22, min_span=4.0, max_stretch=2.5)
    if z:
        ax.set_ylim(z[0], z[1])
    trig = {'NTC_OPEN': 'NTC unplugged', 'NTC_SHORT': 'NTC shorted',
            'FAN_OPEN': 'Fan stalled',
            'NODE_OFFLINE': 'Node offline (I2C)'}.get(reason_name, reason_name)
    ylo, ymax = ax.get_ylim()
    yspan = ymax - ylo
    y_hi = ylo + 0.92 * yspan
    y_md = ylo + 0.55 * yspan
    ax.annotate(f"TRIP\n{trig}", xy=(t_trip, y_hi),
                xytext=(t_trip - margin * 0.55, y_hi), fontsize=8,
                color=C_FAULT, fontweight='bold', ha='left', va='top',
                bbox=dict(boxstyle='round,pad=0.3', fc='#fff5f5', ec=C_FAULT, lw=0.7),
                arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=0.7))
    ax.annotate("Recover", xy=(t_clear, y_md),
                xytext=(t_clear + margin * 0.20, y_md), fontsize=8,
                color=C_PID, fontweight='bold', ha='left', va='center',
                bbox=dict(boxstyle='round,pad=0.3', fc='#f0fdf4', ec=C_PID, lw=0.7),
                arrowprops=dict(arrowstyle='->', color=C_PID, lw=0.7))

    ax.set_title(f"Detail [{idx}]  t={t_trip:.0f}s  duration={t_clear - t_trip:.0f}s",
                 fontsize=10, fontweight='bold', color=C_INK, loc='left')
    ax.set_ylabel("Temp (°C)", fontsize=10, color=C_TEMP_EMA)
    ax.set_xlabel("Time (s)", fontsize=9, color=C_INK_SOFT)
    ax.tick_params(axis='both', labelsize=8)
    ax.tick_params(axis='y', labelcolor=C_TEMP_EMA)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(t_lo, t_hi)

    l1, la1 = ax.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    if l1 or l2:
        ax.legend(l1 + l2, la1 + la2, loc='center left', fontsize=7.5,
                  framealpha=0.92, frameon=True, edgecolor=C_INK_SOFT)
    ax.text(0.02, 0.97, str(idx), transform=ax.transAxes, ha='left', va='top',
            fontsize=12, fontweight='bold', color=C_BADGE_ED,
            bbox=dict(boxstyle='circle,pad=0.4', fc=C_BADGE_BG, ec=C_BADGE_ED, lw=1.2))


FSM_NODES = {'IDLE': (1.6, 7.8), 'PID': (5.0, 7.8), 'THROTTLE': (8.4, 7.8),
             'FAULT': (3.3, 2.9), 'COOLING': (6.7, 2.9)}
FSM_EDGES = [
    ('IDLE', 'PID', 'start', (3.3, 8.3), 0.0),
    ('PID', 'THROTTLE', 'fan maxed', (6.7, 8.75), 0.3),
    ('THROTTLE', 'PID', 'sustainable', (6.7, 6.9), 0.3),
    ('PID', 'FAULT', 'trip', (3.6, 5.6), 0.1),
    ('THROTTLE', 'FAULT', 'trip', (5.6, 4.6), -0.3),
    ('FAULT', 'COOLING', 'recover', (5.0, 2.45), 0.2),
    ('PID', 'COOLING', 'stop', (6.2, 5.6), -0.1),
    ('THROTTLE', 'COOLING', 'stop', (8.3, 5.2), -0.2),
    ('COOLING', 'IDLE', 'cool', (1.35, 5.2), -0.3),
]


def draw_fsm(ax, events=None, counts=True):
    visits = count_state_visits(events) if counts else {}
    trans = count_transitions(events) if counts else {}
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.axis('off')
    if counts:
        ax.text(5.0, 9.8, "FSM (per-zone)", ha='center', va='top', fontsize=13,
                fontweight='bold', color=C_INK)
        ax.text(5.0, 9.25, "counts from this run", ha='center', va='top',
                fontsize=8.5, color=C_INK_SOFT, style='italic')
    bw, bh = 2.25, 1.1
    for name, (x, y) in FSM_NODES.items():
        st = NAME_TO_ID[name]
        n = visits.get(st, 0)
        active = (not counts) or n > 0
        ax.add_patch(FancyBboxPatch((x - bw / 2, y - bh / 2), bw, bh,
                                    boxstyle="round,pad=0.05,rounding_size=0.12",
                                    facecolor=STATE_COLORS[st], edgecolor=C_INK,
                                    linewidth=1.6 if (counts and n) else 1.0,
                                    alpha=0.95 if active else 0.40, zorder=3))
        fs = 10 if len(name) <= 6 else 8.5
        dy = 0.16 if (counts and n) else 0.0
        ax.text(x, y + dy, name, ha='center', va='center', fontsize=fs,
                fontweight='bold', color='white', zorder=4)
        if counts and n:
            ax.text(x, y - 0.28, f"x{n}", ha='center', va='center', fontsize=8.5,
                    color='white', zorder=4)
    for src, dst, label, (lx, ly), rad in FSM_EDGES:
        x0, y0 = FSM_NODES[src]
        x1, y1 = FSM_NODES[dst]
        n = trans.get((NAME_TO_ID[src], NAME_TO_ID[dst]), 0)
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1),
                                     connectionstyle=f"arc3,rad={rad}",
                                     arrowstyle='-|>', mutation_scale=14,
                                     color=C_INK if (counts and n)
                                     else (C_INK_SOFT if not counts else '#cccccc'),
                                     linewidth=2.0 if (counts and n) else 1.0,
                                     shrinkA=25, shrinkB=25, zorder=2))
        lab = label if not (counts and n) else f"{label} x{n}"
        ax.text(lx, ly, lab, ha='center', va='center', fontsize=7.6,
                color=C_INK if (counts and n) else C_INK_SOFT, style='italic',
                bbox=dict(boxstyle='round,pad=0.18', fc='white', ec='none', alpha=0.93),
                zorder=5)


def make_detail_plot(data, fields, events, png_path, show, title_suffix=""):
    apply_style()
    t = col(data, fields, "time_s")
    temp_raw = mask_invalid_temps(col(data, fields, "temp_c"), t, events)
    temp_ema = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    vnode = col(data, fields, "vnode")
    fan_rpm = col(data, fields, "fan_rpm")
    fwins = fault_windows(events)
    # NODE_OFFLINE means no telemetry at all - plotting the last-held or
    # garbage values as if they were measurements is a lie. Blank them.
    off = [w for w in fwins if w[2] == 'NODE_OFFLINE']
    if off:
        offmask = np.zeros_like(t, dtype=bool)
        for (a, b, _) in off:
            offmask |= (t >= a) & (t <= b)
        if vnode is not None:
            vnode = vnode.astype(float).copy(); vnode[offmask] = np.nan
        if fan_rpm is not None:
            fan_rpm = fan_rpm.astype(float).copy(); fan_rpm[offmask] = np.nan
    n_z = min(3, len(fwins))

    fig = plt.figure(figsize=(14, 6.5), facecolor=C_BG)
    gs = fig.add_gridspec(2, 1, height_ratios=[0.45, 4.0], hspace=0.28,
                          left=0.06, right=0.94, top=0.94, bottom=0.07)
    draw_title(fig.add_subplot(gs[0]), "STM32 Chip-Cooling Controller - Detail",
               "FSM with live counts (left) / dynamic per-fault zoom panels (right)",
               title_suffix)

    if n_z == 0:
        draw_fsm(fig.add_subplot(gs[1]), events, counts=True)
    else:
        gsb = gs[1].subgridspec(1, 2, width_ratios=[1.0, 1.4], wspace=0.18)
        draw_fsm(fig.add_subplot(gsb[0]), events, counts=True)
        gz = gsb[1].subgridspec(n_z, 1, hspace=0.65)
        for i in range(n_z):
            t0, t1, r = fwins[i]
            draw_zoom_panel(fig.add_subplot(gz[i]), t, temp_raw, temp_ema, vnode,
                            fan_rpm, t0, t1, r, idx=i + 1)

    if len(fwins) > 3:
        fig.text(0.98, 0.01, f"Showing first 3 of {len(fwins)} fault events",
                 fontsize=8, color=C_INK_SOFT, ha='right')
    save_figure(fig, png_path)
    if show:
        plt.show()
    plt.close(fig)


# =============================================================================
# TERMINAL REPORT
# =============================================================================
def print_report(data, fields, events):
    t = col(data, fields, "time_s")
    heater = col(data, fields, "heater_duty")
    t_end = float(t[-1])
    intervals = state_intervals(events, t_end)
    print("=" * 72)
    print(f"  RUN REPORT   rows={len(data)}  span={t_end - t[0]:.0f}s")
    print("=" * 72)
    for name in fields:
        c = col(data, fields, name)
        print(f"  {name:<12} min={c.min():>9.2f}  max={c.max():>9.2f}  "
              f"mean={c.mean():>9.2f}")

    # Sanity guard for the differential-era reconstruction:
    v = col(data, fields, "vnode")
    if v is not None and np.nanmax(v) > 1.0:
        print("\n  [!] vnode max > 1.0 V - this looks like SINGLE-ENDED-era data")
        print("      or a dtype misalignment. Verify sizeof(ThermalLogEntry)==36")
        print("      and that the firmware logging the run had the v2 sensor chain.")

    if events is not None and len(events):
        ev_hi = float(events['time_s'].max())
        if ev_hi > t_end + 1.0:
            print(f"\n  [!] events reach {ev_hi:.1f}s but the log ends at {t_end:.1f}s")
            print("      - the two dumps are probably from different runs.")

    cycles = throttle_cycles(intervals)
    if cycles and heater is not None:
        print("\n  THROTTLE CYCLES")
        for (t0, t1, prev, nxt) in cycles:
            dv = handover_delta(t, heater, t0)
            print(f"    enter t={t0:7.1f}  exit t={t1:7.1f}  dwell={t1 - t0:6.1f}s"
                  f"  handover step=" + (f"{dv:+.4f}" if dv is not None else "  n/a"))
        print("    (handover step ~ 0 => bumpless transfer verified)")
    print("=" * 72)


# =============================================================================
# MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="Render STM32 thermal-controller black-box dumps into "
                    "annotated reports.")
    ap.add_argument("infile", help="thermal_log dump: RAW binary or Expressions text")
    ap.add_argument("--events", help="thermal_events dump: RAW binary or Expressions text")
    ap.add_argument("-o", "--out-prefix", default=None)
    ap.add_argument("--marks", help="file: time,label[,color] per line (optional)")
    ap.add_argument("--no-show", action="store_true",
                    help="write files without opening interactive windows")
    ap.add_argument("--no-plot", action="store_true",
                    help="alias of --no-show (kept for older invocations)")
    ap.add_argument("--csv-only", action="store_true",
                    help="export CSV and the terminal report, skip all figures")
    ap.add_argument("--csv", action="store_true",
                    help="also export <stem>_log.csv / <stem>_log_events.csv "
                         "(off by default: scripts/run1.csv is a different, "
                         "tracked file written by the older toolchain)")
    args = ap.parse_args()

    fields, data = load_log(args.infile)

    if "time_s" in fields:
        ti = fields.index("time_s")
        data = data[data[:, ti] > 0.0]          # drop never-written ring slots
        data = data[np.argsort(data[:, ti])]    # unwrap ring by timestamp

    if len(data) == 0:
        print("[error] no populated log entries - every time_s was zero. "
              "Was the ring ever written, or did the export start at the wrong "
              "address?", file=sys.stderr)
        return 1

    t_end = float(data[:, fields.index("time_s")][-1])

    events = load_events(args.events)
    events = validate_events(events, t_end, args.events or "<events>")
    marks = load_marks(args.marks)

    print_report(data, fields, events)

    stem = args.out_prefix or os.path.splitext(args.infile)[0]

    if args.csv or args.csv_only:
        export_csv(data, fields, stem + "_log.csv")
        export_events_csv(events, stem + "_log_events.csv")

    if args.csv_only:
        return 0

    show = not (args.no_show or args.no_plot)
    make_overview_plot(data, fields, events, stem + "_overview.png", show, "",
                       marks, None)
    make_mechanism_plot(data, fields, events, stem + "_mechanism.png", show, "", marks)
    make_detail_plot(data, fields, events, stem + "_detail.png", show, "")
    return 0


if __name__ == "__main__":
    sys.exit(main())
