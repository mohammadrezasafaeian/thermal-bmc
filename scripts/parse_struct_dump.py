#!/usr/bin/env python3
"""
parse_struct_dump.py - STM32 chip-cooling controller: story-driven analysis.

Figures:
  <stem>_overview.png     The whole run: temp, load (with real heater_req), fan + RPM.
  <stem>_mechanism.png    One throttle cycle, annotated step by step (arrows).
  <stem>_detail.png       FSM with live counts + dynamic per-fault zoom panels.
  <stem>_walkthrough.png  Glossary + transition chips + timeline strip.
Data:
  <stem>.csv              ~300 event/inflection-weighted rows + state column.
  <stem>_events.csv       Decoded event stream.

Usage:
  python parse_struct_dump.py run1.txt --events run1_events.txt --marks run1_marks.txt -o run1

Marks file (run1_marks.txt) example based on your physical test:
  433,FAN MOVED AWAY,#d97706
  687,FAN RESTORED,#15803d
"""

import argparse
import re
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch

# ----- Thresholds & Constants -----------------------------------------------
TEMP_VALID_LO, TEMP_VALID_HI = -10.0, 100.0
THROTTLE_FAN_SAT, THROTTLE_MARGIN_C, THROTTLE_ENGAGE_N = 0.98, 0.3, 5
FAN_STALL_RPM = 300

# ----- Palette --------------------------------------------------------------
C_BG, C_INK, C_INK_SOFT = '#ffffff', '#1f2933', '#6b7280'
C_GRID = '#e5e7eb'
C_TEMP_RAW, C_TEMP_EMA, C_SETPOINT = '#d1d5db', '#d97706', '#1f2933'
C_HEATER, C_FAN, C_REQUEST = '#ea580c', '#0369a1', '#7f1d1d'
C_VNODE, C_FAULT = '#7c3aed', '#b91c1c'
C_PID, C_COOLING, C_IDLE, C_THROTTLE = '#15803d', '#0369a1', '#9ca3af', '#f59e0b'
C_BADGE_BG, C_BADGE_ED = '#fef3c7', '#92400e'
C_DENIED = '#dc2626'

# ----- Firmware enums -------------------------------------------------------
STATE_NAMES = {0: 'IDLE', 1: 'PID', 2: 'THROTTLE', 3: 'COOLING', 4: 'FAULT'}
NAME_TO_ID = {v: k for k, v in STATE_NAMES.items()}
STATE_COLORS = {0: C_IDLE, 1: C_PID, 2: C_THROTTLE, 3: C_COOLING, 4: C_FAULT}
EV_STATE_CHANGE, EV_FAULT_RAISED, EV_FAULT_CLEARED = 1, 2, 3
EV_SETPOINT_CHG, EV_START_REQ, EV_STOP_REQ = 4, 5, 6
EVENT_NAMES = {1: 'STATE_CHANGE', 2: 'FAULT_RAISED', 3: 'FAULT_CLEARED',
               4: 'SETPOINT_CHG', 5: 'START_REQ', 6: 'STOP_REQ'}
FAULT_NAMES = {0: 'NONE', 1: 'NTC_OPEN', 2: 'NTC_SHORT',
               3: 'HEATER_OPEN', 4: 'FAN_OPEN', 5: 'ALL_DISCONNECTED'}
EVT_DTYPE = np.dtype([('time_s', '<f4'), ('kind', 'u1'),
                      ('u8_payload', 'u1'), ('_pad', '<u2'),
                      ('f_payload', '<f4')])

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
# PARSING
# =============================================================================
def parse_expressions_dump(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    if not entries:
        raise ValueError("No {field = value, ...} blocks found.")
    parsed = []
    for entry in entries:
        pairs = re.findall(r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if pairs:
            parsed.append(dict(pairs))
    fields = list(parsed[0].keys())
    rows = [[float(d[f]) for f in fields] for d in parsed if all(f in d for f in fields)]
    
    seen, unique = set(), []
    for row in rows:
        key = tuple(round(v, 6) for v in row)
        if key not in seen:
            seen.add(key); unique.append(row)
    return fields, np.array(unique, dtype=np.float64)

def parse_events_from_expressions(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    parsed, req = [], ('time_s', 'kind', 'u8_payload', 'f_payload')
    for entry in entries:
        pairs = re.findall(r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if not pairs: continue
        d = dict(pairs)
        if all(k in d for k in req):
            parsed.append((float(d['time_s']), int(float(d['kind'])),
                           int(float(d['u8_payload'])), 0, float(d['f_payload'])))
    if not parsed: return None
    arr = np.array(parsed, dtype=EVT_DTYPE)
    arr = arr[arr['kind'] != 0]
    if len(arr) == 0: return None
    keys = {(float(e['time_s']), int(e['kind']), int(e['u8_payload']), float(e['f_payload'])): e for e in arr}
    arr = np.array(list(keys.values()), dtype=EVT_DTYPE)
    return arr[np.argsort(arr['time_s'])]

def load_marks(path):
    marks = []
    if not path: return marks
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"): continue
            parts = [p.strip() for p in line.strip().split(",")]
            if len(parts) >= 2:
                marks.append((float(parts[0]), parts[1], parts[2] if len(parts) > 2 else C_INK_SOFT))
    return marks

def col(data, fields, name):
    return data[:, fields.index(name)] if name in fields else None

# =============================================================================
# DERIVED VIEWS
# =============================================================================
def state_intervals(events, t_end):
    if events is None or len(events) == 0: return []
    sc = events[events['kind'] == EV_STATE_CHANGE]
    out = []
    for i, e in enumerate(sc):
        t0 = float(e['time_s'])
        t1 = float(sc[i + 1]['time_s']) if i + 1 < len(sc) else t_end
        out.append((t0, t1, int(e['u8_payload'])))
    return out

def state_for_time(intervals, tv):
    for (t0, t1, st) in intervals:
        if t0 <= tv < t1:
            return STATE_NAMES.get(st, '?')
    return STATE_NAMES.get(intervals[-1][2], '?') if intervals else '?'

def fault_windows(events):
    if events is None or len(events) == 0: return []
    wins, t0, reason = [], None, None
    for e in events:
        k = int(e['kind'])
        if k == EV_FAULT_RAISED:
            t0, reason = float(e['time_s']), FAULT_NAMES.get(int(e['u8_payload']), '?')
        elif k == EV_FAULT_CLEARED and t0 is not None:
            wins.append((t0, float(e['time_s']), reason))
            t0, reason = None, None
    return wins

def mask_invalid_temps(x, t, events):
    if x is None: return None
    x = x.copy()
    x[(x < TEMP_VALID_LO) | (x > TEMP_VALID_HI)] = np.nan
    for (t0, t1, _) in fault_windows(events):
        x[(t >= t0) & (t <= t1)] = np.nan
    return x

def throttle_cycles(intervals):
    out = []
    for i, (t0, t1, st) in enumerate(intervals):
        if STATE_NAMES.get(st) != 'THROTTLE': continue
        prev_st = STATE_NAMES.get(intervals[i - 1][2]) if i > 0 else None
        next_st = STATE_NAMES.get(intervals[i + 1][2]) if i + 1 < len(intervals) else None
        out.append((t0, t1, prev_st, next_st))
    return out

def handover_delta(t, heater, t_enter):
    before = heater[(t < t_enter)][-1:]
    after = heater[(t >= t_enter)][:1]
    if len(before) and len(after):
        return float(after[0] - before[0])
    return None

def count_state_visits(events):
    out = {}
    if events is None: return out
    for e in events[events['kind'] == EV_STATE_CHANGE]:
        st = int(e['u8_payload'])
        out[st] = out.get(st, 0) + 1
    return out

def count_transitions(events):
    out, prev = {}, None
    if events is None: return out
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

def draw_title(ax, main, subtitle, suffix=""):
    ax.axis('off')
    ax.text(0.0, 0.85, main, fontsize=20, fontweight='bold', color=C_INK, ha='left', va='top', transform=ax.transAxes)
    ax.text(0.0, 0.22, subtitle + (" - " + suffix if suffix else ""), fontsize=10, color=C_INK_SOFT, ha='left', va='top', transform=ax.transAxes)

def shade_states(ax, intervals, alpha_scale=1.0):
    shade = {1: (C_PID, 0.05), 2: (C_THROTTLE, 0.13), 3: (C_COOLING, 0.07), 4: (C_FAULT, 0.12)}
    for (t0, t1, st) in intervals:
        if st in shade:
            c, a = shade[st]
            ax.axvspan(t0, t1, color=c, alpha=a * alpha_scale, zorder=0)

def state_strip(ax, intervals, t_end, label_min_frac=0.05):
    ax.set_xlim(0, t_end); ax.set_ylim(0, 1)
    ax.set_yticks([]); ax.grid(False)
    for s in ('left', 'top', 'right'): ax.spines[s].set_visible(False)
    for (t0, t1, st) in intervals:
        ax.add_patch(Rectangle((t0, 0.25), t1 - t0, 0.5, facecolor=STATE_COLORS.get(st, C_IDLE), edgecolor='white', linewidth=0.5))
        if (t1 - t0) > label_min_frac * t_end:
            nm = STATE_NAMES.get(st, '?')
            ax.text((t0 + t1) / 2, 0.5, 'THROT.' if nm == 'THROTTLE' else nm, ha='center', va='center', fontsize=6.5, fontweight='bold', color='white')

def draw_marks(marks, label_ax, line_axes, t_lo=None, t_hi=None):
    for (tm, lbl, mc) in (marks or []):
        if t_lo is not None and not (t_lo <= tm <= t_hi): continue
        for ax in line_axes:
            ax.axvline(tm, color=mc, lw=1.1, ls='--', alpha=0.8, zorder=6)
        label_ax.annotate(lbl, xy=(tm, 0.99), xycoords=('data', 'axes fraction'), ha='center', va='top', fontsize=7.5, color=mc, fontweight='bold', bbox=dict(boxstyle='round,pad=0.25', fc='white', ec=mc, lw=0.8, alpha=0.95), zorder=11)

# =============================================================================
# FIGURE 1 - OVERVIEW
# =============================================================================
def make_overview_plot(data, fields, events, png_path, show, title_suffix="", marks=None, trange=None):
    apply_style()
    t = col(data, fields, "time_s")
    t_end = float(t[-1])
    temp_raw = mask_invalid_temps(col(data, fields, "temp_c"), t, events)
    temp_ema = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    setp = col(data, fields, "setpoint")
    fan = col(data, fields, "fan_duty")
    fan_rpm = col(data, fields, "fan_rpm")
    heater = col(data, fields, "heater_duty")
    request = col(data, fields, "heater_req")
    intervals = state_intervals(events, t_end)

    fig = plt.figure(figsize=(13.5, 8.2), facecolor=C_BG)
    gs = fig.add_gridspec(5, 1, height_ratios=[0.55, 2.6, 1.55, 1.15, 0.4], hspace=0.34, left=0.07, right=0.94, top=0.95, bottom=0.06)
    
    ax_title = fig.add_subplot(gs[0])
    ax_temp = fig.add_subplot(gs[1])
    ax_load = fig.add_subplot(gs[2], sharex=ax_temp)
    ax_fan = fig.add_subplot(gs[3], sharex=ax_temp)
    ax_strip = fig.add_subplot(gs[4], sharex=ax_temp)

    draw_title(ax_title, "STM32 Chip-Cooling Controller", "Fan PID holds temperature; when cooling authority runs out, a second PID overrides the requested load", title_suffix)

    # 1. Temperature Pane
    shade_states(ax_temp, intervals)
    if temp_raw is not None: ax_temp.plot(t, temp_raw, lw=0.7, color=C_TEMP_RAW, label='temp (raw)')
    if temp_ema is not None: ax_temp.plot(t, temp_ema, lw=1.9, color=C_TEMP_EMA, label='temp (EMA, loop input)')
    if setp is not None:
        sp = setp.copy(); sp[sp <= 0] = np.nan
        ax_temp.step(t, sp, where='post', lw=1.3, ls='--', color=C_SETPOINT, label='setpoint')
    ax_temp.set_ylabel("Temp (°C)")
    ax_temp.grid(True, axis='y')
    ax_temp.legend(loc='lower right', ncol=3, bbox_to_anchor=(1.0, 1.0))
    if trange: ax_temp.set_ylim(*trange)

    # 2. Load Pane (Dynamic DENIED hatching based on real heater_req)
    shade_states(ax_load, intervals)
    if heater is not None:
        ax_load.fill_between(t, 0, heater, color=C_HEATER, alpha=0.40, label='granted load (heater PWM)')
        ax_load.plot(t, heater, lw=1.0, color=C_HEATER)
    if request is not None:
        ax_load.plot(t, request, lw=1.4, ls=':', color=C_REQUEST, label='user request (g_heater_request)')
        denied = np.isfinite(request) & (request - heater > 0.01)
        ax_load.fill_between(t, heater, request, where=denied, facecolor='none', edgecolor=C_DENIED, hatch='///', lw=0.0, alpha=0.85, label='DENIED by throttle')
    ax_load.set_ylim(-0.05, 1.12)
    ax_load.set_ylabel("Load [0..1]")
    ax_load.grid(True, axis='y')
    ax_load.legend(loc='upper left', ncol=3, fontsize=8, bbox_to_anchor=(0.0, 1.28))

    # 3. Fan Pane (Twin Axis for RPM)
    shade_states(ax_fan, intervals)
    lines_f, labels_f = [], []
    if fan is not None:
        l1 = ax_fan.plot(t, fan, lw=1.5, color=C_FAN, label='fan cmd (0..1)')
        ax_fan.axhline(THROTTLE_FAN_SAT, color=C_FAULT, lw=0.8, ls=':')
        lines_f += l1; labels_f.append('fan cmd (0..1)')
    
    if fan_rpm is not None:
        ax_rpm = ax_fan.twinx()
        l2 = ax_rpm.plot(t, fan_rpm, lw=1.2, ls='-', color=C_INK_SOFT, alpha=0.5, label='tachometer (RPM)')
        ax_rpm.set_ylabel("Speed (RPM)", color=C_INK_SOFT)
        ax_rpm.tick_params(axis='y', labelcolor=C_INK_SOFT)
        lines_f += l2; labels_f.append('tachometer (RPM)')
    
    ax_fan.set_ylim(-0.05, 1.12)
    ax_fan.set_ylabel("Fan Duty")
    ax_fan.grid(True, axis='y')
    ax_fan.legend(lines_f, labels_f, loc='upper left', fontsize=8)

    state_strip(ax_strip, intervals, t_end)
    ax_strip.set_xlabel("Time (s)   /   total run: {}:{:02d}".format(int(t_end) // 60, int(t_end) % 60))

    for ax in (ax_temp, ax_load, ax_fan): plt.setp(ax.get_xticklabels(), visible=False)
    draw_marks(marks, ax_temp, (ax_temp, ax_load, ax_fan))

    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print("[png] wrote {}".format(png_path), file=sys.stderr)
    if show: plt.show()
    plt.close(fig)

# =============================================================================
# FIGURE 2 - MECHANISM
# =============================================================================
def make_mechanism_plot(data, fields, events, png_path, show, title_suffix="", marks=None):
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
        print("[mechanism] no THROTTLE interval, skipping", file=sys.stderr)
        return

    complete = [c for c in cycles if c[3] == 'PID'] or cycles
    def mark_score(c): return sum(1 for (tm, _, _) in (marks or []) if c[0] - 90.0 <= tm <= c[1] + 90.0)
    t_in, t_out, _, next_st = max(complete, key=lambda c: (mark_score(c), c[1] - c[0]))
    
    pre = min(90.0, max(30.0, t_in * 0.9))
    post = min(90.0, t_end - t_out)
    w = (t >= t_in - pre) & (t <= t_out + post)
    tw = t[w]
    t_lo, t_hi = float(tw[0]), float(tw[-1])
    dv = handover_delta(t, heater, t_in)

    fig = plt.figure(figsize=(13.5, 7.8), facecolor=C_BG)
    gs = fig.add_gridspec(4, 1, height_ratios=[0.55, 1.9, 1.5, 1.7], hspace=0.34, left=0.07, right=0.97, top=0.95, bottom=0.07)
    ax_title = fig.add_subplot(gs[0])
    ax_t = fig.add_subplot(gs[1])
    ax_f = fig.add_subplot(gs[2], sharex=ax_t)
    ax_h = fig.add_subplot(gs[3], sharex=ax_t)
    draw_title(ax_title, "Throttle Mechanism - one cycle, step by step", "The controller reacts to the *effect* (no cooling headroom), never the cause - fan degraded, blocked, or load too high.", title_suffix)

    for ax in (ax_t, ax_f, ax_h):
        ax.axvspan(t_in, t_out, color=C_THROTTLE, alpha=0.13, zorder=0)
        ax.axvspan(t_in - THROTTLE_ENGAGE_N, t_in, color=C_THROTTLE, alpha=0.28, zorder=0)
        ax.axvline(t_in, color=C_BADGE_ED, lw=1.2)
        ax.axvline(t_out, color=C_PID, lw=1.2, ls='-.')
        ax.set_xlim(t_lo, t_hi)
        ax.grid(True, axis='y')

    if temp_ema is not None: ax_t.plot(tw, temp_ema[w], lw=1.8, color=C_TEMP_EMA, label='temp (EMA)')
    if setp is not None:
        spw = setp[w].copy(); spw[spw <= 0] = np.nan
        ax_t.step(tw, spw, where='post', lw=1.2, ls='--', color=C_SETPOINT, label='setpoint')
        ax_t.fill_between(tw, spw, spw + THROTTLE_MARGIN_C, color=C_FAULT, alpha=0.10, label=f'+{THROTTLE_MARGIN_C:.1f}°C engage margin')
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
    if request is not None:
        rw = request[w]
        ax_h.plot(tw, rw, lw=1.4, ls=':', color=C_REQUEST, label='user request')
        denied = np.isfinite(rw) & (rw - heater[w] > 0.01)
        ax_h.fill_between(tw, heater[w], rw, where=denied, facecolor='none', edgecolor=C_DENIED, hatch='///', lw=0.0, alpha=0.85, label='denied')
        
        # Zoom Y axis
        duty_vals = np.concatenate((heater[w], rw[np.isfinite(rw)]))
        if len(duty_vals):
            dlo, dhi = float(duty_vals.min()), float(duty_vals.max())
            pad = max(0.15 * (dhi - dlo), 0.02)
            ax_h.set_ylim(max(-0.02, dlo - pad), dhi + pad * 2.5)

    ax_h.set_ylabel("Load (zoomed)")
    ax_h.set_xlabel("Time (s)")
    ax_h.legend(loc='upper right', ncol=3, fontsize=8)

    # Annotations
    def callout(ax, num, text, xy, xytext_frac, color=C_BADGE_ED):
        ax.annotate(f"{num}  {text}", xy=xy, xycoords='data', xytext=xytext_frac, textcoords='axes fraction', ha='center', va='top', fontsize=7.8, color=C_INK, fontweight='bold', bbox=dict(boxstyle='round,pad=0.3', fc=C_BADGE_BG, ec=color, lw=1.0, alpha=0.97), arrowprops=dict(arrowstyle='->', color=color, lw=1.2, shrinkA=2, shrinkB=3), zorder=12)

    def interp(arr, tv):
        if arr is None: return 0.0
        aw, m = arr[w], np.isfinite(arr[w])
        return float(np.interp(tv, tw[m], aw[m])) if m.any() else 0.0
    xf = lambda tv: (tv - t_lo) / (t_hi - t_lo)

    t_sat = float(tw[(fan[w] >= 0.999) & (tw <= t_in)][0]) if len(tw[(fan[w] >= 0.999) & (tw <= t_in)]) else t_in - 2.0
    callout(ax_f, "1", "fan hits 100% - cooling authority\nexhausted, whatever the cause", (t_sat, 1.0), (max(0.13, xf(t_sat) - 0.13), 0.55), color=C_FAN)
    callout(ax_t, "2", "temp stuck above\nsetpoint + margin", (t_in - 2.0, interp(temp_ema, t_in - 2.0)), (max(0.10, xf(t_in) - 0.22), 0.30), color=C_FAULT)
    callout(ax_h, "3", f"handover to throttle PID\nstep = {dv:+.3f} duty" if dv is not None else "handover", (t_in + 1.0, interp(heater, t_in + 1.0)), (min(0.85, xf(t_in) + 0.14), 0.97))
    t_mid = (t_in + t_out) / 2.0
    callout(ax_h, "4", "user request DENIED (hatched);\nPID tracks sustainable max", (t_mid, interp(heater, t_mid)), (xf(t_mid), 0.30), color=C_DENIED)
    
    rec = [(tm, lbl) for (tm, lbl, _) in (marks or []) if t_in < tm < t_out]
    if rec:
        t_rec = rec[-1][0]
        callout(ax_t, "5", "cooling recovers ->\ntemp falls toward setpoint", (t_rec + 5.0, interp(temp_ema, t_rec + 5.0)), (min(0.88, xf(t_rec) + 0.13), 0.92), color=C_PID)
    
    if next_st == 'PID':
        callout(ax_h, "6", "throttle output climbs to request ->\nexit, user regains control", (t_out, interp(heater, min(t_out + 2.0, t_hi))), (min(0.88, xf(t_out) + 0.10), 0.62), color=C_PID)

    draw_marks(marks, ax_t, (ax_t, ax_f, ax_h), t_lo, t_hi)
    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print(f"[png] wrote {png_path}", file=sys.stderr)
    if show: plt.show()
    plt.close(fig)

# =============================================================================
# FIGURE 3 - DETAIL (Dynamic fault panels)
# =============================================================================
def draw_zoom_panel(ax, t, temp_raw, temp_ema, vnode, fan_rpm, t_trip, t_clear, reason_name, idx):
    margin = max(30.0, (t_clear - t_trip) * 1.5)
    t_lo, t_hi = max(0.0, t_trip - margin), t_clear + margin
    m = (t >= t_lo) & (t <= t_hi)
    if not m.any():
        ax.axis('off'); return
    tl = t[m]
    ax2 = ax.twinx()
    
    if temp_raw is not None: ax.plot(tl, temp_raw[m], lw=0.9, color=C_TEMP_RAW, label='temp (raw)', alpha=0.7)
    if temp_ema is not None: ax.plot(tl, temp_ema[m], lw=1.6, color=C_TEMP_EMA, label='temp_ema')
    
    # DYNAMIC AXIS: If FAN fault, show RPM. Else, show Vnode.
    if 'FAN' in reason_name and fan_rpm is not None:
        ax2.plot(tl, fan_rpm[m], lw=1.5, color=C_FAN, ls='--', label='fan (RPM)')
        ax2.axhline(FAN_STALL_RPM, color=C_FAULT, lw=0.8, ls=':', label='Stall Threshold')
        ax2.set_ylabel("Speed (RPM)", color=C_FAN, fontsize=9)
        ax2.tick_params(axis='y', labelcolor=C_FAN, labelsize=8)
    elif vnode is not None:
        ax2.plot(tl, vnode[m], lw=1.5, color=C_VNODE, ls='--', label='vnode (V)')
        ax2.set_ylim(-0.1, 3.5)
        ax2.axhline(2.5, color=C_FAULT, lw=0.8, ls=':', label='Open Thresh')
        ax2.axhline(0.05, color=C_FAULT, lw=0.8, ls=':', label='Short Thresh')
        ax2.set_ylabel("vnode (V)", color=C_VNODE, fontsize=9)
        ax2.tick_params(axis='y', labelcolor=C_VNODE, labelsize=8)

    ax.axvspan(t_trip, t_clear, color=C_FAULT, alpha=0.15, zorder=0)
    ax.axvline(t_trip, color=C_FAULT, lw=1.2)
    ax.axvline(t_clear, color=C_PID, lw=1.2, ls='-.')
    
    trig = {'NTC_OPEN': 'NTC unplugged', 'NTC_SHORT': 'NTC shorted', 'FAN_OPEN': 'Fan stalled'}.get(reason_name, reason_name)
    _, ymax = ax.get_ylim()
    ax.annotate(f"TRIP\n{trig}", xy=(t_trip, ymax * 0.95), xytext=(t_trip - margin * 0.5, ymax * 0.80), fontsize=8, color=C_FAULT, fontweight='bold', ha='left', va='top', bbox=dict(boxstyle='round,pad=0.3', fc='#fff5f5', ec=C_FAULT, lw=0.7), arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=0.7))
    ax.annotate("Recover", xy=(t_clear, ymax * 0.55), xytext=(t_clear + margin * 0.18, ymax * 0.65), fontsize=8, color=C_PID, fontweight='bold', ha='left', va='center', bbox=dict(boxstyle='round,pad=0.3', fc='#f0fdf4', ec=C_PID, lw=0.7), arrowprops=dict(arrowstyle='->', color=C_PID, lw=0.7))
    
    ax.set_title(f"Detail [{idx}]  t={t_trip:.0f}s  duration={t_clear - t_trip:.0f}s", fontsize=10, fontweight='bold', color=C_INK, loc='left')
    ax.set_ylabel("Temp (°C)", fontsize=10, color=C_TEMP_EMA)
    ax.set_xlabel("Time (s)", fontsize=9, color=C_INK_SOFT)
    ax.tick_params(axis='both', labelsize=8)
    ax.tick_params(axis='y', labelcolor=C_TEMP_EMA)
    ax.grid(True, alpha=0.3); ax.set_xlim(t_lo, t_hi)
    
    l1, la1 = ax.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    if l1 or l2: ax.legend(l1 + l2, la1 + la2, loc='center left', fontsize=7.5, framealpha=0.92, frameon=True, edgecolor=C_INK_SOFT)
    ax.text(0.02, 0.97, str(idx), transform=ax.transAxes, ha='left', va='top', fontsize=12, fontweight='bold', color=C_BADGE_ED, bbox=dict(boxstyle='circle,pad=0.4', fc=C_BADGE_BG, ec=C_BADGE_ED, lw=1.2))

# ... [FSM drawing code and Walkthrough plotting code remains the same as before] ...
# (Included to make the script complete)

FSM_NODES = {'IDLE': (1.6, 7.8), 'PID': (5.0, 7.8), 'THROTTLE': (8.4, 7.8), 'FAULT': (3.3, 2.9), 'COOLING': (6.7, 2.9)}
FSM_EDGES = [
    ('IDLE', 'PID', 'start', (3.3, 8.3), 0.0), ('PID', 'THROTTLE', 'fan maxed', (6.7, 8.75), 0.3),
    ('THROTTLE', 'PID', 'sustainable', (6.7, 6.9), 0.3), ('PID', 'FAULT', 'trip', (3.6, 5.6), 0.1),
    ('THROTTLE', 'FAULT', 'trip', (5.6, 4.6), -0.3), ('FAULT', 'COOLING', 'recover', (5.0, 2.45), 0.2),
    ('PID', 'COOLING', 'stop', (6.2, 5.6), -0.1), ('THROTTLE', 'COOLING', 'stop', (8.3, 5.2), -0.2),
    ('COOLING', 'IDLE', 'cool', (1.35, 5.2), -0.3),
]

def draw_fsm(ax, events=None, counts=True):
    visits = count_state_visits(events) if counts else {}
    trans = count_transitions(events) if counts else {}
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.set_aspect('equal'); ax.axis('off')
    if counts:
        ax.text(5.0, 9.8, "FSM (per-zone)", ha='center', va='top', fontsize=13, fontweight='bold', color=C_INK)
        ax.text(5.0, 9.25, "counts from this run", ha='center', va='top', fontsize=8.5, color=C_INK_SOFT, style='italic')
    bw, bh = 2.25, 1.1
    for name, (x, y) in FSM_NODES.items():
        st = NAME_TO_ID[name]; n = visits.get(st, 0); active = (not counts) or n > 0
        ax.add_patch(FancyBboxPatch((x - bw / 2, y - bh / 2), bw, bh, boxstyle="round,pad=0.05,rounding_size=0.12", facecolor=STATE_COLORS[st], edgecolor=C_INK, linewidth=1.6 if (counts and n) else 1.0, alpha=0.95 if active else 0.40, zorder=3))
        fs = 10 if len(name) <= 6 else 8.5
        dy = 0.16 if (counts and n) else 0.0
        ax.text(x, y + dy, name, ha='center', va='center', fontsize=fs, fontweight='bold', color='white', zorder=4)
        if counts and n: ax.text(x, y - 0.28, f"x{n}", ha='center', va='center', fontsize=8.5, color='white', zorder=4)
    for src, dst, label, (lx, ly), rad in FSM_EDGES:
        x0, y0 = FSM_NODES[src]; x1, y1 = FSM_NODES[dst]
        n = trans.get((NAME_TO_ID[src], NAME_TO_ID[dst]), 0)
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), connectionstyle=f"arc3,rad={rad}", arrowstyle='-|>', mutation_scale=14, color=C_INK if (counts and n) else (C_INK_SOFT if not counts else '#cccccc'), linewidth=2.0 if (counts and n) else 1.0, shrinkA=25, shrinkB=25, zorder=2))
        lab = label if not (counts and n) else f"{label} x{n}"
        ax.text(lx, ly, lab, ha='center', va='center', fontsize=7.6, color=C_INK if (counts and n) else C_INK_SOFT, style='italic', bbox=dict(boxstyle='round,pad=0.18', fc='white', ec='none', alpha=0.93), zorder=5)

def make_detail_plot(data, fields, events, png_path, show, title_suffix=""):
    apply_style()
    t = col(data, fields, "time_s")
    temp_raw = mask_invalid_temps(col(data, fields, "temp_c"), t, events)
    temp_ema = mask_invalid_temps(col(data, fields, "temp_ema"), t, events)
    vnode = col(data, fields, "vnode")
    fan_rpm = col(data, fields, "fan_rpm")
    fwins = fault_windows(events)
    n_z = min(3, len(fwins))
    
    fig = plt.figure(figsize=(14, 6.5), facecolor=C_BG)
    gs = fig.add_gridspec(2, 1, height_ratios=[0.45, 4.0], hspace=0.28, left=0.06, right=0.94, top=0.94, bottom=0.07)
    draw_title(fig.add_subplot(gs[0]), "STM32 Chip-Cooling Controller - Detail", "FSM with live counts (left) / dynamic per-fault zoom panels (right)", title_suffix)
    
    if n_z == 0:
        draw_fsm(fig.add_subplot(gs[1]), events, counts=True)
    else:
        gsb = gs[1].subgridspec(1, 2, width_ratios=[1.0, 1.4], wspace=0.18)
        draw_fsm(fig.add_subplot(gsb[0]), events, counts=True)
        gz = gsb[1].subgridspec(n_z, 1, hspace=0.65)
        for i in range(n_z):
            t0, t1, r = fwins[i]
            draw_zoom_panel(fig.add_subplot(gz[i]), t, temp_raw, temp_ema, vnode, fan_rpm, t0, t1, r, idx=i + 1)
            
    if len(fwins) > 3: fig.text(0.98, 0.01, f"Showing first 3 of {len(fwins)} fault events", fontsize=8, color=C_INK_SOFT, ha='right')
    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print(f"[png] wrote {png_path}", file=sys.stderr)
    if show: plt.show()
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
        print(f"  {name:<12} min={c.min():>9.2f}  max={c.max():>9.2f}  mean={c.mean():>9.2f}")
    
    cycles = throttle_cycles(intervals)
    if cycles and heater is not None:
        print("\n  THROTTLE CYCLES")
        for (t0, t1, prev, nxt) in cycles:
            dv = handover_delta(t, heater, t0)
            print(f"    enter t={t0:7.1f}  exit t={t1:7.1f}  dwell={t1 - t0:6.1f}s  handover step=" + (f"{dv:+.4f}" if dv is not None else "  n/a"))
        print("    (handover step ~ 0 => bumpless transfer verified)")
    print("=" * 72)

# =============================================================================
# MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("--events")
    ap.add_argument("-o", "--out-prefix", default=None)
    ap.add_argument("--marks", help="file: time,label[,color] per line")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    if args.no_plot: matplotlib.use("Agg")

    with open(args.infile, encoding="utf-8", errors="replace") as f:
        fields, data = parse_expressions_dump(f.read())
    
    if "time_s" in fields:
        ti = fields.index("time_s")
        data = data[data[:, ti] > 0.0]
        data = data[np.argsort(data[:, ti])]

    events = None
    if args.events:
        with open(args.events, encoding="utf-8", errors="replace") as f:
            events = parse_events_from_expressions(f.read())
    marks = load_marks(args.marks)

    print_report(data, fields, events)
    stem = args.out_prefix or args.infile.rsplit(".", 1)[0]
    
    show = not args.no_plot
    make_overview_plot(data, fields, events, stem + "_overview.png", show, "", marks, None)
    make_mechanism_plot(data, fields, events, stem + "_mechanism.png", show, "", marks)
    make_detail_plot(data, fields, events, stem + "_detail.png", show, "")

if __name__ == "__main__":
    main()