#!/usr/bin/env python3
"""
parse_struct_dump.py - STM32 thermal controller portfolio plotter.

Three figures, one capture:
  Figure 1 (overview)     -> run1_overview.png      Title + temperature + duty.
  Figure 2 (detail)       -> run1_detail.png        FSM diagram + fault zooms.
  Figure 3 (walkthrough)  -> run1_walkthrough.png   FSM ref + pivot insets + strip.
"""

import argparse
import re
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch

DISP = 2
TEMP_VALID_LO, TEMP_VALID_HI = -10.0, 100.0   # 10D-9 NTC physical sanity range

# ----- Palette ------------------------------------------------------------
C_BG, C_INK, C_INK_SOFT = '#ffffff', '#1f2933', '#6b7280'
C_GRID = '#e5e7eb'
C_TEMP_RAW, C_TEMP_EMA, C_SETPOINT = '#d1d5db', '#d97706', '#1f2933'
C_HEATER, C_FAN = '#ea580c', '#0369a1'
C_VNODE, C_FAULT = '#7c3aed', '#b91c1c'
C_PID, C_COOLING, C_IDLE = '#15803d', '#0369a1', '#9ca3af'
C_BADGE_BG, C_BADGE_ED = '#fef3c7', '#92400e'

STATE_COLORS = {0: C_IDLE, 1: C_PID, 2: C_COOLING, 3: C_FAULT}


# =========================================================================
# PARSERS
# =========================================================================
def parse_expressions_dump(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    if not entries:
        raise ValueError("No {field = value, ...} blocks found.")
    parsed = []
    for entry in entries:
        pairs = re.findall(
            r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if pairs:
            parsed.append(dict(pairs))
    fields = list(parsed[0].keys())
    rows = [[float(d[f]) for f in fields]
            for d in parsed if all(f in d for f in fields)]
    seen, unique = set(), []
    for row in rows:
        key = tuple(round(v, 6) for v in row)
        if key not in seen:
            seen.add(key); unique.append(row)
    return fields, np.array(unique, dtype=np.float64)


def drop_buffer_padding(data, fields):
    if "time_s" not in fields:
        return data
    t = data[:, fields.index("time_s")]
    keep = t > 0.0
    n = int((~keep).sum())
    if n:
        print("[clean] dropped {} padding rows".format(n), file=sys.stderr)
    return data[keep]


def sort_by_time(data, fields):
    if "time_s" in fields:
        return data[np.argsort(data[:, fields.index("time_s")])]
    return data


def col(data, fields, name):
    return data[:, fields.index(name)] if name in fields else None


# ----- Sparse event log ---------------------------------------------------
EV_STATE_CHANGE, EV_FAULT_RAISED, EV_FAULT_CLEARED = 1, 2, 3
EV_SETPOINT_CHG, EV_START_REQ, EV_STOP_REQ = 4, 5, 6

EVENT_NAMES = {1: 'STATE_CHANGE', 2: 'FAULT_RAISED', 3: 'FAULT_CLEARED',
               4: 'SETPOINT_CHG', 5: 'START_REQ', 6: 'STOP_REQ'}
STATE_NAMES = {0: 'IDLE', 1: 'PID', 2: 'COOLING', 3: 'FAULT'}
FAULT_NAMES = {0: 'NONE', 1: 'NTC_OPEN', 2: 'NTC_SHORT',
               3: 'HEATER_OPEN', 4: 'FAN_OPEN', 5: 'ALL_DISCONNECTED'}

EVT_DTYPE = np.dtype([('time_s', '<f4'), ('kind', 'u1'),
                      ('u8_payload', 'u1'), ('_pad', '<u2'),
                      ('f_payload', '<f4')])


def parse_events_from_expressions(text):
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    parsed = []
    req = ('time_s', 'kind', 'u8_payload', 'f_payload')
    for entry in entries:
        pairs = re.findall(
            r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if not pairs:
            continue
        d = dict(pairs)
        if not all(k in d for k in req):
            continue
        parsed.append((float(d['time_s']), int(float(d['kind'])),
                       int(float(d['u8_payload'])), 0,
                       float(d['f_payload'])))
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


def print_events(events):
    if events is None or len(events) == 0:
        print("\n  EVENT TRACE: (none)"); return
    print("\n  EVENT TRACE")
    print("  " + "-" * 50)
    for e in events:
        name = EVENT_NAMES.get(int(e['kind']), '?')
        extra = ''
        if name == 'STATE_CHANGE':
            extra = " -> {}".format(STATE_NAMES.get(int(e['u8_payload']), '?'))
        elif name in ('FAULT_RAISED', 'FAULT_CLEARED'):
            extra = " ({})".format(FAULT_NAMES.get(int(e['u8_payload']), '?'))
        elif name == 'SETPOINT_CHG':
            extra = " {:.2f} C".format(float(e['f_payload']))
        print("  t={:8.2f}s  {}{}".format(float(e['time_s']), name, extra))


# =========================================================================
# DERIVED WINDOWS / COUNTS / MASKS
# =========================================================================
def state_intervals(events, t_end):
    if events is None or len(events) == 0:
        return []
    sc = events[events['kind'] == EV_STATE_CHANGE]
    if len(sc) == 0:
        return []
    out = []
    for i, e in enumerate(sc):
        t0 = float(e['time_s'])
        t1 = float(sc[i + 1]['time_s']) if i + 1 < len(sc) else t_end
        out.append((t0, t1, int(e['u8_payload'])))
    return out


def fault_windows(events):
    if events is None or len(events) == 0:
        return []
    wins, t0, reason = [], None, None
    for e in events:
        k = int(e['kind'])
        if k == EV_FAULT_RAISED:
            t0 = float(e['time_s'])
            reason = FAULT_NAMES.get(int(e['u8_payload']), '?')
        elif k == EV_FAULT_CLEARED and t0 is not None:
            wins.append((t0, float(e['time_s']), reason))
            t0, reason = None, None
    return wins


def setpoint_steps(events, t_end):
    if events is None:
        return None, None
    sp = events[events['kind'] == EV_SETPOINT_CHG]
    if len(sp) == 0:
        return None, None
    ts = np.concatenate([sp['time_s'], [t_end]])
    vs = np.concatenate([sp['f_payload'], [sp['f_payload'][-1]]])
    return ts, vs


def mask_invalid_temps(x, t, events):
    if x is None:
        return None
    x = x.copy()
    x[(x < TEMP_VALID_LO) | (x > TEMP_VALID_HI)] = np.nan
    for (t0, t1, _) in fault_windows(events):
        x[(t >= t0) & (t <= t1)] = np.nan
    return x


def count_state_visits(events):
    if events is None:
        return {}
    sc = events[events['kind'] == EV_STATE_CHANGE]
    out = {}
    for e in sc:
        st = int(e['u8_payload'])
        out[st] = out.get(st, 0) + 1
    return out


def count_transitions(events):
    if events is None:
        return {}
    sc = events[events['kind'] == EV_STATE_CHANGE]
    out, prev = {}, None
    for e in sc:
        st = int(e['u8_payload'])
        if prev is not None:
            out[(prev, st)] = out.get((prev, st), 0) + 1
        prev = st
    return out


# =========================================================================
# STYLE
# =========================================================================
def apply_style():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'axes.titlesize': 11,
        'axes.titleweight': 'bold',
        'axes.labelsize': 10,
        'axes.edgecolor': C_INK_SOFT,
        'axes.linewidth': 0.8,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'grid.color': C_GRID,
        'grid.linewidth': 0.6,
        'legend.frameon': False,
        'legend.fontsize': 9,
        'xtick.color': C_INK_SOFT,
        'ytick.color': C_INK_SOFT,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
    })


def draw_title(ax, main, subtitle, suffix=""):
    ax.axis('off')
    ax.text(0.0, 0.85, main,
            fontsize=22, fontweight='bold', color=C_INK,
            ha='left', va='top', transform=ax.transAxes)
    full_sub = subtitle + (" - " + suffix if suffix else "")
    ax.text(0.0, 0.25, full_sub,
            fontsize=10.5, color=C_INK_SOFT,
            ha='left', va='top', transform=ax.transAxes)


# =========================================================================
# STORY CALLOUTS (Figure 1) - short, anti-collision, arrow-anchored
# =========================================================================
def add_story_callouts(ax, t, temp_ema_c, events, t_end):
    """Short labels with arrows. Two-row staggering when events are close."""
    if events is None or temp_ema_c is None:
        return
    finite_mask = np.isfinite(temp_ema_c)
    if not finite_mask.any():
        return

    def temp_near(time_val):
        return float(np.interp(time_val,
                               t[finite_mask], temp_ema_c[finite_mask]))

    # Build short, recruiter-readable beats from state transitions only.
    beats, prev = [], None
    for e in events:
        if int(e['kind']) != EV_STATE_CHANGE:
            continue
        sn = STATE_NAMES.get(int(e['u8_payload']), '?')
        te = float(e['time_s'])
        if sn == 'PID' and prev == 'IDLE':
            beats.append((te, "START", C_PID))
        elif sn == 'FAULT':
            reason = None
            for fe in events:
                if (int(fe['kind']) == EV_FAULT_RAISED
                        and abs(float(fe['time_s']) - te) < 2.0):
                    reason = FAULT_NAMES.get(int(fe['u8_payload']), '?')
                    break
            lbl = "FAULT" if not reason else "FAULT\n({})".format(reason)
            beats.append((te, lbl, C_FAULT))
        elif sn == 'IDLE' and prev == 'FAULT':
            beats.append((te, "RECOVER", C_PID))
        elif sn == 'COOLING':
            beats.append((te, "STOP\n(cool first)", C_COOLING))
        elif sn == 'IDLE' and prev == 'COOLING':
            beats.append((te, "COOLED", C_COOLING))
        prev = sn

    if not beats:
        return

    # Anti-collision: stagger Y when consecutive events are close in time.
    min_gap = t_end * 0.06
    placed, last_t, last_high = [], -1e9, False
    for (te, label, color) in beats:
        too_close = (te - last_t) < min_gap
        y_high = (not last_high) if too_close else True
        y_frac = 0.94 if y_high else 0.78
        placed.append((te, label, color, y_frac))
        last_t, last_high = te, y_high

    for (te, label, color, y_frac) in placed:
        y_anchor = temp_near(te)
        ax.annotate(
            label,
            xy=(te, y_anchor),
            xytext=(te, y_frac),
            xycoords='data',
            textcoords=('data', 'axes fraction'),
            ha='center', va='top',
            fontsize=8.5, color=color, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', fc='white',
                      ec=color, lw=0.8, alpha=0.96),
            arrowprops=dict(arrowstyle='->', color=color, lw=0.8,
                            shrinkA=3, shrinkB=4),
            zorder=10,
        )


# =========================================================================
# FSM DIAGRAMS
# =========================================================================
def draw_fsm_diagram(ax, events):
    """Full FSM with live counts. Figure 2 only."""
    visits = count_state_visits(events)
    trans = count_transitions(events)

    ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    ax.set_aspect('equal'); ax.axis('off')

    ax.text(5.0, 9.7, "FSM (per-zone)",
            ha='center', va='top', fontsize=13, fontweight='bold',
            color=C_INK)
    ax.text(5.0, 9.15,
            "counts from this run",
            ha='center', va='top', fontsize=8.5, color=C_INK_SOFT,
            style='italic')

    nodes = {
        'IDLE':    (2.2, 6.4),
        'PID':     (7.8, 6.4),
        'COOLING': (7.8, 2.4),
        'FAULT':   (2.2, 2.4),
    }
    name_to_id = {'IDLE': 0, 'PID': 1, 'COOLING': 2, 'FAULT': 3}
    box_w, box_h = 2.4, 1.1

    for name, (x, y) in nodes.items():
        st_id = name_to_id[name]
        color = STATE_COLORS[st_id]
        n_visits = visits.get(st_id, 0)
        active = n_visits > 0
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.05,rounding_size=0.12",
            facecolor=color, edgecolor=C_INK,
            linewidth=1.6 if active else 0.8,
            alpha=0.95 if active else 0.40,
            zorder=3)
        ax.add_patch(box)
        ax.text(x, y + 0.18, name, ha='center', va='center',
                fontsize=12, fontweight='bold', color='white', zorder=4)
        if active:
            ax.text(x, y - 0.24, "x{}".format(n_visits),
                    ha='center', va='center', fontsize=8.5,
                    color='white', zorder=4)

    edges = [
        ('IDLE',    'PID',     'start',   (5.0, 7.1), 0.0),
        ('PID',     'COOLING', 'stop',    (8.7, 4.4), 0.0),
        ('COOLING', 'IDLE',    'cool',    (5.0, 3.1), -0.25),
        ('PID',     'FAULT',   'trip',    (5.0, 5.1), 0.0),
        ('FAULT',   'IDLE',    'recover', (1.3, 4.4), 0.0),
    ]
    for src, dst, label, (lx, ly), rad in edges:
        x0, y0 = nodes[src]; x1, y1 = nodes[dst]
        n_fires = trans.get((name_to_id[src], name_to_id[dst]), 0)
        active = n_fires > 0
        color = C_INK if active else '#cccccc'
        lw = 2.0 if active else 0.9
        arrow = FancyArrowPatch(
            (x0, y0), (x1, y1),
            connectionstyle="arc3,rad={}".format(rad),
            arrowstyle='-|>', mutation_scale=16,
            color=color, linewidth=lw,
            shrinkA=28, shrinkB=28, zorder=2)
        ax.add_patch(arrow)
        full_label = label if not active else "{}  x{}".format(label, n_fires)
        ax.text(lx, ly, full_label,
                ha='center', va='center', fontsize=8.5,
                color=C_INK if active else C_INK_SOFT,
                style='italic',
                bbox=dict(boxstyle='round,pad=0.25', fc='white',
                          ec='none', alpha=0.92),
                zorder=5)


def draw_fsm_reference(ax):
    """Clean reference diagram. No counts. Figure 3 only.
       Purpose: explain the FSM topology to a first-time reader, not the run."""
    ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    ax.set_aspect('equal'); ax.axis('off')

    nodes = {
        'IDLE':    (2.4, 6.5),
        'PID':     (7.6, 6.5),
        'COOLING': (7.6, 2.8),
        'FAULT':   (2.4, 2.8),
    }
    name_to_id = {'IDLE': 0, 'PID': 1, 'COOLING': 2, 'FAULT': 3}
    box_w, box_h = 2.4, 1.1

    for name, (x, y) in nodes.items():
        color = STATE_COLORS[name_to_id[name]]
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.05,rounding_size=0.12",
            facecolor=color, edgecolor=C_INK, linewidth=1.0,
            alpha=0.92, zorder=3)
        ax.add_patch(box)
        ax.text(x, y, name, ha='center', va='center',
                fontsize=11, fontweight='bold', color='white', zorder=4)

    edges = [
        ('IDLE',    'PID',     'start',   0.0),
        ('PID',     'COOLING', 'stop',    0.0),
        ('COOLING', 'IDLE',    'cool',   -0.25),
        ('PID',     'FAULT',   'trip',    0.0),
        ('FAULT',   'IDLE',    'recover', 0.0),
    ]
    for src, dst, label, rad in edges:
        x0, y0 = nodes[src]; x1, y1 = nodes[dst]
        arrow = FancyArrowPatch(
            (x0, y0), (x1, y1),
            connectionstyle="arc3,rad={}".format(rad),
            arrowstyle='-|>', mutation_scale=14,
            color=C_INK_SOFT, linewidth=1.1,
            shrinkA=24, shrinkB=24, zorder=2)
        ax.add_patch(arrow)
        # Position label at the midpoint
        if rad == 0:
            lx, ly = (x0 + x1) / 2, (y0 + y1) / 2
        else:
            lx = (x0 + x1) / 2 + rad * (y1 - y0) * 0.5
            ly = (y0 + y1) / 2 - rad * (x1 - x0) * 0.5
        ax.text(lx, ly, label,
                ha='center', va='center', fontsize=9,
                color=C_INK_SOFT, style='italic',
                bbox=dict(boxstyle='round,pad=0.22', fc='white',
                          ec='none', alpha=0.96),
                zorder=5)


def draw_mini_fsm(ax, active_from, active_to):
    """Tiny FSM for walkthrough insets. Active state bright, others faded.
       Only the transition arrow that fired is drawn."""
    ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    ax.set_aspect('equal'); ax.axis('off')

    nodes = {
        'IDLE':    (2.5, 7.0),
        'PID':     (7.5, 7.0),
        'COOLING': (7.5, 3.0),
        'FAULT':   (2.5, 3.0),
    }
    name_to_id = {'IDLE': 0, 'PID': 1, 'COOLING': 2, 'FAULT': 3}
    box_w, box_h = 3.2, 2.0

    active_name = active_to if active_to is not None else active_from

    for name, (x, y) in nodes.items():
        color = STATE_COLORS[name_to_id[name]]
        is_active = (name == active_name)
        box = FancyBboxPatch(
            (x - box_w / 2, y - box_h / 2), box_w, box_h,
            boxstyle="round,pad=0.05,rounding_size=0.25",
            facecolor=color, edgecolor=C_INK,
            linewidth=1.6 if is_active else 0.6,
            alpha=1.0 if is_active else 0.25,
            zorder=3)
        ax.add_patch(box)
        ax.text(x, y, name[0],
                ha='center', va='center',
                fontsize=11, fontweight='bold',
                color='white', alpha=1.0 if is_active else 0.55,
                zorder=4)

    if (active_from is not None and active_to is not None
            and active_from in nodes and active_to in nodes):
        x0, y0 = nodes[active_from]
        x1, y1 = nodes[active_to]
        arrow = FancyArrowPatch(
            (x0, y0), (x1, y1),
            arrowstyle='-|>', mutation_scale=14,
            color=C_INK, linewidth=1.8,
            shrinkA=22, shrinkB=22, zorder=5)
        ax.add_patch(arrow)


# =========================================================================
# ZOOM PANEL
# =========================================================================
def draw_zoom_panel(ax, t, temp_raw, temp_ema, vnode,
                    t_trip, t_clear, reason_name, idx):
    margin = max(30.0, (t_clear - t_trip) * 1.5)
    t_lo = max(0.0, t_trip - margin)
    t_hi = t_clear + margin

    m = (t >= t_lo) & (t <= t_hi)
    if not m.any():
        ax.axis('off'); return
    t_local = t[m]

    ax2 = ax.twinx()

    if temp_raw is not None:
        ax.plot(t_local, temp_raw[m], lw=0.9, color=C_TEMP_RAW,
                label='temp (raw)', alpha=0.7)
    if temp_ema is not None:
        ax.plot(t_local, temp_ema[m], lw=1.6, color=C_TEMP_EMA,
                label='temp_ema')
    if vnode is not None:
        ax2.plot(t_local, vnode[m], lw=1.0, color=C_VNODE,
                 linestyle='--', label='vnode')
        ax2.set_ylim(-0.1, 3.5)
        ax2.axhline(2.5, color=C_FAULT, lw=0.4, ls=':')
        ax2.axhline(0.02, color=C_FAULT, lw=0.4, ls=':')
        ax2.set_ylabel("vnode (V)", color=C_VNODE, fontsize=9, labelpad=6)
        ax2.tick_params(axis='y', labelcolor=C_VNODE, labelsize=8)

    ax.axvspan(t_trip, t_clear, color=C_FAULT, alpha=0.15, zorder=0)
    ax.axvline(t_trip, color=C_FAULT, lw=1.2)
    ax.axvline(t_clear, color=C_PID, lw=1.2, ls='-.')

    trigger = {'NTC_OPEN': 'NTC unplugged',
               'NTC_SHORT': 'NTC shorted'}.get(reason_name, reason_name)
    _, ymax = ax.get_ylim()
    ax.annotate(
        "TRIP\n{}".format(trigger),
        xy=(t_trip, ymax * 0.95),
        xytext=(t_trip - margin * 0.5, ymax * 0.80),
        fontsize=8, color=C_FAULT, fontweight='bold',
        ha='left', va='top',
        bbox=dict(boxstyle='round,pad=0.3', fc='#fff5f5',
                  ec=C_FAULT, lw=0.7),
        arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=0.7))
    ax.annotate(
        "Recover",
        xy=(t_clear, ymax * 0.55),
        xytext=(t_clear + margin * 0.18, ymax * 0.65),
        fontsize=8, color=C_PID, fontweight='bold',
        ha='left', va='center',
        bbox=dict(boxstyle='round,pad=0.3', fc='#f0fdf4',
                  ec=C_PID, lw=0.7),
        arrowprops=dict(arrowstyle='->', color=C_PID, lw=0.7))

    duration = t_clear - t_trip
    ax.set_title(
        "Detail [{}]   t = {:.0f}s   duration = {:.0f}s".format(
            idx, t_trip, duration),
        fontsize=10, fontweight='bold', color=C_INK, loc='left')

    # The crucial left-axis label - make it big and padded so it can't be cropped
    ax.set_ylabel("Temperature (°C)",
                  fontsize=10, color=C_TEMP_EMA, labelpad=8)
    ax.set_xlabel("Time (s)", fontsize=9, color=C_INK_SOFT)
    ax.tick_params(axis='both', labelsize=8)
    ax.tick_params(axis='y', labelcolor=C_TEMP_EMA)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(t_lo, t_hi)

    # Combined twinx legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    if lines1 or lines2:
        ax.legend(lines1 + lines2, labels1 + labels2,
                  loc='upper right', fontsize=7.5, framealpha=0.92,
                  frameon=True, edgecolor=C_INK_SOFT)

    ax.text(0.02, 0.97, str(idx),
            transform=ax.transAxes, ha='left', va='top',
            fontsize=12, fontweight='bold', color=C_BADGE_ED,
            bbox=dict(boxstyle='circle,pad=0.4', fc=C_BADGE_BG,
                      ec=C_BADGE_ED, lw=1.2))


# =========================================================================
# WALKTHROUGH PIVOT SELECTION
# =========================================================================
def select_walkthrough_pivots(events, max_insets=8):
    if events is None or len(events) == 0:
        return []
    sc = [e for e in events if int(e['kind']) == EV_STATE_CHANGE]
    sp = [e for e in events if int(e['kind']) == EV_SETPOINT_CHG]

    pivots = []
    prev_state_name = None
    for e in sc:
        to_state = STATE_NAMES.get(int(e['u8_payload']), '?')
        pivots.append({'kind': 'state', 't': float(e['time_s']),
                       'from': prev_state_name, 'to': to_state, 'reason': None})
        prev_state_name = to_state

    fault_raised = [e for e in events if int(e['kind']) == EV_FAULT_RAISED]
    for p in pivots:
        if p['to'] != 'FAULT':
            continue
        for fr in fault_raised:
            if abs(float(fr['time_s']) - p['t']) < 2.0:
                p['reason'] = FAULT_NAMES.get(int(fr['u8_payload']), '?')
                break

    for e in sp:
        pivots.append({'kind': 'setpoint', 't': float(e['time_s']),
                       'from': None, 'to': None,
                       'value': float(e['f_payload'])})

    pivots.sort(key=lambda p: p['t'])

    if len(pivots) > max_insets:
        state_pivots = [p for p in pivots if p['kind'] == 'state']
        if len(state_pivots) <= max_insets:
            extras = [p for p in pivots if p['kind'] == 'setpoint']
            slots_left = max_insets - len(state_pivots)
            pivots = sorted(state_pivots + extras[:slots_left],
                            key=lambda p: p['t'])
        else:
            pivots = state_pivots[:max_insets]
    return pivots


# =========================================================================
# FIGURE 1 - OVERVIEW
# =========================================================================
def make_overview_plot(data, fields, events, png_path, show, title_suffix=""):
    """Title + temperature + duty. No KPI banner. No state strip.
       Y-axis is computed from real data (percentile-based), not the
       garbage temperatures recorded during a fault."""
    apply_style()

    t = col(data, fields, "time_s")
    if t is None:
        t = np.arange(len(data), dtype=float)
    t_end = float(t[-1])

    temp_raw = col(data, fields, "temp_c")
    temp_ema = col(data, fields, "temp_ema")
    duty = col(data, fields, "duty_cmd")

    temp_raw_c = mask_invalid_temps(temp_raw, t, events)
    temp_ema_c = mask_invalid_temps(temp_ema, t, events)

    sp_t, sp_v = setpoint_steps(events, t_end)
    intervals = state_intervals(events, t_end)

    fig = plt.figure(figsize=(13, 6.5), facecolor=C_BG)
    gs = fig.add_gridspec(
        3, 1, height_ratios=[0.6, 3.6, 1.4],
        hspace=0.30, left=0.07, right=0.97, top=0.93, bottom=0.09)

    ax_title = fig.add_subplot(gs[0])
    ax_temp  = fig.add_subplot(gs[1])
    ax_duty  = fig.add_subplot(gs[2], sharex=ax_temp)

    draw_title(ax_title, "STM32 Thermal Controller",
               "Closed-loop PI control with state-machine safety, verified on hardware",
               title_suffix)

    # Subtle state shading on temperature panel
    for (t0, t1, st) in intervals:
        if st == 1:
            ax_temp.axvspan(t0, t1, color=C_PID, alpha=0.05, zorder=0)
        elif st == 3:
            ax_temp.axvspan(t0, t1, color=C_FAULT, alpha=0.12, zorder=0)
        elif st == 2:
            ax_temp.axvspan(t0, t1, color=C_COOLING, alpha=0.07, zorder=0)

    if temp_raw_c is not None:
        ax_temp.plot(t, temp_raw_c, lw=0.8, color=C_TEMP_RAW,
                     label='temp_c (raw)', zorder=2)
    if temp_ema_c is not None:
        ax_temp.plot(t, temp_ema_c, lw=2.0, color=C_TEMP_EMA,
                     label='temp_ema (PID input)', zorder=3)
    if sp_t is not None:
        ax_temp.step(sp_t, sp_v, where='post', lw=1.4,
                     color=C_SETPOINT, linestyle='--',
                     label='setpoint', alpha=0.9, zorder=4)

    ax_temp.set_ylabel("Temperature (°C)", color=C_INK)
    ax_temp.grid(True, axis='y')
    ax_temp.legend(loc='lower right', ncol=3, bbox_to_anchor=(1.0, 1.005))

    # ---- Y-axis: percentile-based, ignoring outliers and including setpoints ----
    finite_vals = []
    for arr in (temp_raw_c, temp_ema_c):
        if arr is not None:
            f = arr[np.isfinite(arr)]
            if len(f):
                finite_vals.append(f)
    if finite_vals:
        all_t = np.concatenate(finite_vals)
        ymin = float(np.nanpercentile(all_t, 1))
        ymax = float(np.nanpercentile(all_t, 99))
        if sp_v is not None and len(sp_v):
            ymin = min(ymin, float(np.min(sp_v)))
            ymax = max(ymax, float(np.max(sp_v)))
        span = max(ymax - ymin, 1.0)
        # Modest top headroom for callouts (no more 140°C ceiling)
        ax_temp.set_ylim(ymin - 0.10 * span, ymax + 0.30 * span)

    add_story_callouts(ax_temp, t, temp_ema_c, events, t_end)

    # ---- Duty panel ----
    if duty is not None:
        ax_duty.fill_between(t, 0, duty, where=(duty > 0),
                             color=C_HEATER, alpha=0.6, label='heater')
        ax_duty.fill_between(t, 0, duty, where=(duty < 0),
                             color=C_FAN, alpha=0.6, label='fan')
        ax_duty.plot(t, duty, lw=0.7, color=C_INK)
        ax_duty.axhline(0, color=C_INK, lw=0.5)
    ax_duty.set_ylim(-1.1, 1.1)
    ax_duty.set_ylabel("Duty [-1..+1]", color=C_INK)
    ax_duty.grid(True, axis='y')
    ax_duty.legend(loc='upper right', ncol=2, bbox_to_anchor=(1.0, 1.18))
    ax_duty.set_xlabel(
        "Time (s)   /   total run: {}:{:02d}".format(
            int(t_end) // 60, int(t_end) % 60),
        color=C_INK)

    plt.setp(ax_temp.get_xticklabels(), visible=False)

    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print("[png] wrote {}".format(png_path), file=sys.stderr)
    if show:
        plt.show()
    plt.close(fig)


# =========================================================================
# FIGURE 2 - DETAIL
# =========================================================================
def make_detail_plot(data, fields, events, png_path, show, title_suffix=""):
    apply_style()

    t = col(data, fields, "time_s")
    if t is None:
        t = np.arange(len(data), dtype=float)

    temp_raw = col(data, fields, "temp_c")
    temp_ema = col(data, fields, "temp_ema")
    vnode = col(data, fields, "vnode")
    temp_raw_c = mask_invalid_temps(temp_raw, t, events)
    temp_ema_c = mask_invalid_temps(temp_ema, t, events)

    fault_wins = fault_windows(events)
    n_zooms = min(3, len(fault_wins))

    fig = plt.figure(figsize=(14, 6.5), facecolor=C_BG)
    gs = fig.add_gridspec(
        2, 1, height_ratios=[0.45, 4.0], hspace=0.28,
        left=0.06, right=0.98, top=0.94, bottom=0.07)

    ax_title = fig.add_subplot(gs[0])
    draw_title(ax_title, "STM32 Thermal Controller - Detail",
               "FSM with live counts (left)   /   per-fault zoom panels (right)",
               title_suffix)

    if n_zooms == 0:
        ax_fsm = fig.add_subplot(gs[1])
        draw_fsm_diagram(ax_fsm, events)
    else:
        gs_body = gs[1].subgridspec(1, 2, width_ratios=[1.0, 1.4], wspace=0.18)
        ax_fsm = fig.add_subplot(gs_body[0])
        draw_fsm_diagram(ax_fsm, events)
        gs_zoom = gs_body[1].subgridspec(n_zooms, 1, hspace=0.65)
        for i in range(n_zooms):
            ax_z = fig.add_subplot(gs_zoom[i])
            t_trip, t_clear, reason = fault_wins[i]
            draw_zoom_panel(ax_z, t, temp_raw_c, temp_ema_c, vnode,
                            t_trip, t_clear, reason, idx=i + 1)

    if len(fault_wins) > 3:
        fig.text(0.98, 0.01,
                 "Showing first 3 of {} fault events".format(len(fault_wins)),
                 fontsize=8, color=C_INK_SOFT, ha='right')

    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print("[png] wrote {}".format(png_path), file=sys.stderr)
    if show:
        plt.show()
    plt.close(fig)


# =========================================================================
# FIGURE 3 - WALKTHROUGH
# =========================================================================
def make_walkthrough_plot(data, fields, events, png_path, show, title_suffix=""):
    apply_style()

    t = col(data, fields, "time_s")
    if t is None:
        t = np.arange(len(data), dtype=float)
    t_end = float(t[-1])

    intervals = state_intervals(events, t_end)
    pivots = select_walkthrough_pivots(events, max_insets=8)
    n = len(pivots)

    if n == 0:
        print("[walkthrough] no events, skipping", file=sys.stderr)
        return

    fig = plt.figure(figsize=(15, 5.8), facecolor=C_BG)
    gs = fig.add_gridspec(
        4, 1, height_ratios=[1.8, 2.0, 0.85, 0.65],
        hspace=0.22, left=0.04, right=0.98, top=0.97, bottom=0.10)

    # Top row: title (left) + clean FSM reference (right)
    gs_top = gs[0].subgridspec(1, 2, width_ratios=[1.6, 1.0], wspace=0.05)
    ax_title = fig.add_subplot(gs_top[0]); ax_title.axis('off')
    ax_ref   = fig.add_subplot(gs_top[1])

    ax_title.text(0.0, 0.85, "FSM Walkthrough",
                  fontsize=22, fontweight='bold', color=C_INK,
                  ha='left', va='top', transform=ax_title.transAxes)
    ax_title.text(0.0, 0.55,
                  "What the controller actually did, step by step"
                  + (" - " + title_suffix if title_suffix else ""),
                  fontsize=10.5, color=C_INK_SOFT,
                  ha='left', va='top', transform=ax_title.transAxes)
    ax_title.text(0.0, 0.20,
                  "Reference key on the right.",
                  fontsize=9, color=C_INK_SOFT, style='italic',
                  ha='left', va='top', transform=ax_title.transAxes)

    draw_fsm_reference(ax_ref)

    # Inset row
    gs_insets = gs[1].subgridspec(1, n, wspace=0.20)
    inset_axes = []
    for i, p in enumerate(pivots):
        ax_i = fig.add_subplot(gs_insets[i])
        if p['kind'] == 'state':
            draw_mini_fsm(ax_i, p['from'], p['to'])
        else:
            draw_mini_fsm(ax_i, None, None)
            ax_i.text(5.0, 5.0,
                      "SP\n{:.0f} C".format(p['value']),
                      ha='center', va='center',
                      fontsize=10, fontweight='bold', color=C_SETPOINT,
                      bbox=dict(boxstyle='round,pad=0.3',
                                fc='white', ec=C_SETPOINT, lw=1.0))
        inset_axes.append(ax_i)

    # Caption block
    gs_caps = gs[2].subgridspec(1, n, wspace=0.20)
    caption_axes = []
    for i, p in enumerate(pivots):
        ax_c = fig.add_subplot(gs_caps[i])
        ax_c.set_xlim(0, 1); ax_c.set_ylim(0, 1); ax_c.axis('off')
        caption_axes.append(ax_c)

        ax_c.text(0.5, 0.92, "t = {:.0f} s".format(p['t']),
                  ha='center', va='top', fontsize=9,
                  fontweight='bold', color=C_INK,
                  transform=ax_c.transAxes)
        if p['kind'] == 'state':
            arrow_str = "{} -> {}".format(p['from'] or 'boot', p['to'])
            ax_c.text(0.5, 0.62, arrow_str,
                      ha='center', va='top', fontsize=8.5,
                      color=C_INK_SOFT, transform=ax_c.transAxes)
            if p['reason']:
                ax_c.text(0.5, 0.30, "FR_{}".format(p['reason']),
                          ha='center', va='top', fontsize=8,
                          color=C_FAULT, fontweight='bold',
                          transform=ax_c.transAxes)
        else:
            ax_c.text(0.5, 0.62, "Setpoint change",
                      ha='center', va='top', fontsize=8.5,
                      color=C_INK_SOFT, transform=ax_c.transAxes)
            ax_c.text(0.5, 0.30, "-> {:.0f} C".format(p['value']),
                      ha='center', va='top', fontsize=8,
                      color=C_SETPOINT, fontweight='bold',
                      transform=ax_c.transAxes)

    # Timeline strip
    ax_strip = fig.add_subplot(gs[3])
    ax_strip.set_xlim(0, t_end); ax_strip.set_ylim(0, 1)
    ax_strip.set_yticks([])
    ax_strip.spines['left'].set_visible(False)
    ax_strip.grid(False)

    for (t0, t1, st) in intervals:
        ax_strip.add_patch(Rectangle(
            (t0, 0.20), t1 - t0, 0.55,
            facecolor=STATE_COLORS.get(st, C_IDLE),
            edgecolor='white', linewidth=0.5))

    for p in pivots:
        ax_strip.axvline(p['t'], color=C_INK, lw=0.6, alpha=0.35, zorder=2)
        ax_strip.plot(p['t'], 0.475, marker='v', markersize=7,
                      color=C_BADGE_ED, markeredgecolor='white',
                      markeredgewidth=0.8, zorder=4)

    ax_strip.set_xlabel(
        "Time (s)   /   total run: {}:{:02d}".format(
            int(t_end) // 60, int(t_end) % 60),
        color=C_INK)

    fig.canvas.draw()
    bbox_strip = ax_strip.get_position()
    for i, p in enumerate(pivots):
        bbox_c = caption_axes[i].get_position()
        x_top = bbox_c.x0 + bbox_c.width * 0.5
        y_top = bbox_c.y0 - 0.005
        x_bot = bbox_strip.x0 + bbox_strip.width * (p['t'] / t_end)
        y_bot = bbox_strip.y0 + bbox_strip.height * 0.95
        arrow = FancyArrowPatch(
            (x_top, y_top), (x_bot, y_bot),
            transform=fig.transFigure,
            arrowstyle='-|>', mutation_scale=10,
            color=C_BADGE_ED, linewidth=0.9,
            connectionstyle="arc3,rad=0.0",
            shrinkA=2, shrinkB=2, zorder=10)
        fig.patches.append(arrow)

    fig.savefig(png_path, dpi=160, bbox_inches='tight', facecolor=C_BG)
    print("[png] wrote {}".format(png_path), file=sys.stderr)
    if show:
        plt.show()
    plt.close(fig)


# =========================================================================
# TERMINAL FINGERPRINT
# =========================================================================
_BLOCKS = "_.-=*#%@"


def sparkline(values, width=64):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return "(no data)", 0.0, 0.0
    if len(v) > width:
        edges = np.linspace(0, len(v), width + 1).astype(int)
        v = np.array([v[edges[i]:edges[i + 1]].mean() for i in range(width)])
    lo, hi = float(np.nanmin(v)), float(np.nanmax(v))
    if hi - lo < 1e-9:
        return _BLOCKS[0] * len(v), lo, hi
    idx = np.clip(((v - lo) / (hi - lo) * (len(_BLOCKS) - 1)).astype(int),
                  0, len(_BLOCKS) - 1)
    return "".join(_BLOCKS[i] for i in idx), lo, hi


def print_fingerprint(data, fields, events):
    t = col(data, fields, "time_s")
    temp = col(data, fields, "temp_c")
    duty = col(data, fields, "duty_cmd")
    if duty is None:
        duty = col(data, fields, "heater_duty")
    print("=" * 72)
    print("  CURVE FINGERPRINT")
    print("=" * 72)
    print("rows={}".format(len(data)), end="")
    if t is not None and len(t) > 1:
        dt = np.diff(t)
        print("   span={:.2f}s   dt mean={:.2f}s".format(
            t[-1] - t[0], dt.mean()), end="")
    print()
    for name in fields:
        c = col(data, fields, name)
        print("  {:<12} min={:>9.2f}  max={:>9.2f}  mean={:>9.2f}  std={:>8.2f}"
              .format(name, c.min(), c.max(), c.mean(), c.std()))
    if temp is not None:
        clean = mask_invalid_temps(temp, t, events) if t is not None else temp
        spark, lo, hi = sparkline(clean, 64)
        print("\n  temp (fault-masked) [{:.2f}..{:.2f} C]".format(lo, hi))
        print("  " + spark)
    if duty is not None and np.abs(duty).max() > 1e-6:
        spark, lo, hi = sparkline(duty, 64)
        print("  duty                [{:.2f}..{:.2f}]".format(lo, hi))
        print("  " + spark)
    print("=" * 72)


# =========================================================================
# CSV + MAIN
# =========================================================================
def save_csv(path, data, fields):
    np.savetxt(path, data, delimiter=",", header=",".join(fields),
               comments="", fmt="%.6f")
    print("[csv] wrote {} rows -> {}".format(len(data), path), file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="STM32 thermal portfolio plotter.")
    ap.add_argument("infile", help="Expressions-view dump for thermal_log[]")
    ap.add_argument("--events", help="Expressions-view dump for thermal_events[]")
    ap.add_argument("-o", "--csv", help="write dense data to this CSV")
    ap.add_argument("--title", default="")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    if args.no_plot:
        matplotlib.use("Agg")

    with open(args.infile, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    fields, data = parse_expressions_dump(text)
    data = drop_buffer_padding(data, fields)
    data = sort_by_time(data, fields)

    events = None
    if args.events:
        with open(args.events, "r", encoding="utf-8", errors="replace") as f:
            evt_text = f.read()
        events = parse_events_from_expressions(evt_text)

    print_fingerprint(data, fields, events)
    print_events(events)

    if args.csv:
        save_csv(args.csv, data, fields)

    stem = (args.csv or args.infile).rsplit(".", 1)[0]
    make_overview_plot(data, fields, events, stem + "_overview.png",
                       show=not args.no_plot, title_suffix=args.title)
    make_detail_plot(data, fields, events, stem + "_detail.png",
                     show=not args.no_plot, title_suffix=args.title)
    make_walkthrough_plot(data, fields, events, stem + "_walkthrough.png",
                          show=not args.no_plot, title_suffix=args.title)


if __name__ == "__main__":
    main()