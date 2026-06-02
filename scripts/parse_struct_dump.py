#!/usr/bin/env python3
"""
parse_struct_dump.py — Parse STM32CubeIDE Expressions-view dumps for the
dual-stream thermal log, plot, and emit a text "fingerprint" of the curve.

INPUT STREAMS
-------------
Stream 1  (dense, per-tick):   thermal_log[]      → 5 floats per entry
Stream 2  (sparse, on-change): thermal_events[]   → time + kind + payload

Both arrive as STM32CubeIDE Expressions-view text dumps:
    Details:{{time_s = 1.96, temp_c = 29.8, ...}, {...}}

USAGE
    # dense only (back-compat)
    python parse_struct_dump.py run1.txt

    # dense + events overlaid
    python parse_struct_dump.py run1.txt --events run1_events.txt

    # with CSV export, custom row count, headless plot
    python parse_struct_dump.py run1.txt --events run1_events.txt \\
        -o run1.csv --rows 40 --no-plot

PRECISION POLICY
----------------
CSV and matplotlib keep FULL float precision (for K/τ/θ extraction).
Only the pasteable text fingerprint is rounded to DISP decimals.

INVALID-ROW POLICY
------------------
thermal_log[] is a zero-initialized circular buffer; unwritten slots are all
zeros. A real entry always has time_s > 0 (HAL_GetTick). Rows with time_s == 0
are dropped as padding. Same rule for thermal_events[] using kind == 0.
"""

import argparse
import re
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

DISP = 2   # decimals for the pasteable text fingerprint only


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  STREAM 1 — DENSE LOG                                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def parse_expressions_dump(text):
    """Extract every {field = value, ...} block. Returns (field_names, ndarray).
       Works for any struct with all-numeric fields — name-driven."""
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    if not entries:
        raise ValueError("No {field = value, ...} blocks found in input.")

    parsed = []
    for entry in entries:
        pairs = re.findall(
            r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if pairs:
            parsed.append(dict(pairs))
    if not parsed:
        raise ValueError("Found blocks but no field=value pairs.")

    fields = list(parsed[0].keys())            # canonical order = first entry

    rows = [[float(d[f]) for f in fields]
            for d in parsed if all(f in d for f in fields)]

    # Deduplicate (Expressions view repeats the array AND its slices)
    seen, unique = set(), []
    for row in rows:
        key = tuple(round(v, 6) for v in row)
        if key not in seen:
            seen.add(key)
            unique.append(row)
    if not unique:
        raise ValueError("No complete entries after deduplication.")

    return fields, np.array(unique, dtype=np.float64)


def drop_buffer_padding(data, fields):
    """Remove zero-initialized log slots: rows where time_s == 0."""
    if "time_s" not in fields:
        return data
    t = data[:, fields.index("time_s")]
    keep = t > 0.0
    dropped = int((~keep).sum())
    if dropped:
        print(f"[clean] dropped {dropped} zero-time padding row(s)", file=sys.stderr)
    return data[keep]


def sort_by_time(data, fields):
    if "time_s" in fields:
        return data[np.argsort(data[:, fields.index("time_s")])]
    return data


def col(data, fields, name):
    """Return a column by name, or None if absent."""
    return data[:, fields.index(name)] if name in fields else None


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  STREAM 2 — SPARSE EVENT LOG                                             ║
# ╚══════════════════════════════════════════════════════════════════════════╝

EVENT_NAMES = {
    1: 'STATE_CHANGE', 2: 'FAULT_RAISED', 3: 'FAULT_CLEARED',
    4: 'SETPOINT_CHG', 5: 'START_REQ',    6: 'STOP_REQ',
}
STATE_NAMES = {0: 'IDLE', 1: 'PID', 2: 'COOLING', 3: 'FAULT'}
FAULT_NAMES = {0: 'NONE',     1: 'NTC_OPEN',  2: 'NTC_SHORT',
               3: 'HEATER_OPEN', 4: 'FAN_OPEN', 5: 'ALL_DISCONNECTED'}

EVT_DTYPE = np.dtype([
    ('time_s',     '<f4'),
    ('kind',       'u1'),
    ('u8_payload', 'u1'),
    ('_pad',       '<u2'),
    ('f_payload',  '<f4'),
])


def parse_events_from_expressions(text):
    """Parse thermal_events[] from an Expressions-view text dump.
       Returns a structured ndarray (EVT_DTYPE) sorted by time, or None."""
    entries = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    parsed = []
    required = ('time_s', 'kind', 'u8_payload', 'f_payload')

    for entry in entries:
        pairs = re.findall(
            r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', entry)
        if not pairs:
            continue
        d = dict(pairs)
        if not all(k in d for k in required):
            continue
        parsed.append((
            float(d['time_s']),
            int(float(d['kind'])),
            int(float(d['u8_payload'])),
            0,
            float(d['f_payload']),
        ))

    if not parsed:
        return None

    arr = np.array(parsed, dtype=EVT_DTYPE)
    arr = arr[arr['kind'] != 0]                  # drop zero-padded entries
    arr = arr[np.argsort(arr['time_s'])]
    # Deduplicate (Expressions view repeats slices)
    keys = list({(float(e['time_s']), int(e['kind']),
                  int(e['u8_payload']), float(e['f_payload'])): e
                 for e in arr}.values())
    arr = np.array(keys, dtype=EVT_DTYPE)
    arr = arr[np.argsort(arr['time_s'])]
    return arr


def print_events(events):
    if events is None or len(events) == 0:
        print("\n  EVENT TRACE  (sparse log)\n  (no events)")
        return
    print("\n  EVENT TRACE  (sparse log)")
    print("  " + "-" * 50)
    for e in events:
        name = EVENT_NAMES.get(int(e['kind']), '?')
        extra = ''
        if name == 'STATE_CHANGE':
            extra = f" -> {STATE_NAMES.get(int(e['u8_payload']), '?')}"
        elif name in ('FAULT_RAISED', 'FAULT_CLEARED'):
            extra = f" ({FAULT_NAMES.get(int(e['u8_payload']), '?')})"
        elif name == 'SETPOINT_CHG':
            extra = f" {e['f_payload']:.2f} C"
        print(f"  t={e['time_s']:8.2f}s  {name}{extra}")


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TEXT FINGERPRINT  (rounded to DISP decimals — paste to share)           ║
# ╚══════════════════════════════════════════════════════════════════════════╝

_BLOCKS = "▁▂▃▄▅▆▇█"


def sparkline(values, width=64):
    """Downsample to `width` bins (mean) and map each to a block glyph."""
    v = np.asarray(values, dtype=float)
    if len(v) > width:
        edges = np.linspace(0, len(v), width + 1).astype(int)
        v = np.array([v[edges[i]:edges[i + 1]].mean() for i in range(width)])
    lo, hi = np.nanmin(v), np.nanmax(v)
    if hi - lo < 1e-9:
        return _BLOCKS[0] * len(v), lo, hi
    idx = np.clip(((v - lo) / (hi - lo) * 8).astype(int), 0, 7)
    return "".join(_BLOCKS[i] for i in idx), lo, hi


def detect_step_time(t, duty, thresh=0.05):
    """First time |duty| crosses thresh — works for unsigned heater_duty
       (legacy) or signed duty_cmd (new)."""
    if duty is None:
        return None
    above = np.where(np.abs(duty) > thresh)[0]
    return float(t[above[0]]) if len(above) else None


def print_fingerprint(data, fields, n_rows):
    """Plain-text summary block; copy & paste to share."""
    t    = col(data, fields, "time_s")
    temp = col(data, fields, "temp_c")
    # Prefer signed duty_cmd (new schema); fall back to heater_duty (legacy)
    duty = col(data, fields, "duty_cmd")
    if duty is None:
        duty = col(data, fields, "heater_duty")

    print("=" * 68)
    print("  CURVE FINGERPRINT  — copy everything between the lines")
    print("=" * 68)

    # ---- summary stats ----
    print(f"rows={len(data)}", end="")
    if t is not None and len(t) > 1:
        dt = np.diff(t)
        print(f"   span={t[-1]-t[0]:.{DISP}f}s   dt mean={dt.mean():.{DISP}f}s "
              f"min={dt.min():.{DISP}f} max={dt.max():.{DISP}f}", end="")
    print()

    for name in fields:
        c = col(data, fields, name)
        print(f"  {name:<12} min={c.min():>8.{DISP}f}  max={c.max():>8.{DISP}f}  "
              f"mean={c.mean():>8.{DISP}f}  std={c.std():>7.{DISP}f}")

    # ---- step landmark ----
    if temp is not None and duty is not None and t is not None:
        t_step = detect_step_time(t, duty)
        if t_step is not None:
            pre  = temp[t <  t_step]
            post = temp[t >= t_step]
            y0   = pre[-20:].mean()  if len(pre)  else float("nan")
            yinf = post[-20:].mean() if len(post) else float("nan")
            print(f"\n  step fired @ t={t_step:.{DISP}f}s   "
                  f"y0(pre)≈{y0:.{DISP}f}°C   y_end(last20)≈{yinf:.{DISP}f}°C   "
                  f"rise≈{yinf - y0:.{DISP}f}°C")
            tail  = temp[-10:]
            slope = (tail[-1] - tail[0]) / max(1, (len(tail) - 1))
            verdict = "SETTLED" if abs(tail[-1] - tail[0]) < 0.3 else "STILL RISING"
            print(f"  tail slope (last 10 pts): {slope:.{DISP}f} °C/sample -> {verdict}")

    # ---- sparkline ----
    if temp is not None:
        spark, lo, hi = sparkline(temp, width=64)
        print(f"\n  temp_c  [{lo:.{DISP}f} .. {hi:.{DISP}f} °C]")
        print(f"  {spark}")
    if duty is not None and np.abs(duty).max() > 1e-6:
        spark, lo, hi = sparkline(duty, width=64)
        print(f"  duty    [{lo:.{DISP}f} .. {hi:.{DISP}f}]")
        print(f"  {spark}")

    # ---- decimated table ----
    print(f"\n  decimated table ({n_rows} rows, evenly spaced):")
    print("   " + "".join(f"{f:>12}" for f in fields))
    print("   " + "-" * (12 * len(fields)))
    pick = np.linspace(0, len(data) - 1, min(n_rows, len(data))).astype(int)
    for i in pick:
        print("   " + "".join(f"{round(data[i, j], DISP):>12}"
                              for j in range(len(fields))))

    print("=" * 68)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CSV  (full precision)                                                   ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def save_csv(path, data, fields):
    np.savetxt(path, data, delimiter=",",
               header=",".join(fields), comments="", fmt="%.6f")
    print(f"[csv] wrote {len(data)} rows -> {path}", file=sys.stderr)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  PLOT  (3 panels + event markers)                                        ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# Event-kind → vertical-line color (translucent overlay on all panels)
_EVENT_COLORS = {
    1: 'tab:green',   # STATE_CHANGE
    2: 'tab:red',     # FAULT_RAISED
    3: 'tab:orange',  # FAULT_CLEARED
    4: 'tab:blue',    # SETPOINT_CHG
    5: 'tab:gray',    # START_REQ
    6: 'tab:gray',    # STOP_REQ
}


def plot(data, fields, events, png_path, show):
    """Three stacked panels sharing time axis:
         (1) temperature: raw + EMA + setpoint (from events, step plot)
         (2) signed duty: heater positive, fan negative, zero line drawn
         (3) vnode: raw ADC voltage with horizontal threshold lines
       Event-stream entries are drawn as faint vertical lines on all panels."""
    t    = col(data, fields, "time_s")
    x    = t if t is not None else np.arange(len(data))
    xlab = "time (s)" if t is not None else "sample"

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 9))
    ax_t, ax_d, ax_v = axes

    # ── Panel 1: temperature (raw + EMA + setpoint) ─────────────────────────
    temp = col(data, fields, "temp_c")
    ema  = col(data, fields, "temp_ema")
    if temp is not None:
        ax_t.plot(x, temp, label="temp_c (raw)", lw=1.0, alpha=0.6)
    if ema is not None:
        ax_t.plot(x, ema, label="temp_ema (PID input)", lw=1.4)

    # Setpoint comes from event stream — render as a step function
    if events is not None and len(events):
        sp_evts = events[events['kind'] == 4]   # EV_SETPOINT_CHG
        if len(sp_evts):
            sp_t = np.concatenate([sp_evts['time_s'], [x[-1]]])
            sp_v = np.concatenate([sp_evts['f_payload'],
                                   [sp_evts['f_payload'][-1]]])
            ax_t.step(sp_t, sp_v, where='post', linestyle='--',
                      label='setpoint (events)', lw=1.0, color='k')

    ax_t.set_ylabel("°C"); ax_t.grid(True, alpha=0.3); ax_t.legend(loc="best")
    ax_t.set_title("Thermal response (dense)  +  FSM events (sparse)")

    # ── Panel 2: signed duty ────────────────────────────────────────────────
    duty = col(data, fields, "duty_cmd")
    if duty is None:
        # Legacy fallback: heater + fan separate
        hd = col(data, fields, "heater_duty")
        fd = col(data, fields, "fan_duty")
        if hd is not None: ax_d.plot(x, hd, label="heater_duty", lw=1.2)
        if fd is not None and fd.max() > 1e-6:
            ax_d.plot(x, fd, label="fan_duty", lw=1.0)
    else:
        ax_d.plot(x, duty, label="duty_cmd [-1..+1]", lw=1.2)
        ax_d.axhline(0, color='k', lw=0.5)

    ax_d.set_ylabel("duty"); ax_d.grid(True, alpha=0.3); ax_d.legend(loc="best")

    # ── Panel 3: vnode (sensor truth) with fault thresholds ─────────────────
    vnode = col(data, fields, "vnode")
    if vnode is not None:
        ax_v.plot(x, vnode, label="vnode (V)", lw=1.0, color='tab:purple')
        ax_v.axhline(2.5,   color='r', lw=0.5, linestyle=':',
                     label='V_OPEN_THRESH (2.5 V)')
        ax_v.axhline(0.020, color='r', lw=0.5, linestyle=':',
                     label='V_SHORT_THRESH (20 mV)')
    else:
        ax_v.text(0.5, 0.5, "(no vnode column - legacy log?)",
                  ha='center', va='center', transform=ax_v.transAxes)

    ax_v.set_ylabel("V"); ax_v.set_xlabel(xlab)
    ax_v.grid(True, alpha=0.3); ax_v.legend(loc="best")

    # ── Event markers on ALL panels (translucent vertical lines) ────────────
    if events is not None and len(events):
        for e in events:
            c = _EVENT_COLORS.get(int(e['kind']), 'k')
            for ax in axes:
                ax.axvline(e['time_s'], color=c, lw=0.6, alpha=0.35)

    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    print(f"[png] wrote {png_path}", file=sys.stderr)
    if show:
        plt.show()


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MAIN                                                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def main():
    ap = argparse.ArgumentParser(
        description="Parse + plot + fingerprint thermal log (dense + sparse).")
    ap.add_argument("infile",
                    help="Expressions-view dump text file for thermal_log[]")
    ap.add_argument("--events",
                    help="Expressions-view dump text file for thermal_events[]")
    ap.add_argument("-o", "--csv", help="write dense data to this CSV")
    ap.add_argument("--rows", type=int, default=32,
                    help="rows in decimated table")
    ap.add_argument("--no-plot", action="store_true",
                    help="skip matplotlib window (still saves PNG)")
    args = ap.parse_args()

    if args.no_plot:
        matplotlib.use("Agg")

    # ─── Stream 1: dense log ────────────────────────────────────────────────
    with open(args.infile, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()

    fields, data = parse_expressions_dump(text)
    data = drop_buffer_padding(data, fields)
    data = sort_by_time(data, fields)
    print_fingerprint(data, fields, args.rows)

    # ─── Stream 2: sparse event log (optional) ──────────────────────────────
    events = None
    if args.events:
        with open(args.events, "r", encoding="utf-8", errors="replace") as f:
            evt_text = f.read()
        events = parse_events_from_expressions(evt_text)
        print_events(events)

    if args.csv:
        save_csv(args.csv, data, fields)

    png = (args.csv or args.infile).rsplit(".", 1)[0] + ".png"
    plot(data, fields, events, png, show=not args.no_plot)


if __name__ == "__main__":
    main()