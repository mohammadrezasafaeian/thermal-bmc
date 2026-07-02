#!/usr/bin/env python3
"""
thermal_report.py — STM32 chip-cooler log analyzer.

Inputs : debugger memory dumps of thermal_log[] and thermal_events[]
         (GDB / CubeIDE "{field=val, ...}" struct-array format)
Outputs:
  run.csv      ~300 rows, event/inflection-weighted, with reconstructed state
  events.csv   decoded event stream (state names, fault reasons, payloads)
  report.png   two-pane story plot: temperature loop + actuators, state bands

Usage:
  python thermal_report.py log_dump.txt --events-dump events_dump.txt
"""
import argparse, re, sys
import numpy as np

# ── firmware enums (keep in sync with thermal_app.h) ────────────────────────
STATE_NAMES = {0: "IDLE", 1: "PID", 2: "THROTTLE", 3: "COOLING", 4: "FAULT"}
EVENT_KINDS = {0: "NONE", 1: "STATE_CHANGE", 2: "FAULT_RAISED",
               3: "FAULT_CLEARED", 4: "SETPOINT_CHG", 5: "START_REQ", 6: "STOP_REQ"}
FAULT_NAMES = {0: "NONE", 1: "NTC_OPEN", 2: "NTC_SHORT", 3: "HEATER_OPEN",
               4: "FAN_OPEN", 5: "ALL_DISCONNECTED"}
STATE_COLORS = {"IDLE": "#d9d9d9", "PID": "#c6e6c3", "THROTTLE": "#f9d67a",
                "COOLING": "#a8d3f0", "FAULT": "#f4a6a6"}

# ── parsing ──────────────────────────────────────────────────────────────────
def parse_struct_blocks(text):
    """{field=val,...} blocks -> list of dicts (numeric fields only)."""
    rows = []
    for b in re.findall(r'\{([^{}]*=[^{}]*)\}', text):
        pairs = re.findall(
            r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', b)
        if pairs:
            rows.append({k: float(v) for k, v in pairs})
    if not rows:
        raise ValueError("no {field=value} blocks found in dump")
    return rows

def load_log(path):
    rows = parse_struct_blocks(open(path, encoding="utf-8",
                                    errors="replace").read())
    fields = [f for f in ("time_s", "temp_c", "temp_ema", "vnode",
                          "fan_duty", "heater_duty", "setpoint")
              if f in rows[0]]
    data = np.array([[r[f] for f in fields] for r in rows
                     if all(f in r for f in fields)])
    # ring buffer: drop unwritten (time==0) slots, drop exact dupes, sort
    data = data[data[:, 0] > 0.0]
    data = np.unique(np.round(data, 6), axis=0)
    data = data[np.argsort(data[:, 0])]
    return fields, data

def load_events(path):
    rows = parse_struct_blocks(open(path, encoding="utf-8",
                                    errors="replace").read())
    ev = []
    for r in rows:
        t, kind = r.get("time_s", 0.0), int(r.get("kind", 0))
        if t <= 0.0 or kind == 0:
            continue
        u8, fp = int(r.get("u8_payload", 0)), r.get("f_payload", 0.0)
        kname = EVENT_KINDS.get(kind, f"K{kind}")
        if kname == "STATE_CHANGE":
            detail = STATE_NAMES.get(u8, f"S{u8}")
        elif kname in ("FAULT_RAISED", "FAULT_CLEARED"):
            detail = FAULT_NAMES.get(u8, f"F{u8}")
        elif kname == "SETPOINT_CHG":
            detail = f"{fp:.1f}C"
        else:
            detail = ""
        ev.append((t, kname, detail, u8, fp))
    ev.sort(key=lambda e: e[0])
    return ev

# ── state reconstruction ─────────────────────────────────────────────────────
def state_timeline(events, t0, t1):
    """[(t_start, t_end, state_name), ...] covering [t0, t1]."""
    changes = [(t, d) for t, k, d, *_ in events if k == "STATE_CHANGE"]
    segs, cur, tcur = [], "PID" if changes else "PID", t0
    # state before first logged change is unknown; assume predecessor of first
    if changes:
        first = changes[0][1]
        cur = {"THROTTLE": "PID", "PID": "IDLE", "COOLING": "PID",
               "FAULT": "PID", "IDLE": "COOLING"}.get(first, "IDLE")
    for t, name in changes:
        if t > tcur:
            segs.append((tcur, t, cur))
        cur, tcur = name, t
    segs.append((tcur, t1, cur))
    return segs

def state_column(segs, times):
    out = np.empty(len(times), dtype=object)
    for i, t in enumerate(times):
        out[i] = next((s for a, b, s in segs if a <= t < b), segs[-1][2])
    return out

# ── event/inflection-weighted decimation ─────────────────────────────────────
def smart_decimate(data, fields, events, max_rows=300, event_halfwin=6.0):
    t = data[:, 0]
    n = len(data)
    if n <= max_rows:
        return np.arange(n)
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    # 1) always keep every sample within ±halfwin of any event
    for te, *_ in events:
        keep |= np.abs(t - te) <= event_halfwin
    # 2) curvature importance on key signals
    imp = np.zeros(n)
    for f in ("temp_ema", "heater_duty", "fan_duty"):
        if f in fields:
            y = data[:, fields.index(f)]
            rng = np.ptp(y) or 1.0
            d2 = np.abs(np.diff(y, 2, prepend=y[0], append=y[-1])) / rng
            imp += d2
    budget = max_rows - keep.sum()
    if budget > 0:
        cand = np.where(~keep)[0]
        order = cand[np.argsort(imp[cand])[::-1]]
        half = budget // 2
        keep[order[:half]] = True                      # sharpest points
        cand = np.where(~keep)[0]                      # 3) uniform fill
        if len(cand) and budget - half > 0:
            keep[cand[np.linspace(0, len(cand) - 1,
                                  budget - half).astype(int)]] = True
    return np.where(keep)[0]

# ── plotting ─────────────────────────────────────────────────────────────────
def make_plot(fields, data, events, segs, png):
    import matplotlib.pyplot as plt
    t = data[:, 0]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 2]})
    # state bands + labels (both axes)
    for a, b, s in segs:
        for ax in (ax1, ax2):
            ax.axvspan(a, b, color=STATE_COLORS.get(s, "#eee"),
                       alpha=0.35, lw=0)
        if b - a > 0.03 * (t[-1] - t[0]):
            ax1.text((a + b) / 2, 0.985, s, transform=ax1.get_xaxis_transform(),
                     ha="center", va="top", fontsize=8, weight="bold",
                     color="#444")
    col = lambda f: data[:, fields.index(f)]
    ax1.plot(t, col("temp_c"),   color="#bbb", lw=0.7, label="T raw")
    ax1.plot(t, col("temp_ema"), color="#c0392b", lw=1.8, label="T ema")
    ax1.plot(t, col("setpoint"), color="#2c3e50", ls="--", lw=1.2,
             label="setpoint")
    ax1.set_ylabel("temperature [°C]")
    ax1.legend(loc="upper left", fontsize=8, ncol=3)
    ax2.plot(t, col("fan_duty"),    color="#2980b9", lw=1.5, label="fan")
    ax2.plot(t, col("heater_duty"), color="#e67e22", lw=1.5, label="heater")
    ax2.set_ylabel("duty [0–1]"); ax2.set_ylim(-0.05, 1.1)
    ax2.set_xlabel("time [s]")
    ax2.legend(loc="upper left", fontsize=8, ncol=2)
    # non-state events as markers
    for te, k, d, *_ in events:
        if k in ("FAULT_RAISED", "FAULT_CLEARED", "SETPOINT_CHG"):
            for ax in (ax1, ax2):
                ax.axvline(te, color="#8e44ad", ls=":", lw=1, alpha=0.7)
            ax1.text(te, 0.02, f"{k}:{d}", rotation=90, fontsize=6.5,
                     transform=ax1.get_xaxis_transform(), color="#8e44ad")
    fig.suptitle("Chip-cooling controller — closed-loop run", fontsize=12)
    fig.tight_layout()
    fig.savefig(png, dpi=160)
    print(f"[plot]   {png}", file=sys.stderr)

# ── main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_dump")
    ap.add_argument("--events-dump", required=True)
    ap.add_argument("--max-rows", type=int, default=300)
    ap.add_argument("-o", "--out-prefix", default="run")
    args = ap.parse_args()

    fields, data = load_log(args.log_dump)
    events = load_events(args.events_dump)
    print(f"[parse]  {len(data)} log rows, {len(events)} events, "
          f"span {data[0,0]:.0f}–{data[-1,0]:.0f}s", file=sys.stderr)

    segs = state_timeline(events, data[0, 0], data[-1, 0])
    idx  = smart_decimate(data, fields, events, args.max_rows)
    out  = data[idx]
    states = state_column(segs, out[:, 0])

    # run.csv — with state column, for interpretation
    csv = args.out_prefix + ".csv"
    with open(csv, "w") as f:
        f.write(",".join(fields) + ",state\n")
        for row, s in zip(out, states):
            f.write(",".join(f"{v:.4f}" for v in row) + f",{s}\n")
    print(f"[csv]    {len(out)} rows -> {csv}", file=sys.stderr)

    # events.csv — decoded
    ecsv = args.out_prefix + "_events.csv"
    with open(ecsv, "w") as f:
        f.write("time_s,kind,detail,u8,f_payload\n")
        for t, k, d, u8, fp in events:
            f.write(f"{t:.3f},{k},{d},{u8},{fp:.4f}\n")
    print(f"[events] {len(events)} -> {ecsv}", file=sys.stderr)

    # per-state dwell summary
    for a, b, s in segs:
        print(f"  {s:<9} {a:8.1f} -> {b:8.1f}  ({b-a:6.1f}s)", file=sys.stderr)

    make_plot(fields, data, events, segs, args.out_prefix + ".png")

if __name__ == "__main__":
    main()