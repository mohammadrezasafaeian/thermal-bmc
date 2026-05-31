#!/usr/bin/env python3
"""
parse_struct_dump.py — Parse STM32CubeIDE Expressions-view dump, plot, and
emit a text "fingerprint" of the curve that can be pasted into chat/log.

PRECISION POLICY
----------------
CSV and matplotlib keep FULL float precision (for K/tau/theta extraction).
Only the pasteable text fingerprint is rounded to 2 decimals for readability.

INVALID-ROW POLICY
------------------
thermal_log is a zero-initialized circular buffer; unwritten slots are all
zeros. A real entry always has time_s > 0 (HAL_GetTick). So rows with
time_s == 0 are dropped as buffer padding.

INPUT FORMAT  (STM32CubeIDE Expressions view, named-field text)
    Details:{{time_s = 1.96, temp_c = 29.8, setpoint_c = 30,
              heater_duty = 0, fan_duty = 0}, {...}}

USAGE
    python parse_struct_dump.py step_test_01.txt
    python parse_struct_dump.py step_test_01.txt -o step_test_01.csv
    python parse_struct_dump.py step_test_01.txt --rows 30 --no-plot
"""

import argparse
import re
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

DISP = 2   # decimals for the pasteable text fingerprint only


# ────────────────────────────────────────────────────────────────────────────
# 1. PARSE
# ────────────────────────────────────────────────────────────────────────────

def parse_expressions_dump(text):
    """Extract every {field = value, ...} block. Returns (field_names, ndarray)."""
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


# ────────────────────────────────────────────────────────────────────────────
# 2. TEXT FINGERPRINT  (rounded to DISP decimals — paste this to share)
# ────────────────────────────────────────────────────────────────────────────

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
    """First time the drive crosses `thresh` — marks start of the step (≈ θ ref)."""
    if duty is None:
        return None
    above = np.where(duty > thresh)[0]
    return float(t[above[0]]) if len(above) else None


def print_fingerprint(data, fields, n_rows):
    """Everything below the banner is plain text — copy & paste to share."""
    t    = col(data, fields, "time_s")
    temp = col(data, fields, "temp_c")
    hd   = col(data, fields, "heater_duty")

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
    if temp is not None and hd is not None and t is not None:
        t_step = detect_step_time(t, hd)
        if t_step is not None:
            pre  = temp[t < t_step]
            post = temp[t >= t_step]
            y0   = pre[-20:].mean()  if len(pre)  else float("nan")
            yinf = post[-20:].mean() if len(post) else float("nan")
            print(f"\n  step fired @ t={t_step:.{DISP}f}s   "
                  f"y0(pre)≈{y0:.{DISP}f}°C   y_end(last20)≈{yinf:.{DISP}f}°C   "
                  f"rise≈{yinf - y0:.{DISP}f}°C")
            tail = temp[-10:]
            slope = (tail[-1] - tail[0]) / max(1, (len(tail) - 1))
            verdict = "SETTLED" if abs(tail[-1] - tail[0]) < 0.3 else "STILL RISING"
            print(f"  tail slope (last 10 pts): {slope:.{DISP}f} °C/sample -> {verdict}")

    # ---- sparkline ----
    if temp is not None:
        spark, lo, hi = sparkline(temp, width=64)
        print(f"\n  temp_c  [{lo:.{DISP}f} .. {hi:.{DISP}f} °C]")
        print(f"  {spark}")
    if hd is not None and hd.max() > 1e-6:
        spark, lo, hi = sparkline(hd, width=64)
        print(f"  heater  [{lo:.{DISP}f} .. {hi:.{DISP}f}]")
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


# ────────────────────────────────────────────────────────────────────────────
# 3. CSV  (full precision)
# ────────────────────────────────────────────────────────────────────────────

def save_csv(path, data, fields):
    np.savetxt(path, data, delimiter=",",
               header=",".join(fields), comments="", fmt="%.3f")
    print(f"[csv] wrote {len(data)} rows -> {path}", file=sys.stderr)


# ────────────────────────────────────────────────────────────────────────────
# 4. PLOT  (full precision)
# ────────────────────────────────────────────────────────────────────────────

def plot(data, fields, png_path, show):
    t    = col(data, fields, "time_s")
    x    = t if t is not None else np.arange(len(data))
    xlab = "time (s)" if t is not None else "sample"

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))

    temp = col(data, fields, "temp_c")
    sp   = col(data, fields, "setpoint_c")
    if temp is not None:
        ax1.plot(x, temp, label="temp_c", lw=1.3)
    if sp is not None:
        ax1.plot(x, sp, "--", label="setpoint_c", lw=1.0)
    ax1.set_ylabel("°C"); ax1.grid(True, alpha=0.3); ax1.legend(loc="best")
    ax1.set_title("Thermal step response")

    hd = col(data, fields, "heater_duty")
    fd = col(data, fields, "fan_duty")
    if hd is not None:
        ax2.plot(x, hd, label="heater_duty", lw=1.3)
    if fd is not None and fd.max() > 1e-6:
        ax2.plot(x, fd, label="fan_duty", lw=1.0)
    ax2.set_ylabel("duty"); ax2.set_xlabel(xlab)
    ax2.grid(True, alpha=0.3); ax2.legend(loc="best")

    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    print(f"[png] wrote {png_path}", file=sys.stderr)
    if show:
        plt.show()


# ────────────────────────────────────────────────────────────────────────────
# 5. MAIN
# ────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Parse + plot + fingerprint thermal log.")
    ap.add_argument("infile", help="Expressions-view dump text file")
    ap.add_argument("-o", "--csv", help="write parsed data to this CSV")
    ap.add_argument("--rows", type=int, default=32, help="rows in decimated table")
    ap.add_argument("--no-plot", action="store_true", help="skip matplotlib window")
    args = ap.parse_args()

    if args.no_plot:
        matplotlib.use("Agg")

    with open(args.infile, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()

    fields, data = parse_expressions_dump(text)
    data = drop_buffer_padding(data, fields)   # <-- kills the (0,0,...) rows
    data = sort_by_time(data, fields)

    print_fingerprint(data, fields, args.rows)

    if args.csv:
        save_csv(args.csv, data, fields)

    png = (args.csv or args.infile).rsplit(".", 1)[0] + ".png"
    plot(data, fields, png, show=not args.no_plot)


if __name__ == "__main__":
    main()