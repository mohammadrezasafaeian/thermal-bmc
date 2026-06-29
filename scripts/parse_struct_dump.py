#!/usr/bin/env python3
"""parse_struct_dump.py - parse STM32 thermal log dump -> tidy CSV (<=500 rows)."""
import argparse, re, sys
import numpy as np

def parse_dump(text):
    """Extract {field=val,...} blocks into (fields, ndarray)."""
    blocks = re.findall(r'\{([^{}]*=[^{}]*)\}', text)
    rows = []
    for b in blocks:
        pairs = re.findall(r'(\w+)\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)', b)
        if pairs:
            rows.append(dict(pairs))
    if not rows:
        raise ValueError("no {field=value} blocks found")
    fields = list(rows[0].keys())
    data = np.array([[float(r[f]) for f in fields]
                     for r in rows if all(f in r for f in fields)],
                    dtype=np.float64)
    return fields, data

def dedup(data):
    """Remove exact duplicate rows (memory dump repeats)."""
    seen, out = set(), []
    for row in data:
        key = tuple(np.round(row, 6))
        if key not in seen:
            seen.add(key); out.append(row)
    return np.array(out)

def clean(data, fields):
    """Drop padding (time<=0), sort by time."""
    if "time_s" in fields:
        ti = fields.index("time_s")
        data = data[data[:, ti] > 0.0]
        data = data[np.argsort(data[:, ti])]
    return data

def decimate(data, max_rows):
    n = len(data)
    if n <= max_rows:
        return data
    idx = np.linspace(0, n - 1, max_rows).astype(int)
    return data[idx]

def load_events(path):
    """Parse 'time,label' (or 'time label') lines -> [(time, label), ...]."""
    events = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r'[,\s]+', line, maxsplit=1)
            if len(parts) < 2:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            events.append((t, parts[1]))
    return events

def plot_data(fields, data, events, png_path):
    import matplotlib.pyplot as plt
    ti = fields.index("time_s") if "time_s" in fields else None
    x = data[:, ti] if ti is not None else np.arange(len(data))
    plot_fields = [f for i, f in enumerate(fields) if i != ti]

    fig, ax = plt.subplots(figsize=(10, 5))
    for f in plot_fields:
        ax.plot(x, data[:, fields.index(f)], label=f)
    for t, label in events:
        ax.axvline(t, color="gray", linestyle="--", alpha=0.5)
        ax.text(t, ax.get_ylim()[1], label, rotation=90,
                 va="top", ha="right", fontsize=7, color="gray")
    ax.set_xlabel("time_s" if ti is not None else "sample")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    print("[plot] wrote -> {}".format(png_path), file=sys.stderr)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("-o", "--csv", default="run.csv")
    ap.add_argument("--max-rows", type=int, default=500)
    ap.add_argument("--events", help="path to event-marker file (time,label per line)")
    ap.add_argument("--no-plot", action="store_true", help="skip generating a PNG plot")
    args = ap.parse_args()

    with open(args.infile, encoding="utf-8", errors="replace") as f:
        text = f.read()
    fields, data = parse_dump(text)
    data = dedup(data)
    data = clean(data, fields)
    print("[parse] {} unique rows, span {:.1f}s".format(
        len(data), data[-1, fields.index("time_s")] if "time_s" in fields else -1),
        file=sys.stderr)
    out = decimate(data, args.max_rows)
    np.savetxt(args.csv, out, delimiter=",", header=",".join(fields),
               comments="", fmt="%.4f")
    print("[csv] wrote {} rows -> {}".format(len(out), args.csv), file=sys.stderr)
    for i, name in enumerate(fields):
        c = out[:, i]
        print("  {:<12} min={:8.3f} max={:8.3f} mean={:8.3f}".format(
            name, c.min(), c.max(), c.mean()), file=sys.stderr)

    events = load_events(args.events) if args.events else []
    if not args.no_plot:
        png_path = re.sub(r'\.csv$', '', args.csv, flags=re.I) + ".png"
        plot_data(fields, out, events, png_path)

if __name__ == "__main__":
    main()