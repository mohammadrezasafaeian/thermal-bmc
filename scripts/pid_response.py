#!/usr/bin/env python3
"""
pid_response.py — Closed-loop PID step-response analyzer.
Reads CSV from parse_struct_dump.py. Extracts control metrics and plots.

METRICS (the scorecard):
  steady-state error  -> Q1 (integrator kills it?)
  settling time       -> Q2 (~4*tau_c?)
  overshoot %         -> Q3 (~9%?)
  peak duty           -> Q4 (gentle nudge?)
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

BAND = 0.5   # +/- tolerance band (°C) — your spec claim


def col(d, name):
    return d[name] if name in d.dtype.names else None


def find_step(sp):
    """Index where setpoint is first 'active' (nonzero). Here setpoint is
    constant 30, so we instead use the first sample as t0 of the response."""
    return 0


def metrics(t, y, sp, duty, ema_alpha=0.05):
    """Compute step-response metrics on an EMA-smoothed temp (noise is ±1.5°C,
    bigger than the overshoot, so raw is useless for peak detection)."""
    # smooth for peak/settling; raw kept for SS error stats
    ys = np.empty_like(y)
    ys[0] = y[0]
    for i in range(1, len(y)):
        ys[i] = ys[i - 1] + ema_alpha * (y[i] - ys[i - 1])

    r = sp[-1]                      # target (30)
    y0 = ys[0]                      # start (~28)
    span = r - y0                   # commanded change

    # --- steady state: mean of last 60 s (raw, to expose true offset) ---
    ss = y[t >= t[-1] - 60].mean()
    ss_err = r - ss

    # --- peak / overshoot (smoothed) ---
    peak = ys.max()
    peak_t = t[np.argmax(ys)]
    overshoot = max(0.0, (peak - r) / span * 100.0) if span > 0 else 0.0

    # --- rise time: 10% -> 90% of span (smoothed) ---
    def cross(level):
        idx = np.argmax(ys >= level)
        return t[idx] if ys[idx] >= level else np.nan
    t10, t90 = cross(y0 + 0.1 * span), cross(y0 + 0.9 * span)
    rise = t90 - t10

    # --- settling time: last time |error| leaves the band (smoothed) ---
    outside = np.abs(ys - r) > BAND
    settle = t[np.where(outside)[0][-1]] if outside.any() else t[0]

    return dict(r=r, y0=y0, ss=ss, ss_err=ss_err, peak=peak, peak_t=peak_t,
                overshoot=overshoot, rise=rise, settle=settle,
                duty_peak=duty.max() * 100, ys=ys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--alpha", type=float, default=0.05, help="EMA for peak detect")
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True)
    t, y = d["time_s"], d["temp_c"]
    sp = col(d, "setpoint_c")
    duty = col(d, "heater_duty")
    if sp is None: sp = np.full_like(y, y[-1])
    if duty is None: duty = np.zeros_like(y)

    m = metrics(t, y, sp, duty, args.alpha)

    # ---- text scorecard (paste this) ----
    print("=" * 56)
    print("  PID STEP-RESPONSE SCORECARD")
    print("=" * 56)
    print(f"  target          = {m['r']:.2f} C")
    print(f"  start           = {m['y0']:.2f} C   (span {m['r']-m['y0']:.2f} C)")
    print(f"  steady state    = {m['ss']:.2f} C")
    print(f"  SS error    [Q1]= {m['ss_err']:+.3f} C   (want ~0)")
    print(f"  rise 10-90%     = {m['rise']:.0f} s")
    print(f"  settle ±{BAND}C [Q2]= {m['settle']:.0f} s   (predicted ~160 s)")
    print(f"  peak            = {m['peak']:.2f} C @ t={m['peak_t']:.0f}s")
    print(f"  overshoot   [Q3]= {m['overshoot']:.1f} %   (predicted ~9%)")
    print(f"  peak duty   [Q4]= {m['duty_peak']:.0f} %   (gentle?)")
    print("=" * 56)

    # ---- 3-panel plot ----
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 9))

    ax[0].plot(t, y, lw=0.8, alpha=0.4, label="temp (raw)")
    ax[0].plot(t, m["ys"], lw=1.6, label="temp (EMA)")
    ax[0].plot(t, sp, "k--", lw=1.0, label="setpoint")
    ax[0].fill_between(t, sp - BAND, sp + BAND, color="green", alpha=0.12,
                       label=f"±{BAND}°C band")
    ax[0].axvline(m["settle"], color="purple", ls=":", label="settle")
    ax[0].set_ylabel("°C"); ax[0].legend(loc="best", fontsize=8)
    ax[0].set_title("Closed-loop PID response")

    err = sp - y
    ax[1].axhline(0, color="k", lw=0.6)
    ax[1].fill_between(t, -BAND, BAND, color="green", alpha=0.12)
    ax[1].plot(t, err, lw=0.9, color="tab:red")
    ax[1].set_ylabel("error °C"); ax[1].grid(alpha=0.3)

    ax[2].plot(t, duty * 100, lw=1.0, color="tab:orange", label="heater %")
    fan = col(d, "fan_duty")
    if fan is not None and fan.max() > 0:
        ax[2].plot(t, fan * 100, lw=1.0, color="tab:blue", label="fan %")
    ax[2].set_ylabel("duty %"); ax[2].set_xlabel("time (s)")
    ax[2].set_ylim(-5, 105); ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)

    fig.tight_layout()
    png = args.csv.rsplit(".", 1)[0] + "_pid.png"
    fig.savefig(png, dpi=110)
    print(f"[png] {png}")
    plt.show()


if __name__ == "__main__":
    main()