import re
import sys
import matplotlib.pyplot as plt
import numpy as np

# --- Load raw debugger dump ---
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = 'thermal_dump.txt'

with open(filename, 'r') as f:
    raw = f.read()

# --- Parse all entries ---
pattern = re.compile(
    r'\{time_s\s*=\s*([^,]+),\s*'
    r'temp_c\s*=\s*([^,]+),\s*'
    r'setpoint_c\s*=\s*([^,]+),\s*'
    r'heater_duty\s*=\s*([^,]+),\s*'
    r'fan_duty\s*=\s*([^}]+)\}'
)

matches = pattern.findall(raw)

entries = []
seen_times = set()
for m in matches:
    t = float(m[0])
    if t < 0.001:
        continue
    if t in seen_times:
        continue
    seen_times.add(t)
    entries.append({
        't': t,
        'T': float(m[1]),
        'sp': float(m[2]),
        'H': float(m[3]),
        'F': float(m[4])
    })

entries.sort(key=lambda e: e['t'])
print(f"Parsed {len(entries)} unique entries")
print(f"Time range: {entries[0]['t']:.1f} to {entries[-1]['t']:.1f} seconds")

# --- Extract columns as numpy arrays ---
times = np.array([e['t'] for e in entries])
temps = np.array([e['T'] for e in entries])
setpoints = np.array([e['sp'] for e in entries])
duty = np.array([e['H'] for e in entries])

# Derive actual actuator signals
heater_actual = np.maximum(duty, 0)
fan_actual = np.maximum(-duty, 0)
fan_display = -fan_actual    # negative side for display

# --- EMA smoothing for display ---
def ema_filter(signal, alpha=0.15):
    out = np.zeros_like(signal)
    out[0] = signal[0]
    for i in range(1, len(signal)):
        out[i] = out[i-1] + alpha * (signal[i] - out[i-1])
    return out

temps_smooth = ema_filter(temps, alpha=0.15)
duty_smooth = ema_filter(duty, alpha=0.15)
heater_smooth = np.maximum(duty_smooth, 0)
fan_smooth = -np.maximum(-duty_smooth, 0)

# --- Detect setpoint changes for annotation ---
sp_changes = []
for i in range(1, len(setpoints)):
    if setpoints[i] != setpoints[i-1]:
        sp_changes.append((times[i], setpoints[i]))

# --- Plot ---
fig, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True,
                          gridspec_kw={'height_ratios': [3, 2, 2]})
fig.patch.set_facecolor('#f8f8f8')

# ── Panel 1: Temperature ──
axes[0].fill_between(times, temps, setpoints, where=temps > setpoints,
                     color='#ff6b6b', alpha=0.15, label='Above setpoint')
axes[0].fill_between(times, temps, setpoints, where=temps < setpoints,
                     color='#4dabf7', alpha=0.15, label='Below setpoint')
axes[0].plot(times, temps, color='#ff6b6b', linewidth=0.5, alpha=0.3)
axes[0].plot(times, temps_smooth, color='#e03131', linewidth=1.5,
             label='Temperature (filtered)')
axes[0].step(times, setpoints, color='#1971c2', linewidth=2,
             linestyle='--', label='Setpoint', where='post')

for t_change, sp_val in sp_changes:
    axes[0].annotate(f'{sp_val:.0f}°C', xy=(t_change, sp_val),
                     xytext=(10, 15), textcoords='offset points',
                     fontsize=8, color='#1971c2', fontweight='bold',
                     arrowprops=dict(arrowstyle='->', color='#1971c2',
                                    lw=1))

axes[0].set_ylabel('Temperature (°C)', fontsize=11)
axes[0].legend(loc='upper right', fontsize=9)
axes[0].grid(True, alpha=0.2)
axes[0].set_title('Bidirectional PID — Dual Zone Thermal Controller',
                  fontsize=14, fontweight='bold', pad=10)

# ── Panel 2: Raw PID output ──
axes[1].fill_between(times, duty, 0, where=duty >= 0,
                     color='#ff6b6b', alpha=0.2)
axes[1].fill_between(times, duty, 0, where=duty < 0,
                     color='#4dabf7', alpha=0.2)
axes[1].plot(times, duty, color='#868e96', linewidth=0.5, alpha=0.3,
             label='Raw')
axes[1].plot(times, duty_smooth, color='#2f9e44', linewidth=1.5,
             label='PID output (filtered)')
axes[1].axhline(0, color='black', linewidth=0.8)
axes[1].axhline(1, color='#ff6b6b', linewidth=0.5, linestyle=':',
                alpha=0.5)
axes[1].axhline(-1, color='#4dabf7', linewidth=0.5, linestyle=':',
                alpha=0.5)
axes[1].set_ylabel('PID Output [-1, +1]', fontsize=11)
axes[1].set_ylim(-1.15, 1.15)
axes[1].legend(loc='upper right', fontsize=9)
axes[1].grid(True, alpha=0.2)

# Add labels on the sides
axes[1].text(times[0], 0.85, 'HEATING →', fontsize=8, color='#e03131',
             alpha=0.7)
axes[1].text(times[0], -0.85, '← COOLING', fontsize=8, color='#1971c2',
             alpha=0.7)

# ── Panel 3: Actuator split ──
axes[2].fill_between(times, heater_smooth, 0,
                     color='#ff6b6b', alpha=0.3)
axes[2].fill_between(times, fan_smooth, 0,
                     color='#4dabf7', alpha=0.3)
axes[2].plot(times, heater_actual, color='#ff6b6b', linewidth=0.4,
             alpha=0.3)
axes[2].plot(times, heater_smooth, color='#e03131', linewidth=1.5,
             label='Heater duty')
axes[2].plot(times, fan_display, color='#4dabf7', linewidth=0.4,
             alpha=0.3)
axes[2].plot(times, fan_smooth, color='#1971c2', linewidth=1.5,
             label='Fan duty')
axes[2].axhline(0, color='black', linewidth=0.8)
axes[2].set_ylabel('Actuator Duty', fontsize=11)
axes[2].set_xlabel('Time (seconds)', fontsize=11)
axes[2].set_ylim(-1.15, 1.15)
axes[2].legend(loc='upper right', fontsize=9)
axes[2].grid(True, alpha=0.2)

axes[2].text(times[-1], 0.85, 'HEATER ↑', fontsize=8, color='#e03131',
             alpha=0.7, ha='right')
axes[2].text(times[-1], -0.85, 'FAN ↓', fontsize=8, color='#1971c2',
             alpha=0.7, ha='right')

plt.tight_layout()
plt.savefig('thermal_bidirectional.png', dpi=150, bbox_inches='tight',
            facecolor='#f8f8f8')
plt.show()
