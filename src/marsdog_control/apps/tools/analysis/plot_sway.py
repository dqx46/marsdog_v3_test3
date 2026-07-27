import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260628_170816.csv"

times, roll, pitch, period, p_mm, d_mm = [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if 'trot_fwd' in r['mode'] and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))
                period.append(float(r['period_s']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
p = np.split(pitch, split_indices)[longest_idx]
p_val = np.split(p_mm, split_indices)[longest_idx]
d_val = np.split(d_mm, split_indices)[longest_idx]

t = t - t[0]

# Zoom in to a 4 second window
mask = (t > 2.0) & (t < 6.0)
t = t[mask]
r = r[mask]
p = p[mask]
p_val = p_val[mask]
d_val = d_val[mask]

plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.title(f"Body Sway (Roll & Pitch) - Period: {period[0]:.2f}s", fontsize=14)
plt.plot(t, r, 'g-', linewidth=2, label="Roll (Left/Right Sway)")
plt.plot(t, p, 'm-', linewidth=2, label="Pitch (Front/Back Tilt)")
plt.axhline(0, color='black', linestyle='--', alpha=0.5)
plt.ylabel("Angle (deg)", fontsize=12)
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.7)

plt.subplot(2, 1, 2)
plt.title("IMU Controller Compensation (Roll Axis)", fontsize=14)
plt.plot(t, p_val, 'b-', linewidth=1.5, label="P Term (Proportional)")
plt.plot(t, d_val, 'r-', linewidth=1.5, label="D Term (Damping)")
plt.plot(t, p_val + d_val, 'k--', linewidth=2, label="Total Compensation")
plt.axhline(0, color='black', linestyle='--', alpha=0.5)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Leg Extension (mm)", fontsize=12)
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig("sway_analysis.png", dpi=150)
