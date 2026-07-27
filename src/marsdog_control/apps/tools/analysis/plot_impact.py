import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260628_172304.csv"

times, roll, p_mm, d_mm, act, tgt = [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if 'trot_fwd' in r['mode'] and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))
                act.append(float(r['actual_deg']))
                tgt.append(float(r['target_deg']))

times = np.array(times)
roll = np.array(roll)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)
act = np.array(act)
tgt = np.array(tgt)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
p = np.split(p_mm, split_indices)[longest_idx]
d = np.split(d_mm, split_indices)[longest_idx]
a = np.split(act, split_indices)[longest_idx]
tg = np.split(tgt, split_indices)[longest_idx]

t = t - t[0]

# Zoom in to a 3 second window
mask = (t > 2.0) & (t < 5.0)
t = t[mask]
r = r[mask]
p = p[mask]
d = d[mask]
a = a[mask]
tg = tg[mask]

fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axs[0].set_title("Body Roll (Sway) - 0.75s Period, 0 Sway", fontsize=14)
axs[0].plot(t, r, 'g-', linewidth=2)
axs[0].set_ylabel("Roll (deg)")
axs[0].grid(True, linestyle=':', alpha=0.7)
axs[0].axhline(0, color='k', linestyle='--', alpha=0.5)

axs[1].set_title("IMU Compensation (Z-Height Adjustment)", fontsize=12)
axs[1].plot(t, p, 'b-', label="P-Term (Proportional)")
axs[1].plot(t, d, 'r-', label="D-Term (Damping)")
axs[1].plot(t, p + d, 'k--', linewidth=2, label="Total Z Comp")
axs[1].set_ylabel("Leg Ext (mm)")
axs[1].legend(loc='upper right')
axs[1].grid(True, linestyle=':', alpha=0.7)

axs[2].set_title("Motor 3 (FL Calf) Tracking & Impact", fontsize=12)
axs[2].plot(t, tg, 'r--', label="Target")
axs[2].plot(t, a, 'b-', label="Actual")
axs[2].set_ylabel("Angle (deg)")
axs[2].set_xlabel("Time (s)")
axs[2].legend(loc='upper right')
axs[2].grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig("impact_analysis.png", dpi=150)
