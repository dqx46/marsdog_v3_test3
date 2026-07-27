import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260628_170816.csv"

times, roll, p_mm, d_mm, sway, reactive = [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if 'trot_fwd' in r['mode'] and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))
                sway.append(float(r['lateral_sway_mm']))
                reactive.append(float(r['reactive_deg']))

times = np.array(times)
roll = np.array(roll)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)
sway = np.array(sway)
reactive = np.array(reactive)

# Find the longest continuous segment
diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
t_segs = np.split(times, split_indices)
roll_segs = np.split(roll, split_indices)
p_segs = np.split(p_mm, split_indices)
d_segs = np.split(d_mm, split_indices)
sway_segs = np.split(sway, split_indices)
react_segs = np.split(reactive, split_indices)

longest_idx = np.argmax([len(s) for s in t_segs])
t = t_segs[longest_idx]
r = roll_segs[longest_idx]
p_val = p_segs[longest_idx]
d_val = d_segs[longest_idx]
s_val = sway_segs[longest_idx]
react_val = react_segs[longest_idx]

t = t - t[0] # Normalize time

# Let's plot only a 4-second window to see the details clearly
mask = (t > 2.0) & (t < 6.0)
t = t[mask]
r = r[mask]
p_val = p_val[mask]
d_val = d_val[mask]
s_val = s_val[mask]
react_val = react_val[mask]

fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

axs[0].set_title("Body Roll (Sway) - 0.75s Period", fontsize=14)
axs[0].plot(t, r, 'g-', linewidth=2)
axs[0].set_ylabel("Roll (deg)")
axs[0].grid(True, linestyle=':', alpha=0.7)
axs[0].axhline(0, color='k', linestyle='--', alpha=0.5)

axs[1].set_title("IMU Compensation (Z-Height Adjustment)", fontsize=12)
axs[1].plot(t, p_val, 'b-', label="P-Term (Proportional)")
axs[1].plot(t, d_val, 'r-', label="D-Term (Damping)")
axs[1].plot(t, p_val + d_val, 'k--', linewidth=2, label="Total Z Comp")
axs[1].set_ylabel("Leg Ext (mm)")
axs[1].legend(loc='upper right')
axs[1].grid(True, linestyle=':', alpha=0.7)

axs[2].set_title("Lateral Sway (CoM Shift)", fontsize=12)
axs[2].plot(t, s_val, 'm-', linewidth=2)
axs[2].set_ylabel("Sway (mm)")
axs[2].grid(True, linestyle=':', alpha=0.7)

axs[3].set_title("Raibert Reactive Foot Placement", fontsize=12)
axs[3].plot(t, react_val, 'c-', linewidth=2)
axs[3].set_ylabel("Correction (deg)")
axs[3].set_xlabel("Time (s)")
axs[3].grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig("roll_analysis.png", dpi=150)
print("Plot saved to roll_analysis.png")
