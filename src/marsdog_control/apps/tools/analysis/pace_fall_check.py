import csv
import numpy as np
import math
import matplotlib.pyplot as plt

LOG = "walk_log_20260629_111923.csv"

times, roll, sway = [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == 'pace_fwd' and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                sway.append(float(r['lateral_sway_mm']))

times = np.array(times)
roll = np.array(roll)
sway = np.array(sway)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
s = np.split(sway, split_indices)[longest_idx]
t = t - t[0]

plt.figure(figsize=(10, 6))
mask = (t > 2.0) & (t < 6.0)
plt.plot(t[mask], r[mask], 'g-', label='Actual Roll (deg)')
plt.plot(t[mask], s[mask] / 0.24 * 180 / math.pi, 'm--', label='Commanded Sway (deg equivalent)')
plt.legend()
plt.grid()
plt.savefig("pace_fall_check.png")
