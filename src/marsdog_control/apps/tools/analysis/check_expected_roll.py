import csv
import numpy as np
import matplotlib.pyplot as plt
import math

LOG = "walk_log_20260629_113627.csv"

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

# Expected roll from sway
# sway is in mm. lat_offset = sway / 1000
# sway_angle = -lat_offset / body_height
# Wait, get_expected_roll returns: math.degrees(lat_offset / self.body_height)
# So expected_roll = (sway / 1000) / 0.24 * 180 / math.pi
exp = (s / 1000.0) / 0.24 * 180.0 / math.pi

# But there is a delay of 0.18s in get_expected_roll!
# We can approximate it by shifting `exp` by 0.18s
dt = np.mean(np.diff(t))
shift_idx = int(0.18 / dt)
exp_delayed = np.zeros_like(exp)
if shift_idx < len(exp):
    exp_delayed[shift_idx:] = exp[:-shift_idx]

eff = r - exp_delayed

plt.figure(figsize=(10, 6))
plt.plot(t, r, 'g-', label="Raw IMU Roll")
plt.plot(t, exp_delayed, 'm--', label="Expected Roll (Delayed)")
plt.plot(t, eff, 'b-', label="Effective Roll (Fed to Ctrl)")
plt.legend()
plt.grid()
plt.savefig("expected_roll_check.png")
