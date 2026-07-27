import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

LOG = sys.argv[1]

times, roll, expected, react, sway, act, tgt = [], [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == 'pace_fwd' and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                expected.append(float(r['expected_roll']))
                react.append(float(r['reactive_deg']))
                sway.append(float(r['lateral_sway_mm']))
                act.append(float(r['actual_deg']))
                tgt.append(float(r['target_deg']))

if not times:
    print("No pace data found.")
    exit()

times = np.array(times)
roll = np.array(roll)
expected = np.array(expected)
react = np.array(react)
sway = np.array(sway)
act = np.array(act)
tgt = np.array(tgt)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
exp = np.split(expected, split_indices)[longest_idx]
re = np.split(react, split_indices)[longest_idx]
sw = np.split(sway, split_indices)[longest_idx]
a = np.split(act, split_indices)[longest_idx]
tg = np.split(tgt, split_indices)[longest_idx]
t = t - t[0]

print(f"--- Pace Analysis ---")
print(f"Duration: {t[-1]:.2f}s")
print(f"Raw Roll Range: {np.min(r):.1f}° to {np.max(r):.1f}°")
print(f"Expected Roll Range: {np.min(exp):.1f}° to {np.max(exp):.1f}°")
print(f"Max Reactive: {np.max(np.abs(re)):.2f}°")
print(f"Max Tracking Error: {np.max(np.abs(a - tg)):.1f}°")
