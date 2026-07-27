import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260629_113120.csv"

times, roll, pitch, sway, act, tgt = [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == 'pace_fwd' and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))
                sway.append(float(r['lateral_sway_mm']))
                act.append(float(r['actual_deg']))
                tgt.append(float(r['target_deg']))

if not times:
    print("No pace data found.")
    exit()

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)
sway = np.array(sway)
act = np.array(act)
tgt = np.array(tgt)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
p = np.split(pitch, split_indices)[longest_idx]
s = np.split(sway, split_indices)[longest_idx]
a = np.split(act, split_indices)[longest_idx]
tg = np.split(tgt, split_indices)[longest_idx]
t = t - t[0]

print(f"--- Fly Analysis ---")
print(f"Duration: {t[-1]:.2f}s")
print(f"Roll Range: {np.min(r):.1f}° to {np.max(r):.1f}° (Amp: {np.max(r)-np.min(r):.1f}°)")
print(f"Pitch Range: {np.min(p):.1f}° to {np.max(p):.1f}° (Amp: {np.max(p)-np.min(p):.1f}°)")
print(f"Max Tracking Error (Motor 3): {np.max(np.abs(a - tg)):.1f}°")

plt.figure(figsize=(10, 8))
plt.subplot(3,1,1)
plt.plot(t, r, 'r-', label='Roll (deg)')
plt.legend()
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, p, 'b-', label='Pitch (deg)')
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, tg, 'k--', label='Target')
plt.plot(t, a, 'g-', label='Actual')
plt.legend()
plt.grid()

plt.tight_layout()
plt.savefig("fly_analysis.png")
