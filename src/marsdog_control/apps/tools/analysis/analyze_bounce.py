import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260629_113627.csv"

times, roll, pitch, p_mm, d_mm, react, sway, act, tgt = [], [], [], [], [], [], [], [], []
mode_filter = 'pace_fwd'
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == mode_filter and r['motor_id'] == '3': # Motor 3 = FL_calf or FL_hip? Wait, 3 is FL_calf usually, wait, let's check motor IDs.
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))
                react.append(float(r['reactive_deg']))
                sway.append(float(r['lateral_sway_mm']))
                act.append(float(r['actual_deg']))
                tgt.append(float(r['target_deg']))

if not times:
    print(f"No {mode_filter} data found.")
    exit()

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)
react = np.array(react)
sway = np.array(sway)
act = np.array(act)
tgt = np.array(tgt)

diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
p = np.split(pitch, split_indices)[longest_idx]
p_val = np.split(p_mm, split_indices)[longest_idx]
d_val = np.split(d_mm, split_indices)[longest_idx]
react_val = np.split(react, split_indices)[longest_idx]
s_val = np.split(sway, split_indices)[longest_idx]
a_val = np.split(act, split_indices)[longest_idx]
tg_val = np.split(tgt, split_indices)[longest_idx]

t = t - t[0]

print(f"--- Bounce Analysis ({mode_filter}) ---")
print(f"Duration: {t[-1]:.2f}s")
print(f"Roll Range: {np.min(r):.1f}° to {np.max(r):.1f}° (Amp: {np.max(r)-np.min(r):.1f}°)")
print(f"Pitch Range: {np.min(p):.1f}° to {np.max(p):.1f}° (Amp: {np.max(p)-np.min(p):.1f}°)")
print(f"Max P Comp: {np.max(np.abs(p_val)):.1f} mm")
print(f"Max D Comp: {np.max(np.abs(d_val)):.1f} mm")
print(f"Max Reactive: {np.max(np.abs(react_val)):.2f}°")
print(f"Max Tracking Error: {np.max(np.abs(a_val - tg_val)):.1f}°")

plt.figure(figsize=(12, 10))

plt.subplot(4,1,1)
plt.title("Roll & Pitch (deg)")
plt.plot(t, r, 'g-', label="Roll")
plt.plot(t, p, 'b-', label="Pitch")
plt.legend()
plt.grid()

plt.subplot(4,1,2)
plt.title("IMU Z-Compensation (mm)")
plt.plot(t, p_val, 'r-', label="P-Term")
plt.plot(t, d_val, 'm-', label="D-Term")
plt.legend()
plt.grid()

plt.subplot(4,1,3)
plt.title("Raibert Reactive (deg)")
plt.plot(t, react_val, 'c-', label="Reactive")
plt.legend()
plt.grid()

plt.subplot(4,1,4)
plt.title("Motor Tracking Error (deg)")
plt.plot(t, a_val - tg_val, 'k-', label="Error (Actual - Target)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.savefig("bounce_analysis.png")
print("Plot saved to bounce_analysis.png")
